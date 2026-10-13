#!/usr/bin/env python3
"""Convert an HM3D split with deterministic navmesh spawns and resumable imports."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
IMPORTER = ROOT / 'workspaces/ws_sim/src/gz_sim_worlds/scripts/import_hm3d.py'
sys.path.insert(0, str(IMPORTER.parent))
from import_hm3d import safe_destination, sha256  # noqa: E402

# Runs in an existing Habitat environment; it never starts or writes a simulator.
SPAWNS = '''
import habitat_sim, json, math, pathlib, sys
root, height, seed = pathlib.Path(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3])
poses = {}
for source in sorted(root.rglob('*.basis.glb')):
    navmesh = source.with_suffix('.navmesh')
    finder = habitat_sim.PathFinder()
    if not finder.load_nav_mesh(str(navmesh)) or not finder.num_islands:
        raise RuntimeError('Cannot load navmesh: ' + str(navmesh))
    island = max(range(finder.num_islands), key=finder.island_area)
    finder.seed(seed)
    points = [finder.get_random_navigable_point(island_index=island) for _ in range(256)]
    point = max(points, key=lambda p: finder.distance_to_closest_obstacle(p, 3.0))
    x, y, z = map(float, point)
    poses[source.parent.name] = {'position': [x, -z, y + height], 'yaw': 0.0,
        'habitat_floor_position': [x, y, z], 'navmesh_island': island,
        'floor_clearance_m': finder.distance_to_closest_obstacle(point, 3.0)}
print(json.dumps(poses))
'''


def main() -> None:
    """Import every scene, preserve completed imports, and record a batch report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True, help='HM3D root containing hm3d/val or val')
    parser.add_argument('--split', choices=['train', 'val'], default='val')
    parser.add_argument('--output', type=Path, default=ROOT / 'generated/hm3d')
    parser.add_argument('--basisu', type=Path, default=ROOT / 'generated/hm3d_tools/basis_universal/bin/basisu')
    parser.add_argument('--habitat-python', required=True, help='Python executable with habitat_sim installed')
    parser.add_argument('--height', type=float, default=1.5, help='Robot height above the navmesh floor, metres')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--reserve-gib', type=float, default=8, help='Stop new imports below this free-space reserve')
    args = parser.parse_args()
    if args.height <= 0 or args.workers < 1 or args.reserve_gib < 0:
        parser.error('height/workers must be positive and reserve-gib nonnegative')
    output = safe_destination(args.output, args.dataset)
    split = args.dataset / args.split
    if not split.is_dir():
        split = args.dataset / 'hm3d' / args.split
    sources = sorted(split.rglob('*.basis.glb'))
    if not sources:
        parser.error(f'No Basis GLB scenes found in {split}')
    output.mkdir(parents=True, exist_ok=True)
    poses_path = output / f'{args.split}_spawns.json'
    if poses_path.exists():
        poses = json.loads(poses_path.read_text())
    else:
        result = subprocess.run([args.habitat_python, '-c', SPAWNS, str(split), str(args.height), str(args.seed)],
                                capture_output=True, text=True, check=True)
        poses = json.loads(result.stdout)
        poses_path.write_text(json.dumps(poses, indent=2) + '\n')
    missing = [source.parent.name for source in sources if source.parent.name not in poses]
    if missing:
        raise ValueError(f'Saved spawn manifest is missing scenes: {missing}')
    report = {'split': args.split, 'dataset': str(args.dataset.resolve()), 'scenes': {},
              'spawn_manifest': str(poses_path), 'reserve_gib': args.reserve_gib}
    report_path = output / f'{args.split}_conversion_report.json'

    def convert(source: Path) -> tuple[str, dict]:
        """Verify an existing import or convert one scene in an isolated process."""
        scene = source.parent.name
        model = 'hm3d_' + scene.replace('-', '_')
        world = output / 'worlds' / f'{model}.sdf'
        metadata = world.with_suffix('.json')
        digest = sha256(source)
        if metadata.exists():
            saved = json.loads(metadata.read_text())
            if saved['source_sha256'] != digest or not world.is_file() or not (output / 'models' / model / 'meshes/scene.obj').is_file():
                raise ValueError('Existing import is incomplete or source checksum differs')
            return scene, {'status': 'existing', 'source_sha256': digest}
        if shutil.disk_usage(output).free < args.reserve_gib * 2**30:
            raise RuntimeError('Free disk space below configured reserve; resume after freeing space')
        pose = poses[scene]
        command = [sys.executable, str(IMPORTER), '--dataset', str(split), '--scene', scene,
                   '--output', str(output), '--basisu', str(args.basisu), '--spawn',
                   *map(str, pose['position']), '--spawn-yaw', str(pose['yaw'])]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        if sha256(source) != digest:
            raise RuntimeError('Source checksum changed during import')
        saved = json.loads(metadata.read_text())
        return scene, {'status': 'converted', 'source_sha256': digest, 'triangles': saved['triangles'],
                       'texture_count': saved['texture_count']}

    print(f'Converting {len(sources)} {args.split} scenes into {output}', flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        jobs = {executor.submit(convert, source): source.parent.name for source in sources}
        for number, job in enumerate(as_completed(jobs), 1):
            scene = jobs[job]
            try:
                _, entry = job.result()
            except Exception as error:
                entry = {'status': 'failed', 'error': str(error)}
            report['scenes'][scene] = entry
            report_path.write_text(json.dumps(report, indent=2) + '\n')
            print(f'[{number}/{len(sources)}] {scene}: {entry["status"]}' +
                  (f' — {entry["error"]}' if 'error' in entry else ''), flush=True)
    failed = sum(entry['status'] == 'failed' for entry in report['scenes'].values())
    print(f'Completed: {len(sources) - failed}; failed: {failed}; report: {report_path}', flush=True)
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
