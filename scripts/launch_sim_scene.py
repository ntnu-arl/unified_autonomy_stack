#!/usr/bin/env python3
"""Launch one local scene through Compose, without running an evaluation trial."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
COLLECTIONS = ('hm3d', 'gz_sim_worlds', 'subt_cave_sim')
SPAWN_CONFIG = ROOT / 'workspaces/robot_bringup/config/simulation/scene_spawns.yaml'


def scene_plan(collection: str, scene: str, spawn: str = '', assets: Path | None = None) -> dict:
    """Resolve a local scene and validated world-frame spawn.

    :param collection: Scene collection name.
    :param scene: Filename, filename stem, or imported HM3D scene id.
    :param spawn: Optional space-separated x y z yaw, in metres and radians.
    :param assets: Directory of converted HM3D assets.
    :return: Simulator command and any extra read-only mounts.
    """
    if collection not in COLLECTIONS:
        raise ValueError(f'Unknown collection; choose {", ".join(COLLECTIONS)}')
    if not scene:
        scene = {'hm3d': '00800-TEEsavR23oF', 'gz_sim_worlds': 'rmf_office',
                 'subt_cave_sim': 'darpa_cave_01'}[collection]
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', scene) or scene in {'.', '..'}:
        raise ValueError('SCENE must be a local world filename or scene id, without directories')
    presets = yaml.safe_load(SPAWN_CONFIG.read_text())
    physics_step = presets['physics_step_sec']
    if (type(physics_step) not in (int, float) or not math.isfinite(physics_step)
            or physics_step <= 0):
        raise ValueError('physics_step_sec must be a finite positive number')
    volumes = []
    pose = None
    extra = []
    if collection == 'hm3d':
        assets = (assets or ROOT / 'generated/hm3d').resolve()
        model = 'hm3d_' + scene.replace('-', '_')
        world = assets / 'worlds' / f'{model}.sdf'
        metadata = json.loads(world.with_suffix('.json').read_text())
        if metadata['scene_id'] != scene or metadata['model_name'] != model:
            raise ValueError('HM3D metadata does not match the requested scene')
        pose = [*metadata['spawn']['position'], metadata['spawn']['yaw']]
        package = 'gz_sim_worlds'
        extra = ['extra_resource_path:=/hm3d/models']
        volumes = [{'type': 'bind', 'source': str(assets), 'target': '/hm3d',
                    'read_only': True, 'bind': {'create_host_path': False}}]
    else:
        directory = ROOT / 'workspaces/ws_sim/src' / collection / 'worlds'
        candidates = ([directory / scene] if Path(scene).suffix else
                      [directory / (scene + suffix) for suffix in ('.sdf', '.world')])
        matches = [path for path in candidates if path.is_file()]
        if len(matches) != 1:
            raise ValueError(f'Scene absent or ambiguous in {directory}; use an exact filename from make list-scenes')
        world = matches[0]
        package = collection
        pose = presets.get(collection, {}).get(world.name)
    if not world.is_file():
        raise ValueError(f'World missing: {world}; import/build the scene first')
    worlds = ET.parse(world).getroot().findall('world')
    if len(worlds) != 1 or not worlds[0].get('name'):
        raise ValueError(f'Expected exactly one named Gazebo world in {world}')
    if spawn:
        pose = [float(value) for value in spawn.split()]
    if pose is None:
        raise ValueError('No saved spawn for this world. Supply SPAWN="x y z yaw" using a known free-space position')
    if len(pose) != 4 or not all(math.isfinite(value) for value in pose):
        raise ValueError('SPAWN must contain four finite values: world-frame x y z and yaw in radians')
    runtime_world = ROOT / 'generated/simulation/worlds' / collection / world.name
    volumes.append({'type': 'bind', 'source': str(runtime_world),
                    'target': '/simulation/world.sdf', 'read_only': True,
                    'bind': {'create_host_path': False}})
    command = ['ros2', 'launch', 'robot_bringup', 'uav_sim_unipilot.launch.xml',
               f'world_package:={package}', f'world:={world.stem}',
               'world_file:=/simulation/world.sdf', f'world_name:={worlds[0].get("name")}',
               *[f'{name}:={value}' for name, value in zip(('x', 'y', 'z', 'Y'), pose)],
               'R:=0.0', 'P:=0.0', *extra]
    return {'collection': collection, 'scene': scene, 'world': str(world),
            'world_name': worlds[0].get('name'), 'spawn': pose,
            'command': command, 'volumes': volumes,
            'physics_step_sec': physics_step, 'runtime_world': str(runtime_world)}


def prepare_world(plan: dict) -> None:
    """Create a runtime world with the configured UAV physics step.

    :param plan: Resolved source world, runtime path and physics step.
    :return: None; the source dataset world is left untouched.
    """
    tree = ET.parse(plan['world'])
    world = tree.getroot().find('world')
    physics = world.find('physics')
    if physics is None:
        physics = ET.SubElement(world, 'physics', name='uav_physics', type='ode')
    step = physics.find('max_step_size')
    if step is None:
        step = ET.SubElement(physics, 'max_step_size')
    step.text = str(plan['physics_step_sec'])
    path = Path(plan['runtime_world'])
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding='utf-8', xml_declaration=True)


def list_scenes(assets: Path) -> None:
    """Print available scene filenames/ids without requiring Docker.

    :param assets: Converted HM3D assets directory.
    :return: None.
    """
    for collection in COLLECTIONS:
        directory = (assets / 'worlds' if collection == 'hm3d' else
                     ROOT / 'workspaces/ws_sim/src' / collection / 'worlds')
        names = []
        for path in sorted(directory.glob('*')):
            if collection == 'hm3d' and path.suffix == '.json':
                names.append(json.loads(path.read_text())['scene_id'])
            elif collection != 'hm3d' and path.suffix in {'.sdf', '.world'}:
                names.append(path.name)
        print(f'{collection}:\n  ' + '\n  '.join(names or ['No imported scenes']))


def main() -> None:
    """Resolve Makefile settings and run the unchanged autonomy services.

    :return: None; exit with the Compose process result.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    assets = Path(os.environ.get('HM3D_ASSETS') or ROOT / 'generated/hm3d')
    if args.list:
        list_scenes(assets)
        return
    try:
        plan = scene_plan(os.environ.get('COLLECTION') or 'gz_sim_worlds',
                          os.environ.get('SCENE', ''), os.environ.get('SPAWN', ''), assets)
    except (ValueError, OSError, KeyError, ET.ParseError) as error:
        parser.error(str(error))
    print(f"{plan['collection']}/{plan['scene']}; world={plan['world_name']}; "
          f"spawn={plan['spawn']}; physics_step={plan['physics_step_sec']} s", flush=True)
    print(shlex.join(plan['command']), flush=True)
    if args.dry_run:
        return
    try:
        prepare_world(plan)
    except (ValueError, OSError, ET.ParseError) as error:
        parser.error(str(error))
    compose = ['docker', 'compose', '--project-directory', str(ROOT),
               '-f', str(ROOT / 'docker-compose.uav_nmpc_unipilot_sim.yml')]
    scene_graph = (os.environ.get('WITH_SCENE_GRAPH') or 'false').lower()
    if scene_graph not in {'true', 'false'}:
        parser.error('WITH_SCENE_GRAPH must be true or false')
    if scene_graph == 'true':
        compose += ['-f', str(ROOT / 'docker-compose.scene_graph.yml')]
    override = {'services': {'ros2_launch_uav_sim': {
        'command': plan['command'], 'volumes': plan['volumes']}}}
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', prefix='autonomy-scene-') as stream:
        json.dump(override, stream)
        stream.flush()
        try:
            result = subprocess.run([*compose, '-f', stream.name, '--profile', 'launch', 'up',
                                     *shlex.split(os.environ.get('DOCKER_ARGS', ''))], cwd=ROOT)
        except KeyboardInterrupt:
            raise SystemExit(130)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
