#!/usr/bin/env python3
"""Extract evaluator-only world-space visual triangles from selected static SDF assets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Transform XYZ points while retaining their leading dimensions."""
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def pose_matrix(text: str | None) -> np.ndarray:
    """Convert an SDF xyz/rpy pose into a homogeneous transform."""
    x, y, z, roll, pitch, yaw = map(float, (text or "0 0 0 0 0 0").split())
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    matrix[:3, 3] = [x, y, z]
    return matrix


def obj_triangles(path: Path) -> np.ndarray:
    """Read OBJ polygon faces, triangulating each face with a vertex fan."""
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append(list(map(float, fields[1:4])))
        elif fields[0] == "f":
            indices = [int(token.split("/")[0]) for token in fields[1:]]
            indices = [i - 1 if i > 0 else len(vertices) + i for i in indices]
            faces.extend(
                (indices[0], indices[i], indices[i + 1])
                for i in range(1, len(indices) - 1)
            )
    if not vertices or not faces:
        raise ValueError(f"No visual triangles in {path}")
    return np.asarray(vertices, dtype=float)[np.asarray(faces)]


def dae_triangles(path: Path) -> np.ndarray:
    """Read COLLADA triangle geometry and its scene transforms (Z-up only).

    Unsupported geometry fails explicitly instead of silently changing ground truth.
    """
    root = ET.parse(path).getroot()
    # Strip namespaces locally so source assets remain untouched.
    for element in root.iter():
        element.tag = element.tag.split("}")[-1]
    if root.findtext("asset/up_axis", "Y_UP") != "Z_UP":
        raise ValueError(f"Only Z_UP COLLADA is supported: {path}")
    unit = root.find("asset/unit")
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    meshes = {}
    for geometry in root.findall("library_geometries/geometry"):
        mesh = geometry.find("mesh")
        sources = {}
        for source in mesh.findall("source"):
            accessor = source.find("technique_common/accessor")
            stride = int(accessor.get("stride", "1"))
            values = np.fromstring(source.findtext("float_array"), sep=" ")
            offset = int(accessor.get("offset", "0"))
            sources[source.get("id")] = values[offset:].reshape(-1, stride)
        vertices = {
            v.get("id"): v.find("input[@semantic='POSITION']").get("source")[1:]
            for v in mesh.findall("vertices")
        }
        parts = []
        for primitive in mesh:
            if primitive.tag in ("source", "vertices"):
                continue
            if primitive.tag != "triangles":
                raise ValueError(
                    f"Unsupported COLLADA primitive {primitive.tag}: {path}"
                )
            inputs = primitive.findall("input")
            vertex = primitive.find("input[@semantic='VERTEX']")
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            indices = np.fromstring(
                primitive.findtext("p"), sep=" ", dtype=int
            ).reshape(-1, stride)
            indices = indices[:, int(vertex.get("offset", "0"))].reshape(-1, 3)
            parts.append(sources[vertices[vertex.get("source")[1:]]][:, :3][indices])
        meshes[geometry.get("id")] = np.concatenate(parts)
    parts = []

    def visit(node: ET.Element, parent: np.ndarray) -> None:
        matrix = np.eye(4)
        for child in node:
            change = np.eye(4)
            if child.tag == "matrix":
                change = np.fromstring(child.text, sep=" ").reshape(4, 4)
            elif child.tag == "translate":
                change[:3, 3] = np.fromstring(child.text, sep=" ")
            elif child.tag == "scale":
                change[np.arange(3), np.arange(3)] = np.fromstring(child.text, sep=" ")
            elif child.tag == "rotate":
                vector = np.fromstring(child.text, sep=" ")
                axis = vector[:3] / np.linalg.norm(vector[:3])
                angle = np.deg2rad(vector[3])
                cross = np.array(
                    [
                        [0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0],
                    ]
                )
                change[:3, :3] = (
                    np.eye(3) * np.cos(angle)
                    + (1 - np.cos(angle)) * np.outer(axis, axis)
                    + np.sin(angle) * cross
                )
            elif child.tag in (
                "lookat",
                "skew",
                "instance_controller",
                "instance_node",
            ):
                raise ValueError(f"Unsupported COLLADA transform {child.tag}: {path}")
            else:
                continue
            matrix = matrix @ change
        matrix = parent @ matrix
        for instance in node.findall("instance_geometry"):
            parts.append(transform(meshes[instance.get("url")[1:]], matrix))
        for child in node.findall("node"):
            visit(child, matrix)

    scene_url = root.find("scene/instance_visual_scene").get("url")[1:]
    scene = root.find(f"library_visual_scenes/visual_scene[@id='{scene_url}']")
    for node in scene.findall("node"):
        visit(node, np.eye(4))
    return np.concatenate(parts) * scale


def extract_task(
    world_file: Path, models_root: Path, targets: list[dict], output_directory: Path
) -> dict:
    """Save world-space target triangles and source hashes for one trial.

    :param world_file: Original SDF world containing named include instances.
    :param models_root: Directory resolving model:// asset URIs.
    :param targets: Task targets with visibility.instances and visibility.policy.
    :param output_directory: Directory for ground_truth.json and ground_truth_meshes.npz.
    :return: Metadata describing the extracted visual geometry.
    """
    world_file, models_root = Path(world_file), Path(models_root)
    world = ET.parse(world_file).find("world")
    includes = {i.findtext("name"): i for i in world.findall("include")}
    arrays, objects, target_metadata = {}, [], []
    for target in targets:
        visibility = target.get("visibility")
        if not visibility:
            raise ValueError(
                f"Target {target['id']} lacks visibility.instances; approximate region centers are not geometry"
            )
        instances = visibility["instances"]
        policy = visibility.get("policy", "any")
        if policy not in ("all", "any") or not instances:
            raise ValueError(f"Invalid visibility selection for {target['id']}")
        target_metadata.append(
            {"id": target["id"], "policy": policy, "instances": instances}
        )
        for name in instances:
            include = includes[name]
            uri = include.findtext("uri")
            if not uri.startswith("model://"):
                raise ValueError(f"Unsupported model URI: {uri}")
            sdf = models_root / uri.removeprefix("model://") / "model.sdf"
            model = ET.parse(sdf).find("model")
            if (
                include.findtext("static", model.findtext("static", "false")).lower()
                != "true"
            ):
                raise ValueError(f"Ground truth requires a static instance: {name}")
            if model.findall("model") or model.findall("include"):
                raise ValueError(f"Nested SDF models are not supported: {name}")
            parts, assets = [], []
            for link in model.findall("link"):
                for visual in link.findall("visual"):
                    mesh = visual.find("geometry/mesh")
                    if mesh is None:
                        raise ValueError(f"Unsupported non-mesh visual in {name}")
                    for element in (include, model, link, visual):
                        pose = element.find("pose")
                        if pose is not None and pose.get("relative_to"):
                            raise ValueError(f"Unsupported relative_to in {name}")
                    if mesh.find("submesh") is not None:
                        raise ValueError(f"Mesh sub-selection is not supported: {name}")
                    mesh_uri = mesh.findtext("uri")
                    if not mesh_uri.startswith("model://"):
                        raise ValueError(f"Unsupported visual mesh URI: {mesh_uri}")
                    mesh_path = models_root / mesh_uri.removeprefix("model://")
                    if mesh_path.suffix not in (".obj", ".dae"):
                        raise ValueError(f"Unsupported mesh format: {mesh_path}")
                    triangles = (
                        obj_triangles(mesh_path)
                        if mesh_path.suffix == ".obj"
                        else dae_triangles(mesh_path)
                    )
                    triangles *= np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ")
                    matrix = (
                        pose_matrix(include.findtext("pose"))
                        @ pose_matrix(model.findtext("pose"))
                        @ pose_matrix(link.findtext("pose"))
                        @ pose_matrix(visual.findtext("pose"))
                    )
                    parts.append(transform(triangles, matrix))
                    assets.append(
                        {
                            "uri": mesh_uri,
                            "sha256": hashlib.sha256(
                                mesh_path.read_bytes()
                            ).hexdigest(),
                        }
                    )
            triangles = np.concatenate(parts).astype(np.float64)
            key = f"mesh_{len(objects)}"
            arrays[key] = triangles
            bounds = np.stack([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))])
            objects.append(
                {
                    "instance": name,
                    "target_id": target["id"],
                    "mesh_key": key,
                    "center": bounds.mean(axis=0).tolist(),
                    "size": (bounds[1] - bounds[0]).tolist(),
                    "triangle_count": len(triangles),
                    "provenance": {
                        "assets": assets,
                        "model_sdf": str(sdf.relative_to(models_root)),
                        "model_sdf_sha256": hashlib.sha256(
                            sdf.read_bytes()
                        ).hexdigest(),
                        "include_pose": include.findtext("pose", "0 0 0 0 0 0"),
                    },
                }
            )
    metadata = {
        "version": 1,
        "frame_id": "map",
        "mesh_file": "ground_truth_meshes.npz",
        "world_file": world_file.name,
        "world_sha256": hashlib.sha256(world_file.read_bytes()).hexdigest(),
        "objects": objects,
        "targets": target_metadata,
        "limitations": "Static visual geometry; depth consistency is geometric evidence, not semantic recognition. Human completion review remains required.",
    }
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_directory / metadata["mesh_file"], **arrays)
    (output_directory / "ground_truth.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    return metadata


def enrich_trial(
    run_directory: Path, manifest: dict, worlds_root: Path, overwrite: bool = False
) -> dict:
    """Add frozen visibility sidecars to an older trial without changing its task.

    :param run_directory: Existing trial folder containing trial.json.
    :param manifest: Manifest supplying explicit instance identities and thresholds.
    :param worlds_root: Source gz_sim_worlds directory.
    :param overwrite: Explicitly permit replacing prior visibility geometry.
    :return: Extracted geometry and enrichment provenance.
    """
    run_directory, worlds_root = Path(run_directory), Path(worlds_root)
    if (run_directory / "ground_truth.json").exists() and not overwrite:
        raise FileExistsError(
            "Ground truth already exists; use --overwrite only for intentional regeneration"
        )
    trial = json.loads((run_directory / "trial.json").read_text())
    fixture = manifest["worlds"][trial["world_id"]]
    task = fixture["tasks"][trial["task_id"]]
    for key in ("prompt", "stages"):
        if task[key] != trial["task"][key]:
            raise ValueError(f"Manifest {key} differs from frozen trial")
    if [t["id"] for t in task["targets"]] != [
        t["id"] for t in trial["task"]["targets"]
    ]:
        raise ValueError("Manifest target IDs differ from frozen trial")
    world_file = run_directory / "world.sdf"
    expected = trial.get("evaluated_world_sha256")
    if not world_file.exists():
        world_file = worlds_root / "worlds" / fixture["world"]
        expected = trial.get("world_sha256")
    if not expected or hashlib.sha256(world_file.read_bytes()).hexdigest() != expected:
        raise ValueError(
            "World hash differs from frozen trial; do not reinterpret old data with modified assets"
        )
    result = extract_task(
        world_file, worlds_root / "models", task["targets"], run_directory
    )
    result["visibility"] = trial.get("visibility", manifest.get("visibility", {}))
    result["enrichment"] = {
        "trial_sha256": hashlib.sha256(
            (run_directory / "trial.json").read_bytes()
        ).hexdigest(),
        "source": "explicit manifest instance identities; original trial unchanged",
        "historical_asset_provenance_verified": False,
        "asset_provenance_note": "World hash matches the frozen trial. Mesh/model hashes describe current checkout assets; their equality to assets at original recording time has not been independently verified.",
    }
    (run_directory / "ground_truth.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


def main() -> None:
    """Extract one manifest fixture or enrich a frozen trial without starting ROS."""
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--worlds-root", type=Path, required=True)
    parser.add_argument("--world")
    parser.add_argument("--task")
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--trial", type=Path, help="Enrich an existing trial without editing trial.json"
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    manifest = yaml.safe_load(args.manifest.read_text())
    if args.trial:
        result = enrich_trial(args.trial, manifest, args.worlds_root, args.overwrite)
        output = args.trial
    else:
        if not args.world or not args.task or not args.output:
            parser.error("Provide --trial or all of --world, --task and --output")
        if (args.output / "ground_truth.json").exists() and not args.overwrite:
            parser.error("Ground truth already exists; pass --overwrite to replace it")
        fixture = manifest["worlds"][args.world]
        result = extract_task(
            args.worlds_root / "worlds" / fixture["world"],
            args.worlds_root / "models",
            fixture["tasks"][args.task]["targets"],
            args.output,
        )
        output = args.output
    print(f"Extracted {len(result['objects'])} static object instances to {output}")


if __name__ == "__main__":
    main()
