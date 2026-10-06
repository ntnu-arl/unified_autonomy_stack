"""Verify fixture geometry extraction independently of ROS and Gazebo."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "ground_truth_assets", ROOT / "scripts/ground_truth_assets.py"
)
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


class GeometryTests(unittest.TestCase):
    def test_obj_negative_indices_and_fan(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "quad.obj"
            path.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf -4 -3 -2 -1\n")
            triangles = assets.obj_triangles(path)
            self.assertEqual(triangles.shape, (2, 3, 3))
            np.testing.assert_equal(triangles[1], [[0, 0, 0], [1, 1, 0], [0, 1, 0]])

    def test_pose_rotation_translation(self):
        output = assets.transform(
            np.array([[1.0, 0, 0]]), assets.pose_matrix("2 3 4 0 0 1.5707963267948966")
        )
        np.testing.assert_allclose(output, [[2, 4, 4]], atol=1e-12)

    def test_collada_scene_transform_and_units(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "triangle.dae"
            path.write_text("""<COLLADA><asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="g"><mesh><source id="p"><float_array>0 0 0 1 0 0 0 1 0</float_array><technique_common><accessor stride="3"/></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2</p></triangles></mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="s"><node><scale>2 2 2</scale><node><matrix>1 0 0 0 0 1 0 0 0 0 1 3 0 0 0 1</matrix><instance_geometry url="#g"/></node></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#s"/></scene></COLLADA>""")
            np.testing.assert_allclose(
                assets.dae_triangles(path),
                [[[0, 0, 0.06], [0.02, 0, 0.06], [0, 0.02, 0.06]]],
            )

    def test_extract_fixture_instances_and_hashes(self):
        import yaml

        manifest_path = (
            ROOT / "workspaces/robot_bringup/config/evaluation/agentic_benchmark.yaml"
        )
        worlds_root = ROOT / "workspaces/ws_sim/src/gz_sim_worlds"
        if not manifest_path.exists() or not worlds_root.exists():
            self.skipTest("Nested world/bringup checkout unavailable")
        manifest = yaml.safe_load(manifest_path.read_text())
        with tempfile.TemporaryDirectory() as folder:
            for world in manifest["worlds"].values():
                result = assets.extract_task(
                    worlds_root / "worlds" / world["world"],
                    worlds_root / "models",
                    world["tasks"]["sequential"]["targets"],
                    Path(folder),
                )
                with np.load(Path(folder) / result["mesh_file"]) as data:
                    self.assertEqual(len(data.files), len(result["objects"]))
                    for obj in result["objects"]:
                        self.assertTrue(np.isfinite(data[obj["mesh_key"]]).all())
                        self.assertGreater(obj["triangle_count"], 0)
                        self.assertEqual(len(obj["provenance"]["model_sdf_sha256"]), 64)
                self.assertEqual(
                    json.loads((Path(folder) / "ground_truth.json").read_text()), result
                )

    def test_enrichment_is_frozen_and_requires_matching_task(self):
        import hashlib

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "world.sdf").write_text('<sdf><world name="test"/></sdf>')
            digest = hashlib.sha256((root / "world.sdf").read_bytes()).hexdigest()
            task = {"prompt": "Explore", "stages": [], "targets": []}
            trial = {
                "world_id": "w",
                "task_id": "t",
                "task": task,
                "evaluated_world_sha256": digest,
            }
            text = json.dumps(trial)
            (root / "trial.json").write_text(text)
            manifest = {"worlds": {"w": {"tasks": {"t": task}}}}
            result = assets.enrich_trial(root, manifest, root)
            self.assertEqual(result["objects"], [])
            self.assertEqual((root / "trial.json").read_text(), text)
            with self.assertRaises(FileExistsError):
                assets.enrich_trial(root, manifest, root)
            manifest["worlds"]["w"]["tasks"]["t"] = dict(task, prompt="Different")
            with self.assertRaisesRegex(ValueError, "prompt differs"):
                assets.enrich_trial(root, manifest, root, overwrite=True)

    def test_missing_geometry_is_not_approximate_center(self):
        worlds_root = ROOT / "workspaces/ws_sim/src/gz_sim_worlds"
        if not worlds_root.exists():
            self.skipTest("Nested world checkout unavailable")
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "lacks visibility.instances"):
                assets.extract_task(
                    worlds_root / "worlds/rmf_office.sdf",
                    worlds_root / "models",
                    [{"id": "x", "position": [1, 2, 3]}],
                    Path(folder),
                )


if __name__ == "__main__":
    unittest.main()
