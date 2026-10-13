# HM3D scenes in Gazebo

HM3D scenes can use the existing Unipilot robot, RGB-D camera, LiDAR, controller,
and ROS bridges. The importer converts the textured Basis GLB into Gazebo OBJ,
MTL and PNG assets, with the same geometry used for static collision.
Habitat is not required at runtime. Import one scene at a time rather than
decoding the entire dataset.

All generated files are stored under `generated/` (ignored by Git). The source
dataset is read-only, and the importer rejects output anywhere on
`/media/albert/ExtremeAlbert1`, including paths redirected there by symlinks.
Existing imports cannot be overwritten. Generated scans retain HM3D's licensing
and access restrictions and should not be committed or redistributed.

## Convert a complete split

The batch importer uses an existing Habitat Python environment to read each
scene's navigation mesh. It samples 256 points on the largest navigable island,
selects the point with greatest floor-level obstacle clearance, and places the
robot 1.5 m above it. Sampling uses seed 42. These are suggested initial poses;
floor navigability does not guarantee aerial clearance or sensor visibility.
The poses are saved in `generated/hm3d/val_spawns.json` for repeatability.

```bash
python3 scripts/import_hm3d_split.py \
  --dataset /media/albert/ExtremeAlbert1/habitat-sim-data/versioned_data/hm3d-0.2 \
  --split val \
  --habitat-python /home/albert/miniforge3/envs/grapheqa/bin/python \
  --workers 3
```

Run the same command to resume. Completed imports are retained and their source
checksums verified. The progress/failure report is
`generated/hm3d/val_conversion_report.json`. New imports stop below an 8 GiB
free-space reserve; active imports may consume additional space. Converted assets
are considerably larger than the compressed GLB files. `--height`, `--seed` and
`--reserve-gib` can be set on the command line; height/seed affect only newly
created spawn manifests. To use different poses, supply a different output
directory or edit the saved spawn manifest before importing new scenes.

Launch a converted validation scene by its original directory id, for example:

```bash
HM3D_SCENE=00800-TEEsavR23oF make launch \
  DOCKER_COMPOSE_FILE=docker-compose.uav_nmpc_unipilot_hm3d_sim.yml
```

All 100 validation scenes have been converted locally: approximately 16.7 GiB
of models, 39,312,379 triangles and 4,910 textures. The full-split asset audit
verified source checksums, world/model XML, mesh presence, texture references and
image integrity; results are in `generated/hm3d/val_asset_validation.json`.
This is an asset validation, not a live sensor/navigation test of every scene.

## Prepare and import

Run from the stack root. The decoder build needs Git, CMake and a C++ compiler;
the importer needs Python 3 and NumPy.

```bash
bash scripts/setup_hm3d_tools.sh

python3 workspaces/ws_sim/src/gz_sim_worlds/scripts/import_hm3d.py \
  --dataset /media/albert/ExtremeAlbert1/habitat-sim-data/versioned_data/hm3d-0.2 \
  --list

python3 workspaces/ws_sim/src/gz_sim_worlds/scripts/import_hm3d.py \
  --dataset /media/albert/ExtremeAlbert1/habitat-sim-data/versioned_data/hm3d-0.2 \
  --scene 00001-UVdNNRcVyV1 \
  --basisu generated/hm3d_tools/basis_universal/bin/basisu \
  --output generated/hm3d \
  --spawn -0.30367583 0.106891215 1.65 --spawn-yaw 2.52897
```

That scene is already imported locally. To import another scene, change the
scene id and supply a known free-space spawn. Native HM3D meshes are Z-up and in
metres; no scene rotation is applied. Habitat agent coordinates `(x, y, z)` map
to Gazebo `(x, -z, y)`. Choose Z at the desired robot height above the local floor.
Yaw is in radians about Gazebo Z. An optional `--scene-pose X Y Z ROLL PITCH YAW`
transforms the entire mesh; the spawn must then be expressed in that transformed
world frame. No automatic safe-spawn or navigation validation is performed.

The output contains a world SDF, a model directory with mesh/textures, and JSON
metadata with the spawn, geometry counts/bounds and source SHA-256. If reimporting
an existing scene, choose a new output directory.

## Build and launch

Build the affected simulation packages after updating the source:

```bash
make build-sim

HM3D_SCENE=00001-UVdNNRcVyV1 make launch \
  DOCKER_COMPOSE_FILE=docker-compose.uav_nmpc_unipilot_hm3d_sim.yml
```

The Compose file inherits the existing Unipilot stack and overrides only the
Gazebo launch. It mounts `generated/hm3d` read-only at `/hm3d`; the original dataset
drive is not mounted. Use `HM3D_ASSETS=/absolute/path/to/converted/assets` for a
different output directory. Its `worlds/` and `models/` must remain siblings.
Missing assets fail with an error rather than creating an empty directory.

```bash
make stop DOCKER_COMPOSE_FILE=docker-compose.uav_nmpc_unipilot_hm3d_sim.yml
```

The normal subt and RMF simulation Compose files continue to use their existing
worlds. The HM3D launch passes the imported world name into the existing sensor
bridges, so their world-dependent Gazebo topic names resolve correctly.

## Sensor topics and repeatable check

The existing ROS 2 topics remain:

| Data | Topic |
| --- | --- |
| RGB | `/rmf_unipilot/cam/rgb` |
| Metric depth, `32FC1` metres | `/rmf_unipilot/cam/depth` |
| RGB calibration | `/rmf_unipilot/cam/rgb/camera_info` |
| Depth calibration | `/rmf_unipilot/cam/camera_info` |
| LiDAR | `/rmf_unipilot/lidar/points` |
| Poses | `/tf`, `/tf_static` |

The existing ROS 1 bridge and RViz displays can consume these as in other
Unipilot worlds. To capture and validate real data, run the check from a Humble
container on the simulation's ROS domain. From the stack root while it runs:

```bash
container=$(docker ps --filter label=com.docker.compose.service=ros2_launch_agentic_uas \
  --format '{{.ID}}')
docker cp scripts/check_sim_sensors.py "$container":/tmp/check_sim_sensors.py
docker exec "$container" bash -lc \
  'source /opt/ros/humble/setup.bash && python3 /tmp/check_sim_sensors.py --output /tmp/hm3d-check --timeout 120 --frames 10'
docker cp "$container":/tmp/hm3d-check generated/hm3d-validation-repeat
```

Select the intended container explicitly if multiple stacks are running. The
script exits nonzero on failure and saves RGB, metric depth and LiDAR arrays,
a depth preview, and `sensor_report.json`. It checks calibration, nonblank textured
RGB, variable positive depth, spatial LiDAR returns, and an exact RGB/depth pair
with camera TF at capture time. Reported rates use simulated timestamps.

## Verified scene

Gazebo Harmonic with scene `00001-UVdNNRcVyV1`, the spawn above, and the existing
Unipilot robot passed the live check with at least ten messages per stream:

- RGB/depth: 480×270, both 20 Hz, matching capture stamps and valid camera TF.
- Depth: 99.4% valid pixels, 1.36–7.48 m in the saved frame.
- LiDAR: 10 Hz, 23,544 valid points from 24,000 rays.
- Textured interior visually inspected; source GLB checksum unchanged.

Evidence is in `generated/hm3d_validation/`. The imported scene has 306,339
triangles and occupies approximately 296 MiB after texture decoding. Other
scenes have not been individually tested. Scan holes and incomplete surfaces
remain part of the source geometry; this importer does not repair them. Sensor
data is validated here, not autonomous navigation performance throughout the scan.
