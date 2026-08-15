# quail setup notes: SLAM + GBPlanner + NMPC on the Orin NX

**Status: this is the first configuration confirmed running mimosa (SLAM) + gbplanner (planner) + NMPC
(control) together on the real quail hardware**, live-tested via `make launch`. The baseline was
committed on both repos (commit `first full working setup on quail` — `434972d` on `robot_bringup`
branch `quail/main`, `b2bef4b` on `unified_autonomy_stack`) on 2026-08-13. See §9 for tuning/fixes applied
*after* that commit (currently uncommitted) — most notably a real bug fix that corrects earlier guidance
in this same document (§2a used to say `flags.simulation: True` was fine to leave as-is; it was not). §10
adds a RealSense D455 as an auxiliary, unintegrated sensor (topics only, for rviz/recording) — not part
of the SLAM/planner/NMPC pipeline itself.

This documents everything that differs from a clean `main` checkout of `unified_autonomy_stack` and
`robot_bringup` to get **mimosa (SLAM) → gbplanner (planner) → NMPC (control, SDF collision avoidance
disabled) → PX4** running on quail. It covers both changes made specifically to wire up NMPC/gbplanner
(this pass) and pre-existing local, quail-specific hardware changes already on disk (documented here for
completeness, since "what's needed to set up on this robot" includes both).

No Docker images needed rebuilding for any of this — everything below is either a workspace (code) build
inside the existing images, or a config/launch-file change. CBF (`ws_cbf`/`ws_control`'s
`cbf_pc_selector`/`composite_cbf`) is intentionally **not** wired up — NMPC's own SDF-based avoidance is
present but disabled by default (see "NMPC behavior" below).

Section 0 below covers general Docker/JetPack/vcstool/image-build issues hit while first setting this box
up (unrelated to the NMPC/gbplanner work, but part of "what it took to get this stack running on quail") —
all verified against this checkout. `docs/installation.md` itself was **not** modified; everything lives
here instead.

---

## 0. Installation issues hit on this Orin NX (Docker / JetPack / vcstool / image build)

All verified against this checkout — see the reproduction/verification note under each.

### `docker run hello-world` fails with an iptables error

```
failed to set up container networking ... Unable to enable DIRECT ACCESS FILTERING - DROP rule:
(iptables failed: iptables --wait -t raw -A PREROUTING ...): iptables v1.8.7 (legacy): can't initialize
iptables table 'raw': Table does not exist (do you need to insmod?)
```

Seen on Jetson platforms with a Docker version newer than what the installed Jetson Linux (L4T) kernel's
iptables modules support. JetPack 6.2.1 (Jetson Linux 36.4.4) resolves this compatibility gap.
**Verified:** this box is on L4T 36.4.4 / Docker 29.1.3 and `docker run hello-world` now runs clean —
issue resolved on this checkout. If you hit it on an older JetPack/L4T, matching your Docker version to
your L4T release (rather than installing Docker's latest) is the workaround.

### vcstool: `sudo apt install python3-vcstool` fails to install

Typo in `docs/installation.md`'s "Install other dependencies" step — the correct package name has a
trailing "s": **`python3-vcstools`**. **Verified:** `apt-cache policy python3-vcstool` returns nothing;
`python3-vcstools` (0.1.42-4) is installed and correct on this box.

### `ws_nmpc` fails to pull / model weight files are broken

Missing `git-lfs`. `sdf-nmpc` (inside `ws_nmpc`) tracks its trained model weights (`*.pt` files) via Git
LFS — without `git-lfs` installed *before* cloning, you silently get small LFS pointer text files instead
of the actual weights, not a clone error. **Verified:** `ws_nmpc/src/sdf-nmpc/.gitattributes` contains
`*.pt filter=lfs diff=lfs merge=lfs -text`; `git-lfs` (3.0.2) is installed on this box.

Fix (install then re-pull the affected repo):
```bash
sudo apt install git-lfs
cd workspaces/ws_nmpc/src/sdf_nmpc   # or whichever package has LFS-tracked files
git clean -fd
cd -
./scripts/import_all_repos.sh
```

### `make images` fails with "permission denied ... docker.sock"

```
ERROR: permission denied while trying to connect to the docker API at unix:///var/run/docker.sock
```

Docker group membership issue, not a build issue:
```bash
sudo usermod -aG docker $USER
newgrp docker  # or log out/in
```
**Verified:** `arl` is in the `docker` group on this box.

*(A `xhost: unable to open display ""` warning at shell startup, e.g. over SSH with no X server, is
harmless — safe to ignore.)*

### `make images` fails building `ros2_ros1_bridge` (or anything `FROM unified_autonomy:ros2_base`)

```
target ros2_ros1_bridge: failed to solve: process "/bin/sh -c apt-get update && apt-get install -y
ros-humble-desktop-full ..." did not complete successfully: exit code: 100
```

The failure is reported against whichever target you built, but it actually happens while building the
shared `ros2_base` image that target depends on (`Dockerfile.ros2_base`) — **verified**:
`Dockerfile.ros2_ros1_bridge` is just `FROM unified_autonomy:ros2_base` with no separate apt-get of its
own, so the fix only needs to happen once, in `ros2_base`. `ros-humble-desktop-full` pulls in
Gazebo/Ignition, and apt resolved an inconsistent dependency set for it at build time (e.g.
`libignition-sensors6` version mismatch) — an upstream repository issue, not something wrong with this
repo.

**Fix (already applied on this checkout — see §1d):** in `Dockerfile.ros2_base`, replace
`ros-humble-desktop-full` with `ros-humble-ros-base`. Avoids Gazebo/Ignition entirely (not needed to run
the stack on real hardware) and sidesteps the broken dependency set.

### `make build` fails pulling `unified_autonomy:ros2_heli_sim`

```
✘ Image unified_autonomy:ros2_heli_sim Error pull access denied for unified_autonomy, repository does
not exist or may require 'docker login'
```

Downstream of the `ros2_base` fix above: `ros2_heli_sim` depends on Gazebo/Ignition, so once that's
removed from `ros2_base` this target can no longer build and should be skipped instead.

**Fix (already applied on this checkout — see §1d):** comment out the `ros2_heli_sim`-related
target/services in `docker-bake.hcl`, `docker-compose.build.yml`, and `docker-compose.yml`. **Verified:**
no active (uncommented) `heli_sim` references remain in any of the three files on this checkout.

### `make build` fails with `could not select device driver "nvidia"`

```
Error response from daemon: could not select device driver "nvidia" with capabilities: [[gpu]]
```

NVIDIA Container Toolkit isn't installed/configured (common on a fresh Jetson image):
```bash
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
Verify with `docker info | grep -i runtimes` — should list `nvidia` alongside `runc`.
**Verified:** `nvidia-container-toolkit` 1.16.2 is installed and the `nvidia` runtime is present and
working on this box (used it directly for the NMPC solver build in §1c).

---

## 1. `unified_autonomy_stack` (top-level repo)

### 1a. Changes made to wire up SLAM + GBPlanner + NMPC

**`docker-compose.robot.yml`**
- Uncommented `ros1_launch_gbplanner` (`roslaunch robot_bringup gbplanner_robot.launch`, image
  `unified_autonomy:ros1_gbplanner`) — unchanged from what was already there, just re-enabled.
- Uncommented `ros2_launch_nmpc` (image `unified_autonomy:ros2_nmpc`), with its command extended to pass
  explicit overrides:
  `ros2 launch robot_bringup nmpc_full.launch.py cfg:=quail.yaml use_sim_time:=false odometry:=/msf_core/odometry_50hz input_image:=/img_node/range_image`
- Added a new service `ros1_launch_dome_lidar_processor` (image `unified_autonomy:ros1_base`,
  `roslaunch dome_lidar_processor dome_lidar_slicer.launch`) — feeds NMPC's range-image observation.
- CBF services (`ros2_launch_cbf_pc_select`, `ros2_launch_composite_cbf`) left commented out on purpose.

### 1b. New workspace: `workspaces/ws_control` (not tracked by any `repos/*.repos` manifest)

Ported by hand from magpie's private-repo workspace of the same name (only the piece we need —
`dome_lidar_processor`, **not** `cbf_pc_selector`/`rl_pc_selector`/`control_launcher`, since CBF stays
disabled here):

- `workspaces/ws_control/src/dome_lidar_processor/` — ROS1 catkin package. Converts a point cloud into the
  range image NMPC's VAE/SDF pipeline expects. Adapted from magpie's version:
  - `launch/dome_lidar_slicer.launch`: `input_topic` default changed to
    `/mimosa_node/lidar/manager/points_full_res` (quail's mimosa-deskewed cloud; magpie's was
    `/rslidar_points`, a RoboSense-specific topic that doesn't exist on quail's Hesai setup).
    `frame_id` stays `mimosa_lidar`. Output remaps to `/img_node/range_image` unchanged (already the topic
    `bridge_robot.yaml` bridges).
  - **Built and verified** against the existing `unified_autonomy:ros1_base` image (already has
    `ros-noetic-image-transport`/`cv-bridge` — no Dockerfile change needed):
    ```bash
    docker run --rm -v ~/unified_autonomy_stack/workspaces/ws_control:/workspace -w /workspace \
      unified_autonomy:ros1_base bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCMAKE_BUILD_TYPE=Release"
    ```
  - **TODO / verify**: `slice_thickness`/`center_elevation` in `dome_lidar_slicer.launch` were tuned for
    magpie's solid-state RSM1 lidar, not quail's spinning Hesai JT128 (different native vertical FOV).
    Visualize the output range image before trusting it and retune if it looks clipped or mostly padding.

### 1c. `workspaces/ws_nmpc` — build fix + one-time solver build

- Added an empty `COLCON_IGNORE` file to `workspaces/ws_nmpc/src/dome_lidar_processor/` (the **ROS2**
  variant of dome_lidar_processor bundled in this workspace). It was breaking the colcon build
  (`ros-humble-image-transport` missing from `unified_autonomy:ros2_nmpc`) and blocking `sdf_nmpc_ros`
  from building at all. We use the ROS1 port in `ws_control` instead (matches magpie's proven setup), so
  the ROS2 copy is simply skipped, not fixed.
- Rebuilt the workspace so `sdf_nmpc_ros` actually compiles/installs (previously it never had):
  ```bash
  docker run --rm --runtime=nvidia -v ~/unified_autonomy_stack/workspaces/ws_nmpc:/workspace -w /workspace \
    -e PYTHONPATH=/workspace/src/sdf-nmpc:/home/developer/acados/interfaces/acados_template \
    -e ACADOS_SOURCE_DIR=/home/developer/acados \
    -e LD_LIBRARY_PATH=/usr/local/lib:/home/developer/acados/lib \
    unified_autonomy:ros2_nmpc bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
  ```
- **One-time acados solver build for the `quail` config** (required — `sdfnmpc_node.py` loads a
  pre-compiled solver, it does not generate one at launch; without this the NMPC container crashes on
  startup). The ROS-wrapped helper (`sdf_nmpc_ros/scripts/build_solver.py`) is broken on this repo (it
  imports `collision_predictor_mpc.ocp`, a module name that only exists in magpie's private-repo fork —
  quail's package is `sdf_nmpc`), so the solver was built by calling the underlying function directly:
  ```bash
  docker run --rm --runtime=nvidia \
    -e PYTHONPATH=/workspace/src/sdf-nmpc:/home/developer/acados/interfaces/acados_template \
    -e ACADOS_SOURCE_DIR=/home/developer/acados \
    -e LD_LIBRARY_PATH=/usr/local/lib:/home/developer/acados/lib \
    -v ~/unified_autonomy_stack/workspaces/ws_nmpc:/workspace \
    -v ~/unified_autonomy_stack/workspaces/robot_bringup:/workspace/src/robot_bringup:ro \
    -w /workspace unified_autonomy:ros2_nmpc bash -c \
    'source /opt/ros/humble/setup.bash && python3 -c "from sdf_nmpc.ocp import build_solver; build_solver(\"/workspace/src/robot_bringup/config/ros2/quail.yaml\")"'
  ```
  Output lands in `workspaces/ws_nmpc/src/sdf-nmpc/sdf_nmpc/codegen/quail/` (bind-mounted, so it's found
  automatically by `ros2_launch_nmpc` at runtime). **Re-run this any time `quail.yaml`'s `mpc`/`robot`/
  `sensor` sections change** — runtime-only values (mission `ref`, the `sdf_flag` runtime toggle) don't
  need a rebuild.
  - Note for whoever eventually commits this: `sdf-nmpc` is itself a nested git repo
    (`workspaces/ws_nmpc/src/sdf-nmpc`), and this codegen output now shows up as an **untracked**
    `sdf_nmpc/codegen/` directory in its `git status` — it's a compiled build artifact (`.so`/`.o`/acados
    C code), not source, so it belongs in that repo's `.gitignore` rather than being committed.

### 1d. Pre-existing local changes (already on disk before this work, unrelated to NMPC/gbplanner — noted
for completeness since they're also part of "what differs from `main` on this robot")

- **`Dockerfile.ros2_base`**: `ros-humble-desktop-full` → `ros-humble-ros-base` (lighter install, no GUI
  tooling baked into the base image — reasonable for an onboard Orin NX).
- **`docker-bake.hcl`**: `ros2_heli_sim` build target commented out (not applicable to this platform).
- **`docker-compose.yml`, `docker-compose.build.yml`, `docker-compose.uav_rl_cbf_eval.yml`**:
  simulation-only services (`build_sim`, `ros2_launch_uav_sim`, etc.) commented out — not relevant on
  real hardware.
- **`repos/robot_bringup.repos`**: pinned `robot_bringup`'s version from `main` to `quail/main` (see below).

---

## 2. `robot_bringup` (separate repo, `git@github.com:ntnu-arl/robot_bringup.git`, branch `quail/main`)

### 2a. Changes made to wire up SLAM + GBPlanner + NMPC

- **`launch/ros1/msf_mimosa.launch`**: added a `topic_tools drop` node
  (`/msf_core/odometry` → `/msf_core/odometry_50hz`, keeping 1 in 4) — matches magpie's proven config;
  this throttled odometry is what NMPC actually consumes.
- **`launch/ros2/nmpc_full.launch.py`**:
  - `cmd/acc` remap changed from `/sdf_nmpc/cmd/acc` (a dead end — nothing bridges it to ROS1, confirmed
    against magpie's bridge config) to `/mavros/setpoint_raw/local` directly.
  - `use_sim_time` default `true` → `false` (no `/clock` published on real hardware; the sim default was
    silently stalling every downstream node).
  - `input_image` default `/rmf/lidar/range` → `/img_node/range_image`.
  - `odometry` default `/rmf/odom` → `/msf_core/odometry_50hz`.
  - `cfg` default `nmpc_sim_lidar.yaml` → `quail.yaml`.
- **`launch/ros1/gbplanner_robot.launch`**: `use_sim_time` param `true` → `false` (same sim-default bug).
- **`config/ros1/bridge_params/bridge_robot.yaml`**: added `/msf_core/odometry_50hz`
  (`nav_msgs/msg/Odometry`, ros1→ros2) to the ROS1↔ROS2 bridge topic list.
- **New file `config/ros2/quail.yaml`** (NMPC config; committed in `434972d`, see §9 for changes made
  after that commit): built from magpie's
  flight-tested `sdf_nmpc_ros/config/magpie.yaml` as a template, since the two drones share the same
  airframe/motors/props and differ only in lidar/radar:
  - **Copied unchanged** (shared airframe, or fixed by the trained network weights — do not touch):
    `mpc.*` weights/horizon/braking_dist, `robot.inertia`, `robot.alloc` (motor layout), `robot.size`,
    `robot.limits`, `nn.*`, `sensor.*` (hfov/vfov/aspect_ratio/dmax/shape_imgs/dtype).
  - **Verified against quail's own config** (not copied blind): `ros.frames` (`world: world`,
    `body: mimosa_imu`, `sensor: mimosa_imu`) — checked against `robot_bringup`'s own
    `config/ros1/mimosa.yaml` and `launch/ros1/mimosa.launch`, confirmed identical convention to magpie.
  - **Candidate, needs physical verification before autonomous flight**:
    - `robot.mass: 2.0` — copied from magpie; quail carries a different lidar/radar payload (Hesai JT128
      spinning lidar vs magpie's solid-state RSM1) — weigh the actual assembled airframe and update.
    - `robot.sensor_extrinsics` (`position: [-0.06605, -0.01878, 0.034]`,
      `orientation: [1.5724, -0.4276, -1.5724]` rad) — derived programmatically from quail's own
      *already-calibrated* `mimosa.yaml` lidar `T_B_S` transform (position + quaternion), converted to the
      roll/pitch/yaw convention `collision_predictor_mpc/utils/math.py`'s `euler2rot` expects. Good-faith
      starting value, not flight-verified — cross-check via the `viz_sdf_3D_node` visualization (does the
      reconstructed SDF/point cloud line up with real obstacles?) before trusting it in an autonomous
      mission. A sign error here means the collision model reasons about obstacles in the wrong place.
  - `flags.enable_sdf: True` (solver has SDF machinery compiled in) but the **runtime** flag
    (`/sdf_nmpc/set_flag`, backed by `sdfnmpc_node.py`'s `self.sdf_flag`) defaults to `False` on every
    node start — so out of the box NMPC runs as a pure path/reference tracker, no code or config change
    needed. See "NMPC behavior" below.
  - `flags.simulation` — **correction, see §9**: this document originally said `True` was fine to leave
    (matching magpie's checked-out config at the time) since it's "separate from ROS `use_sim_time`".
    That's true for `use_sim_time`, but incomplete — this flag *also* selects the message type
    `sdfnmpc_node.py` uses for its `cmd/acc` publisher (`Twist` if `True`, `PositionTarget` if `False`),
    and the bridge only knows how to carry `PositionTarget` on that topic. Must be `False` on real
    hardware. See §9 for the full explanation and consequence.

### 2b. Pre-existing local hardware-calibration changes (already on disk, quail-specific — genuinely part
of "what's needed to set up on this robot", just not from this session)

- **`config/ros1/mimosa.yaml`**: `lidar.T_B_S` and `radar.T_B_S` extrinsics (position+quaternion) updated
  to quail's actual calibrated lidar/radar-to-IMU transforms (this is the calibration the new
  `quail.yaml`'s `sensor_extrinsics` candidate was derived from).
- **`config/ros1/jt128.yaml`** (Hesai driver): `use_timestamp_type` `0` → `1` (use packet receive
  timestamp instead of the lidar's own point-cloud timestamp — a sync fix for this specific unit).
- **`config/ros1/vn100.yaml`** (VectorNav IMU): `port` `/dev/ttyTHS2` → `/dev/ttyTHS1`,
  `baud_rate` `230400` → `115200` — quail's IMU wiring/UART config.
- **`launch/ros1/mimosa.launch`**: lidar input remap `/rslidar_points` → `/lidar_points` (matches the
  Hesai driver's actual output topic; the template default was RoboSense-named).
- **`launch/ros1/bag_rec.launch`**: rosbag output path prefix `parrot` → `quail` (cosmetic, per-robot
  bag naming).

---

## 3. Other workspace repos with local changes (pre-existing, unrelated to this work)

A systematic scan of every git repo under `workspaces/` (top-level and nested under each `src/`) for
uncommitted changes turned up one more, beyond `unified_autonomy_stack` and `robot_bringup`:

- **`workspaces/ws_mmwave_ti_ros/src/mmwave_ti_ros`**: a large pre-existing restructuring — the old nested
  vendor layout (`autonomous_robotics_ros/`, `ros_driver/`, including a full copy of turtlebot/kobuki
  integration packages unrelated to a drone) has been deleted and replaced with a flat layout
  (`CMakeLists.txt`, `package.xml`, `src/`, `include/`, `launch/`, `cfg/`, `msg/`, `srv/`, `udev/`, etc.
  directly at the repo root — currently untracked, i.e. not yet `git add`ed). This predates this session,
  I did not make it, and I have not verified its correctness or the reasoning behind it — flagging it here
  only because it's a real local diff on this robot's checkout that "what differs from `main`" should
  include. Worth a proper commit message from whoever did this, since right now it's just a large
  uncommitted working-tree change.

Everything else under `workspaces/` (`ws_gbplanner`, `ws_mimosa`, `ws_ethzasl_msf`, `ws_hesailidar`,
`ws_vectornav`, `ws_mavros`, `ws_cbf`, `ws_nmpc/src/sdf_nmpc_ros`, etc.) has a clean `git status` — no
local modifications beyond what's listed in §1 and §2.

---

## 4. NMPC behavior: SDF collision avoidance is present but off by default

`quail.yaml` has `flags.enable_sdf: True` — the acados solver has the SDF constraint machinery compiled
in — but `sdfnmpc_node.py` resets `self.sdf_flag = False` on every startup, and the constraint evaluates
to inert (`sdf.max_df`, i.e. "no obstacle") whenever that runtime flag is `False`:

```bash
# confirm/keep it disabled (this is already the default - no-op unless someone armed it)
rosservice call /sdf_nmpc/set_flag "data: false"

# arm the SDF constraint later, once you want it (refuses if no valid image/latent received yet)
rosservice call /sdf_nmpc/set_flag "data: true"
```

With `quail.yaml`'s `vfov_constraint: False` and `hfov: 3.1415` (≥ π, so the horizontal FOV constraint
isn't added either), there's no other geometric constraint lingering — with the runtime flag off, NMPC is
purely tracking the reference path/waypoints (`wps` ← `/gbplanner_path`).

---

## 5. Bringing it up

```bash
# 1. Autonomy stack (SLAM + planner + NMPC + bridge) - must come up first, owns the shared roscore
cd ~/unified_autonomy_stack
make launch DOCKER_COMPOSE_FILE=docker-compose.robot.yml

# 2. Flight controller connection - after step 1, so it attaches to the existing roscore
cd ~/ros1_docker_dev
./start.sh && ./enter.sh
roslaunch rmf_obelix px4_qgc.launch
```

Sanity checks:
```bash
rostopic echo -n1 /mavros/state              # connected: True
rostopic hz /msf_core/odometry_50hz          # SLAM+fusion, throttled odom feeding NMPC
rostopic hz /img_node/range_image            # dome_lidar_processor output
docker compose -f docker-compose.robot.yml logs -f ros2_launch_nmpc   # no crash on the missing-solver error
```

I have **not** traced the exact arm/takeoff/mission-trigger procedure end-to-end (e.g. the precise
sequencing of `/sdf_nmpc/takeoff`, PX4 offboard mode, and telling gbplanner to start exploring) — treat
that as an open item to verify tethered/on the bench before an autonomous flight.

---

## 6. Bugs found during the first bench-test launch (props off)

Everything above was individually build-verified but had never been launched together through
`make launch`. First run (props off) surfaced four real bugs — none were guesses, all confirmed by
reading logs/`ldd`/timestamps before fixing. Fixed in place; `sdf_nmpc_ros`/`sdf-nmpc` use
`--symlink-install`, so source edits there took effect immediately without a rebuild.

### 6a. Every `sdf_nmpc_ros` node imports a module that doesn't exist on this repo

```
ModuleNotFoundError: No module named 'collision_predictor_mpc'
```

Not just `build_solver.py` (§1c) — **every** script in `workspaces/ws_nmpc/src/sdf_nmpc_ros/scripts/`
(`vae_node.py`, `ref_gen_node.py`, `sdfnmpc_node.py`, `viz_vae_node.py`, `viz_sdf_2D_node.py`,
`viz_sdf_3D_node.py`) imports from `collision_predictor_mpc` — magpie's private-fork module name;
quail's actual package is `sdf_nmpc`. Fixed with a straight rename across all of them:
```bash
SCRIPTS=~/unified_autonomy_stack/workspaces/ws_nmpc/src/sdf_nmpc_ros/scripts
sed -i "s/collision_predictor_mpc/sdf_nmpc/g" $SCRIPTS/*.py
```
One exception needing manual fixing, not a blind rename: `viz_sdf_2D_node.py`/`viz_sdf_3D_node.py`
imported `COLPREDMPC_TMP_DIR` (a constant, doesn't exist under `sdf_nmpc`) — replaced with `sdf_nmpc`'s
equivalent `default_data_dir()` function (same role: `f'{default_data_dir()}/{cfg.nn.sdf_weights}'`).

### 6b. `robot_bringup`'s `quail.yaml` wasn't actually installed — stale CMake configure cache

```
FileNotFoundError: [Errno 2] No such file or directory:
'/workspace/install/robot_bringup/share/robot_bringup/config/ros2/quail.yaml'
```
`robot_bringup/CMakeLists.txt` installs config via `install(DIRECTORY config DESTINATION ...)` — a plain
directory copy evaluated at CMake *configure* time. The §1c colcon rebuild ran without
`--cmake-force-configure`, so it reused a configure cache from before `quail.yaml` existed and never
picked up the new file, even though the file existed in the mounted source. Same root cause as the
`ws_nmpc`/`dome_lidar_processor` build issue in §1c, hitting a different package. Fix: clear the stale
build dir and rebuild that package specifically (needs the `robot_bringup` source mount, not just
`ws_nmpc`):
```bash
rm -rf ~/unified_autonomy_stack/workspaces/ws_nmpc/build/robot_bringup ~/unified_autonomy_stack/workspaces/ws_nmpc/install/robot_bringup
docker run --rm --runtime=nvidia \
  -v ~/unified_autonomy_stack/workspaces/ws_nmpc:/workspace \
  -v ~/unified_autonomy_stack/workspaces/robot_bringup:/workspace/src/robot_bringup:rw \
  -w /workspace unified_autonomy:ros2_nmpc bash -c \
  "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select robot_bringup"
```
**General takeaway**: after editing anything under `robot_bringup/config` or `robot_bringup/launch`, do a
clean (not incremental) rebuild of the `robot_bringup` package specifically, or the change won't reach
`install/` silently.

### 6c. `ros1_launch_gbplanner`'s GPU request crashed the container, not the app

```
Error running prestart hook #0: ... invoking the NVIDIA Container Runtime Hook directly ... is not
supported. Please use the NVIDIA Container Runtime (e.g. specify the --runtime=nvidia flag) instead.
```
The re-enabled block (§1a) carried over its original GPU declaration verbatim: Compose's
`deploy.resources.reservations.devices` style GPU request. That mechanism doesn't work on this
Docker/NVIDIA Container Toolkit setup (same failure class as running `docker run --gpus all` without
`--runtime=nvidia`, hit earlier in this whole process). `ros2_launch_nmpc` already uses the plain
`runtime: nvidia` compose key and works — switched `ros1_launch_gbplanner` to match, dropped the
`deploy:` block entirely. (This was latent/never-tested in the original commented-out block — not
something this session's edits introduced.)

### 6d. `ros1_bridge`'s `parameter_bridge` binary won't start — missing shared libraries

```
error while loading shared libraries: libexample_interfaces__rosidl_typesupport_cpp.so: cannot open
shared object file: No such file or directory
```
`ldd` on the binary showed three missing libs: `libexample_interfaces`, `libmap_msgs`,
`libturtlesim` (all `__rosidl_typesupport_cpp.so`). Root cause: `ros1_bridge` was compiled against a full
`ros-humble-desktop` environment (it auto-discovers every available ROS2 message package at build time
to generate bridging code), but the runtime image (`unified_autonomy:ros2_ros1_bridge`) is built
`FROM unified_autonomy:ros2_base`, which no longer ships these demo/tutorial packages since the
`ros-humble-desktop-full` → `ros-humble-ros-base` fix in §1d. Direct fallout of that fix, not a new,
unrelated problem.

This network's route to the ROS/Ubuntu package mirrors turned out to be unreliable (DNS partially
resolves, plain HTTP to a domain root works instantly, but `apt-get update` inside a container hung for
several minutes with zero output, twice, even after forcing IPv4) — so rather than fight that, the fix
pulls the exact same libraries from `unified_autonomy:ros1-bridge-builder`, the intermediate image used
to *build* `ros1_bridge` in the first place, which still has the full desktop environment from before
the final image was slimmed down. No network access needed:
```bash
mkdir -p ~/unified_autonomy_stack/workspaces/ws_ros1_bridge_extra_libs
cid=$(docker create unified_autonomy:ros1-bridge-builder)
docker cp $cid:/opt/ros/humble/lib/libexample_interfaces__rosidl_typesupport_cpp.so ~/unified_autonomy_stack/workspaces/ws_ros1_bridge_extra_libs/
docker cp $cid:/opt/ros/humble/lib/libmap_msgs__rosidl_typesupport_cpp.so ~/unified_autonomy_stack/workspaces/ws_ros1_bridge_extra_libs/
docker cp $cid:/opt/ros/humble/lib/libturtlesim__rosidl_typesupport_cpp.so ~/unified_autonomy_stack/workspaces/ws_ros1_bridge_extra_libs/
docker rm $cid
```
Wired into `docker-compose.robot.yml`'s `ros2_launch_ros1_bridge` service as a new bind mount
(`./workspaces/ws_ros1_bridge_extra_libs:/extra_libs:ro`) plus
`LD_LIBRARY_PATH=/extra_libs:/opt/ros/humble/lib`. Verified via `ldd` (zero unresolved) and by actually
starting `parameter_bridge` standalone (it ran and only complained about no roscore being reachable in
that isolated test — expected, since no master was running there).

**Note for whoever eventually commits this**: `ws_ros1_bridge_extra_libs/` is a workaround, not a proper
fix — the real fix belongs in whatever Dockerfile produces `unified_autonomy:ros2_ros1_bridge`, adding
`ros-humble-example-interfaces`, `ros-humble-map-msgs`, `ros-humble-turtlesim` to its apt install list
(or copying them from the builder stage in a multi-stage build) so the image is self-contained again.

### 6e. `sdfnmpc_node.py` — renamed module, but the class name differs too

```
ImportError: cannot import name 'NMPC' from 'sdf_nmpc.controller' (/workspace/src/sdf-nmpc/sdf_nmpc/controller.py)
```
The blanket `collision_predictor_mpc` → `sdf_nmpc` rename in §6a fixed the module path but not this: the
controller class is named `Nmpc` in quail's `sdf_nmpc.controller` (confirmed via
`grep '^class' controller.py`), not `NMPC` like magpie's fork. Fixed the two real symbol references in
`sdfnmpc_node.py` (the import on line 20, the instantiation `Nmpc(self.cfg)` on line 38) — left the
string `'NMPC FAILED, disabling constraints'` log message alone since it's just text, not a symbol.

**Verified live**: on the next `make launch`, `sdfnmpc_node.py` started with no traceback and logged
`node started successfully` followed by `first reference received, starting mpc` — it picked up
`gbplanner`'s reference and began actually running the control loop.

### 6f. `ros2_launch_ros1_bridge` — missing dependency on `bridge_params` finishing first

```
The parameter 'topics' either doesn't exist or isn't an array
The parameter 'services_1_to_2' either doesn't exist or isn't an array
The parameter 'services_2_to_1' either doesn't exist or isn't an array
```
`parameter_bridge` reads its topic/service list from ROS1 parameters (`/topics` etc.), which
`ros1_launch_bridge_params` loads via `rosparam load` and then exits (a one-shot job — its own log
confirms `/topics` gets set correctly: `* /topics: [{'topic': '/msf_...`). But
`ros2_launch_ros1_bridge` only `depends_on: ros1_launch_roscore: condition: service_healthy` — nothing
ties it to `bridge_params` actually finishing, so it can start reading `/topics` before that one-shot
loader has written it. Real params on the roscore persist fine once set (the process exiting doesn't
clear them), so this is a startup-order race, not a data problem. Fix: added a second `depends_on` entry:
```yaml
depends_on:
  ros1_launch_roscore:
    condition: service_healthy
  ros1_launch_bridge_params:
    condition: service_completed_successfully
```
**Verified live**: on the next `make launch`, the "topics doesn't exist" warning was gone and the log
instead showed real bridging activity for every configured topic (`Trying to create bidirectional bridge
for topic '/mavros/setpoint_raw/local' ...`, etc.), plus actual message traffic (`Passing message from
ROS 1 nav_msgs/Odometry to ROS 2 nav_msgs/msg/Odometry`, and the same for the image and
path/`horizon_ref` topics going the other way). The ordering fix worked.

**New, separate finding from that same log**: `services_1_to_2`/`services_2_to_1` still log as "doesn't
exist or isn't an array", even though `topics` now resolves fine. Root cause is almost certainly
`bridge_robot.yaml`'s structure — the two `/sdf_nmpc/takeoff` and `/sdf_nmpc/set_flag` entries are listed
as `- service: ...` items *inside* the same flat `topics:` list, not under separate top-level
`services_1_to_2`/`services_2_to_1` keys the way `parameter_bridge` expects. Practical effect: **topic
bridging works (odometry, image, path, setpoints — everything the tracking pipeline needs), but the
takeoff/set_flag *service* calls documented in §3 are not actually reaching the ROS2 side right now.**
Not yet fixed — needs the service entries restructured into the correct parameter shape and re-verified;
low priority since nothing in the core SLAM→planner→NMPC→PX4 path depends on those two services, but
don't rely on `rosservice call /sdf_nmpc/takeoff` or `/sdf_nmpc/set_flag` until this is addressed.

### 6g. `hesai_ros_driver_node` prints an unconditional per-frame log line

Not a bug, just terminal noise: `ws_hesailidar/src/HesaiLidar_ROS_2.0/src/manager/source_driver_ros1.hpp`
had a bare `printf("%s frame:%d points:%u packet:%d start time:%lf end time:%lf\n", ...)` inside
`SourceDriver::ToRosMsg`, called once per lidar frame (~10+ Hz) with no config flag or ROS log-level
gating it — it's a raw C `printf`, not a `ROS_INFO`/`ROS_DEBUG` call, so nothing in `jt128.yaml` or
`roslaunch`'s log-level machinery could suppress it. Commented out the line and rebuilt:
```bash
docker run --rm -v ~/unified_autonomy_stack/workspaces/ws_hesailidar:/workspace \
  -v ~/unified_autonomy_stack/workspaces/robot_bringup:/workspace/src/robot_bringup:rw \
  -w /workspace unified_autonomy:ros1_base \
  bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCMAKE_BUILD_TYPE=Release"
```
Builds clean (only harmless "unused variable" warnings for the now-unused format args — expected, not
an error).

### 6h. First full successful run — status and two open (not yet fixed) observations

With 6a–6g applied, `make launch DOCKER_COMPOSE_FILE=docker-compose.robot.yml` came up clean for the
first time: all 11 services stayed up (no crash-loop), `mimosa`/`ethzasl_msf` produced odometry,
`gbplanner` received it and started its planning loop, `sdfnmpc_node` received a reference and started
controlling, and the ROS1↔ROS2 bridge passed real traffic both directions. This is the first time the
whole SLAM→planner→NMPC chain has run together end to end.

Two things observed in that run, investigated but **not fixed** — flagged as open items, not silently
ignored:

- **`gbplanner` recurring warning**: `Input pointcloud queue getting too long! Dropping some
  pointclouds.` — recurred 3 times, roughly 60s apart, not a one-off startup blip. Checked the TF tree
  before assuming a frame-naming bug (`rostopic echo /tf`): `world` and `mimosa_body` (the point cloud's
  frame) are siblings under a common parent `mimosa_world`, which TF can resolve fine — so this doesn't
  look like a frame mismatch. More likely explanation: **compute contention** — mimosa's SLAM, the NMPC's
  VAE/SDF neural net inference, and gbplanner's TSDF integration are all real-time-hungry and were now
  running simultaneously on the Orin NX for the first time. See §7 (power mode) below — worth
  re-checking whether this clears up under `MAXN_SUPER` before treating it as a real bug to chase further.
- **`TF_REPEATED_DATA` warning**: `ignoring data with redundant timestamp for frame mimosa_body (parent
  mimosa_world) ... according to authority unknown_publisher` — suggests something is publishing the
  same `mimosa_body` transform twice at an identical timestamp. Not investigated further yet; low
  priority since TF2 handles it gracefully (drops the duplicate, doesn't crash), but worth a look if
  odometry ever looks glitchy downstream.

---

## 7. Power mode (Orin NX)

Given the compute contention observed in §6h (mimosa + NMPC's neural net + gbplanner all running at
once), it's worth running the companion computer at max performance rather than the Jetson's default
power-capped mode:
```bash
sudo nvpmodel -m 0      # MAXN_SUPER - unrestricted power budget, all cores online
sudo jetson_clocks       # locks CPU/GPU/EMC clocks to max - without this, DVFS can still scale down
                          # under light load even in MAXN_SUPER mode
```
Check the current mode with `nvpmodel -q` (this box defaults to mode `4`, "40W"). I don't have sudo
access on this box, so these need to be run manually — not something I can verify/apply myself.

**Trade-off worth knowing**: MAXN_SUPER draws significantly more power and generates more heat. Fine for
bench testing on wall power; for actual battery-powered flight, running the companion computer at max
draw eats into flight battery life faster, so it's a real trade-off to weigh, not a strictly-better
setting.

---

## 8. `./data` was root-owned — recorder now runs as your user instead

`mkdir` inside `~/unified_autonomy_stack/data` was failing with "Permission denied". Root cause: none of
the compose services override the container's default user, and `unified_autonomy:ros1_base`'s default
user is `root` — so `ros1_launch_recorder` (the only service that writes to `./data`, via the bind mount
`*recorded-data-volume`) created every rosbag file as root, and that ownership propagated to the whole
`data/` directory tree, blocking normal writes from the host `arl` account without `sudo`.

**Fix** (`docker-compose.robot.yml`, `ros1_launch_recorder`): added `user: "1000:1000"` so the container
runs as the host `arl` UID/GID instead of root. This alone isn't sufficient, though — verified via a
quick `docker run` test first: **UID 1000 has no `/etc/passwd` entry in this image**, so `$HOME` defaults
to `/` (not writable), and `roslaunch` writes its own run logs to `$HOME/.ros/log/...` before it even
gets to launching `rosbag record` — so without an explicit `HOME`, the service would fail to start at all
under the new user. Added `environment: - HOME=/tmp` (confirmed writable for UID 1000) alongside the
other required env vars. **Important YAML gotcha**: adding an `environment:` key directly in this
service's block *replaces* (not merges with) `*system-template`'s own `environment:` list via YAML
anchor semantics — so all of that template's original entries (`DISPLAY`, `NVIDIA_VISIBLE_DEVICES`, etc.)
had to be repeated explicitly alongside `HOME`, or they'd have silently been dropped for just this one
service.

**One-time manual step still needed** (I don't have sudo on this box): the existing `data/` directory
itself is still owned by `root:root` from before this fix, so it needs a one-time ownership fix before
the recorder (now running as UID 1000) can write into it at all:
```bash
sudo chown arl:arl ~/unified_autonomy_stack/data
```
After that, every file the recorder creates going forward will be `arl`-owned, so this shouldn't recur.

**Note**: while investigating this, `data/` was found completely empty (the prior rosbags visible earlier
in this session, and a `test/` dir, were both gone) — confirmed with the user this was their own manual
cleanup, unrelated to anything in this fix.

---

## 9. Post-commit: a real bug fix (corrects §2a) and safety-conscious tuning

Everything through §8 was committed on both repos on 2026-08-13 (`434972d` on `robot_bringup`, `b2bef4b`
on `unified_autonomy_stack`, both messaged "first full working setup on quail"). The changes below were
made *after* that commit — found via a systematic `git status` sweep of `unified_autonomy_stack` and
every workspace repo (same method as §3/§6), currently uncommitted on `robot_bringup`. Not made by me;
documented here for the same reason as the pre-existing hardware-calibration changes in §2b — genuinely
part of "what's needed to run this on quail."

### 9a. `config/ros2/quail.yaml` — `flags.simulation` must be `False` on real hardware (bug fix, corrects §2a)

```diff
- simulation: True  # kept True to match magpie's flight-tested config
+ simulation: False  # must be False on real hardware
```

**This corrects guidance earlier in this document (§2a) and in chat during this session — that guidance
was wrong.** The reasoning at the time ("`flags.simulation` is separate from ROS `use_sim_time`, magpie
leaves it `True` in real flights") is true as far as it goes, but incomplete: I verified in
`sdfnmpc_node.py` that this flag *also* selects the message type used for the `cmd/acc` publisher:

```python
if self.cfg.flags['simulation']:
    self.pub_cmd = self.create_publisher(Twist, 'cmd/acc', 1)          # simulation: True
else:
    self.pub_cmd = self.create_publisher(PositionTarget, 'cmd/acc', 1)  # simulation: False
```

`nmpc_full.launch.py` remaps `cmd/acc` → `/mavros/setpoint_raw/local`, and `bridge_robot.yaml` declares
that bridge topic's type as `mavros_msgs/msg/PositionTarget`. With `simulation: True`, the node actually
publishes `geometry_msgs/Twist` there instead — a type the bridge has no matching rule for on that topic,
so it silently fails to bridge it. **Practical consequence: during the "first full working" bench test
in §6h, `quail.yaml` still had `simulation: True`, meaning NMPC's actual acceleration commands almost
certainly never reached mavros/PX4** — even though the control loop, reference tracking, and message
bridging on every *other* topic (odometry, image, horizon_ref, path) all genuinely worked and were
correctly verified. The pipeline was real; this one topic wasn't reaching the flight controller. Fixed
now; **retest and re-verify `/mavros/setpoint_raw/local` is actually receiving messages** before relying
on this for anything beyond another bench test.

### 9b. Safety-conscious speed/bounds tuning (not a bug — deliberate, conservative tuning)

Three related changes, all reducing speed or tightening the operating envelope compared to the committed
baseline — consistent with preparing for a cautious first real test rather than fixing anything broken:

- **`config/ros2/quail.yaml`**: `ref.vref` (reference velocity) `2.0` → `1.0 m/s`; `ref.wzref` (reference
  yaw rate) `1` → `0.7 rad/s`.
- **`config/ros1/gbplanner/robot/planner_control_interface_config.yaml`**: `RobotDynamics.v_max`,
  `v_init_max`, `v_homing_max` all `1.0` → `0.3 m/s` — gbplanner's own commanded speed cap, roughly a 3x
  reduction.
- **`config/ros1/gbplanner/robot/gbplanner_config.yaml`**: `BoundedSpaceParams.Global` exploration volume
  narrowed from `min_val: [-1.0, -300.0, -2.0], max_val: [32.0, 2.0, 1.75]` (a 302m-long Y-axis range —
  looks like a leftover long-corridor default) to `min_val: [-2.0, -40.0, 0.5], max_val: [30.0, 2.5, 2.0]`
  — a much more tightly scoped volume, presumably matching an actual real test space.

No action needed here — just documenting the change. Worth knowing these are *more* conservative than the
committed baseline, not less, if comparing behavior against the §6h bench test.

---

## 10. RealSense D455 (USB) — auxiliary camera, topics only, not consumed by the pipeline

quail has a RealSense D455 plugged into a USB port. Requirement: it should come up automatically with
`make launch`, its topics should exist and be visualizable in rviz, and `bag_rec.launch` should record
the compressed versions — but **nothing else in this stack (mimosa/gbplanner/NMPC) subscribes to it or
depends on it**. It's a new, fully independent branch of the pipeline.

### 10a. New image: `Dockerfile.ros2_realsense`

Built from `unified_autonomy:ros2_base`. Two build-time issues hit, both infrastructure, not code:

- **`docker buildx bake` silently rebuilds the parent target from scratch.** The bake target's
  `contexts = { "unified_autonomy:ros2_base" = "target:ros2_base" }` directive resolves that named
  context by *rebuilding* `ros2_base`, not reusing the already-built, already-tagged local image — it got
  stuck 30+ minutes re-running `ros2_base`'s very first `apt-get install ros-humble-ros-base` layer from
  zero. Confirmed via `docker buildx history ls` showing both Dockerfiles "Running" simultaneously,
  started at the same instant. **Fix: build with plain `docker build -f Dockerfile.ros2_realsense
  --network host -t unified_autonomy:ros2_realsense .` instead** — this resolves `FROM
  unified_autonomy:ros2_base` against the existing local image with no rebuild. (`ros2_realsense` stays
  defined in `docker-bake.hcl` for consistency with the other targets, it's just not what was actually
  used to build it here.)
- **`apt-file`'s hook downloads a ~383MB index on every `apt-get update`, and it stalled indefinitely
  against this network's mirror.** `ros2_base` installs `apt-file`, which drops
  `/etc/apt/apt.conf.d/50apt-file.conf` — unlike its sibling `Contents-udeb`/`Contents-dsc` targets, the
  `Contents-deb` target in that file has no `DefaultEnabled "false"` line, so it fetches by default.
  Confirmed genuinely stuck (not just slow) via a log timestamp frozen for 11+ minutes and an independent
  `curl` to the same file timing out with 0 bytes. A `-o
  Acquire::IndexTargets::deb::Contents-deb::DefaultEnabled=false` command-line override did **not**
  reliably suppress it on retry. **Fix: `RUN rm -f /etc/apt/apt.conf.d/50apt-file.conf` as the first line
  in the Dockerfile**, before any `apt-get update`.

Build steps, in order: remove the apt-file hook → install librealsense's build deps (`libssl-dev`,
`freeglut3-dev`, `libusb-1.0-0-dev`, `pkg-config`, `libgtk-3-dev`) → build+install `librealsense` from
source (`realsenseai/librealsense`, `-DFORCE_LIBUVC=true` — uses the standard USB Video Class kernel
driver, no custom kernel patching needed, matches `scripts/libuvc_installation.sh`) → install the ROS2
wrapper's dependencies (`ros-humble-image-transport`, `ros-humble-image-transport-plugins`,
`ros-humble-cv-bridge`, `ros-humble-camera-info-manager`, `ros-humble-diagnostic-updater`,
`ros-humble-tf2-ros` — none of these ship with `ros-humble-ros-base`, same gap pattern already documented
for `ros2_nmpc`/`ros2_ros1_bridge` elsewhere in this file). `image-transport-plugins` specifically is what
makes the `/compressed` and `/compressedDepth` sibling topics exist automatically.

### 10b. New workspace: `workspaces/ws_realsense`

`git clone --branch ros2-master --depth 1 https://github.com/realsenseai/realsense-ros.git` into
`src/realsense-ros/`. Added empty `COLCON_IGNORE` files to `realsense2_rgbd_plugin/` and
`realsense2_ros_mqtt_bridge/` (not needed, scopes the build to `realsense2_camera` +
`realsense2_camera_msgs` + `realsense2_description`). `robot_bringup` is also built into this workspace
(empty `src/robot_bringup/` placeholder + bind-mounted at container runtime, same pattern as every other
workspace) so the new launch file below is discoverable via `ros2 launch robot_bringup ...`.

### 10c. New launch file: `robot_bringup/launch/ros2/realsense_d455.launch.py`

Thin wrapper around `realsense2_camera`'s own `rs_launch.py`. Color + depth only (no IR, no pointcloud,
no align — kept light given known compute contention on this Orin NX, see §6h/§7), 640x480 @ 15fps:

- `camera_name: 'd455'`, **`camera_namespace: ''`** — leaving `camera_namespace` at its own default
  (`'camera'`) doubles up with `camera_name` and produces `/camera/camera/...`-style topics (a known
  quirk of this package's launch defaults); setting it empty gives clean `/d455/...` topics.
- Resulting topics: `/d455/color/image_raw(/compressed)`, `/d455/color/camera_info`,
  `/d455/depth/image_rect_raw(/compressedDepth)`, `/d455/depth/camera_info`.

### 10d. New compose service: `ros2_launch_realsense`

Added to the `AUXILIARY SENSORS` section of `docker-compose.robot.yml`, picked up automatically by `make
launch` (no Makefile change needed — it matches the `ros2_launch_*` service-name convention already
scanned for). `privileged: true` + full `/dev:/dev` mount, needed because `-DFORCE_LIBUVC=true` means
librealsense talks to the camera directly over `libusb` (raw `/dev/bus/usb/*` access), not through a
kernel v4l2 driver.

**Real bug found and fixed here, not specific to RealSense — worth knowing about for any future
container that publishes ROS2 data cross-container:** the service also sets **`user: "1000:1000"` +
`group_add: [root]`** (plus `HOME=/tmp`, same reasoning as `ros1_launch_recorder`'s existing `user:`
override — uid 1000 has no `/etc/passwd` entry in these images, so `$HOME` defaults to `/`, unwritable).
This isn't just hardening — it's required for the bridge to actually receive this camera's messages:

- `Dockerfile.ros2_realsense` has no `USER` directive, so its process runs as **root** by default.
  `ros2_launch_ros1_bridge` (and `ros2_launch_nmpc`) run as **uid 1000** (`developer`, set via their own
  Dockerfiles). FastDDS's shared-memory transport creates `/dev/shm/fastrtps_*` segments owned by
  whichever uid published them, readable only by that same uid (or root) — a root-owned publisher's
  segments are invisible over SHM to a uid-1000 subscriber.
- **Symptom was silent and easy to miss**: `ros2 topic list` / `rostopic list` on both sides showed the
  topics fine, `ros2 topic info -v` showed the correct publisher/subscriber pair with compatible QoS, the
  bridge logged `create bidirectional bridge for topic /d455/...` with no error — but zero messages ever
  arrived on the ROS1 side (`rostopic hz` sat at "no new messages" indefinitely). Everything about the
  *control plane* (discovery, topic graph, QoS negotiation) looked correct; only actual data delivery was
  broken.
- **Root-caused by elimination**, not guesswork: reproduced the identical failure with a bare
  `unified_autonomy:ros2_base` container publishing a trivial `std_msgs/String` to a bare
  `unified_autonomy:ros2_ros1_bridge`-image container — confirming it was a uid mismatch between *any*
  root-owned publisher and *any* uid-1000 subscriber, nothing specific to realsense, `CompressedImage`, or
  this bridge config. `docker exec -u root <bridge container> ros2 topic echo ...` immediately started
  receiving, confirming the fix before touching the compose file.
- Since `/dev/bus/usb/*` device nodes are `root:root` with group-`rw` (`crw-rw-r--`), **`group_add:
  [root]`** gives the uid-1000 process the group membership it needs to still open the USB device
  read-write despite not running as root itself. `privileged: true` alone does not grant this — it only
  bypasses the cgroup device whitelist, not standard file-permission (DAC) checks on the device node.

### 10e. `bridge_robot.yaml` — 4 new `ros2_to_ros1` entries

```yaml
- topic: /d455/color/image_raw/compressed
  type: sensor_msgs/msg/CompressedImage
- topic: /d455/color/camera_info
  type: sensor_msgs/msg/CameraInfo
- topic: /d455/depth/image_rect_raw/compressedDepth
  type: sensor_msgs/msg/CompressedImage
- topic: /d455/depth/camera_info
  type: sensor_msgs/msg/CameraInfo
```

(`queue_size: 10`, `direction: ros2_to_ros1` on all four, same as the existing `cam_front`/`cam_left`/
`cam_right` entries this was modeled on.) Depth uses **`compressedDepth`**, not `compressed` — depth is
16-bit (`Z16`), and `compressedDepth` (PNG-based) is `image_transport`'s correct plugin for that; plain
`compressed` (JPEG) is for 8-bit color and isn't meaningful for raw depth values. `camera_info` is bridged
alongside each image so rviz has the intrinsics needed to interpret it properly.

### 10f. `bag_rec.launch` — the same 4 topics added to the recorded topic list

### 10g. Verified live, end to end (not just "should work")

Brought up `roscore` + `bridge_params` + `ros1_bridge` + `ros2_launch_realsense` + `recorder` via compose
(`--profile launch up -d <services>`), then confirmed at every hop:

- Camera detected: `rs-enumerate-devices` → `RealSense D455`, serial `341222300967`, FW `5.13.0.55`.
- ROS2 side publishing: `ros2 topic hz` → ~15Hz on both `color/image_raw/compressed` and
  `depth/image_rect_raw/compressedDepth`.
- Bridged to ROS1: `rostopic hz` on the same two topics → ~15Hz, matching.
- Recorder subscribed: recorder log shows `Subscribing to /d455/...` for all 4 topics.
- Actually landed in the bag: `rosbag info` on the resulting `.bag` shows all 4 topics with ~1430 messages
  each (`sensor_msgs/CompressedImage` / `sensor_msgs/CameraInfo`), matching every other topic's message
  count for the same recording window.

Test containers/bags from this verification pass were cleaned up afterward except the resulting `.bag`
file under `./data/`, left in place as evidence (`quail_2026-08-15-11-29-27_0.bag`) — delete it if not
wanted.

---

## 11. `launch_stack` / `launch_px4` shell aliases

Two shell functions in `~/.bash_aliases` on quail (sourced automatically by `~/.bashrc`, which already had
a `[ -f ~/.bash_aliases ] && . ~/.bash_aliases` guard — no changes needed there), bundling the two
commands normally run by hand in separate terminals to bring the robot up:

```bash
launch_stack() {
    (cd ~/unified_autonomy_stack && make launch DOCKER_COMPOSE_FILE=docker-compose.robot.yml)
    make -C ~/unified_autonomy_stack stop
}

launch_px4() {
    (cd ~/ros1_docker_dev && ./start.sh && docker exec -it ros1-dev bash -ic "roslaunch rmf_obelix px4_qgc.launch")
    (cd ~/ros1_docker_dev && ./stop.sh)
}
```

Both follow the same shape: run the thing in the foreground, then unconditionally tear it back down once
it exits — including via Ctrl+C. That's why each is written as two separate statements (not `&&`): `make
launch`/`roslaunch` exiting non-zero on SIGINT would skip a chained `&&` follow-up, so `make stop`/
`./stop.sh` needs its own statement to run regardless of how the first one ended. The `cd` happens inside
a subshell (`( ... )`) so the alias doesn't leave your terminal parked in a different directory afterward.

- **`launch_stack`** — the main `unified_autonomy_stack` (mimosa/gbplanner/NMPC/bridge/recorder/etc, per
  `docker-compose.robot.yml`). Direct wrapper around the `make launch DOCKER_COMPOSE_FILE=...` / `make
  stop` pair already documented throughout this file.
- **`launch_px4`** — the separate `ros1_docker_dev` container (`ros1-dev`, a generic ROS1 Noetic dev
  environment unrelated to `unified_autonomy_stack`'s own images) that runs `roslaunch rmf_obelix
  px4_qgc.launch` — the PX4/MAVROS driver connecting to the flight controller. `./start.sh` is idempotent
  (checks if the container is already running before starting it). This container shares `network_mode:
  host` with the rest of the stack, so it talks to the same `roscore` as `unified_autonomy_stack` without
  any extra `ROS_MASTER_URI` configuration.

**One real bug hit and fixed here**: the first version used `docker exec -it ros1-dev bash -c
"roslaunch ..."`, which failed with `bash: roslaunch: command not found`. `ros1_docker_dev`'s Dockerfile
sources `/opt/ros/noetic/setup.bash` and the workspace's `devel/setup.bash` from `~/.bashrc` — but
`~/.bashrc` is only sourced by *interactive* shells, and `bash -c "cmd"` runs non-interactively regardless
of `docker exec -it`'s `-it` (that only allocates a tty / keeps stdin open; it doesn't make `bash -c`
itself behave like an interactive shell). **Fix: `bash -c` → `bash -ic`** — the `-i` flag forces bash to
treat itself as interactive, which makes it source `~/.bashrc` (and therefore `roslaunch`'s `PATH`) before
running the given command. Verified with `docker exec ros1-dev bash -ic 'which roslaunch'` →
`/opt/ros/noetic/bin/roslaunch`.

---

## 12. Hesai lidar `pkt loss freq` warnings — kernel UDP receive buffer too small (host-level fix)

Seen intermittently in `ros1_launch_hesailidar` logs, sometimes severe (e.g. `12139/14707` — over 80% of
packets in that second):

```
ros1_launch_hesailidar-1  | [2026-08-15 12:43:35][WARNING]pkt loss freq: 12139/14707
```

**Root cause, confirmed (not guessed) via `/proc/net/snmp`**: `Udp: ... RcvbufErrors: 5525` — real kernel-
level UDP socket receive-buffer overflow. The lidar's dedicated NIC (`enP8p1s0`, `192.168.6.1/24`) showed
**zero** drops at the interface level (`ip -s link`), so packets were arriving fine; they were being
dropped *after* the NIC, in the kernel socket buffer, before `hesai_ros_driver`'s parser thread could
drain them. The system-wide cap, `net.core.rmem_max = 212992` (~208KB), was almost certainly clamping
down whatever buffer size the driver actually requests via `setsockopt(SO_RCVBUF, ...)` — the kernel
silently caps any larger request to this ceiling, so raising the app's own buffer request (not exposed in
`config.yaml` anyway) wouldn't have helped without also raising this.

Secondary/contributing factor: `load average` was `13.08` on an 8-core box at the time — genuine CPU
oversubscription (NMPC's two Python nodes, `realsense2_camera`, `hesai_ros_driver`, `mimosa_node`, the
ROS1↔ROS2 bridge, and `gbplanner_node` all contending simultaneously), which makes the parser thread more
likely to fall behind and let the (too-small) buffer overflow. Jetson was already at `MAXN_SUPER` (max
performance power mode) — no free power-mode headroom to reach for there.

**Fix (host-level `sysctl`, not a driver/code change)**:

```bash
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.rmem_default=8388608
# persisted across reboots:
echo -e "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608" | sudo tee /etc/sysctl.d/99-lidar-rmem.conf
```

**Verified fixed**, not just applied: restarted the stack (`launch_stack`) after the change, then watched
both sides over a live ~25-35s window with the lidar actively streaming (~9000 UDP packets/sec):
- `RcvbufErrors` in `/proc/net/snmp` stayed **exactly flat** (`6205` → `6205`) while `InDatagrams`
  increased by ~225,000 — heavy real traffic, zero new kernel-level drops.
- Zero `pkt loss freq` warnings logged by the driver itself over the same window (previously several
  per second, some over 80% loss).
- `load average` also dropped to `6.71` post-restart (still nontrivial for 8 cores, but no longer badly
  oversubscribed — likely just startup settling).

If this recurs after a fresh reboot and the `/etc/sysctl.d/99-lidar-rmem.conf` file is somehow missing or
not applied (check with `sysctl net.core.rmem_max` — should read `8388608`, not `212992`), that's the
first thing to check before assuming it's a new problem.
