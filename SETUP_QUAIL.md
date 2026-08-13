# quail setup notes: SLAM + GBPlanner + NMPC on the Orin NX

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
- **New file `config/ros2/quail.yaml`** (NMPC config, not yet in git — untracked): built from magpie's
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
