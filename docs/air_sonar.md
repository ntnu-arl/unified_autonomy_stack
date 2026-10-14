# PICASO air sonar (ROS 2 Humble)

The driver lives in `workspaces/ws_air_sonar/src/picaso-drone`, on
`feat/ros2-driver`. The workspace contains the `picaso_msgs` interfaces and
`picaso_driver`. It publishes packed PDM microphone captures, not estimated
ranges or a point cloud. No simulator or navigation stack is needed.

## Build

The local clone is already in place. On a new checkout, import the workspace
after the driver branch has been pushed:

```bash
mkdir -p workspaces/ws_air_sonar
vcs import workspaces/ws_air_sonar < repos/ws_air_sonar.repos
```

Build only the sonar image and workspace:

```bash
docker buildx bake ros2_air_sonar --load --allow=network.host
make build-air_sonar
```

The workspace build also installs `robot_bringup`. The lean image contains
Humble, serial and optional webcam dependencies; it does not require a GPU.
The sonar image and workspace are also included in `make images` and `make build`.

## USB setup on the host

Install the repository's USB rule on the **host**, once:

```bash
sudo cp workspaces/ws_air_sonar/src/picaso-drone/99-picaso.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
```

This is the same host-side rule installation used by VectorNav; there is no
sonar-specific installer or udev daemon in the container. Installation and reload
can be done with the sensor disconnected. `/dev/picaso` appears only after a
matching sensor is connected; check it then with `ls -l /dev/picaso`.

The rule matches STM32 USB CDC VID/PID `0483:5740`, creates `/dev/picaso`, and
grants access to the `dialout` group. Reconnect the sensor after installation
if necessary. For native, non-Docker use, add your user to `dialout` and log
in again. When several matching STM32 devices are present, specialize the rule
with `ATTRS{serial}` and use the driver's `serial_number` parameter.
Set `port: ""` in the YAML to enable automatic discovery by VID/PID and optional
serial number. An explicit port bypasses discovery.

The Compose service mounts host `/dev` so the symlink and replacement device
nodes remain visible after unplug/replug. Device-cgroup rules allow only ttyACM,
ttyUSB, and optional V4L2 camera device classes; `privileged` is not used. The
standalone container runs as root without capabilities. Udev continues to run on
the host. In the robot stack, sonar runs as uid 1000 with host IPC to match the
ROS 1 bridge's FastDDS shared-memory permissions. Serial and optional camera
access use the host `dialout` and `video` groups (GIDs 20 and 44 by default;
override `AIR_SONAR_DIALOUT_GID` / `AIR_SONAR_VIDEO_GID` if needed).

## Launch, configure, inspect and stop

To run the sonar alongside the general robot stack:

```bash
make images
make build
make launch DOCKER_COMPOSE_FILE=docker-compose.robot.yml
```

The robot stack reuses the standalone sonar service, including its serial-device
permissions and configuration. To launch the sonar by itself:

```bash
make launch DOCKER_COMPOSE_FILE=docker-compose.air_sonar.yml
```

This launches **only** `ros2_launch_air_sonar`. It keeps retrying if the sensor
is absent, so hardware can be connected later. ROS domain comes from `.env`
`DOMAIN_ID` (205 by default). To override the serial path:

```bash
AIR_SONAR_PORT=/dev/ttyACM0 make launch DOCKER_COMPOSE_FILE=docker-compose.air_sonar.yml
```

Edit `workspaces/robot_bringup/config/ros2/air_sonar.yaml` for capture mode,
events, sweep, capture period, timeouts, reconnect interval or optional webcam.
An empty `waveform_file` selects the sweep installed with the driver.
`camera_index: -1` disables the webcam. Restart the service after changing
parameters; the bringup directory is mounted read-only in the runtime container.

Attach in another terminal:

```bash
make attach-air_sonar DOCKER_COMPOSE_FILE=docker-compose.air_sonar.yml
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 topic echo /air_sonar/status
ros2 interface show picaso_msgs/msg/Capture
ros2 topic hz /air_sonar/capture
```

The launch entrypoint sources Humble and the built workspace; source them as
above in a new attached shell. A manual
launch from an attached shell, after stopping the existing node, can override
parameters and logging:

```bash
ros2 launch robot_bringup air_sonar.launch.py port:=/dev/picaso log_level:=debug
```

Stop the standalone service without touching the rest of the stack:

```bash
docker compose -f docker-compose.air_sonar.yml --profile launch down
```

## Published data

- `/air_sonar/capture`: `picaso_msgs/msg/Capture`, including a standard ROS
  header, capture identifier/mode, microphone bit ordering, event
  count, sample rate, packed `uint16[] pdm_words`, and the actual transmitted
  waveform for active captures. All receive/timing fields formerly saved in
  metadata are preserved, including realtime/monotonic nanoseconds and UTC
  strings. See `ros2 interface show` and the driver's README for field semantics.
- `/air_sonar/status`: `diagnostic_msgs/msg/DiagnosticArray`, including serial
  connection/reconnect state and acquisition errors.
- `/air_sonar/image/compressed`: optional paired `sensor_msgs/msg/CompressedImage`
  when `camera_index >= 0`. The capture includes the associated image timestamp.

The PDM payload remains lossless and typed, rather than being wrapped as an
opaque `.bin` byte blob. Its words are bit-packed samples, not PCM or ranges.
Frames describe the sensor; the driver does not invent a body-to-sensor TF.
Publish the measured mounting transform separately when integrating hardware.

Record data with standard ROS tools:

```bash
ros2 bag record -o /workspace/recordings/sonar \
  /air_sonar/capture /air_sonar/status /air_sonar/image/compressed
```

Recordings under `/workspace` persist in the host `workspaces/ws_air_sonar`.
The standalone launch does not start the ROS 1 bridge. ROS 2 consumers can use
the capture directly in the matching ROS domain.

## Recording with the robot stack's ROS 1 rosbag

The robot bridge configuration forwards `/air_sonar/capture` and
`/air_sonar/image/compressed` to ROS 1, and `bag_rec.launch` includes both in
its topic list. Set `RECORD=true` in the root `.env` to enable the recorder;
bags are saved under the host `data/` directory by default. The image topic
has data only when the optional webcam is enabled (`camera_index >= 0`).

The custom capture requires rebuilding the bridge with both versions of
`picaso_msgs`. The builder uses the driver's ROS 2 message as the source of
truth, generating the ROS 1 counterpart with native `time` for `image_stamp`.
It includes the ROS 2 message libraries in the extracted bridge workspace.
After updating these files, rebuild and extract the bridge before launching:

```bash
docker buildx bake ros1-bridge-builder --load --allow=network.host
make build-ros1_bridge
make launch DOCKER_COMPOSE_FILE=docker-compose.robot.yml
```

Rebuild the sonar workspace too (`make build-air_sonar`) if the driver's
message definition changes. The recorder stores the full packed PDM payload
and timestamps as `picaso_msgs/Capture` in the ROS 1 bag.

## Testing without the sensor

The driver repository includes protocol tests and a serial pseudo-terminal
emulator for acquisition and reconnect tests. Run its pytest tests in the image
after building the workspace (see the driver README for the emulator command).
The same tests can run with the production Compose permissions and mounts:

```bash
docker compose -f docker-compose.air_sonar.yml --profile launch run --rm --no-deps -T \
  -e ROS_DOMAIN_ID=224 ros2_launch_air_sonar \
  python3 -m pytest src/picaso-drone/picaso_driver/test -q -s
```

Real firmware timing, acoustic output, and actual USB reconnects still require
the hardware. The ordinary Compose launch also checks the disconnected-sensor
path: the node should remain alive and publish diagnostics while retrying.
