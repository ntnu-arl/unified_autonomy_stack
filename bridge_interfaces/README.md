# Custom bridge interfaces

`ros1/picaso_msgs` contains the catkin build files for the ROS 1 counterpart
of the driver's ROS 2 `picaso_msgs` package. During the bridge image build,
`Dockerfile.ros1_bridge_builder` copies the driver's `Capture.msg` into this
package, replacing `builtin_interfaces/Time` with ROS 1's native `time`.
All field names and payload types otherwise remain identical so ros1_bridge
can generate the conversion automatically.

The source of truth is
`workspaces/ws_air_sonar/src/picaso-drone/picaso_msgs/msg/Capture.msg`.
Import `repos/ws_air_sonar.repos` before building the bridge image. Changes
to that message require rebuilding both the sonar workspace and bridge image,
then extracting the bridge workspace with `make build-ros1_bridge`.
