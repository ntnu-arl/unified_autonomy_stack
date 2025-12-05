#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
# setup workspace if it exists
if [ -f /workspace/devel/setup.bash ]; then source /workspace/devel/setup.bash; fi
if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi

exec "$@"
