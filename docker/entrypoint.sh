#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
exec "$@"
