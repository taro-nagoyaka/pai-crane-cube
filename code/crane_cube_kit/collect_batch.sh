#!/usr/bin/env bash
# ROS/system Python only. Existing data and learned models are never overwritten.
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
/usr/bin/python3 prepare_scene.py > /dev/null
/usr/bin/python3 prepare_teacher.py
(
  cd teacher_ws
  colcon build --packages-select crane_cube_teacher --cmake-args -DCMAKE_BUILD_TYPE=Release
)
source teacher_ws/install/setup.bash
exec /usr/bin/python3 collect_batch.py "$@"
