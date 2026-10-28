#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
/usr/bin/python3 prepare_teacher.py
if ! command -v colcon >/dev/null 2>&1; then
  echo 'colconが見つかりません。既存ROSワークスペースをビルドした環境で実行してください。'
  exit 1
fi
(
  cd teacher_ws
  colcon build --packages-select crane_cube_teacher \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)
source teacher_ws/install/setup.bash
exec /usr/bin/python3 auto_record.py "$@"
