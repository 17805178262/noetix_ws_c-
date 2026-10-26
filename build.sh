#!/usr/bin/env bash
# 编译真机部署包及MuJoCo仿真后端；始终跳过Gazebo，不切换策略语言或安装系统依赖。
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  cat <<'HELP'
Usage: ./build.sh [colcon build arguments]
Build this C++ workspace in Release mode, excluding noetix_gazebo.
MuJoCo, GMR, hardware and policy packages remain included.
Example: ./build.sh --parallel-workers 2
HELP
  exit 0
fi

TASK_WS_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '[ERROR] ROS 2 Humble setup not found: /opt/ros/humble/setup.bash' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
cd "$TASK_WS_ROOT"
command -v colcon >/dev/null || { echo '[ERROR] colcon is not installed.' >&2; exit 1; }

# 源包只枚举当前工作区的部署包，Gazebo不会被扫描或构建。
package_paths=()
for package_path in "$TASK_WS_ROOT"/src/*; do
  [[ -d "$package_path" && $(basename -- "$package_path") != noetix_gazebo ]] || continue
  package_paths+=("$package_path")
done
colcon build --base-paths "${package_paths[@]}" --packages-skip noetix_gazebo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
printf '\n[INFO] Build completed. Start hardware: %s/start.sh real\n' "$TASK_WS_ROOT"
printf '[INFO] Start MuJoCo simulation: %s/start.sh sim\n' "$TASK_WS_ROOT"
