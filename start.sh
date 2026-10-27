#!/usr/bin/env bash
# real启动真机底层与策略；sim启动MuJoCo与策略。只启动进程，不自动使能机器人。
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  cat <<'HELP'
Usage: ./start.sh [real|sim] [launch arguments]
  real  Hardware bridge and C++ policy switcher (default).
  sim   MuJoCo simulation and C++ policy switcher; Gazebo is not used.
Examples:
  ./start.sh real robot_name:=n3 use_rviz:=false
  ./start.sh real robot_name:=n3 device:=pico human_height:=1.78 visualize:=true
  ./start.sh sim robot_name:=n3 gui:=false
  ./start.sh real robot_name:=n3 joystick_config:=/home/nano/deploy/joystick.yaml
Press Ctrl+C to stop the processes started by this script.
HELP
  exit 0
fi

mode=real
case ${1:-} in
  real|sim) mode=$1; shift ;;
  ''|*:=*) ;;
  *) echo '[ERROR] Expected real or sim. Use --help for usage.' >&2; exit 2 ;;
esac
TASK_WS_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$TASK_WS_ROOT/install/setup.bash" ]]; then
  echo '[ERROR] ROS or workspace setup is missing. Run ./build.sh first.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$TASK_WS_ROOT/install/setup.bash"
cd "$TASK_WS_ROOT"

if [[ $mode == sim ]]; then
  # exec保留launch的信号处理；其子节点随同一个launch正常退出。
  exec ros2 launch noetix_mujoco sim.launch.py launch_policy:=true "$@"
fi

command -v setsid >/dev/null || { echo '[ERROR] setsid is not installed.' >&2; exit 1; }
real_args=()
policy_args=(use_sim_time:=false)
for argument in "$@"; do
  case "$argument" in
    joystick_config:=*) policy_args+=("$argument") ;;
    use_sim_time:=*) echo '[ERROR] Hardware mode uses wall time; use sim for simulation.' >&2; exit 2 ;;
    *) real_args+=("$argument") ;;
  esac
done

# 两个launch分别占用进程组，退出时只停止本脚本创建的进程，不影响其他终端。
launch_pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${launch_pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  for pid in "${launch_pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch noetix_bringup real.launch.py "${real_args[@]}" &
launch_pids+=("$!")
setsid ros2 launch noetix_policy policy.launch.py "${policy_args[@]}" &
launch_pids+=("$!")

# 任意一个launch结束后退出，并清理另一个，避免只剩一半部署进程继续运行。
set +e
wait -n "${launch_pids[@]}"
result=$?
set -e
exit "$result"
