#!/usr/bin/env bash
# MuJoCo 3.3.7 for noetix_mujoco / ROS 2 Humble (Ubuntu 22.04).
# Supports x86_64 and aarch64; installs the wheel into the SAME Python
# interpreter used by the existing ROS 2 mujoco_backend entry point.
# Does not install or upgrade NumPy through pip (ROS ABI compatibility).
set -Eeuo pipefail

VERSION="${MUJOCO_VERSION:-3.3.7}"
ARCH="$(uname -m)"
case "$ARCH" in
  x86_64|aarch64) ;;
  *) echo "[ERROR] Unsupported architecture: $ARCH (expected x86_64 or aarch64)" >&2; exit 1 ;;
esac

if [[ "${EUID}" -eq 0 ]]; then
  echo '[ERROR] Run this installer as your normal ROS 2 user, not with sudo.' >&2
  exit 1
fi

# ROS 2 ament_python entry points have a fixed Python shebang.
# Prefer that exact interpreter over shell 'python3' (which may be Conda).
ENTRY="${MUJOCO_ENTRYPOINT:-${PWD}/install/noetix_mujoco/lib/noetix_mujoco/mujoco_backend}"
PY="${MUJOCO_PYTHON:-}"
if [[ -z "$PY" && -f "$ENTRY" ]]; then
  read -r SHEBANG < "$ENTRY" || true
  if [[ "$SHEBANG" =~ ^\#!(/[^[:space:]]+/python3([.]?[0-9]+)?)([[:space:]]|$) ]]; then
    PY="${BASH_REMATCH[1]}"
  fi
fi
PY="${PY:-/usr/bin/python3}"
if [[ ! -x "$PY" ]]; then
  echo "[ERROR] Python executable not found: $PY" >&2
  exit 1
fi

echo "[INFO] Architecture : $ARCH"
echo "[INFO] Python       : $PY"
echo "[INFO] MuJoCo       : $VERSION"
if [[ -f "$ENTRY" ]]; then
  echo "[INFO] ROS entry    : $ENTRY"
else
  echo '[INFO] No existing ROS entry point found; using system Python by default.'
fi

"$PY" - <<'PY'
import site, sys
print('[INFO] Python ver   :', sys.version.split()[0])
if sys.version_info < (3, 10):
    raise SystemExit('[ERROR] ROS 2 Humble installation requires Python >= 3.10 for this MuJoCo installer.')
if site.ENABLE_USER_SITE is not True:
    raise SystemExit('[ERROR] Python user-site is disabled. Unset PYTHONNOUSERSITE or select the interpreter used by the ROS entry point.')
PY

# Install only the graphics/runtime packages we need; no CUDA or CUDA toolkit.
sudo apt-get update
sudo apt-get install -y \
  python3-pip python3-numpy python3-yaml \
  libglfw3 libgl1 libegl1 libglx0 libglvnd0 libosmesa6 libglew2.2 \
  ca-certificates

# Keep ROS / apt NumPy intact. Installing latest NumPy 2 via pip can break
# binary ROS packages such as cv_bridge built against NumPy 1.x.
"$PY" -m pip install --user --only-binary=:all: \
  'absl-py' 'etils[epath]' 'glfw' 'PyOpenGL'
"$PY" -m pip install --user --only-binary=:all: --no-deps \
  "mujoco==${VERSION}"

# Headless physics smoke test, no DISPLAY required.
"$PY" - <<'PY'
import sys
import numpy as np
import mujoco
from pathlib import Path

print('[OK] Interpreter   :', sys.executable)
print('[OK] NumPy version :', np.__version__)
print('[OK] MuJoCo version:', mujoco.__version__)
print('[OK] MuJoCo path   :', Path(mujoco.__file__).resolve())
xml = '''<mujoco><worldbody>
<body name="ball" pos="0 0 1"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body>
</worldbody></mujoco>'''
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
mujoco.mj_step(m, d)
print('[OK] MuJoCo model load + physics step')
if int(np.__version__.split('.')[0]) >= 2:
    print('[WARN] NumPy 2 detected; some ROS 2 Humble binary modules may expect NumPy 1.x.')
PY

if [[ -r /opt/ros/humble/setup.bash ]]; then
  if "$PY" -c 'import rclpy' >/dev/null 2>&1; then
    echo '[OK] rclpy import with the same Python interpreter'
  else
    echo '[NOTE] rclpy import was not available in this shell.'
    echo '       Source /opt/ros/humble/setup.bash and your workspace install/setup.bash before launching.'
  fi
fi

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo '[NOTE] No desktop display detected. MuJoCo physics works, but GUI requires a graphical session.'
fi

echo
echo '[SUCCESS] MuJoCo installed into the Python interpreter used by noetix_mujoco.'
echo 'Next: source /opt/ros/humble/setup.bash && source install/setup.bash'
echo '      ros2 launch noetix_mujoco sim.launch.py robot_name:=n3 launch_policy:=false'
