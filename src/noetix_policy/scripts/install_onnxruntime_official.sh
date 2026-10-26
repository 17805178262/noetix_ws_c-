#!/usr/bin/env bash
# Official ONNX Runtime 1.23.2 CPU C++ SDK installer (x86_64 / aarch64).
# Uses upstream onnxruntimeConfig.cmake and onnxruntime::onnxruntime.
# NO custom CMake Config and NO directory compatibility symlinks.
set -Eeuo pipefail

VERSION="1.23.2"
PREFIX="/opt/onnxruntime"
ARCHIVE=""
SOURCE_DIR=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [--archive /path/sdk.tgz | --sdk-dir /path/sdk]

Default: reuse a matching ONNX Runtime $VERSION SDK from $PREFIX if possible;
otherwise download the official CPU archive for this CPU architecture.

Installs files in the layout required by the official CMake Config:
  $PREFIX/include/onnxruntime/*.h
  $PREFIX/lib64/libonnxruntime.so*
  $PREFIX/lib/cmake/onnxruntime/onnxruntimeConfig.cmake

The package is found by:
  find_package(onnxruntime CONFIG REQUIRED)
  target_link_libraries(target PRIVATE onnxruntime::onnxruntime)
EOF
}

while (($#)); do
  case "$1" in
    --archive)
      (($# >= 2)) || { echo "--archive needs a path" >&2; exit 2; }
      ARCHIVE="$2"; shift 2 ;;
    --sdk-dir)
      (($# >= 2)) || { echo "--sdk-dir needs a path" >&2; exit 2; }
      SOURCE_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ -n "$ARCHIVE" && -n "$SOURCE_DIR" ]]; then
  echo "Use either --archive or --sdk-dir, not both." >&2
  exit 2
fi

case "$(uname -m)" in
  x86_64) SDK_ARCH="x64"; ELF_PATTERN='x86-64' ;;
  aarch64|arm64) SDK_ARCH="aarch64"; ELF_PATTERN='aarch64|ARM aarch64' ;;
  *) echo "Unsupported platform: $(uname -m). Requires x86_64 or aarch64." >&2; exit 1 ;;
esac

install_tools_if_missing() {
  local missing=()
  command -v cmake >/dev/null || missing+=(cmake)
  command -v g++ >/dev/null || missing+=(g++)
  command -v file >/dev/null || missing+=(file)
  command -v tar >/dev/null || missing+=(tar)
  command -v curl >/dev/null || missing+=(curl)
  command -v ldconfig >/dev/null || [[ -x /sbin/ldconfig ]] || missing+=(libc-bin)
  if ((${#missing[@]})); then
    echo "Installing missing system tools: ${missing[*]}"
    sudo apt-get update
    sudo apt-get install -y ca-certificates "${missing[@]}"
  fi
}
install_tools_if_missing

if [[ -n "$ARCHIVE" ]]; then
  [[ -f "$ARCHIVE" ]] || { echo "Archive not found: $ARCHIVE" >&2; exit 1; }
fi
if [[ -n "$SOURCE_DIR" ]]; then
  [[ -d "$SOURCE_DIR" ]] || { echo "SDK directory not found: $SOURCE_DIR" >&2; exit 1; }
fi

TMPDIR_ORT="$(mktemp -d -t ort-official.XXXXXXXX)"
trap 'rm -rf "$TMPDIR_ORT"' EXIT

# A valid official Config should be present. Never reuse a pip-only SDK.
find_sdk_library() {
  local root="$1"
  for d in "$root/lib64" "$root/lib"; do
    if [[ -f "$d/libonnxruntime.so.${VERSION}" ]]; then
      printf '%s\n' "$d/libonnxruntime.so.${VERSION}"
      return 0
    fi
  done
  return 1
}

if [[ -z "$ARCHIVE" && -z "$SOURCE_DIR" ]]; then
  if [[ -f "$PREFIX/lib/cmake/onnxruntime/onnxruntimeConfig.cmake" ]] \
    && { [[ -f "$PREFIX/include/onnxruntime/onnxruntime_cxx_api.h" ]] \
         || [[ -f "$PREFIX/include/onnxruntime_cxx_api.h" ]]; } \
    && find_sdk_library "$PREFIX" >/dev/null; then
    SOURCE_DIR="$PREFIX"
    echo "Using existing official $VERSION SDK in $PREFIX."
  else
    ARCHIVE="$TMPDIR_ORT/onnxruntime-linux-${SDK_ARCH}-${VERSION}.tgz"
    URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/onnxruntime-linux-${SDK_ARCH}-${VERSION}.tgz"
    echo "Downloading official ONNX Runtime $VERSION (${SDK_ARCH}, CPU):"
    echo "  $URL"
    curl --fail --location --retry 3 --connect-timeout 20 --output "$ARCHIVE" "$URL"
  fi
fi

if [[ -n "$ARCHIVE" ]]; then
  SOURCE_DIR="$TMPDIR_ORT/raw"
  mkdir -p "$SOURCE_DIR"
  tar -xzf "$ARCHIVE" --strip-components=1 -C "$SOURCE_DIR"
fi

[[ -f "$SOURCE_DIR/lib/cmake/onnxruntime/onnxruntimeConfig.cmake" ]] || {
  echo "Official onnxruntimeConfig.cmake was not found in the SDK." >&2
  exit 1
}

STAGE="$TMPDIR_ORT/staged-sdk"
mkdir -p "$STAGE"
cp -a "$SOURCE_DIR/." "$STAGE/"

# Normalize only the SDK release's incorrect folder layout.
# Upstream config is kept byte-for-byte, under lib/cmake/onnxruntime.
if [[ -L "$STAGE/include/onnxruntime" ]]; then
  rm "$STAGE/include/onnxruntime"
fi
if [[ -f "$STAGE/include/onnxruntime_cxx_api.h" ]]; then
  mkdir -p "$STAGE/include/onnxruntime"
  find "$STAGE/include" -mindepth 1 -maxdepth 1 ! -name onnxruntime \
    -exec mv -t "$STAGE/include/onnxruntime" -- {} +
fi
if [[ -L "$STAGE/lib64" ]]; then
  rm "$STAGE/lib64"
fi
mkdir -p "$STAGE/lib64"
shopt -s nullglob
for libfile in "$STAGE/lib/"*.so*; do
  mv -- "$libfile" "$STAGE/lib64/"
done
shopt -u nullglob

# The actual imported target expects lib64 and include/onnxruntime.
[[ -f "$STAGE/include/onnxruntime/onnxruntime_cxx_api.h" ]] || {
  echo "C++ API header missing after installation layout normalization." >&2; exit 1;
}
[[ -e "$STAGE/lib64/libonnxruntime.so" ]] || {
  echo "Shared library missing after installation layout normalization." >&2; exit 1;
}
[[ -f "$STAGE/lib64/libonnxruntime.so.${VERSION}" ]] || {
  echo "Expected libonnxruntime.so.${VERSION}, wrong SDK version or incomplete SDK." >&2; exit 1;
}
file -L "$STAGE/lib64/libonnxruntime.so" | grep -Eiq "$ELF_PATTERN" || {
  echo "SDK ELF architecture does not match $(uname -m)." >&2; exit 1;
}
API_VERSION="$(awk '$1=="#define" && $2=="ORT_API_VERSION" {print $3; exit}' \
  "$STAGE/include/onnxruntime/onnxruntime_c_api.h")"
if [[ ! "$API_VERSION" =~ ^[0-9]+$ ]] || ((API_VERSION < 23)); then
  echo "ONNX Runtime C API must be >= 23. Found: ${API_VERSION:-missing}" >&2
  exit 1
fi

# Verify the official imported target really configures, compiles and runs.
SMOKE="$TMPDIR_ORT/smoke"
mkdir -p "$SMOKE"
cat > "$SMOKE/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(official_ort_smoke LANGUAGES CXX)
find_package(onnxruntime CONFIG REQUIRED)
if(NOT TARGET onnxruntime::onnxruntime)
  message(FATAL_ERROR "Official onnxruntime::onnxruntime target not exported")
endif()
add_executable(official_ort_smoke main.cpp)
target_link_libraries(official_ort_smoke PRIVATE onnxruntime::onnxruntime)
CMAKE
cat > "$SMOKE/main.cpp" <<'CPP'
#include <onnxruntime_cxx_api.h>
#include <iostream>
int main() {
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "noetix-ort-install-check");
  std::cout << "ONNX Runtime " << Ort::GetVersionString() << '\n';
  return 0;
}
CPP

smoke_test() {
  local prefix="$1"
  local out_dir="$2"
  cmake -S "$SMOKE" -B "$out_dir" \
    -Donnxruntime_DIR="$prefix/lib/cmake/onnxruntime" \
    -DCMAKE_BUILD_TYPE=Release
  cmake --build "$out_dir" --parallel 2
  "$out_dir/official_ort_smoke"
}

echo "Testing official CMake Config in staged SDK ..."
smoke_test "$STAGE" "$TMPDIR_ORT/build-before-install"

# Allow repeat invocations without rewriting an already correct installation.
if [[ "$SOURCE_DIR" == "$PREFIX" && \
      -d "$PREFIX/lib64" && ! -L "$PREFIX/lib64" && \
      -f "$PREFIX/include/onnxruntime/onnxruntime_cxx_api.h" && \
      -f "$PREFIX/lib64/libonnxruntime.so.${VERSION}" ]]; then
  echo "SDK is already in the official Config layout. No replacement needed."
else
  echo "Installing normalized official SDK under $PREFIX ..."
  BACKUP="${PREFIX}.backup.$(date +%Y%m%d%H%M%S).$$"
  HAD_OLD=0
  sudo install -d /opt
  if [[ -e "$PREFIX" ]]; then
    sudo mv "$PREFIX" "$BACKUP"
    HAD_OLD=1
  fi
  if ! sudo mkdir -p "$PREFIX" || ! sudo cp -a "$STAGE/." "$PREFIX/"; then
    echo "Installation copy failed. Restoring previous SDK." >&2
    sudo rm -rf "$PREFIX"
    if ((HAD_OLD)); then sudo mv "$BACKUP" "$PREFIX"; fi
    exit 1
  fi
  if ((HAD_OLD)); then
    echo "Previous SDK backed up at: $BACKUP"
  fi
fi

# The dynamic loader uses real lib64, never a compatibility directory symlink.
printf '%s\n' "$PREFIX/lib64" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf >/dev/null
sudo ldconfig

# Verify from the final fixed install prefix too.
echo "Testing installed official CMake Config ..."
smoke_test "$PREFIX" "$TMPDIR_ORT/build-installed"

echo
echo "SUCCESS: official ONNX Runtime $VERSION CPU C++ SDK installed."
echo "  Config: $PREFIX/lib/cmake/onnxruntime/onnxruntimeConfig.cmake"
echo "  Header: $PREFIX/include/onnxruntime/onnxruntime_cxx_api.h"
echo "  Library: $PREFIX/lib64/libonnxruntime.so"
echo "  CMake target: onnxruntime::onnxruntime"
echo "Run 'colcon build --cmake-clean-cache' once after replacing noetix_policy."
echo "Later builds: colcon build"
