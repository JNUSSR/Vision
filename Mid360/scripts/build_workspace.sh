#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${MID360_WORKSPACE:-$(dirname -- "$(dirname -- "$project_dir")")/mid360_ws}"
sdk_prefix="${MID360_SDK_PREFIX:-$workspace/deps/sdk}"
ros_setup="${MID360_ROS_SETUP:-/opt/ros/humble/setup.bash}"

source "$ros_setup"
mkdir -p "$workspace/src" "$workspace/deps"
for package in fast_lio_comparison fast_livo2_ros2_src livox_ros_driver2 rpg_vikit_rational_polynomial mid360_live; do
  destination="$workspace/src/$package"
  source_dir="$project_dir/$package"
  if [[ -e "$destination" || -L "$destination" ]]; then
    if [[ "$(readlink -f "$destination")" != "$(readlink -f "$source_dir")" ]]; then
      echo "工作空间中已有不同的 $destination，请选择新的 MID360_WORKSPACE。" >&2
      exit 1
    fi
  else
    ln -s "$source_dir" "$destination"
  fi
done

cmake -S "$project_dir/Livox-SDK2" -B "$workspace/deps/sdk-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$sdk_prefix"
cmake --build "$workspace/deps/sdk-build" --parallel "${MID360_BUILD_JOBS:-2}"
cmake --install "$workspace/deps/sdk-build"
export LD_LIBRARY_PATH="$sdk_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAKEFLAGS="-j${MID360_BUILD_JOBS:-2}"
export CMAKE_BUILD_PARALLEL_LEVEL="${MID360_BUILD_JOBS:-2}"
cd "$workspace"
colcon build --executor sequential --packages-up-to fast_lio fast_livo mid360_live \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  "-DLIVOX_LIDAR_SDK_LIBRARY=$sdk_prefix/lib/liblivox_lidar_sdk_shared.so" \
  "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$sdk_prefix/include" \
  "-Dvikit_common_DIR=$workspace/build/vikit_common"
