#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash mid360_ros.sh driver
  bash mid360_ros.sh fastlio baseline|weighted
  bash mid360_ros.sh live [--algorithm lio|livo2|both] [--no-driver]
  bash mid360_ros.sh record-raw BAG_PATH
  bash mid360_ros.sh record-odom BAG_PATH
  bash mid360_ros.sh play BAG_PATH
  bash mid360_ros.sh replay-compare BAG_PATH
  bash mid360_ros.sh info BAG_PATH
  bash mid360_ros.sh inspect BAG_PATH
  bash mid360_ros.sh topics
EOF
}

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${MID360_WORKSPACE:-$(dirname -- "$(dirname -- "$project_dir")")/mid360_ws}"
livox_ws="${MID360_LIVOX_WS:-$workspace}"
livo_ws="${MID360_LIVO_WS:-$workspace}"
sdk_prefix="${MID360_SDK_PREFIX:-$livox_ws/deps/sdk}"
ros_setup="${MID360_ROS_SETUP:-/opt/ros/humble/setup.bash}"

export LD_LIBRARY_PATH="$sdk_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source "$ros_setup"
source "$livox_ws/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$project_dir/config/cyclonedds.xml}"

command="${1:-}"
case "$command" in
  driver)
    exec ros2 launch livox_ros_driver2 msg_MID360s_launch.py
    ;;
  fastlio)
    mode="${2:-}"
    case "$mode" in
      baseline) weighted=false ;;
      weighted) weighted=true ;;
      *) usage; exit 2 ;;
    esac
    mkdir -p "$project_dir/fast_lio_comparison/PCD" "$project_dir/fast_lio_comparison/Log"
    exec ros2 launch fast_lio mapping.launch.py rviz:=false "static_confidence:=$weighted" "config_path:=$project_dir/fast_lio_comparison/config"
    ;;
  live)
    if [[ "$livo_ws" != "$livox_ws" ]]; then
      set +u
      source "$livo_ws/install/setup.bash"
      set -u
    fi
    shift
    exec python3 "$project_dir/scripts/live_coordinates.py" "$@"
    ;;
  record-raw)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    exec python3 "$project_dir/scripts/record_livox_bag.py" "$2"
    ;;
  record-odom)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    exec ros2 bag record -o "$2" /Odometry
    ;;
  play)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    exec ros2 bag play "$2"
    ;;
  replay-compare)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    [[ -t 0 ]] || { echo "replay-compare must run in an interactive Ubuntu terminal." >&2; exit 2; }
    exec ros2 bag play --start-paused "$2"
    ;;
  inspect)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    exec python3 "$project_dir/scripts/inspect_livox_bag.py" "$2"
    ;;
  info)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    exec ros2 bag info "$2"
    ;;
  topics)
    exec ros2 topic list --no-daemon --spin-time 10 -t
    ;;
  *) usage; exit 2 ;;
esac
