#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-${ROS_DISTRO:-}}" in
  humble)
    printf 'Generated native wrapper selection.\n' > "$root/noetic/COLCON_IGNORE"
    printf 'Generated native wrapper selection.\n' > "$root/noetic/AMENT_IGNORE"
    rm -f "$root/humble/CATKIN_IGNORE"
    ;;
  noetic)
    printf 'Generated native wrapper selection.\n' > "$root/humble/CATKIN_IGNORE"
    rm -f "$root/noetic/COLCON_IGNORE" "$root/noetic/AMENT_IGNORE"
    ;;
  *) printf 'Usage: %s humble|noetic\n' "$0" >&2; exit 2 ;;
esac
