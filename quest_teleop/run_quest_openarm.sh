#!/usr/bin/env bash
# Meta Quest -> OpenArm arms in the ITRI_project Isaac Sim 5.1 scene (default: the right arm only).
# Uses the ROS environment of the ITRI chain flow (scripts/vla/run_tote_chain_flow.sh): ROS_DOMAIN_ID 42,
# rmw_fastrtps_cpp + its FastDDS profile, NOT ros_env.sh (ROS_LOCALHOST_ONLY=1 there would not see the chain).
# Held-tote attempt: pass --run-dir <run_dir> of a chain started with TOTE_CHAIN_ASSIST=1 TOTE_CHAIN_RECORD=1.
# Extra args pass through, e.g. --arms right, --yaw-deg 180, --scale 0.5, --rot-frame base
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ITRI_PROJECT="${ITRI_PROJECT:-$HOME/Documents/ITRI_project}"
export ITRI_PROJECT
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ITRI_PROJECT/ros2_ws/src/itri_bringup/config/tote_collection_fastdds_large_shm.xml}"
unset ROS_LOCALHOST_ONLY
cd "$HERE"
exec "$HERE/../.venv_quest/bin/python" "$HERE/quest_openarm_teleop.py" "$@"
