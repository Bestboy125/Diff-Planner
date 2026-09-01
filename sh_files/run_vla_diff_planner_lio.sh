#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:---bridge-only}"
LIVE_OUTPUT="${VLA_BRIDGE_LIVE_OUTPUT_ENABLED:-false}"
HOST_IP="${VLA_HOST_ONBOARD_IP:-192.168.14.250}"
AUTH_TOKEN="${VLA_BRIDGE_AUTH_TOKEN:-}"
ODOM_TOPIC="${VLA_ODOM_TOPIC:-/ekf/ekf_odom}"
START_RVIZ="${VLA_START_RVIZ:-false}"
PIDS=()

if [[ -z "${AUTH_TOKEN}" || "${AUTH_TOKEN}" == "REQUIRED" ]]; then
  echo "VLA_BRIDGE_AUTH_TOKEN must be set to the same non-default secret as the host backend." >&2
  exit 2
fi

if [[ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  echo "Missing ${WORKSPACE_ROOT}/devel/setup.bash; build this catkin workspace on Ubuntu first." >&2
  exit 2
fi

if [[ "${MODE}" != "--bridge-only" && "${MODE}" != "--full-lio" ]]; then
  echo "Usage: $0 [--bridge-only|--full-lio]" >&2
  exit 2
fi

if [[ "${MODE}" == "--full-lio" && "${VLA_ALLOW_HARDWARE_STACK:-}" != "I_UNDERSTAND_HARDWARE_STACK" ]]; then
  echo "Set VLA_ALLOW_HARDWARE_STACK=I_UNDERSTAND_HARDWARE_STACK before --full-lio." >&2
  exit 2
fi

if [[ "${LIVE_OUTPUT}" == "true" && "${VLA_ALLOW_LIVE_OUTPUT:-}" != "I_UNDERSTAND_LIVE_OUTPUT" ]]; then
  echo "Live goal publishing requires VLA_ALLOW_LIVE_OUTPUT=I_UNDERSTAND_LIVE_OUTPUT." >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  wait || true
}
trap cleanup EXIT INT TERM

launch_bg() {
  "$@" &
  PIDS+=("$!")
}

if [[ "${MODE}" == "--full-lio" ]]; then
  launch_bg roslaunch mavros px4.launch
  sleep 2
  rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 || true
  rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 || true
  rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 || true
  rosrun mavros mavcmd long 511 147 5000 0 0 0 0 0 || true
  rosrun mavros mavcmd long 511 106 5000 0 0 0 0 0 || true
  launch_bg roslaunch faster_lio mapping_mid360.launch
  sleep 10
  launch_bg roslaunch ekf ekf_lidar.launch
  sleep 5
  launch_bg roslaunch diff_planner run_exp_single_lio.launch
  sleep 3
  launch_bg roslaunch px4ctrl run_ctrl_lio.launch
  sleep 3
fi

launch_bg roslaunch vla_diff_bridge vla_diff_bridge.launch \
  auth_token:="${AUTH_TOKEN}" \
  allowed_host_ip:="${HOST_IP}" \
  live_publish_enabled:="${LIVE_OUTPUT}" \
  odom_topic:="${ODOM_TOPIC}"

if [[ "${START_RVIZ}" == "true" ]]; then
  launch_bg roslaunch diff_planner exp_rviz.launch
fi

echo "VLA bridge started in ${MODE} mode; live_publish_enabled=${LIVE_OUTPUT}."
wait
