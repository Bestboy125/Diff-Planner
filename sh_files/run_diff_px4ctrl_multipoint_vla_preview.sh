#!/usr/bin/env bash
set -Eeuo pipefail

# One-shot launcher for the real-flight planning/control processes plus the
# isolated VLA preview uplink. This script deliberately contains no MAVROS arm,
# takeoff, land, mode-switch or actuator command.
#
# Prerequisites (started separately):
#   - ROS master and MAVROS telemetry
#   - FAST-LIO and EKF (/ekf/ekf_odom)
#   - registered point cloud (/laserMapping/cloud_registered)
#   - RealSense color stream and CameraInfo
#   - Windows VLA backend
#
# VLA remains preview-only:
#   /vla/preview_goal, /vla/preview_yaw
# It is never remapped to /goal or /setpoints_cmd by this launcher.

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly LOG_ROOT="${VLA_STACK_LOG_DIR:-/tmp/vla_diff_full_stack_$(date +%Y%m%d_%H%M%S)}"

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

fail() {
  log "ERROR: $*" >&2
  exit 1
}

cleanup() {
  local index
  trap - EXIT INT TERM
  if ((${#CHILD_PIDS[@]} > 0)); then
    log 'Stopping launch groups started by this script...'
    for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; index--)); do
      if kill -0 "${CHILD_PIDS[index]}" 2>/dev/null; then
        log "Stopping ${CHILD_NAMES[index]} (pid=${CHILD_PIDS[index]})"
        kill -INT "${CHILD_PIDS[index]}" 2>/dev/null || true
      fi
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "required command not found: $1"
}

require_message() {
  local topic="$1"
  local timeout_sec="${2:-5}"
  if ! timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    fail "required live topic unavailable: ${topic}"
  fi
}

require_node_absent() {
  local node="$1"
  if rosnode list 2>/dev/null | grep -Fxq "${node}"; then
    fail "node is already running: ${node}"
  fi
}

start_launch() {
  local name="$1"
  shift
  local log_file="${LOG_ROOT}/${name}.log"

  log "Starting ${name}; log=${log_file}"
  roslaunch "$@" >"${log_file}" 2>&1 &
  local pid=$!
  CHILD_NAMES+=("${name}")
  CHILD_PIDS+=("${pid}")

  sleep 2
  if ! kill -0 "${pid}" 2>/dev/null; then
    tail -n 80 "${log_file}" >&2 || true
    fail "${name} exited during startup"
  fi
}

start_process() {
  local name="$1"
  shift
  local log_file="${LOG_ROOT}/${name}.log"

  log "Starting ${name}; log=${log_file}"
  "$@" >"${log_file}" 2>&1 &
  local pid=$!
  CHILD_NAMES+=("${name}")
  CHILD_PIDS+=("${pid}")

  sleep 2
  if ! kill -0 "${pid}" 2>/dev/null; then
    tail -n 80 "${log_file}" >&2 || true
    fail "${name} exited during startup"
  fi
}

wait_for_node() {
  local node="$1"
  local attempts="${2:-20}"
  local attempt
  for ((attempt=1; attempt<=attempts; attempt++)); do
    if rosnode info "${node}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  fail "node did not become ready: ${node}"
}

monitor_children() {
  local index
  while true; do
    for ((index=0; index<${#CHILD_PIDS[@]}; index++)); do
      if ! kill -0 "${CHILD_PIDS[index]}" 2>/dev/null; then
        fail "${CHILD_NAMES[index]} exited; inspect ${LOG_ROOT}/${CHILD_NAMES[index]}.log"
      fi
    done
    sleep 1
  done
}

: "${ENABLE_FLIGHT_STACK:?set ENABLE_FLIGHT_STACK to I_UNDERSTAND_THIS_STARTS_PX4CTRL}"
: "${VLA_BACKEND_URL:?set VLA_BACKEND_URL, for example http://192.168.5.2:8080}"
: "${VLA_BRIDGE_TOKEN:?set VLA_BRIDGE_TOKEN}"
: "${VLA_OBSERVATION_TOKEN:?set VLA_OBSERVATION_TOKEN}"
: "${VLA_CALIBRATION_ID:?set the validated VLA_CALIBRATION_ID}"
: "${VLA_CALIBRATION_VALIDATED:?confirm CameraInfo and body<-camera TF first}"

[[ "${ENABLE_FLIGHT_STACK}" == 'I_UNDERSTAND_THIS_STARTS_PX4CTRL' ]] || \
  fail 'ENABLE_FLIGHT_STACK confirmation does not match'
[[ "${VLA_CALIBRATION_VALIDATED}" == 'I_VALIDATED_CAMERA_INFO_AND_TF' ]] || \
  fail 'VLA_CALIBRATION_VALIDATED confirmation does not match'
[[ "${VLA_CALIBRATION_ID}" != 'REQUIRED' ]] || \
  fail 'VLA_CALIBRATION_ID cannot be REQUIRED'
[[ "${VLA_CALIBRATION_ID}" != comm-test-* ]] || \
  fail 'communication-test calibration IDs are forbidden for this launcher'

readonly MULTIPOINT_START_PLAN="${MULTIPOINT_START_PLAN:-0}"
readonly MULTIPOINT_BACK_PLAN="${MULTIPOINT_BACK_PLAN:-0}"
readonly MULTIPOINT_AUTO_PLANNING="${MULTIPOINT_AUTO_PLANNING:-0}"
readonly MULTIPOINT_AUTO_LANDING="${MULTIPOINT_AUTO_LANDING:-0}"

[[ "${MULTIPOINT_START_PLAN}" =~ ^[01]$ ]] || fail 'MULTIPOINT_START_PLAN must be 0 or 1'
[[ "${MULTIPOINT_BACK_PLAN}" =~ ^[01]$ ]] || fail 'MULTIPOINT_BACK_PLAN must be 0 or 1'
[[ "${MULTIPOINT_AUTO_PLANNING}" =~ ^[01]$ ]] || fail 'MULTIPOINT_AUTO_PLANNING must be 0 or 1'
[[ "${MULTIPOINT_AUTO_LANDING}" == '0' ]] || fail 'automatic landing is forbidden in this launcher'

if [[ "${MULTIPOINT_START_PLAN}" == '1' || "${MULTIPOINT_AUTO_PLANNING}" == '1' ]]; then
  [[ "${ENABLE_MULTIPOINT_AUTOSTART:-}" == 'I_ACCEPT_AUTOMATIC_GOAL_PUBLICATION' ]] || \
    fail 'set ENABLE_MULTIPOINT_AUTOSTART=I_ACCEPT_AUTOMATIC_GOAL_PUBLICATION'
fi

if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
  fail '/opt/ros/noetic/setup.bash is unavailable'
fi
if [[ ! -r "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  fail "workspace is not built: ${WORKSPACE_DIR}/devel/setup.bash is unavailable"
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/devel/setup.bash"

export DRONE_ID="${DRONE_ID:-0}"
[[ "${DRONE_ID}" == '0' ]] || fail 'the current LIO launch files only support DRONE_ID=0'

umask 077
mkdir -p "${LOG_ROOT}"

require_command roslaunch
require_command rosnode
require_command rospack
require_command rosrun
require_command rostopic
require_command curl
require_command timeout

rosnode list >/dev/null 2>&1 || fail 'ROS master is unavailable'

readonly MAVROS_STATE="$(timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
grep -q '^connected: True$' <<<"${MAVROS_STATE}" || fail 'MAVROS is not connected'
grep -q '^armed: False$' <<<"${MAVROS_STATE}" || fail 'vehicle must be disarmed before startup'

require_message /ekf/ekf_odom 5
require_message /laserMapping/cloud_registered 5
require_message "${VLA_CAMERA_INFO_TOPIC:-/camera/color/camera_info}" 5
require_message "${VLA_IMAGE_COMPRESSED_TOPIC:-/camera/color/image_raw/compressed}" 5

curl --fail --silent --show-error --max-time 3 \
  "${VLA_BACKEND_URL%/}/api/missions/current" >/dev/null || \
  fail "Windows VLA backend is unavailable: ${VLA_BACKEND_URL}"

require_node_absent /vla_diff_bridge
require_node_absent /onboard_observation_uplink
require_node_absent /px4ctrl
require_node_absent /multipointplan
require_node_absent /drone_0_traj_server

start_launch vla_preview \
  vla_diff_bridge vla_fastlio_diff_preview_stack.launch \
  start_diff_planner_preview:=false \
  start_network_bridge:=true \
  start_observation_uplink:=true \
  backend_url:="${VLA_BACKEND_URL}" \
  allowed_host_ip:="${VLA_HOST_IP:-192.168.5.2}" \
  bridge_token:="${VLA_BRIDGE_TOKEN}" \
  observation_token:="${VLA_OBSERVATION_TOKEN}" \
  calibration_id:="${VLA_CALIBRATION_ID}" \
  calibration_validated:=true \
  odom_topic:=/ekf/ekf_odom \
  cloud_topic:=/laserMapping/cloud_registered \
  camera_info_topic:="${VLA_CAMERA_INFO_TOPIC:-/camera/color/camera_info}" \
  image_compressed_topic:="${VLA_IMAGE_COMPRESSED_TOPIC:-/camera/color/image_raw/compressed}" \
  image_raw_topic:="${VLA_IMAGE_RAW_TOPIC:-/camera/color/image_raw}" \
  image_transport:="${VLA_IMAGE_TRANSPORT:-compressed}" \
  world_frame:="${VLA_WORLD_FRAME:-world}" \
  body_frame:="${VLA_BODY_FRAME:-base_link}" \
  camera_frame:="${VLA_CAMERA_FRAME:-camera_color_optical_frame}"

wait_for_node /vla_diff_bridge
wait_for_node /onboard_observation_uplink

start_launch diff_planner diff_planner run_exp_single_lio.launch
wait_for_node /drone_0_traj_server

start_launch px4ctrl px4ctrl run_ctrl_lio.launch
wait_for_node /px4ctrl

readonly MULTIPOINT_YAML_PATH="$(rospack find multipoint)/config/points.yaml"
start_process multipoint \
  rosrun multipoint multipointplan \
  __name:=multipointplan \
  odom_topic:=/ekf/ekf_odom \
  _yaml_path:="${MULTIPOINT_YAML_PATH}" \
  _next_distance:="${MULTIPOINT_NEXT_DISTANCE:-0.2}" \
  _fligt_type:="${MULTIPOINT_FLIGHT_TYPE:-1}" \
  _start_plan:="${MULTIPOINT_START_PLAN}" \
  _back_plan:="${MULTIPOINT_BACK_PLAN}" \
  _auto_planning:="${MULTIPOINT_AUTO_PLANNING}" \
  _auto_landing:="${MULTIPOINT_AUTO_LANDING}"
wait_for_node /multipointplan

readonly MAVROS_STATE_AFTER="$(timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
grep -q '^armed: False$' <<<"${MAVROS_STATE_AFTER}" || \
  fail 'vehicle armed during startup; stopping all launch groups'

log 'All launch groups are ready.'
log 'VLA is preview-only on /vla/preview_goal and /vla/preview_yaw.'
log "Multipoint start_plan=${MULTIPOINT_START_PLAN}; auto_planning=${MULTIPOINT_AUTO_PLANNING}."
log "Logs: ${LOG_ROOT}"
log 'Press Ctrl-C to stop all launch groups started by this script.'

monitor_children
