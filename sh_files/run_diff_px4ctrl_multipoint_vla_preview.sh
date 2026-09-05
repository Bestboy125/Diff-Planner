#!/usr/bin/env bash
set -Eeuo pipefail

# One-shot launcher for the real-flight planning/control processes plus the
# safety-gated VLA bridge/uplink. This script deliberately contains no MAVROS arm,
# takeoff, land, mode-switch or actuator command.
#
# Prerequisites (started separately):
#   - ROS master and MAVROS telemetry
#   - FAST-LIO and EKF (/ekf/ekf_odom)
#   - registered point cloud (/laserMapping/cloud_registered)
#   - The KINGSEN monocular USB camera is connected. This launcher starts it by
#     default; set VLA_START_USB_CAMERA=0 only when a compatible driver is already running.
#   - Windows VLA backend
#
# VLA_BRIDGE_MODE defaults to preview. Selecting live additionally requires
# ENABLE_VLA_LIVE_CONTROL=I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION.

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

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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

require_topic_frame() {
  local topic="$1"
  local expected_frame="$2"
  local message actual_frame
  message="$(timeout 5 rostopic echo -n 1 "${topic}" 2>/dev/null || true)"
  actual_frame="$(sed -n 's/^[[:space:]]*frame_id:[[:space:]]*//p' <<<"${message}" | tr -d "\"'" | head -n 1)"
  [[ "${actual_frame}" == "${expected_frame}" ]] || \
    fail "topic ${topic} frame_id=${actual_frame:-missing}; expected ${expected_frame}"
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
: "${VLA_BACKEND_URL:?set VLA_BACKEND_URL, for example http://HOST_ONBOARD_IP:8080}"
: "${VLA_BRIDGE_TOKEN:?set VLA_BRIDGE_TOKEN}"
: "${VLA_OBSERVATION_TOKEN:?set VLA_OBSERVATION_TOKEN}"
: "${VLA_CALIBRATION_ID:?set the validated VLA_CALIBRATION_ID}"
: "${VLA_CALIBRATION_VALIDATED:?confirm CameraInfo and body<-camera TF first}"
: "${VLA_HOST_IP:?set VLA_HOST_IP to the host address on the onboard network}"

[[ "${ENABLE_FLIGHT_STACK}" == 'I_UNDERSTAND_THIS_STARTS_PX4CTRL' ]] || \
  fail 'ENABLE_FLIGHT_STACK confirmation does not match'
[[ "${VLA_CALIBRATION_VALIDATED}" == 'I_VALIDATED_CAMERA_INFO_AND_TF' ]] || \
  fail 'VLA_CALIBRATION_VALIDATED confirmation does not match'
[[ "${VLA_CALIBRATION_ID}" != 'REQUIRED' ]] || \
  fail 'VLA_CALIBRATION_ID cannot be REQUIRED'
[[ "${VLA_CALIBRATION_ID}" != comm-test-* ]] || \
  fail 'communication-test calibration IDs are forbidden for this launcher'

readonly VLA_BRIDGE_MODE="${VLA_BRIDGE_MODE:-preview}"
[[ "${VLA_BRIDGE_MODE}" =~ ^(preview|live)$ ]] || fail 'VLA_BRIDGE_MODE must be preview or live'
if [[ "${VLA_BRIDGE_MODE}" == 'live' ]]; then
  [[ "${ENABLE_VLA_LIVE_CONTROL:-}" == 'I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION' ]] || \
    fail 'live bridge requires ENABLE_VLA_LIVE_CONTROL=I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION'
fi

readonly VLA_START_USB_CAMERA="${VLA_START_USB_CAMERA:-1}"
readonly VLA_USB_VIDEO_DEVICE="${VLA_USB_VIDEO_DEVICE:-/dev/v4l/by-id/usb-KINGSEN_KS2A418-2.0-video-index0}"
readonly VLA_USB_CAMERA_INFO_URL="${VLA_USB_CAMERA_INFO_URL:-file://${HOME}/.ros/camera_info/head_camera.yaml}"
readonly VLA_CAMERA_INFO_TOPIC="${VLA_CAMERA_INFO_TOPIC:-/vla_usb_camera/camera_info}"
readonly VLA_IMAGE_COMPRESSED_TOPIC="${VLA_IMAGE_COMPRESSED_TOPIC:-/vla_usb_camera/image_raw/compressed}"
readonly VLA_IMAGE_RAW_TOPIC="${VLA_IMAGE_RAW_TOPIC:-/vla_usb_camera/image_raw}"
readonly VLA_CAMERA_FRAME="${VLA_CAMERA_FRAME:-vla_usb_camera_optical_frame}"
[[ "${VLA_START_USB_CAMERA}" =~ ^[01]$ ]] || fail 'VLA_START_USB_CAMERA must be 0 or 1'

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

if [[ "${VLA_START_USB_CAMERA}" == '1' ]]; then
  [[ -r "${VLA_USB_VIDEO_DEVICE}" ]] || fail "USB camera is unavailable: ${VLA_USB_VIDEO_DEVICE}"
  if [[ "${VLA_USB_CAMERA_INFO_URL}" == file://* ]]; then
    [[ -r "${VLA_USB_CAMERA_INFO_URL#file://}" ]] || \
      fail "USB camera calibration file is unavailable: ${VLA_USB_CAMERA_INFO_URL#file://}"
  fi
  require_node_absent /vla_usb_camera
  start_launch vla_usb_camera \
    vla_diff_bridge vla_usb_camera.launch \
    video_device:="${VLA_USB_VIDEO_DEVICE}" \
    image_width:="${VLA_USB_IMAGE_WIDTH:-640}" \
    image_height:="${VLA_USB_IMAGE_HEIGHT:-480}" \
    framerate:="${VLA_USB_FRAMERATE:-30}" \
    pixel_format:="${VLA_USB_PIXEL_FORMAT:-mjpeg}" \
    camera_frame:="${VLA_CAMERA_FRAME}" \
    camera_info_url:="${VLA_USB_CAMERA_INFO_URL}"
  wait_for_node /vla_usb_camera
fi

require_message /ekf/ekf_odom 5
require_message /laserMapping/cloud_registered 5
require_message "${VLA_CAMERA_INFO_TOPIC}" 5
require_message "${VLA_IMAGE_COMPRESSED_TOPIC}" 5
require_topic_frame "${VLA_CAMERA_INFO_TOPIC}" "${VLA_CAMERA_FRAME}"
require_topic_frame "${VLA_IMAGE_COMPRESSED_TOPIC}" "${VLA_CAMERA_FRAME}"

curl --fail --silent --show-error --max-time 3 \
  "${VLA_BACKEND_URL%/}/api/missions/current" >/dev/null || \
  fail "Windows VLA backend is unavailable: ${VLA_BACKEND_URL}"

require_node_absent /vla_diff_bridge
require_node_absent /onboard_observation_uplink
require_node_absent /semantic_raw_stereo_node
require_node_absent /semantic_orbit_executor
require_node_absent /atomic_skill_executor
require_node_absent /px4ctrl
require_node_absent /multipointplan
require_node_absent /drone_0_traj_server

if [[ "${VLA_BRIDGE_MODE}" == 'preview' ]]; then
  start_launch vla_preview \
    vla_diff_bridge vla_fastlio_diff_preview_stack.launch \
    start_diff_planner_preview:=false \
    action_chunk_sample_count:="${VLA_ACTION_CHUNK_SAMPLE_COUNT:-8}" \
    start_network_bridge:=true \
    start_observation_uplink:=true \
    backend_url:="${VLA_BACKEND_URL}" \
    allowed_host_ip:="${VLA_HOST_IP}" \
    bridge_token:="${VLA_BRIDGE_TOKEN}" \
    observation_token:="${VLA_OBSERVATION_TOKEN}" \
    calibration_id:="${VLA_CALIBRATION_ID}" \
    calibration_validated:=true \
    observation_mode:="${VLA_OBSERVATION_MODE:-calibrated}" \
    odom_topic:=/ekf/ekf_odom \
    cloud_topic:=/laserMapping/cloud_registered \
    camera_info_topic:="${VLA_CAMERA_INFO_TOPIC}" \
    image_compressed_topic:="${VLA_IMAGE_COMPRESSED_TOPIC}" \
    image_raw_topic:="${VLA_IMAGE_RAW_TOPIC}" \
    image_transport:="${VLA_IMAGE_TRANSPORT:-compressed}" \
    world_frame:="${VLA_WORLD_FRAME:-world}" \
    body_frame:="${VLA_BODY_FRAME:-base_link}" \
    camera_frame:="${VLA_CAMERA_FRAME}"
else
  start_launch vla_control_bridge \
    vla_diff_bridge vla_diff_bridge.launch \
    auth_token:="${VLA_BRIDGE_TOKEN}" \
    action_chunk_sample_count:="${VLA_ACTION_CHUNK_SAMPLE_COUNT:-8}" \
    allowed_host_ip:="${VLA_HOST_IP}" \
    live_publish_enabled:=true \
    preview_only_mode:=false \
    planning_preview_enabled:=true \
    operator_task_enabled:=true \
    expected_calibration_id:="${VLA_CALIBRATION_ID}" \
    odom_topic:=/ekf/ekf_odom \
    world_frame:="${VLA_WORLD_FRAME:-world}" \
    body_frame:="${VLA_BODY_FRAME:-base_link}" \
    camera_frame:="${VLA_CAMERA_FRAME}" \
    goal_topic:=/goal \
    yaw_topic:=/planning/yaw \
    takeoff_land_topic:=/px4ctrl/takeoff_land

  start_launch vla_observation_uplink \
    vla_diff_bridge onboard_observation_uplink.launch \
    backend_url:="${VLA_BACKEND_URL}" \
    observation_token:="${VLA_OBSERVATION_TOKEN}" \
    calibration_id:="${VLA_CALIBRATION_ID}" \
    calibration_validated:=true \
    observation_mode:="${VLA_OBSERVATION_MODE:-calibrated}" \
    odom_topic:=/ekf/ekf_odom \
    camera_info_topic:="${VLA_CAMERA_INFO_TOPIC}" \
    image_compressed_topic:="${VLA_IMAGE_COMPRESSED_TOPIC}" \
    image_raw_topic:="${VLA_IMAGE_RAW_TOPIC}" \
    image_transport:="${VLA_IMAGE_TRANSPORT:-compressed}" \
    world_frame:="${VLA_WORLD_FRAME:-world}" \
    body_frame:="${VLA_BODY_FRAME:-base_link}" \
    camera_frame:="${VLA_CAMERA_FRAME}"
fi

wait_for_node /vla_diff_bridge
wait_for_node /onboard_observation_uplink

start_launch diff_planner diff_planner run_exp_single_lio.launch traj_server_executable:=traj_server_heading_hold
wait_for_node /drone_0_traj_server

if [[ "${VLA_BRIDGE_MODE}" == 'live' ]]; then
  # These nodes only expose safety-gated ROS interfaces. Startup never sends a
  # goal, arms PX4, changes mode, or requests takeoff.
  start_launch atomic_skill_executor \
    atomic_skill_executor atomic_skill_executor.launch \
    execution_enabled:=true \
    odom_topic:=/ekf/ekf_odom \
    goal_topic:=/goal \
    yaw_topic:=/planning/yaw
  wait_for_node /atomic_skill_executor

  semantic_start_realsense=true
  if rosnode info /camera/realsense2_camera >/dev/null 2>&1; then
    semantic_start_realsense=false
    require_message /camera/infra1/image_rect_raw 5
    require_message /camera/infra2/image_rect_raw 5
    require_message /camera/infra1/camera_info 5
    require_message /camera/infra2/camera_info 5
  fi

  start_launch semantic_orbit \
    semantic_raw_stereo_localizer semantic_d435_raw_stereo_fastlio.launch \
    start_realsense:="${semantic_start_realsense}" \
    execution_enabled:=false \
    publish_planner_goal:=false \
    auto_publish_stable_goal:=false \
    semantic_orbit_execution_enabled:=true \
    target_class:=chair \
    depth_backend:="${SEMANTIC_DEPTH_BACKEND:-sgbm}" \
    odom_topic:=/ekf/ekf_odom \
    world_frame:="${VLA_WORLD_FRAME:-world}" \
    body_frame:="${VLA_BODY_FRAME:-base_link}" \
    allow_empty_odom_child_frame:=true
  wait_for_node /semantic_raw_stereo_node 60
  wait_for_node /semantic_orbit_executor 20
fi

start_launch px4ctrl vla_diff_bridge px4ctrl_vla.launch
wait_for_node /px4ctrl

# Readback only: never change parameters underneath a running controller.
python3 - <<'PY'
import rospy
if rospy.get_param('/px4ctrl/auto_takeoff_land/enable_auto_arm') is not True:
    raise SystemExit('this operator-approved launch requires auto-arm on TAKEOFF')
if rospy.get_param('/px4ctrl/auto_takeoff_land/no_RC') is not False:
    raise SystemExit('no_RC must be false')
height = float(rospy.get_param('/px4ctrl/auto_takeoff_land/takeoff_height'))
if not abs(height - 0.8) < 1e-6:
    raise SystemExit('PX4Ctrl takeoff height must be 0.8 m')
if not abs(float(rospy.get_param('/vla_diff_bridge/takeoff_height_m')) - height) < 1e-6:
    raise SystemExit('bridge/PX4Ctrl height mismatch')
PY

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
log "VLA bridge mode: ${VLA_BRIDGE_MODE}."
log "Multipoint start_plan=${MULTIPOINT_START_PLAN}; auto_planning=${MULTIPOINT_AUTO_PLANNING}."
log "Logs: ${LOG_ROOT}"
log 'Press Ctrl-C to stop all launch groups started by this script.'

monitor_children
