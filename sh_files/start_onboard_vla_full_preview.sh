#!/usr/bin/env bash
set -Eeuo pipefail

# Historical filename retained: default is isolated preview without PX4Ctrl.
# --live requires explicit confirmation and starts the real control stack.
# No arm, takeoff, mode switch or automatic multipoint command is issued here.

MODE=preview
case "${1:-}" in
  '') ;;
  --live)
    [[ "${2:-}" == 'I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION' && "$#" == 2 ]] || {
      echo 'Live requires: --live I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION' >&2
      exit 2
    }
    MODE=live
    ;;
  --help)
    echo 'Usage: bash start_onboard_vla_full_preview.sh [--live I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION]'
    echo 'Default: USB + FAST-LIO/EKF + isolated preview. --live additionally starts PX4Ctrl.'
    exit 0
    ;;
  *) echo 'Unknown argument; use --help' >&2; exit 2 ;;
esac
readonly MODE

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly ENV_FILE="${VLA_STACK_ENV_FILE:-${HOME}/.config/vla_stack.env}"
readonly LOG_DIR="${VLA_ONECLICK_LOG_DIR:-${HOME}/.local/state/vla_preview_$(date +%Y%m%d_%H%M%S)}"

declare -a OWNED_PIDS=()
declare -a OWNED_NAMES=()

log() { printf '[%s] %s\n' "$(date '+%F %T')" "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

cleanup() {
  local index
  trap - EXIT INT TERM
  if ((${#OWNED_PIDS[@]} > 0)); then
    log 'Stopping prerequisite processes started by this wrapper...'
    for ((index=${#OWNED_PIDS[@]} - 1; index >= 0; index--)); do
      if kill -0 "${OWNED_PIDS[index]}" 2>/dev/null; then
        log "Stopping ${OWNED_NAMES[index]} (pid=${OWNED_PIDS[index]})"
        kill -TERM "${OWNED_PIDS[index]}" 2>/dev/null || true
      fi
    done
    wait "${OWNED_PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

[[ -r "${ENV_FILE}" ]] || fail "private configuration is missing: ${ENV_FILE}"
# shellcheck disable=SC1090
source "${ENV_FILE}"

# CLI mode wins; never inherit live permission from the environment file.
export VLA_BRIDGE_MODE="${MODE}"
if [[ "${MODE}" == live ]]; then
  export ENABLE_FLIGHT_STACK=I_UNDERSTAND_THIS_STARTS_PX4CTRL
  export ENABLE_VLA_LIVE_CONTROL=I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION
fi
[[ "${VLA_OBSERVATION_MODE:-calibrated}" =~ ^(calibrated|image_odom)$ ]] || fail 'invalid VLA_OBSERVATION_MODE'
[[ "${MULTIPOINT_START_PLAN:-0}" == '0' ]] || fail 'MULTIPOINT_START_PLAN must remain 0'
[[ "${MULTIPOINT_BACK_PLAN:-0}" == '0' ]] || fail 'MULTIPOINT_BACK_PLAN must remain 0'
[[ "${MULTIPOINT_AUTO_PLANNING:-0}" == '0' ]] || fail 'MULTIPOINT_AUTO_PLANNING must remain 0'
[[ "${MULTIPOINT_AUTO_LANDING:-0}" == '0' ]] || fail 'MULTIPOINT_AUTO_LANDING must remain 0'

[[ -r /opt/ros/noetic/setup.bash ]] || fail '/opt/ros/noetic/setup.bash is missing'
[[ -r "${WORKSPACE_DIR}/devel/setup.bash" ]] || fail 'catkin workspace is not built'
# shellcheck disable=SC1091
unset _CATKIN_SETUP_DIR
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/devel/setup.bash"

: "${BD_LIST:?configure the actual MID-360 BD_LIST in the private environment}"
export BD_LIST
: "${VLA_BACKEND_URL:?missing VLA_BACKEND_URL}"
: "${VLA_CALIBRATION_VALIDATED:?missing camera input confirmation}"
[[ "${VLA_CALIBRATION_VALIDATED}" == 'I_VALIDATED_CAMERA_INFO_AND_TF' ]] || fail 'camera input confirmation missing'
# Do not partially start the onboard stack if the Windows console is absent.
curl --fail --silent --show-error --max-time 3 "${VLA_BACKEND_URL%/}/api/missions/current" >/dev/null || \
  fail 'Start the Windows model + operator console first'

mkdir -p "${LOG_DIR}"
chmod 700 "${LOG_DIR}"

start_owned() {
  local name="$1"
  shift
  log "Starting ${name}; log=${LOG_DIR}/${name}.log"
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  OWNED_NAMES+=("${name}")
  OWNED_PIDS+=("$!")
}

wait_node() {
  local node="$1"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))
  until rosnode list 2>/dev/null | grep -Fxq -- "${node}"; do
    ((SECONDS < deadline)) || fail "node did not become ready: ${node}"
    sleep 0.5
  done
}

node_exists() {
  local node="$1"
  rosnode list 2>/dev/null | grep -Fxq -- "${node}"
}

wait_mavros_connected_disarmed() {
  local timeout_sec="${1:-30}"
  local deadline=$((SECONDS + timeout_sec))
  local state=''

  while ((SECONDS < deadline)); do
    state="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
    if grep -q '^armed: True$' <<<"${state}"; then
      fail 'vehicle became armed while waiting for MAVROS; refusing to continue'
    fi
    if grep -q '^connected: True$' <<<"${state}" && \
       grep -q '^armed: False$' <<<"${state}"; then
      return 0
    fi
    sleep 0.5
  done

  fail "MAVROS did not reach connected=True, armed=False within ${timeout_sec}s"
}

wait_topic() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/dev/null 2>&1 || \
    fail "topic did not produce a message: ${topic}"
}

monitor_owned() {
  local index
  while true; do
    for ((index=0; index<${#OWNED_PIDS[@]}; index++)); do
      kill -0 "${OWNED_PIDS[index]}" 2>/dev/null || fail "${OWNED_NAMES[index]} exited; see ${LOG_DIR}"
    done
    sleep 1
  done
}

if rosnode list >/dev/null 2>&1; then
  for node in /vla_diff_bridge /onboard_observation_uplink /px4ctrl /multipointplan /drone_0_traj_server; do
    if node_exists "${node}"; then
      fail "Existing stack node ${node}; stop the previous stack on the ground first"
    fi
  done
fi

if ! rosnode list >/dev/null 2>&1; then
  start_owned roscore roscore
  wait_node /rosout 15
fi

if ! node_exists /mavros; then
  start_owned mavros roslaunch mavros px4.launch
  wait_node /mavros 30
fi
wait_mavros_connected_disarmed 30

if ! node_exists /laserMapping; then
  start_owned faster_lio roslaunch faster_lio mapping_mid360.launch
fi
wait_topic /laserMapping/cloud_registered 45

if ! node_exists /ekf; then
  start_owned ekf roslaunch ekf ekf_lidar.launch
fi
wait_topic /ekf/ekf_odom 30

if ! node_exists /vla_usb_camera; then
  start_owned usb_camera roslaunch vla_diff_bridge vla_usb_camera.launch \
    video_device:="${VLA_USB_VIDEO_DEVICE:-/dev/v4l/by-id/usb-KINGSEN_KS2A418-2.0-video-index0}" \
    camera_info_url:="${VLA_USB_CAMERA_INFO_URL:-file://${HOME}/.ros/camera_info/head_camera.yaml}" \
    camera_frame:="${VLA_CAMERA_FRAME:-vla_usb_camera_optical_frame}" \
    image_width:="${VLA_USB_IMAGE_WIDTH:-640}" \
    image_height:="${VLA_USB_IMAGE_HEIGHT:-480}" \
    framerate:="${VLA_USB_FRAMERATE:-30}" \
    pixel_format:="${VLA_USB_PIXEL_FORMAT:-mjpeg}"
fi
wait_topic "${VLA_CAMERA_INFO_TOPIC:-/vla_usb_camera/camera_info}" 30
wait_topic "${VLA_IMAGE_COMPRESSED_TOPIC:-/vla_usb_camera/image_raw/compressed}" 30

if [[ "${VLA_OBSERVATION_MODE:-calibrated}" == calibrated ]]; then
  TF_CHECK="$(timeout 3 rosrun tf tf_echo "${VLA_BODY_FRAME:-base_link}" "${VLA_CAMERA_FRAME:-vla_usb_camera_optical_frame}" 2>/dev/null || true)"
  grep -q 'Translation:' <<<"${TF_CHECK}" || fail 'calibrated mode requires measured body-camera TF'
fi

wait_mavros_connected_disarmed 5
log "Prerequisites ready; vehicle disarmed, mode=${MODE}, observation=${VLA_OBSERVATION_MODE:-calibrated}."
log "Logs: ${LOG_DIR}"
cd "${WORKSPACE_DIR}"
if [[ "${MODE}" == live ]]; then
  export VLA_START_USB_CAMERA=0
  start_owned control_stack bash "${SCRIPT_DIR}/run_diff_px4ctrl_multipoint_vla_preview.sh"
else
  export VLA_START_USB_CAMERA=false
  start_owned preview_stack bash "${WORKSPACE_DIR}/src/integration/vla_diff_bridge/scripts/run_vla_fastlio_diff_preview.sh"
fi
monitor_owned
