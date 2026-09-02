#!/usr/bin/env bash
set -Eeuo pipefail

# One-click, safety-locked real-hardware preview stack. It starts sensing,
# localization and planning processes, but publishes no arm, takeoff, mode or
# automatic multipoint command. Live mode is deliberately rejected here.

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
        kill -INT "${OWNED_PIDS[index]}" 2>/dev/null || true
      fi
    done
    wait "${OWNED_PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

[[ -r "${ENV_FILE}" ]] || fail "private configuration is missing: ${ENV_FILE}"
# shellcheck disable=SC1090
source "${ENV_FILE}"

[[ "${VLA_BRIDGE_MODE:-preview}" == 'preview' ]] || \
  fail 'this one-click launcher only permits VLA_BRIDGE_MODE=preview'
[[ "${MULTIPOINT_START_PLAN:-0}" == '0' ]] || fail 'MULTIPOINT_START_PLAN must remain 0'
[[ "${MULTIPOINT_AUTO_PLANNING:-0}" == '0' ]] || fail 'MULTIPOINT_AUTO_PLANNING must remain 0'
[[ "${MULTIPOINT_AUTO_LANDING:-0}" == '0' ]] || fail 'MULTIPOINT_AUTO_LANDING must remain 0'

[[ -r /opt/ros/noetic/setup.bash ]] || fail '/opt/ros/noetic/setup.bash is missing'
[[ -r "${WORKSPACE_DIR}/devel/setup.bash" ]] || fail 'catkin workspace is not built'
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/devel/setup.bash"

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

if ! rosnode list >/dev/null 2>&1; then
  start_owned roscore roscore
  wait_node /rosout 15
fi

if ! node_exists /mavros; then
  start_owned mavros roslaunch mavros px4.launch
  wait_node /mavros 30
fi
wait_mavros_connected_disarmed 30

if ! timeout 2 rostopic echo -n 1 /laserMapping/cloud_registered >/dev/null 2>&1; then
  start_owned faster_lio roslaunch faster_lio mapping_mid360.launch
fi
wait_topic /laserMapping/cloud_registered 45

if ! timeout 2 rostopic echo -n 1 /ekf/ekf_odom >/dev/null 2>&1; then
  start_owned ekf roslaunch ekf ekf_lidar.launch
fi
wait_topic /ekf/ekf_odom 30

if ! timeout 2 rostopic echo -n 1 /camera/color/camera_info >/dev/null 2>&1; then
  start_owned realsense roslaunch realsense2_camera rs_camera.launch
fi
wait_topic /camera/color/camera_info 30
wait_topic /camera/color/image_raw 30

TF_CHECK="$(timeout 2 rosrun tf tf_echo base_link camera_color_optical_frame 2>/dev/null || true)"
if ! grep -q 'Translation:' <<<"${TF_CHECK}"; then
  # Previously measured aircraft mount transform: base_link -> camera_link.
  start_owned body_camera_tf rosrun tf2_ros static_transform_publisher \
    0.1507507633 0.02274636381 -0.0769869916 \
    0.00289564 -0.00927950 -0.00477240 0.99994136 \
    base_link camera_link
fi
TF_CHECK="$(timeout 3 rosrun tf tf_echo base_link camera_color_optical_frame 2>/dev/null || true)"
grep -q 'Translation:' <<<"${TF_CHECK}" || fail 'base_link -> camera_color_optical_frame TF is unavailable'

log 'Prerequisites ready; vehicle is disarmed and bridge mode is preview.'
log "Logs: ${LOG_DIR}"
cd "${WORKSPACE_DIR}"
"${SCRIPT_DIR}/run_diff_px4ctrl_multipoint_vla_preview.sh"
