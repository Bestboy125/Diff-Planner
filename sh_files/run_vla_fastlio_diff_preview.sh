#!/usr/bin/env bash
set -euo pipefail

# Planner-preview adapter only. No MAVROS, px4ctrl, arming, mode switch,
# takeoff, landing, or actuator command is present in this script.
: "${VLA_BACKEND_URL:?set VLA_BACKEND_URL, e.g. http://192.168.14.250:8080}"
: "${VLA_BRIDGE_TOKEN:?set VLA_BRIDGE_TOKEN}"
: "${VLA_OBSERVATION_TOKEN:?set VLA_OBSERVATION_TOKEN}"
: "${VLA_CALIBRATION_ID:?set VLA_CALIBRATION_ID after validating CameraInfo and TF}"
: "${VLA_CALIBRATION_VALIDATED:?set VLA_CALIBRATION_VALIDATED after validating CameraInfo and TF}"

if [[ "${VLA_CALIBRATION_VALIDATED}" != "I_VALIDATED_CAMERA_INFO_AND_TF" ]]; then
  echo "VLA_CALIBRATION_VALIDATED must equal I_VALIDATED_CAMERA_INFO_AND_TF" >&2
  exit 2
fi

VLA_WORKSPACE_SETUP="${VLA_WORKSPACE_SETUP:-$(pwd)/devel/setup.bash}"
VLA_HOST_IP="${VLA_HOST_IP:-192.168.14.250}"
VLA_ODOM_TOPIC="${VLA_ODOM_TOPIC:-/ekf/ekf_odom}"
VLA_CLOUD_TOPIC="${VLA_CLOUD_TOPIC:-/laserMapping/cloud_registered}"
VLA_CAMERA_INFO_TOPIC="${VLA_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
VLA_IMAGE_COMPRESSED_TOPIC="${VLA_IMAGE_COMPRESSED_TOPIC:-/camera/color/image_raw/compressed}"

if [[ ! -f "${VLA_WORKSPACE_SETUP}" ]]; then
  echo "workspace setup not found: ${VLA_WORKSPACE_SETUP}" >&2
  exit 2
fi
source "${VLA_WORKSPACE_SETUP}"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do kill "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

roslaunch diff_planner run_exp_single_lio_vla_preview.launch \
  odom_topic:="${VLA_ODOM_TOPIC}" cloud_topic:="${VLA_CLOUD_TOPIC}" &
pids+=("$!")

roslaunch vla_diff_bridge vla_diff_bridge.launch \
  auth_token:="${VLA_BRIDGE_TOKEN}" \
  allowed_host_ip:="${VLA_HOST_IP}" \
  live_publish_enabled:=false \
  planning_preview_enabled:=true \
  expected_calibration_id:="${VLA_CALIBRATION_ID}" \
  odom_topic:="${VLA_ODOM_TOPIC}" &
pids+=("$!")

roslaunch vla_diff_bridge onboard_observation_uplink.launch \
  backend_url:="${VLA_BACKEND_URL}" \
  observation_token:="${VLA_OBSERVATION_TOKEN}" \
  calibration_id:="${VLA_CALIBRATION_ID}" \
  calibration_validated:=true \
  odom_topic:="${VLA_ODOM_TOPIC}" \
  camera_info_topic:="${VLA_CAMERA_INFO_TOPIC}" \
  image_compressed_topic:="${VLA_IMAGE_COMPRESSED_TOPIC}" &
pids+=("$!")

wait
