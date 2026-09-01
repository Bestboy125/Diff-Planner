#!/usr/bin/env bash
set -euo pipefail

# This script only starts the isolated planning-preview stack. It contains no
# flight-controller process, mode switch, arming, takeoff, landing or actuator command.
: "${VLA_BACKEND_URL:?set VLA_BACKEND_URL}"
: "${VLA_BRIDGE_TOKEN:?set VLA_BRIDGE_TOKEN}"
: "${VLA_OBSERVATION_TOKEN:?set VLA_OBSERVATION_TOKEN}"
: "${VLA_CALIBRATION_ID:?set VLA_CALIBRATION_ID}"
: "${VLA_CALIBRATION_VALIDATED:?confirm CameraInfo and body<-camera TF first}"

if [[ "${VLA_CALIBRATION_VALIDATED}" != "I_VALIDATED_CAMERA_INFO_AND_TF" ]]; then
  echo "VLA_CALIBRATION_VALIDATED must equal I_VALIDATED_CAMERA_INFO_AND_TF" >&2
  exit 2
fi

exec roslaunch vla_diff_bridge vla_fastlio_diff_preview_stack.launch \
  start_diff_planner_preview:=true \
  start_network_bridge:=true \
  start_observation_uplink:=true \
  backend_url:="${VLA_BACKEND_URL}" \
  allowed_host_ip:="${VLA_HOST_IP:-192.168.14.250}" \
  bridge_token:="${VLA_BRIDGE_TOKEN}" \
  observation_token:="${VLA_OBSERVATION_TOKEN}" \
  calibration_id:="${VLA_CALIBRATION_ID}" \
  calibration_validated:=true \
  odom_topic:="${VLA_ODOM_TOPIC:-/ekf/ekf_odom}" \
  cloud_topic:="${VLA_CLOUD_TOPIC:-/laserMapping/cloud_registered}" \
  camera_info_topic:="${VLA_CAMERA_INFO_TOPIC:-/camera/color/camera_info}" \
  image_compressed_topic:="${VLA_IMAGE_COMPRESSED_TOPIC:-/camera/color/image_raw/compressed}" \
  world_frame:="${VLA_WORLD_FRAME:-world}" \
  body_frame:="${VLA_BODY_FRAME:-base_link}" \
  camera_frame:="${VLA_CAMERA_FRAME:-camera_color_optical_frame}"
