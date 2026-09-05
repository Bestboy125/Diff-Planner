#!/usr/bin/env bash
set -euo pipefail

# This script only starts the isolated planning-preview stack. It contains no
# flight-controller process, mode switch, arming, takeoff, landing or actuator command.
: "${VLA_BACKEND_URL:?set VLA_BACKEND_URL}"
: "${VLA_BRIDGE_TOKEN:?set VLA_BRIDGE_TOKEN}"
: "${VLA_OBSERVATION_TOKEN:?set VLA_OBSERVATION_TOKEN}"
: "${VLA_CALIBRATION_ID:?set VLA_CALIBRATION_ID}"
: "${VLA_CALIBRATION_VALIDATED:?confirm CameraInfo and body<-camera TF first}"
: "${VLA_HOST_IP:?set VLA_HOST_IP to the host address on the onboard network}"

if [[ "${VLA_CALIBRATION_VALIDATED}" != "I_VALIDATED_CAMERA_INFO_AND_TF" ]]; then
  echo "VLA_CALIBRATION_VALIDATED must equal I_VALIDATED_CAMERA_INFO_AND_TF" >&2
  exit 2
fi

exec roslaunch vla_diff_bridge vla_fastlio_diff_preview_stack.launch \
  start_usb_camera:="${VLA_START_USB_CAMERA:-true}" \
  action_chunk_sample_count:="${VLA_ACTION_CHUNK_SAMPLE_COUNT:-8}" \
  start_diff_planner_preview:=true \
  start_network_bridge:=true \
  start_observation_uplink:=true \
  backend_url:="${VLA_BACKEND_URL}" \
  allowed_host_ip:="${VLA_HOST_IP}" \
  bridge_token:="${VLA_BRIDGE_TOKEN}" \
  observation_token:="${VLA_OBSERVATION_TOKEN}" \
  calibration_id:="${VLA_CALIBRATION_ID}" \
  calibration_validated:=true \
  observation_mode:="${VLA_OBSERVATION_MODE:-calibrated}" \
  odom_topic:="${VLA_ODOM_TOPIC:-/ekf/ekf_odom}" \
  cloud_topic:="${VLA_CLOUD_TOPIC:-/laserMapping/cloud_registered}" \
  camera_info_topic:="${VLA_CAMERA_INFO_TOPIC:-/vla_usb_camera/camera_info}" \
  image_compressed_topic:="${VLA_IMAGE_COMPRESSED_TOPIC:-/vla_usb_camera/image_raw/compressed}" \
  image_raw_topic:="${VLA_IMAGE_RAW_TOPIC:-/vla_usb_camera/image_raw}" \
  image_transport:="${VLA_IMAGE_TRANSPORT:-compressed}" \
  usb_video_device:="${VLA_USB_VIDEO_DEVICE:-/dev/v4l/by-id/usb-KINGSEN_KS2A418-2.0-video-index0}" \
  usb_image_width:="${VLA_USB_IMAGE_WIDTH:-640}" \
  usb_image_height:="${VLA_USB_IMAGE_HEIGHT:-480}" \
  usb_framerate:="${VLA_USB_FRAMERATE:-30}" \
  usb_pixel_format:="${VLA_USB_PIXEL_FORMAT:-mjpeg}" \
  usb_camera_info_url:="${VLA_USB_CAMERA_INFO_URL:-file://${HOME}/.ros/camera_info/head_camera.yaml}" \
  world_frame:="${VLA_WORLD_FRAME:-world}" \
  body_frame:="${VLA_BODY_FRAME:-base_link}" \
  camera_frame:="${VLA_CAMERA_FRAME:-vla_usb_camera_optical_frame}"
