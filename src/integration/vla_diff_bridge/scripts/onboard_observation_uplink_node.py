#!/usr/bin/env python3
"""Upload camera + FAST-LIO state without ever publishing a motion command."""

import base64
import json
import queue
import threading
import time
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
import tf2_ros


class ObservationUplink:
    def __init__(self) -> None:
        self.backend_url = str(rospy.get_param("~backend_url", "http://127.0.0.1:8080")).rstrip("/")
        self.token = str(rospy.get_param("~observation_token", "REQUIRED"))
        self.vehicle_id = str(rospy.get_param("~vehicle_id", "uav0"))
        self.world_frame = str(rospy.get_param("~world_frame", "world"))
        self.body_frame = str(rospy.get_param("~body_frame", "base_link"))
        self.camera_frame = str(rospy.get_param("~camera_frame", "vla_usb_camera_optical_frame"))
        self.allow_empty_odom_child_frame = bool(
            rospy.get_param("~allow_empty_odom_child_frame", False)
        )
        self.calibration_id = str(rospy.get_param("~calibration_id", "REQUIRED"))
        self.calibration_validated = bool(rospy.get_param("~calibration_validated", False))
        self.observation_mode = str(rospy.get_param("~observation_mode", "calibrated"))
        if self.observation_mode not in {"calibrated", "image_odom"}:
            raise RuntimeError("~observation_mode must be calibrated or image_odom")
        self.image_transport = str(rospy.get_param("~image_transport", "compressed"))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))
        self.http_timeout_sec = float(rospy.get_param("~http_timeout_sec", 1.0))
        self.max_pair_age_ms = int(rospy.get_param("~max_pair_age_ms", 80))

        if self.token == "REQUIRED":
            raise RuntimeError("~observation_token must be configured")
        if self.image_transport not in {"compressed", "raw"}:
            raise RuntimeError("~image_transport must be compressed or raw")

        self._lock = threading.RLock()
        self._latest_odom: Optional[Odometry] = None
        self._camera_info: Optional[CameraInfo] = None
        self._planner_preview: Optional[PositionCommand] = None
        self._sequence = 0
        self._queue: queue.Queue = queue.Queue(maxsize=2)
        self._shutdown = threading.Event()
        self._tf_buffer = None
        self._tf_listener = None
        if self.observation_mode == "calibrated":
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)

        rospy.Subscriber("~odom", Odometry, self._odom_callback, queue_size=10)
        rospy.Subscriber("~camera_info", CameraInfo, self._camera_info_callback, queue_size=1)
        rospy.Subscriber("~planner_preview", PositionCommand, self._planner_preview_callback, queue_size=1)
        if self.image_transport == "compressed":
            rospy.Subscriber("~image_compressed", CompressedImage, self._compressed_callback, queue_size=1)
        else:
            rospy.Subscriber("~image_raw", Image, self._raw_callback, queue_size=1)

        self._worker = threading.Thread(target=self._upload_worker, name="observation-uplink", daemon=True)
        self._worker.start()
        rospy.on_shutdown(self.shutdown)
        self._publish_status("ready", "observation uplink initialized; no control publishers exist")

    def _odom_callback(self, message: Odometry) -> None:
        with self._lock:
            self._latest_odom = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera_info = message

    def _planner_preview_callback(self, message: PositionCommand) -> None:
        with self._lock:
            self._planner_preview = message

    def _compressed_callback(self, message: CompressedImage) -> None:
        jpeg = bytes(message.data)
        if not jpeg.startswith(b"\xff\xd8"):
            self._publish_status("drop", "compressed camera payload is not JPEG")
            return
        self._enqueue(("jpeg", message.header.stamp, message.header.frame_id, jpeg))

    def _raw_callback(self, message: Image) -> None:
        self._enqueue(("raw", message.header.stamp, message.header.frame_id, message))

    def _enqueue(self, frame: Any) -> None:
        """Camera callbacks only enqueue; TF, encoding and HTTP stay off callback threads."""
        with self._lock:
            item = (frame, self._latest_odom, self._camera_info, self._planner_preview)
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    return

    def _build_payload(
        self,
        stamp: rospy.Time,
        image_frame: str,
        jpeg: bytes,
        odom: Optional[Odometry],
        camera: Optional[CameraInfo],
        planner_preview: Optional[PositionCommand],
    ) -> Dict[str, Any]:
        with self._lock:
            sequence = self._sequence
            self._sequence += 1
        if odom is None:
            raise RuntimeError("FAST-LIO odometry unavailable")
        if camera is None:
            raise RuntimeError("CameraInfo unavailable")
        capture_ms = self._stamp_ms(stamp)
        odom_ms = self._stamp_ms(odom.header.stamp)
        if abs(capture_ms - odom_ms) > self.max_pair_age_ms:
            raise RuntimeError("latest FAST-LIO odometry is not synchronized with image")
        if odom.header.frame_id != self.world_frame:
            raise RuntimeError("FAST-LIO odometry world frame contract mismatch")
        source_child_frame = odom.child_frame_id.strip()
        if source_child_frame and source_child_frame != self.body_frame:
            raise RuntimeError("FAST-LIO odometry child frame contract mismatch")
        if not source_child_frame and not self.allow_empty_odom_child_frame:
            raise RuntimeError("FAST-LIO odometry child frame is empty")
        if image_frame != self.camera_frame or camera.header.frame_id != self.camera_frame:
            raise RuntimeError("camera optical frame contract mismatch")
        body_from_camera = None
        if self.observation_mode == "calibrated":
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.body_frame, self.camera_frame, stamp, rospy.Duration(0.03)
                )
            except Exception as exc:
                raise RuntimeError("body<-camera TF unavailable: {}".format(exc))
            body_from_camera = {
                "parent_frame_id": self.body_frame,
                "child_frame_id": self.camera_frame,
                "translation": self._xyz(transform.transform.translation),
                "rotation": self._xyzw(transform.transform.rotation),
            }
        pose = odom.pose.pose
        twist = odom.twist.twist
        payload: Dict[str, Any] = {
            "schema_version": 1,
            "type": "onboard_observation",
            "vehicle_id": self.vehicle_id,
            "sequence": sequence,
            "capture_unix_ms": capture_ms,
            "image_encoding": "jpeg",
            "image_base64": base64.b64encode(jpeg).decode("ascii"),
            "image_frame_id": image_frame,
            "odometry": {
                "stamp_unix_ms": odom_ms,
                "frame_id": odom.header.frame_id,
                # ekf_lidar currently publishes an empty child_frame_id. The
                # normalized contract remains explicit and calibration-gated.
                "child_frame_id": self.body_frame,
                "pose": {
                    "position": self._xyz(pose.position),
                    "orientation": self._xyzw(pose.orientation),
                },
                "linear_velocity": self._xyz(twist.linear),
                "angular_velocity": self._xyz(twist.angular),
            },
            "camera_intrinsics": {
                "width": camera.width,
                "height": camera.height,
                "distortion_model": camera.distortion_model or "plumb_bob",
                "k": list(camera.K),
                "d": list(camera.D),
            },
            "observation_mode": self.observation_mode,
            "body_from_camera": body_from_camera,
            "calibration_id": self.calibration_id,
            "calibration_validated": self.calibration_validated and self.observation_mode == "calibrated",
        }
        if planner_preview is not None:
            payload["planner_preview"] = {
                "stamp_unix_ms": self._stamp_ms(planner_preview.header.stamp),
                "frame_id": planner_preview.header.frame_id or self.world_frame,
                "position": self._xyz(planner_preview.position),
                "velocity": self._xyz(planner_preview.velocity),
                "acceleration": self._xyz(planner_preview.acceleration),
                "yaw": float(planner_preview.yaw),
            }
        return payload

    def _upload_worker(self) -> None:
        endpoint = self.backend_url + "/api/onboard/observations"
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            try:
                frame, odom, camera, planner_preview = self._queue.get(timeout=0.2)
                image_kind, stamp, frame_id, image_data = frame
            except queue.Empty:
                continue
            try:
                jpeg = (
                    image_data
                    if image_kind == "jpeg"
                    else self._encode_raw_image(image_data)
                )
                payload = self._build_payload(
                    stamp,
                    frame_id or self.camera_frame,
                    jpeg,
                    odom,
                    camera,
                    planner_preview,
                )
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                request = Request(
                    endpoint,
                    data=body,
                    headers={
                        "Content-Type": "application/json",
                        "X-Observation-Token": self.token,
                    },
                    method="POST",
                )
                with urlopen(request, timeout=self.http_timeout_sec) as response:
                    response.read()
                self._publish_status("online", "uploaded observation {}".format(payload["sequence"]))
            except HTTPError as exc:
                try:
                    response_detail = exc.read().decode("utf-8", errors="replace")
                except Exception:
                    response_detail = ""
                detail = "upload failed: {}".format(exc)
                if response_detail:
                    detail = "{}; response={}".format(detail, response_detail[:1000])
                self._publish_status("degraded", detail)
            except (URLError, TimeoutError) as exc:
                self._publish_status("degraded", "upload failed: {}".format(exc))
            except Exception as exc:
                self._publish_status("drop", "observation assembly failed: {}".format(exc))
            finally:
                self._queue.task_done()

    def _encode_raw_image(self, message: Image) -> bytes:
        import cv2
        from cv_bridge import CvBridge

        bgr = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
        ok, encoded = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("cv2.imencode returned false")
        return encoded.tobytes()

    @staticmethod
    def _stamp_ms(stamp: rospy.Time) -> int:
        seconds = stamp.to_sec()
        return int((seconds if seconds > 0 else time.time()) * 1000)

    @staticmethod
    def _xyz(value: Any) -> Dict[str, float]:
        return {"x": float(value.x), "y": float(value.y), "z": float(value.z)}

    @staticmethod
    def _xyzw(value: Any) -> Dict[str, float]:
        return {
            "x": float(value.x),
            "y": float(value.y),
            "z": float(value.z),
            "w": float(value.w),
        }

    def _publish_status(self, status: str, detail: str) -> None:
        self._status_pub.publish(
            String(data=json.dumps({"status": status, "detail": detail, "observation_mode": self.observation_mode, "time_unix_ms": int(time.time() * 1000)}))
        )

    def shutdown(self) -> None:
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)


def main() -> None:
    rospy.init_node("onboard_observation_uplink")
    ObservationUplink()
    rospy.spin()


if __name__ == "__main__":
    main()
