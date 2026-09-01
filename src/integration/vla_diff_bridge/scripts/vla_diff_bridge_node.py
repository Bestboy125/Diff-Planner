#!/usr/bin/env python3
"""ROS node receiving validated VLA trajectory intents over TCP/NDJSON."""

import json
import math
import socketserver
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from vla_diff_bridge.protocol import (
    COMPLETE,
    EMERGENCY_STOP,
    HOLD,
    PLAN_PREVIEW,
    TRACK,
    BridgeCommand,
    ProtocolError,
    make_ack,
    parse_command,
)


class ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class CommandHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        self.connection.settimeout(self.server.socket_timeout_sec)
        peer_ip = self.client_address[0]
        while not rospy.is_shutdown():
            line = self.rfile.readline(self.server.max_message_bytes + 1)
            if not line:
                return
            if len(line) > self.server.max_message_bytes:
                self._reply(make_ack(None, "rejected", "message exceeds maximum size"))
                return
            ack = self.server.bridge.process_wire_message(line, peer_ip)
            self._reply(ack)

    def _reply(self, payload: Dict[str, Any]) -> None:
        self.wfile.write(json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n")
        self.wfile.flush()


class VlaDiffBridge:
    def __init__(self) -> None:
        self.bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 50051))
        self.auth_token = str(rospy.get_param("~auth_token", "REQUIRED"))
        self.allowed_host_ips = set(rospy.get_param("~allowed_host_ips", ["192.168.5.2"]))
        self.live_publish_enabled = bool(rospy.get_param("~live_publish_enabled", False))
        self.preview_only_mode = bool(rospy.get_param("~preview_only_mode", True))
        self.planning_preview_enabled = bool(rospy.get_param("~planning_preview_enabled", False))
        self.world_frame = str(rospy.get_param("~world_frame", "world"))
        self.body_frame = str(rospy.get_param("~body_frame", "base_link"))
        self.camera_frame = str(rospy.get_param("~camera_frame", "camera_color_optical_frame"))
        self.expected_calibration_id = str(rospy.get_param("~expected_calibration_id", "REQUIRED"))
        self.max_source_observation_age_ms = int(
            rospy.get_param("~max_source_observation_age_ms", 1000)
        )
        self.max_ttl_ms = int(rospy.get_param("~max_ttl_ms", 2000))
        self.max_goal_step_m = float(rospy.get_param("~max_goal_step_m", 1.0))
        self.min_goal_z_m = float(rospy.get_param("~min_goal_z_m", 0.1))
        self.max_goal_z_m = float(rospy.get_param("~max_goal_z_m", 2.0))
        self.max_odom_age_ms = int(rospy.get_param("~max_odom_age_ms", 250))
        self.watchdog_timeout_ms = int(rospy.get_param("~watchdog_timeout_ms", 1000))
        self.max_message_bytes = int(rospy.get_param("~max_message_bytes", 65536))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 2.0))

        if not self.auth_token or self.auth_token == "REQUIRED":
            raise RuntimeError("~auth_token must be supplied by the launch environment")
        if self.live_publish_enabled and not self.allowed_host_ips:
            raise RuntimeError("live mode requires at least one allowed_host_ips entry")
        if self.preview_only_mode and self.live_publish_enabled:
            raise RuntimeError("preview_only_mode forbids live_publish_enabled")
        if self.planning_preview_enabled and self.expected_calibration_id == "REQUIRED":
            raise RuntimeError("~expected_calibration_id is required for planning preview")

        self.goal_pub = rospy.Publisher("~goal", PoseStamped, queue_size=1)
        self.yaw_pub = rospy.Publisher("~yaw", PositionCommand, queue_size=1)
        self.preview_goal_pub = rospy.Publisher("~preview_goal", PoseStamped, queue_size=1)
        self.preview_yaw_pub = rospy.Publisher("~preview_yaw", PositionCommand, queue_size=1)
        self.hover_pub = rospy.Publisher("~hover_stop", Empty, queue_size=1)
        self.stop_pub = rospy.Publisher("~mandatory_stop", Empty, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber("~odom", Odometry, self._odom_callback, queue_size=1)

        self._lock = threading.RLock()
        self._latest_odom: Optional[Tuple[float, float, float, float, int]] = None
        self._last_sequence: Dict[str, int] = {}
        self._active_mission_id: Optional[str] = None
        self._terminal_missions = set()
        self._last_track_monotonic: Optional[float] = None
        self._watchdog_latched = False
        self._server: Optional[ReusableThreadingTCPServer] = None

        rospy.Timer(rospy.Duration(0.1), self._watchdog_callback)
        self._publish_status("ready", "bridge initialized")

    def _odom_callback(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        stamp_ms = int(message.header.stamp.to_sec() * 1000)
        if stamp_ms <= 0:
            stamp_ms = int(time.time() * 1000)
        with self._lock:
            self._latest_odom = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
                yaw,
                stamp_ms,
            )

    def process_wire_message(self, raw: bytes, peer_ip: str) -> Dict[str, Any]:
        command = None
        try:
            if self.allowed_host_ips and peer_ip not in self.allowed_host_ips:
                raise ProtocolError("source IP {} is not allowed".format(peer_ip))
            try:
                payload = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ProtocolError("message is not valid UTF-8 JSON") from exc
            if payload.get("auth_token") != self.auth_token:
                raise ProtocolError("authentication failed")
            command = parse_command(payload, max_ttl_ms=self.max_ttl_ms)
            status, reason = self._apply(command)
            self._publish_status(status, reason, command)
            return make_ack(command, status, reason)
        except ProtocolError as exc:
            reason = str(exc)
            self._publish_status("rejected", reason, command)
            return make_ack(command, "rejected", reason)
        except Exception as exc:  # Keep malformed network input from killing the ROS node.
            rospy.logerr("VLA bridge command failed: %s", exc)
            self._publish_status("rejected", "internal bridge error", command)
            return make_ack(command, "rejected", "internal bridge error")

    def _apply(self, command: BridgeCommand) -> Tuple[str, str]:
        with self._lock:
            previous = self._last_sequence.get(command.mission_id, -1)
            if command.sequence <= previous:
                raise ProtocolError("sequence is duplicate or out of order")
            if command.mission_id in self._terminal_missions:
                raise ProtocolError("mission is already terminal")
            if self._active_mission_id not in (None, command.mission_id):
                raise ProtocolError("another mission is active")

            if command.frame_id != self.world_frame:
                raise ProtocolError("frame_id does not match onboard world frame")
            if self.preview_only_mode and command.command != PLAN_PREVIEW:
                raise ProtocolError("legacy control protocol is disabled in preview_only_mode")
            self._validate_goal(command)
            self._last_sequence[command.mission_id] = command.sequence
            self._active_mission_id = command.mission_id

            if command.command == PLAN_PREVIEW:
                self._validate_preview_contract(command)
                if not self.planning_preview_enabled:
                    return "preview_locked", "validated; planner-preview publishing is disabled"
                assert command.target_mission is not None
                self._publish_goal(
                    command.target_mission, self.preview_goal_pub, self.preview_yaw_pub
                )
                return "preview_published", "goal published only to isolated Diff-Planner preview topics"

            if not self.live_publish_enabled:
                if command.command in {COMPLETE, EMERGENCY_STOP}:
                    self._terminal_missions.add(command.mission_id)
                    self._active_mission_id = None
                return "preview", "validated; ROS control publishing is safety-locked"

            if command.command == TRACK:
                assert command.target_mission is not None
                self._publish_goal(command.target_mission)
                self._last_track_monotonic = time.monotonic()
                self._watchdog_latched = False
                return "accepted", "goal and yaw published to Diff-Planner"
            if command.command == HOLD:
                self.hover_pub.publish(Empty())
                self._last_track_monotonic = None
                return "accepted", "recoverable hover-stop published"
            if command.command == COMPLETE:
                self.hover_pub.publish(Empty())
                self._terminal_missions.add(command.mission_id)
                self._active_mission_id = None
                self._last_track_monotonic = None
                return "accepted", "mission completed; recoverable hover-stop published"

            # EMERGENCY_STOP is deliberately the only command mapped to the
            # planner's latched emergency-stop path.
            self.stop_pub.publish(Empty())
            self._terminal_missions.add(command.mission_id)
            self._active_mission_id = None
            self._last_track_monotonic = None
            return "accepted", "mandatory stop published"

    def _validate_goal(self, command: BridgeCommand) -> None:
        if command.command not in {TRACK, PLAN_PREVIEW}:
            return
        assert command.target_mission is not None
        target_x, target_y, target_z, _ = command.target_mission
        odom = self._require_fresh_odom()
        step = math.sqrt(
            (target_x - odom[0]) ** 2 + (target_y - odom[1]) ** 2 + (target_z - odom[2]) ** 2
        )
        if step > self.max_goal_step_m:
            raise ProtocolError("target exceeds max_goal_step_m")
        if not self.min_goal_z_m <= target_z <= self.max_goal_z_m:
            raise ProtocolError("target altitude is outside configured bounds")

    def _require_fresh_odom(self) -> Tuple[float, float, float, float, int]:
        if self._latest_odom is None:
            raise ProtocolError("odometry is not available")
        age_ms = int(time.time() * 1000) - self._latest_odom[4]
        if age_ms < -1000 or age_ms > self.max_odom_age_ms:
            raise ProtocolError("odometry is stale")
        return self._latest_odom

    def _validate_preview_contract(self, command: BridgeCommand) -> None:
        if command.calibration_id != self.expected_calibration_id:
            raise ProtocolError("calibration_id does not match onboard configuration")
        if command.body_frame_id != self.body_frame:
            raise ProtocolError("body_frame_id does not match onboard configuration")
        if command.camera_frame_id != self.camera_frame:
            raise ProtocolError("camera_frame_id does not match onboard configuration")
        assert command.source_capture_unix_ms is not None
        source_age_ms = int(time.time() * 1000) - command.source_capture_unix_ms
        if source_age_ms < -1000 or source_age_ms > self.max_source_observation_age_ms:
            raise ProtocolError("source observation is stale or clock-unsynchronized")

    def _publish_goal(
        self,
        target: Tuple[float, float, float, float],
        goal_publisher: Any = None,
        yaw_publisher: Any = None,
    ) -> None:
        x, y, z, yaw = target
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.world_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        yaw_command = PositionCommand()
        yaw_command.header = goal.header
        yaw_command.yaw = yaw
        (self.yaw_pub if yaw_publisher is None else yaw_publisher).publish(yaw_command)
        (self.goal_pub if goal_publisher is None else goal_publisher).publish(goal)

    def _watchdog_callback(self, _event: Any) -> None:
        with self._lock:
            if not self.live_publish_enabled or self._last_track_monotonic is None or self._watchdog_latched:
                return
            elapsed_ms = (time.monotonic() - self._last_track_monotonic) * 1000.0
            if elapsed_ms <= self.watchdog_timeout_ms:
                return
            self.hover_pub.publish(Empty())
            self._watchdog_latched = True
            self._last_track_monotonic = None
            self._publish_status(
                "watchdog_hold", "command stream timed out; recoverable hover-stop published"
            )

    def _publish_status(
        self, status: str, detail: str, command: Optional[BridgeCommand] = None
    ) -> None:
        payload = {
            "status": status,
            "detail": detail,
            "live_publish_enabled": self.live_publish_enabled,
            "preview_only_mode": self.preview_only_mode,
            "planning_preview_enabled": self.planning_preview_enabled,
            "mission_id": command.mission_id if command else self._active_mission_id,
            "sequence": command.sequence if command else None,
            "time_unix_ms": int(time.time() * 1000),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        log = rospy.logwarn if status in {"rejected", "watchdog_hold", "watchdog_stop"} else rospy.loginfo
        log("[vla_diff_bridge] %s: %s", status, detail)

    def serve(self) -> None:
        server = ReusableThreadingTCPServer((self.bind_host, self.port), CommandHandler)
        server.bridge = self
        server.max_message_bytes = self.max_message_bytes
        server.socket_timeout_sec = self.socket_timeout_sec
        self._server = server
        thread = threading.Thread(target=server.serve_forever, name="vla-bridge-tcp", daemon=True)
        thread.start()
        rospy.loginfo(
            "VLA bridge listening on %s:%d (live=%s, planning_preview=%s)",
            self.bind_host,
            self.port,
            self.live_publish_enabled,
            self.planning_preview_enabled,
        )
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None


def main() -> None:
    rospy.init_node("vla_diff_bridge")
    try:
        VlaDiffBridge().serve()
    except Exception as exc:
        rospy.logfatal("Unable to start VLA bridge: %s", exc)
        raise


if __name__ == "__main__":
    main()
