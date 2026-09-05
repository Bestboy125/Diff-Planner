#!/usr/bin/env python3
"""ROS node receiving validated VLA trajectory intents over TCP/NDJSON."""

import json
import math
import socketserver
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from vla_diff_bridge.protocol import (
    COMPLETE,
    EMERGENCY_STOP,
    HOLD,
    PLAN_PREVIEW,
    OPERATOR_MESSAGE_TYPE,
    TRACK,
    BridgeCommand,
    ActionChunkPlan,
    ProtocolError,
    make_ack,
    parse_command,
    build_action_chunk_plan,
    select_action_chunk_plan_target,
)


@dataclass
class ActiveChunkExecution:
    command: BridgeCommand
    plan: ActionChunkPlan
    sample_index: int


@dataclass
class ActiveOrbitExecution:
    command: BridgeCommand
    waypoints: Tuple[Tuple[float, float, float, float], ...]
    waypoint_index: int


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
        self.allowed_host_ips = set(rospy.get_param("~allowed_host_ips", ["127.0.0.1"]))
        self.live_publish_enabled = bool(rospy.get_param("~live_publish_enabled", False))
        self.preview_only_mode = bool(rospy.get_param("~preview_only_mode", True))
        self.planning_preview_enabled = bool(rospy.get_param("~planning_preview_enabled", False))
        self.operator_task_enabled = bool(rospy.get_param("~operator_task_enabled", False))
        self.world_frame = str(rospy.get_param("~world_frame", "world"))
        self.body_frame = str(rospy.get_param("~body_frame", "base_link"))
        self.camera_frame = str(rospy.get_param("~camera_frame", "vla_usb_camera_optical_frame"))
        self.expected_calibration_id = str(rospy.get_param("~expected_calibration_id", "REQUIRED"))
        self.max_source_observation_age_ms = int(
            rospy.get_param("~max_source_observation_age_ms", 1000)
        )
        self.max_ttl_ms = int(rospy.get_param("~max_ttl_ms", 2000))
        self.action_chunk_lookahead_enabled = bool(
            rospy.get_param("~action_chunk_lookahead_enabled", True)
        )
        self.max_action_chunk_steps = int(rospy.get_param("~max_action_chunk_steps", 10))
        self.action_chunk_sample_count = int(rospy.get_param("~action_chunk_sample_count", 8))
        self.action_chunk_lookahead_distance_m = float(
            rospy.get_param("~action_chunk_lookahead_distance_m", 0.10)
        )
        self.action_chunk_max_cross_track_m = float(
            rospy.get_param("~action_chunk_max_cross_track_m", 1.0)
        )
        self.max_goal_step_m = float(rospy.get_param("~max_goal_step_m", 1.0))
        self.max_operator_step_m = float(rospy.get_param("~max_operator_step_m", 2.0))
        self.max_operator_yaw_rad = float(rospy.get_param("~max_operator_yaw_rad", 1.5708))
        self.orbit_waypoint_spacing_m = float(rospy.get_param("~orbit_waypoint_spacing_m", 0.35))
        self.orbit_arrival_tolerance_m = float(rospy.get_param("~orbit_arrival_tolerance_m", 0.20))
        self.max_orbit_waypoints = int(rospy.get_param("~max_orbit_waypoints", 720))
        self.takeoff_height_m = float(rospy.get_param("~takeoff_height_m", 0.8))
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
        if self.max_action_chunk_steps < 1:
            raise RuntimeError("~max_action_chunk_steps must be >= 1")
        if not math.isfinite(self.max_operator_step_m) or self.max_operator_step_m <= 0.0:
            raise RuntimeError("~max_operator_step_m must be finite and positive")
        if self.action_chunk_sample_count not in (6, 8):
            raise RuntimeError("~action_chunk_sample_count must be 6 or 8")
        if self.action_chunk_lookahead_distance_m < 0.0:
            raise RuntimeError("~action_chunk_lookahead_distance_m must be >= 0")
        if self.action_chunk_max_cross_track_m <= 0.0:
            raise RuntimeError("~action_chunk_max_cross_track_m must be > 0")

        self.goal_pub = rospy.Publisher("~goal", PoseStamped, queue_size=1)
        self.yaw_pub = rospy.Publisher("~yaw", PositionCommand, queue_size=1)
        self.operator_yaw_hold_pub = rospy.Publisher("~operator_yaw_hold", PositionCommand, queue_size=1)
        self.preview_goal_pub = rospy.Publisher("~preview_goal", PoseStamped, queue_size=1)
        self.preview_yaw_pub = rospy.Publisher("~preview_yaw", PositionCommand, queue_size=1)
        self.hover_pub = rospy.Publisher("~hover_stop", Empty, queue_size=1)
        self.stop_pub = rospy.Publisher("~mandatory_stop", Empty, queue_size=1)
        self.takeoff_land_pub = rospy.Publisher("~takeoff_land", TakeoffLand, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)

        self._lock = threading.RLock()
        self._latest_odom: Optional[Tuple[float, float, float, float, int]] = None
        self._last_sequence: Dict[str, int] = {}
        self._active_mission_id: Optional[str] = None
        self._terminal_missions = set()
        self._last_track_monotonic: Optional[float] = None
        self._watchdog_latched = False
        self._server: Optional[ReusableThreadingTCPServer] = None
        self._active_chunk: Optional[ActiveChunkExecution] = None
        self._active_orbit: Optional[ActiveOrbitExecution] = None
        self._latest_fcu_state = None
        self.fcu_state_sub = rospy.Subscriber("~fcu_state", State, self._fcu_state_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("~odom", Odometry, self._odom_callback, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self._watchdog_callback)
        self._publish_status("ready", "bridge initialized")

    def _fcu_state_callback(self, message: State) -> None:
        with self._lock:
            self._latest_fcu_state = (message.connected, message.armed, time.monotonic())

    def _require_safe_takeoff_configuration(self) -> None:
        # These parameters describe the launch configuration, not proof of a
        # hot reload. PX4Ctrl must be restarted on the ground after deployment.
        if rospy.get_param('/px4ctrl/auto_takeoff_land/enable_auto_arm', None) is not True:
            raise ProtocolError('takeoff blocked: PX4Ctrl auto-arm setting does not match the approved workflow')
        if rospy.get_param('/px4ctrl/auto_takeoff_land/no_RC', None) is not False:
            raise ProtocolError('takeoff blocked: PX4Ctrl must require RC')
        height = rospy.get_param('/px4ctrl/auto_takeoff_land/takeoff_height', None)
        if height is None or not math.isfinite(float(height)) or abs(float(height) - self.takeoff_height_m) > 1e-6:
            raise ProtocolError('takeoff blocked: bridge/PX4Ctrl takeoff height mismatch')
        state = self._latest_fcu_state
        if state is None or time.monotonic() - state[2] > 2.5 or not state[0]:
            raise ProtocolError('takeoff blocked: fresh connected FCU state is required')
        # A disarmed vehicle is expected in the operator-approved auto-takeoff
        # workflow. PX4Ctrl retains its landed/static/RC and FCU arming checks.
        self._require_fresh_odom()

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
            command = parse_command(
                payload,
                max_ttl_ms=self.max_ttl_ms,
                max_action_chunk_steps=self.max_action_chunk_steps,
            )
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
            if command.message_type == OPERATOR_MESSAGE_TYPE:
                return self._apply_operator_task(command)
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
            if command.command == PLAN_PREVIEW:
                self._validate_preview_contract(command)
            target, selection_reason, chunk_plan, sample_index = self._resolve_goal_target(command)
            if target is not None:
                self._validate_goal(target)
            self._last_sequence[command.mission_id] = command.sequence
            self._active_mission_id = command.mission_id
            # Only a fully validated, newer command can replace the buffered route.
            self._active_chunk = None
            if command.command != PLAN_PREVIEW:
                self._active_orbit = None

            if command.command in {TRACK, PLAN_PREVIEW} and target is None:
                if command.command == TRACK and self.live_publish_enabled:
                    self.hover_pub.publish(Empty())
                    self._last_track_monotonic = None
                return "stale_chunk", selection_reason

            if command.command == PLAN_PREVIEW:
                if not self.planning_preview_enabled:
                    return (
                        "preview_locked",
                        "validated; planner-preview publishing is disabled; " + selection_reason,
                    )
                assert target is not None
                self._publish_goal(target, self.preview_goal_pub, self.preview_yaw_pub)
                if chunk_plan is not None and sample_index is not None:
                    self._active_chunk = ActiveChunkExecution(command, chunk_plan, sample_index)
                return (
                    "preview_published",
                    "goal published only to isolated Diff-Planner preview topics; "
                    + selection_reason,
                )

            if not self.live_publish_enabled:
                if command.command in {COMPLETE, EMERGENCY_STOP}:
                    self._terminal_missions.add(command.mission_id)
                    self._active_mission_id = None
                return "preview", "validated; ROS control publishing is safety-locked"

            if command.command == TRACK:
                assert target is not None
                self._publish_goal(target)
                if chunk_plan is not None and sample_index is not None:
                    self._active_chunk = ActiveChunkExecution(command, chunk_plan, sample_index)
                self._last_track_monotonic = time.monotonic()
                self._watchdog_latched = False
                return (
                    "accepted",
                    "first remaining sampled goal published; later samples advance with odometry; " + selection_reason,
                )
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

    def _apply_operator_task(self, command: BridgeCommand) -> Tuple[str, str]:
        assert command.task_id is not None
        assert command.magnitude is not None
        previous = self._last_sequence.get("operator:" + command.task_id, -1)
        if command.sequence <= previous:
            raise ProtocolError("operator task sequence is duplicate or out of order")
        if command.frame_id != self.world_frame:
            raise ProtocolError("frame_id does not match onboard world frame")
        if command.body_frame_id != self.body_frame:
            raise ProtocolError("body_frame_id does not match onboard body frame")

        movement_commands = {
            "MOVE_FORWARD",
            "MOVE_BACKWARD",
            "MOVE_LEFT",
            "MOVE_RIGHT",
            "MOVE_UP",
            "MOVE_DOWN",
        }
        if command.command in movement_commands and command.magnitude > self.max_operator_step_m:
            raise ProtocolError("operator movement exceeds max_operator_step_m")
        if command.command in {"YAW_LEFT", "YAW_RIGHT"} and command.magnitude > self.max_operator_yaw_rad:
            raise ProtocolError("operator rotation exceeds max_operator_yaw_rad")
        if command.command == "TAKEOFF" and abs(command.magnitude - self.takeoff_height_m) > 1e-6:
            raise ProtocolError("requested takeoff height does not match PX4Ctrl configuration")
        if command.command == "ORBIT_WORLD" and not 0.5 <= command.magnitude <= 5.0:
            raise ProtocolError("operator orbit radius must be within [0.5, 5.0]")

        if (command.command == "TAKEOFF" and not self.preview_only_mode
                and self.live_publish_enabled and self.operator_task_enabled):
            self._require_safe_takeoff_configuration()

        self._last_sequence["operator:" + command.task_id] = command.sequence
        if self.preview_only_mode or not self.live_publish_enabled or not self.operator_task_enabled:
            return "operator_locked", "validated; operator task publishing is safety-locked"

        if command.command == "HOLD":
            self._active_chunk = None
            self._active_orbit = None
            self._last_track_monotonic = None
            self.hover_pub.publish(Empty())
            return "accepted", "recoverable hover-stop published"
        if command.command in {"TAKEOFF", "LAND"}:
            self._active_chunk = None
            self._active_orbit = None
            self._last_track_monotonic = None
            message = TakeoffLand()
            message.takeoff_land_cmd = 1 if command.command == "TAKEOFF" else 2
            self.takeoff_land_pub.publish(message)
            if command.command == "TAKEOFF":
                return "accepted", "PX4Ctrl takeoff requested: may switch Offboard, arm and climb; execution is not confirmed"
            return "accepted", "PX4Ctrl {} request published; execution is not confirmed by this acknowledgement".format(
                command.command.lower()
            )

        odom = self._require_fresh_odom()
        x, y, z, yaw, _ = odom
        magnitude = command.magnitude
        if command.command == "ORBIT_WORLD":
            waypoints = self._build_orbit_waypoints(command, odom)
            if self.operator_yaw_hold_pub.get_num_connections() < 1:
                raise ProtocolError("operator orbit blocked: heading subscriber unavailable; updated traj_server is required")
            self._active_chunk = None
            self._last_track_monotonic = None
            self._active_orbit = ActiveOrbitExecution(command, waypoints, 0)
            self._publish_goal(waypoints[0], hold_heading=True)
            return "accepted", "operator orbit accepted; entry waypoint published"
        if command.command == "YAW_LEFT":
            target = (x, y, z, math.atan2(math.sin(yaw + magnitude), math.cos(yaw + magnitude)))
        elif command.command == "YAW_RIGHT":
            target = (x, y, z, math.atan2(math.sin(yaw - magnitude), math.cos(yaw - magnitude)))
        else:
            dx_body = 0.0
            dy_body = 0.0
            dz_body = 0.0
            if command.command == "MOVE_FORWARD":
                dx_body = magnitude
            elif command.command == "MOVE_BACKWARD":
                dx_body = -magnitude
            elif command.command == "MOVE_LEFT":
                dy_body = magnitude
            elif command.command == "MOVE_RIGHT":
                dy_body = -magnitude
            elif command.command == "MOVE_UP":
                dz_body = magnitude
            else:
                dz_body = -magnitude
            target = (
                x + math.cos(yaw) * dx_body - math.sin(yaw) * dy_body,
                y + math.sin(yaw) * dx_body + math.cos(yaw) * dy_body,
                z + dz_body,
                yaw,
            )
            if not self.min_goal_z_m <= target[2] <= self.max_goal_z_m:
                raise ProtocolError("operator target altitude is outside configured bounds")
        # Cancel only after the operator target was validated, so a rejected
        # request cannot disable the existing route's watchdog.
        hold_heading = command.command in movement_commands
        if hold_heading and self.operator_yaw_hold_pub.get_num_connections() < 1:
            raise ProtocolError("operator move blocked: heading-hold subscriber unavailable; updated traj_server is required")
        self._active_chunk = None
        self._active_orbit = None
        self._last_track_monotonic = None
        self._publish_goal(target, hold_heading=hold_heading)
        return "accepted", "operator target published to Diff-Planner"

    def _build_orbit_waypoints(self, command: BridgeCommand, odom) -> Tuple[Tuple[float, float, float, float], ...]:
        assert command.orbit_center is not None and command.orbit_laps is not None
        assert command.orbit_direction is not None and command.magnitude is not None
        cx, cy, cz = command.orbit_center
        if not self.min_goal_z_m <= cz <= self.max_goal_z_m:
            raise ProtocolError("orbit height is outside configured bounds")
        x, y, z = odom[:3]
        radial_x, radial_y = x - cx, y - cy
        radial_norm = math.hypot(radial_x, radial_y)
        if radial_norm < 1e-3:
            raise ProtocolError("vehicle is too close to orbit center to define an entry bearing")
        radius = command.magnitude
        start_angle = math.atan2(radial_y, radial_x)
        entry = (cx + radius * math.cos(start_angle), cy + radius * math.sin(start_angle), cz)
        if math.dist((x, y, z), entry) > self.max_operator_step_m:
            raise ProtocolError("orbit entry exceeds max_operator_step_m")
        max_angle_step = min(math.radians(15.0), 2.0 * math.asin(min(1.0, self.orbit_waypoint_spacing_m / (2.0 * radius))))
        total_angle = 2.0 * math.pi * command.orbit_laps
        segment_count = max(1, int(math.ceil(total_angle / max_angle_step)))
        if segment_count + 1 > self.max_orbit_waypoints:
            raise ProtocolError("orbit requires too many waypoints")
        sign = -1.0 if command.orbit_direction == "clockwise" else 1.0
        points = []
        for index in range(segment_count + 1):
            angle = start_angle + sign * total_angle * index / segment_count
            px, py = cx + radius * math.cos(angle), cy + radius * math.sin(angle)
            face_center_yaw = math.atan2(cy - py, cx - px)
            points.append((px, py, cz, face_center_yaw))
        return tuple(points)

    def _advance_active_orbit(self) -> None:
        active = self._active_orbit
        if active is None:
            return
        try:
            odom = self._require_fresh_odom()
            target = active.waypoints[active.waypoint_index]
            if math.dist(odom[:3], target[:3]) > self.orbit_arrival_tolerance_m:
                return
            next_index = active.waypoint_index + 1
            if next_index >= len(active.waypoints):
                self._active_orbit = None
                self._publish_status("orbit_complete", "all geometric orbit waypoints reached", active.command)
                return
            next_target = active.waypoints[next_index]
            if math.dist(odom[:3], next_target[:3]) > self.max_operator_step_m:
                raise ProtocolError("next orbit waypoint exceeds max_operator_step_m")
            self._publish_goal(next_target, hold_heading=True)
            active.waypoint_index = next_index
            self._publish_status("orbit_waypoint_advanced", "published next geometric orbit waypoint", active.command)
        except ProtocolError as exc:
            self._active_orbit = None
            self.hover_pub.publish(Empty())
            self._publish_status("orbit_aborted", str(exc), active.command)

    def _resolve_goal_target(
        self, command: BridgeCommand
    ) -> Tuple[Optional[Tuple[float, float, float, float]], str, Optional[ActionChunkPlan], Optional[int]]:
        if command.command not in {TRACK, PLAN_PREVIEW}:
            return command.target_mission, "not a trajectory command", None, None
        assert command.action_local_delta is not None
        assert command.target_mission is not None
        if (
            not self.action_chunk_lookahead_enabled
            or command.action_chunk is None
            or len(command.action_chunk) <= 1
        ):
            return command.target_mission, "single target or onboard look-ahead disabled", None, None
        odom = self._require_fresh_odom()
        plan = build_action_chunk_plan(
            command.action_local_delta,
            command.target_mission,
            command.action_chunk,
            self.action_chunk_sample_count,
        )
        # Reject unsafe future samples before admitting the buffer, not halfway
        # through execution. Each actual publication is checked against fresh odom too.
        previous = plan.capture_pose
        for waypoint in plan.sampled_waypoints:
            if not self.min_goal_z_m <= waypoint[2] <= self.max_goal_z_m:
                raise ProtocolError("sampled waypoint altitude is outside configured bounds")
            if math.dist(previous[:3], waypoint[:3]) > self.max_goal_step_m:
                raise ProtocolError("sampled waypoint segment exceeds max_goal_step_m")
            previous = waypoint
        result = select_action_chunk_plan_target(
            plan,
            odom[:4],
            self.action_chunk_lookahead_distance_m,
            self.action_chunk_max_cross_track_m,
        )
        reason = "{}; skipped_samples={}; progress={:.3f}m; cross_track={:.3f}m".format(
            result.reason,
            result.skipped_count,
            result.path_progress_m,
            result.cross_track_error_m,
        )
        # A pure-yaw chunk has no spatial progress to advance on; retain its
        # immediate yaw target rather than inventing spatial completion.
        if result.reason == "in_place_chunk":
            return result.target, reason, None, None
        return result.target, reason, plan, result.selected_sample_index

    def _advance_active_chunk(self) -> None:
        """Called under _lock, at most once per timer tick; never burst all goals."""
        active = self._active_chunk
        if active is None:
            return
        preview = active.command.command == PLAN_PREVIEW
        if (preview and not self.planning_preview_enabled) or (
            not preview and (not self.live_publish_enabled or self.preview_only_mode)
        ):
            self._active_chunk = None
            return
        try:
            odom = self._require_fresh_odom()
            result = select_action_chunk_plan_target(
                active.plan, odom[:4], self.action_chunk_lookahead_distance_m,
                self.action_chunk_max_cross_track_m,
            )
            if result.target is None:
                if result.cross_track_error_m > self.action_chunk_max_cross_track_m:
                    raise ProtocolError(result.reason)
                self._active_chunk = None
                self._publish_status("chunk_complete", result.reason, active.command)
                return
            assert result.selected_sample_index is not None
            if result.selected_sample_index <= active.sample_index:
                return  # Never regress/re-publish a sample because odometry jitters.
            self._validate_goal(result.target)
            if preview:
                self._publish_goal(result.target, self.preview_goal_pub, self.preview_yaw_pub)
            else:
                self._publish_goal(result.target)
            active.sample_index = result.selected_sample_index
            # Progress is NOT a cloud heartbeat: do not extend the watchdog lease.
            self._publish_status("waypoint_advanced", result.reason, active.command)
        except ProtocolError as exc:
            self._active_chunk = None
            if not preview:
                self.hover_pub.publish(Empty())
                self._last_track_monotonic = None
            self._publish_status("chunk_aborted", str(exc), active.command)

    def _validate_goal(self, target: Tuple[float, float, float, float]) -> None:
        target_x, target_y, target_z, _ = target
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
        if not all(math.isfinite(value) for value in self._latest_odom[:4]):
            raise ProtocolError("odometry contains non-finite values")
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
        hold_heading: bool = False,
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
        yaw_command.yaw_dot = 0.0
        if hold_heading:
            self.operator_yaw_hold_pub.publish(yaw_command)
        else:
            (self.yaw_pub if yaw_publisher is None else yaw_publisher).publish(yaw_command)
        (self.goal_pub if goal_publisher is None else goal_publisher).publish(goal)

    def _watchdog_callback(self, _event: Any) -> None:
        with self._lock:
            if self._active_orbit is not None:
                self._advance_active_orbit()
            if self._active_chunk is not None and self._active_chunk.command.command == PLAN_PREVIEW:
                # Isolated preview also has a finite lease; it cannot replay forever.
                command = self._active_chunk.command
                if int(time.time() * 1000) > command.sent_at_unix_ms + self.watchdog_timeout_ms:
                    self._active_chunk = None
                else:
                    self._advance_active_chunk()
            if not self.live_publish_enabled or self._last_track_monotonic is None or self._watchdog_latched:
                return
            elapsed_ms = (time.monotonic() - self._last_track_monotonic) * 1000.0
            if elapsed_ms <= self.watchdog_timeout_ms:
                self._advance_active_chunk()
                return
            self._active_chunk = None
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
            "operator_task_enabled": self.operator_task_enabled,
            "mission_id": command.mission_id if command else self._active_mission_id,
            "task_id": command.task_id if command else None,
            "sequence": command.sequence if command else None,
            "time_unix_ms": int(time.time() * 1000),
            "action_chunk_sample_count": self.action_chunk_sample_count,
            "active_chunk": {
                "sequence": self._active_chunk.command.sequence,
                "sample_index": self._active_chunk.sample_index,
                "sampled_source_indices": self._active_chunk.plan.sampled_indices,
                "sampled_waypoints": self._active_chunk.plan.sampled_waypoints,
            } if self._active_chunk else None,
            "active_orbit": {
                "waypoint_index": self._active_orbit.waypoint_index,
                "waypoint_count": len(self._active_orbit.waypoints),
            } if self._active_orbit else None,
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
