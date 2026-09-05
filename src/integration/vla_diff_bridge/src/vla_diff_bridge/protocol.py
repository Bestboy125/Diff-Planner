"""Pure-Python validation for the host-to-onboard trajectory protocol."""

from dataclasses import dataclass
import math
import re
import time
from typing import Any, Dict, Optional, Sequence, Tuple


SCHEMA_VERSION = 1
PREVIEW_SCHEMA_VERSION = 2
OPERATOR_SCHEMA_VERSION = 3
MESSAGE_TYPE = "trajectory_command"
PREVIEW_MESSAGE_TYPE = "planning_preview"
OPERATOR_MESSAGE_TYPE = "operator_task"
TRACK = "TRACK"
HOLD = "HOLD"
COMPLETE = "COMPLETE"
EMERGENCY_STOP = "EMERGENCY_STOP"
PLAN_PREVIEW = "PLAN_PREVIEW"
SUPPORTED_COMMANDS = {TRACK, HOLD, COMPLETE, EMERGENCY_STOP, PLAN_PREVIEW}
SUPPORTED_POLICIES = {"openvla", "pi05"}
OPERATOR_COMMANDS = {
    "TAKEOFF",
    "LAND",
    "HOLD",
    "MOVE_FORWARD",
    "MOVE_BACKWARD",
    "MOVE_LEFT",
    "MOVE_RIGHT",
    "MOVE_UP",
    "MOVE_DOWN",
    "YAW_LEFT",
    "YAW_RIGHT",
    "ORBIT_WORLD",
    "SEMANTIC_ORBIT",
}
ACTION_SEMANTIC = ("dx_body", "dy_body", "dz_body", "d_yaw")
ACTION_UNITS = ("m", "m", "m", "rad")


class ProtocolError(ValueError):
    """Raised when an inbound bridge message is unsafe or malformed."""


@dataclass(frozen=True)
class BridgeCommand:
    mission_id: str
    sequence: int
    sent_at_unix_ms: int
    ttl_ms: int
    policy: str
    command: str
    frame_id: str
    action_local_delta: Optional[Tuple[float, float, float, float]]
    target_mission: Optional[Tuple[float, float, float, float]]
    action_chunk: Optional[Tuple[Tuple[float, float, float, float], ...]] = None
    calibration_id: Optional[str] = None
    body_frame_id: Optional[str] = None
    camera_frame_id: Optional[str] = None
    source_vehicle_id: Optional[str] = None
    source_observation_sequence: Optional[int] = None
    source_capture_unix_ms: Optional[int] = None
    message_type: str = MESSAGE_TYPE
    task_id: Optional[str] = None
    magnitude: Optional[float] = None
    magnitude_unit: Optional[str] = None
    orbit_center: Optional[Tuple[float, float, float]] = None
    orbit_laps: Optional[float] = None
    orbit_direction: Optional[str] = None
    orbit_yaw_mode: Optional[str] = None
    semantic_target_label: Optional[str] = None
    semantic_keep_current_altitude: Optional[bool] = None


def _require_int(payload: Dict[str, Any], key: str, minimum: int = 0) -> int:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ProtocolError("{} must be an integer >= {}".format(key, minimum))
    return value


def _vector4(value: Any, key: str) -> Tuple[float, float, float, float]:
    # Both model gateways currently return [horizon, 4], so the bridge consumes
    # the first row while deliberately rejecting ambiguous shapes.
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 1:
        raise ProtocolError("{} must have shape [1, 4]".format(key))
    row = value[0]
    if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != 4:
        raise ProtocolError("{} must have shape [1, 4]".format(key))
    try:
        result = tuple(float(item) for item in row)
    except (TypeError, ValueError) as exc:
        raise ProtocolError("{} must contain numbers".format(key)) from exc
    if not all(math.isfinite(item) for item in result):
        raise ProtocolError("{} must contain only finite numbers".format(key))
    return result  # type: ignore[return-value]


def _action_chunk(
    value: Any, key: str, max_steps: int
) -> Tuple[Tuple[float, float, float, float], ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or not 1 <= len(value) <= max_steps
    ):
        raise ProtocolError("{} must have shape [1..{}, 4]".format(key, max_steps))
    rows = []
    for row in value:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != 4:
            raise ProtocolError("{} must have shape [1..{}, 4]".format(key, max_steps))
        try:
            normalized = tuple(float(item) for item in row)
        except (TypeError, ValueError) as exc:
            raise ProtocolError("{} must contain numbers".format(key)) from exc
        if not all(math.isfinite(item) for item in normalized):
            raise ProtocolError("{} must contain only finite numbers".format(key))
        rows.append(normalized)
    return tuple(rows)  # type: ignore[return-value]


@dataclass(frozen=True)
class LookAheadResult:
    target: Optional[Tuple[float, float, float, float]]
    selected_index: Optional[int]
    skipped_count: int
    reason: str
    path_progress_m: float
    cross_track_error_m: float
    selected_sample_index: Optional[int] = None
    sampled_count: int = 0


@dataclass(frozen=True)
class ActionChunkPlan:
    """A fixed world-frame action chunk plus selected Diff-Planner waypoints."""

    capture_pose: Tuple[float, float, float, float]
    waypoints: Tuple[Tuple[float, float, float, float], ...]
    sampled_indices: Tuple[int, ...]

    @property
    def sampled_waypoints(self) -> Tuple[Tuple[float, float, float, float], ...]:
        return tuple(self.waypoints[index] for index in self.sampled_indices)


def _wrap_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _capture_pose_from_first_target(
    first_action: Tuple[float, float, float, float],
    first_target: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    dx_body, dy_body, dz_body, d_yaw = first_action
    capture_yaw = _wrap_yaw(first_target[3] - d_yaw)
    dx_world = math.cos(capture_yaw) * dx_body - math.sin(capture_yaw) * dy_body
    dy_world = math.sin(capture_yaw) * dx_body + math.cos(capture_yaw) * dy_body
    return (
        first_target[0] - dx_world,
        first_target[1] - dy_world,
        first_target[2] - dz_body,
        capture_yaw,
    )


def integrate_action_chunk(
    first_action: Tuple[float, float, float, float],
    first_target: Tuple[float, float, float, float],
    action_chunk: Tuple[Tuple[float, float, float, float], ...],
) -> Tuple[
    Tuple[float, float, float, float],
    Tuple[Tuple[float, float, float, float], ...],
]:
    """Recover the capture pose and integrate body deltas in a fixed world frame."""
    pose = _capture_pose_from_first_target(first_action, first_target)
    capture_pose = pose
    targets = []
    for dx_body, dy_body, dz_body, d_yaw in action_chunk:
        x, y, z, yaw = pose
        target = (
            x + math.cos(yaw) * dx_body - math.sin(yaw) * dy_body,
            y + math.sin(yaw) * dx_body + math.cos(yaw) * dy_body,
            z + dz_body,
            _wrap_yaw(yaw + d_yaw),
        )
        targets.append(target)
        pose = target
    return capture_pose, tuple(targets)


def _path_geometry(
    capture_pose: Tuple[float, float, float, float],
    waypoints: Tuple[Tuple[float, float, float, float], ...],
) -> Tuple[
    Tuple[Tuple[float, float, float], ...],
    Tuple[float, ...],
    Tuple[Tuple[Tuple[float, float, float], float], ...],
]:
    points = (capture_pose[:3],) + tuple(target[:3] for target in waypoints)
    cumulative = [0.0]
    segments = []
    for start, end in zip(points, points[1:]):
        vector = tuple(end[index] - start[index] for index in range(3))
        length = math.sqrt(sum(component * component for component in vector))
        cumulative.append(cumulative[-1] + length)
        segments.append((vector, length))
    return points, tuple(cumulative), tuple(segments)


def _sample_waypoint_indices(
    capture_pose: Tuple[float, float, float, float],
    waypoints: Tuple[Tuple[float, float, float, float], ...],
    sample_count: int,
) -> Tuple[int, ...]:
    """Choose exactly sample_count original waypoints, preserving endpoints/order."""
    if sample_count < 1:
        raise ProtocolError("action_chunk_sample_count must be >= 1")
    waypoint_count = len(waypoints)
    if waypoint_count <= sample_count:
        return tuple(range(waypoint_count))
    _, cumulative, _ = _path_geometry(capture_pose, waypoints)
    total_distance = cumulative[-1]
    selected = []
    previous = -1
    for sample_index in range(sample_count):
        if sample_index == 0:
            selected_index = 0
        elif sample_index == sample_count - 1:
            selected_index = waypoint_count - 1
        else:
            minimum = previous + 1
            maximum = waypoint_count - sample_count + sample_index
            if total_distance <= 1e-9:
                desired_index = round(
                    sample_index * (waypoint_count - 1) / (sample_count - 1)
                )
                selected_index = max(minimum, min(maximum, desired_index))
            else:
                desired_progress = total_distance * sample_index / (sample_count - 1)
                selected_index = min(
                    range(minimum, maximum + 1),
                    key=lambda index: abs(cumulative[index + 1] - desired_progress),
                )
        selected.append(selected_index)
        previous = selected_index
    return tuple(selected)


def build_action_chunk_plan(
    first_action: Tuple[float, float, float, float],
    first_target: Tuple[float, float, float, float],
    action_chunk: Tuple[Tuple[float, float, float, float], ...],
    sample_count: int,
) -> ActionChunkPlan:
    """Integrate all model steps, then retain 6/8 original waypoints for execution."""
    if not action_chunk:
        raise ProtocolError("action chunk must not be empty")
    capture_pose, waypoints = integrate_action_chunk(first_action, first_target, action_chunk)
    return ActionChunkPlan(
        capture_pose=capture_pose,
        waypoints=waypoints,
        sampled_indices=_sample_waypoint_indices(capture_pose, waypoints, sample_count),
    )


def select_action_chunk_plan_target(
    plan: ActionChunkPlan,
    current_pose: Tuple[float, float, float, float],
    lookahead_distance_m: float,
    max_cross_track_m: float,
) -> LookAheadResult:
    """Project on the full path, prune stale samples, and select the next sample."""
    current_position = current_pose[:3]
    points, cumulative, segments = _path_geometry(plan.capture_pose, plan.waypoints)

    if cumulative[-1] <= 1e-9:
        error = math.dist(current_position, plan.capture_pose[:3])
        if error > max_cross_track_m:
            return LookAheadResult(None, None, 0,
                                   "current pose is outside the action-chunk corridor",
                                   0.0, error, None, len(plan.sampled_indices))
        first_sample = plan.sampled_indices[0]
        return LookAheadResult(
            target=(current_pose[0], current_pose[1], current_pose[2], plan.waypoints[first_sample][3]),
            selected_index=first_sample,
            skipped_count=0,
            reason="in_place_chunk",
            path_progress_m=0.0,
            cross_track_error_m=math.dist(current_position, plan.capture_pose[:3]),
            selected_sample_index=0,
            sampled_count=len(plan.sampled_indices),
        )

    best_error = float("inf")
    best_progress = 0.0
    for index, ((vector, length), start) in enumerate(zip(segments, points)):
        if length <= 1e-9:
            continue
        offset = tuple(current_position[axis] - start[axis] for axis in range(3))
        dot = sum(offset[axis] * vector[axis] for axis in range(3))
        ratio = max(0.0, min(1.0, dot / (length * length)))
        projected = tuple(start[axis] + ratio * vector[axis] for axis in range(3))
        error = math.dist(current_position, projected)
        progress = cumulative[index] + ratio * length
        if error < best_error - 1e-9 or (
            abs(error - best_error) <= 1e-9 and progress > best_progress
        ):
            best_error = error
            best_progress = progress

    if best_error > max_cross_track_m:
        return LookAheadResult(
            target=None,
            selected_index=None,
            skipped_count=0,
            reason="current pose is outside the action-chunk corridor",
            path_progress_m=best_progress,
            cross_track_error_m=best_error,
            selected_sample_index=None,
            sampled_count=len(plan.sampled_indices),
        )

    if best_progress >= cumulative[-1] - 1e-9:
        return LookAheadResult(
            None, None, len(plan.sampled_indices),
            "all sampled action-chunk waypoints were already traversed",
            best_progress, best_error, None, len(plan.sampled_indices),
        )
    desired_progress = best_progress + lookahead_distance_m
    selected_sample_index = next(
        (
            sample_index
            for sample_index, waypoint_index in enumerate(plan.sampled_indices)
            if cumulative[waypoint_index + 1] + 1e-9 >= desired_progress
        ),
        None,
    )
    if selected_sample_index is None:
        if best_progress >= cumulative[-1] - 1e-9:
            return LookAheadResult(
                target=None,
                selected_index=None,
                skipped_count=len(plan.sampled_indices),
                reason="all sampled action-chunk waypoints were already traversed",
                path_progress_m=best_progress,
                cross_track_error_m=best_error,
                selected_sample_index=None,
                sampled_count=len(plan.sampled_indices),
            )
        selected_sample_index = len(plan.sampled_indices) - 1

    selected_index = plan.sampled_indices[selected_sample_index]
    return LookAheadResult(
        target=plan.waypoints[selected_index],
        selected_index=selected_index,
        skipped_count=selected_sample_index,
        reason="selected sampled waypoint {}/{} from source step {}/{}".format(
            selected_sample_index + 1,
            len(plan.sampled_indices),
            selected_index + 1,
            len(plan.waypoints),
        ),
        path_progress_m=best_progress,
        cross_track_error_m=best_error,
        selected_sample_index=selected_sample_index,
        sampled_count=len(plan.sampled_indices),
    )


def select_action_chunk_target(
    first_action: Tuple[float, float, float, float],
    first_target: Tuple[float, float, float, float],
    action_chunk: Tuple[Tuple[float, float, float, float], ...],
    current_pose: Tuple[float, float, float, float],
    lookahead_distance_m: float,
    max_cross_track_m: float,
) -> LookAheadResult:
    """Drop world-frame waypoints already traversed while cloud inference ran."""
    plan = build_action_chunk_plan(
        first_action,
        first_target,
        action_chunk,
        sample_count=len(action_chunk),
    )
    return select_action_chunk_plan_target(
        plan,
        current_pose,
        lookahead_distance_m,
        max_cross_track_m,
    )


def parse_command(
    payload: Any,
    now_unix_ms: Optional[int] = None,
    max_ttl_ms: int = 2000,
    future_tolerance_ms: int = 1000,
    max_action_chunk_steps: int = 10,
) -> BridgeCommand:
    """Validate and normalize one command from the ground-station backend."""
    if not isinstance(payload, dict):
        raise ProtocolError("message must be a JSON object")
    message_type = payload.get("type")
    is_operator = message_type == OPERATOR_MESSAGE_TYPE
    is_preview = message_type == PREVIEW_MESSAGE_TYPE
    expected_version = (
        OPERATOR_SCHEMA_VERSION
        if is_operator
        else PREVIEW_SCHEMA_VERSION
        if is_preview
        else SCHEMA_VERSION
    )
    if payload.get("schema_version") != expected_version:
        raise ProtocolError("unsupported schema_version")
    if message_type not in {MESSAGE_TYPE, PREVIEW_MESSAGE_TYPE, OPERATOR_MESSAGE_TYPE}:
        raise ProtocolError("unsupported message type")

    identifier_key = "task_id" if is_operator else "mission_id"
    identifier = payload.get(identifier_key)
    if not isinstance(identifier, str) or not identifier.strip() or len(identifier) > 128:
        raise ProtocolError("{} must be a non-empty string of at most 128 characters".format(identifier_key))
    mission_id = identifier
    sequence = _require_int(payload, "sequence")
    sent_at_unix_ms = _require_int(payload, "sent_at_unix_ms", 1)
    ttl_ms = _require_int(payload, "ttl_ms", 1)
    if ttl_ms > max_ttl_ms:
        raise ProtocolError("ttl_ms exceeds onboard maximum")

    now_ms = int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
    if sent_at_unix_ms > now_ms + future_tolerance_ms:
        raise ProtocolError("command timestamp is too far in the future")
    if now_ms > sent_at_unix_ms + ttl_ms:
        raise ProtocolError("command expired")

    command = payload.get("command")
    if is_operator:
        if command not in OPERATOR_COMMANDS:
            raise ProtocolError("unsupported operator command")
        policy = "operator"
    else:
        policy = payload.get("policy")
        if policy not in SUPPORTED_POLICIES:
            raise ProtocolError("policy must be openvla or pi05")
        if command not in SUPPORTED_COMMANDS:
            raise ProtocolError("unsupported command")
    if is_preview != (command == PLAN_PREVIEW):
        raise ProtocolError("PLAN_PREVIEW must use the planning_preview message type")
    frame_id = payload.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ProtocolError("frame_id must be a non-empty string")

    if not is_operator:
        semantic = tuple(payload.get("action_semantic", ()))
        units = tuple(payload.get("action_units", ()))
        if semantic != ACTION_SEMANTIC or units != ACTION_UNITS:
            raise ProtocolError("action semantic or units do not match the VLA contract")

    action = None
    target = None
    if payload.get("action_local_delta") is not None:
        action = _vector4(payload["action_local_delta"], "action_local_delta")
    if payload.get("target_mission") is not None:
        target = _vector4(payload["target_mission"], "target_mission")
    action_chunk = None
    if payload.get("action_chunk") is not None and command in {TRACK, PLAN_PREVIEW}:
        action_chunk = _action_chunk(
            payload["action_chunk"], "action_chunk", max_action_chunk_steps
        )
    if command in {TRACK, PLAN_PREVIEW} and (action is None or target is None):
        raise ProtocolError("TRACK and PLAN_PREVIEW require action_local_delta and target_mission")
    if action_chunk is not None:
        if action is None or any(
            abs(action[index] - action_chunk[0][index]) > 1e-6 for index in range(4)
        ):
            raise ProtocolError("action_chunk first row must match action_local_delta")

    calibration_id = None
    body_frame_id = None
    camera_frame_id = None
    source_vehicle_id = None
    source_observation_sequence = None
    source_capture_unix_ms = None
    magnitude = None
    magnitude_unit = None
    orbit_center = None
    orbit_laps = None
    orbit_direction = None
    orbit_yaw_mode = None
    semantic_target_label = None
    semantic_keep_current_altitude = None
    if is_preview:
        calibration_id = _nonempty_string(payload, "calibration_id")
        body_frame_id = _nonempty_string(payload, "body_frame_id")
        camera_frame_id = _nonempty_string(payload, "camera_frame_id")
        source = payload.get("source_observation")
        if not isinstance(source, dict):
            raise ProtocolError("source_observation must be an object")
        source_vehicle_id = _nonempty_string(source, "vehicle_id")
        source_observation_sequence = _require_int(source, "sequence")
        source_capture_unix_ms = _require_int(source, "capture_unix_ms", 1)
        if source_capture_unix_ms > sent_at_unix_ms + future_tolerance_ms:
            raise ProtocolError("source observation timestamp is after command timestamp")
    if is_operator:
        body_frame_id = _nonempty_string(payload, "body_frame_id")
        value = payload.get("magnitude")
        if isinstance(value, bool):
            raise ProtocolError("magnitude must be a finite number")
        try:
            magnitude = float(value)
        except (TypeError, ValueError) as exc:
            raise ProtocolError("magnitude must be a finite number") from exc
        if not math.isfinite(magnitude) or magnitude < 0.0:
            raise ProtocolError("magnitude must be a non-negative finite number")
        magnitude_unit = payload.get("magnitude_unit")
        expected_unit = "rad" if command in {"YAW_LEFT", "YAW_RIGHT"} else "m"
        if command in {"HOLD", "LAND"}:
            expected_unit = "none"
        if magnitude_unit != expected_unit:
            raise ProtocolError("magnitude_unit does not match operator command")
        if command in {"HOLD", "LAND"} and magnitude != 0.0:
            raise ProtocolError("HOLD and LAND require zero magnitude")
        if command == "ORBIT_WORLD":
            orbit = payload.get("orbit")
            if not isinstance(orbit, dict):
                raise ProtocolError("ORBIT_WORLD requires an orbit object")
            center = orbit.get("center")
            if not isinstance(center, Sequence) or isinstance(center, (str, bytes)) or len(center) != 3:
                raise ProtocolError("orbit.center must have shape [3]")
            try:
                orbit_center = tuple(float(item) for item in center)
                orbit_laps = float(orbit.get("laps"))
            except (TypeError, ValueError) as exc:
                raise ProtocolError("orbit values must be finite numbers") from exc
            if not all(math.isfinite(item) for item in orbit_center) or not math.isfinite(orbit_laps):
                raise ProtocolError("orbit values must be finite numbers")
            if abs(float(orbit.get("radius_m", magnitude)) - magnitude) > 1e-6:
                raise ProtocolError("orbit radius does not match magnitude")
            if not 0.25 <= orbit_laps <= 3.0:
                raise ProtocolError("orbit.laps must be within [0.25, 3.0]")
            orbit_direction = orbit.get("direction")
            if orbit_direction not in {"clockwise", "counterclockwise"}:
                raise ProtocolError("orbit.direction is invalid")
            orbit_yaw_mode = orbit.get("yaw_mode", "face_center")
            if orbit_yaw_mode != "face_center":
                raise ProtocolError("only face_center orbit yaw mode is supported")
        if command == "SEMANTIC_ORBIT":
            semantic_orbit = payload.get("semantic_orbit")
            if not isinstance(semantic_orbit, dict):
                raise ProtocolError("SEMANTIC_ORBIT requires a semantic_orbit object")
            semantic_target_label = semantic_orbit.get("target_label")
            if (
                not isinstance(semantic_target_label, str)
                or re.fullmatch(r"[A-Za-z][A-Za-z-]{0,31}", semantic_target_label) is None
            ):
                raise ProtocolError("semantic target_label must be one English word")
            try:
                semantic_radius = float(semantic_orbit.get("radius_m"))
                orbit_laps = float(semantic_orbit.get("laps"))
            except (TypeError, ValueError) as exc:
                raise ProtocolError("semantic orbit values must be finite numbers") from exc
            if not math.isfinite(semantic_radius) or abs(semantic_radius - 1.5) > 1e-6:
                raise ProtocolError("semantic orbit radius must be exactly 1.5 m")
            if abs(magnitude - semantic_radius) > 1e-6:
                raise ProtocolError("semantic orbit radius does not match magnitude")
            if not math.isfinite(orbit_laps) or abs(orbit_laps - 1.0) > 1e-6:
                raise ProtocolError("semantic orbit laps must be exactly 1")
            orbit_direction = semantic_orbit.get("direction")
            if orbit_direction not in {"clockwise", "counterclockwise"}:
                raise ProtocolError("semantic orbit direction is invalid")
            orbit_yaw_mode = semantic_orbit.get("yaw_mode")
            if orbit_yaw_mode != "face_center":
                raise ProtocolError("semantic orbit must face the target center")
            semantic_keep_current_altitude = semantic_orbit.get("keep_current_altitude")
            if semantic_keep_current_altitude is not True:
                raise ProtocolError("semantic orbit must keep current altitude")

    return BridgeCommand(
        mission_id=mission_id,
        sequence=sequence,
        sent_at_unix_ms=sent_at_unix_ms,
        ttl_ms=ttl_ms,
        policy=policy,
        command=command,
        frame_id=frame_id,
        action_local_delta=action,
        target_mission=target,
        action_chunk=action_chunk,
        calibration_id=calibration_id,
        body_frame_id=body_frame_id,
        camera_frame_id=camera_frame_id,
        source_vehicle_id=source_vehicle_id,
        source_observation_sequence=source_observation_sequence,
        source_capture_unix_ms=source_capture_unix_ms,
        message_type=message_type,
        task_id=identifier if is_operator else None,
        magnitude=magnitude,
        magnitude_unit=magnitude_unit,
        orbit_center=orbit_center,
        orbit_laps=orbit_laps,
        orbit_direction=orbit_direction,
        orbit_yaw_mode=orbit_yaw_mode,
        semantic_target_label=(
            semantic_target_label.lower() if semantic_target_label is not None else None
        ),
        semantic_keep_current_altitude=semantic_keep_current_altitude,
    )


def make_ack(command: Optional[BridgeCommand], status: str, reason: str) -> Dict[str, Any]:
    is_operator = bool(command and command.message_type == OPERATOR_MESSAGE_TYPE)
    return {
        "schema_version": (
            OPERATOR_SCHEMA_VERSION
            if is_operator
            else PREVIEW_SCHEMA_VERSION
            if command and command.command == PLAN_PREVIEW
            else SCHEMA_VERSION
        ),
        "type": (
            "operator_task_ack"
            if is_operator
            else "planning_preview_ack"
            if command and command.command == PLAN_PREVIEW
            else "trajectory_ack"
        ),
        "mission_id": command.mission_id if command and not is_operator else None,
        "task_id": command.task_id if is_operator and command else None,
        "sequence": command.sequence if command else None,
        "status": status,
        "reason": reason,
        "onboard_time_unix_ms": int(time.time() * 1000),
    }


def _nonempty_string(payload: Dict[str, Any], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ProtocolError("{} must be a non-empty string".format(key))
    return value
