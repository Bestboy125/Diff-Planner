"""Pure-Python validation for the host-to-onboard trajectory protocol."""

from dataclasses import dataclass
import math
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


def parse_command(
    payload: Any,
    now_unix_ms: Optional[int] = None,
    max_ttl_ms: int = 2000,
    future_tolerance_ms: int = 1000,
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
    if command in {TRACK, PLAN_PREVIEW} and (action is None or target is None):
        raise ProtocolError("TRACK and PLAN_PREVIEW require action_local_delta and target_mission")

    calibration_id = None
    body_frame_id = None
    camera_frame_id = None
    source_vehicle_id = None
    source_observation_sequence = None
    source_capture_unix_ms = None
    magnitude = None
    magnitude_unit = None
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
