import pathlib
import sys
import time
import unittest


PACKAGE_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from vla_diff_bridge.protocol import ProtocolError, parse_command  # noqa: E402


def valid_payload():
    now_ms = int(time.time() * 1000)
    return {
        "schema_version": 1,
        "type": "trajectory_command",
        "mission_id": "mission-test",
        "sequence": 4,
        "sent_at_unix_ms": now_ms,
        "ttl_ms": 500,
        "policy": "pi05",
        "command": "TRACK",
        "frame_id": "world",
        "action_semantic": ["dx_body", "dy_body", "dz_body", "d_yaw"],
        "action_units": ["m", "m", "m", "rad"],
        "action_local_delta": [[0.2, 0.0, 0.1, 0.05]],
        "target_mission": [[1.2, 2.0, 1.1, 0.55]],
    }


def valid_preview_payload():
    payload = valid_payload()
    payload.update(
        {
            "schema_version": 2,
            "type": "planning_preview",
            "command": "PLAN_PREVIEW",
            "body_frame_id": "base_link",
            "camera_frame_id": "camera_color_optical_frame",
            "calibration_id": "front-rgb-2026-09-01",
            "source_observation": {
                "vehicle_id": "uav0",
                "sequence": 14,
                "capture_unix_ms": payload["sent_at_unix_ms"] - 20,
            },
        }
    )
    return payload


def valid_operator_payload():
    return {
        "schema_version": 3,
        "type": "operator_task",
        "task_id": "operator-test",
        "sequence": 9,
        "sent_at_unix_ms": int(time.time() * 1000),
        "ttl_ms": 500,
        "command": "MOVE_LEFT",
        "frame_id": "world",
        "body_frame_id": "base_link",
        "magnitude": 0.4,
        "magnitude_unit": "m",
    }


class ProtocolTest(unittest.TestCase):
    def test_accepts_common_model_contract(self):
        parsed = parse_command(valid_payload())
        self.assertEqual(parsed.policy, "pi05")
        self.assertEqual(parsed.target_mission, (1.2, 2.0, 1.1, 0.55))

    def test_rejects_expired_command(self):
        payload = valid_payload()
        payload["sent_at_unix_ms"] -= 1000
        with self.assertRaisesRegex(ProtocolError, "expired"):
            parse_command(payload)

    def test_rejects_non_finite_action(self):
        payload = valid_payload()
        payload["action_local_delta"] = [[float("nan"), 0.0, 0.0, 0.0]]
        with self.assertRaisesRegex(ProtocolError, "finite"):
            parse_command(payload)

    def test_rejects_semantic_mismatch(self):
        payload = valid_payload()
        payload["action_units"] = ["cm", "cm", "cm", "deg"]
        with self.assertRaisesRegex(ProtocolError, "semantic or units"):
            parse_command(payload)

    def test_hold_does_not_require_motion_vectors(self):
        payload = valid_payload()
        payload["command"] = "HOLD"
        payload["action_local_delta"] = None
        payload["target_mission"] = None
        parsed = parse_command(payload)
        self.assertEqual(parsed.command, "HOLD")

    def test_accepts_planner_only_preview_contract(self):
        parsed = parse_command(valid_preview_payload())
        self.assertEqual(parsed.command, "PLAN_PREVIEW")
        self.assertEqual(parsed.body_frame_id, "base_link")
        self.assertEqual(parsed.source_observation_sequence, 14)

    def test_rejects_preview_without_calibration_identity(self):
        payload = valid_preview_payload()
        del payload["calibration_id"]
        with self.assertRaisesRegex(ProtocolError, "calibration_id"):
            parse_command(payload)

    def test_rejects_plan_preview_disguised_as_control_message(self):
        payload = valid_preview_payload()
        payload["schema_version"] = 1
        payload["type"] = "trajectory_command"
        with self.assertRaisesRegex(ProtocolError, "PLAN_PREVIEW"):
            parse_command(payload)

    def test_accepts_operator_body_frame_primitive(self):
        parsed = parse_command(valid_operator_payload())
        self.assertEqual(parsed.message_type, "operator_task")
        self.assertEqual(parsed.task_id, "operator-test")
        self.assertEqual(parsed.command, "MOVE_LEFT")
        self.assertEqual(parsed.magnitude, 0.4)

    def test_rejects_operator_rotation_with_wrong_unit(self):
        payload = valid_operator_payload()
        payload["command"] = "YAW_LEFT"
        with self.assertRaisesRegex(ProtocolError, "magnitude_unit"):
            parse_command(payload)


if __name__ == "__main__":
    unittest.main()
