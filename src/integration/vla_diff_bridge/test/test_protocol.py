import pathlib
import sys
import time
import unittest


PACKAGE_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from vla_diff_bridge.protocol import (  # noqa: E402
    ProtocolError,
    build_action_chunk_plan,
    integrate_action_chunk,
    parse_command,
    select_action_chunk_target,
    select_action_chunk_plan_target,
)


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
        "action_chunk": [
            [0.2, 0.0, 0.1, 0.05],
            [0.2, 0.0, 0.0, 0.0],
            [0.2, 0.0, 0.0, 0.0],
        ],
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
        self.assertEqual(len(parsed.action_chunk), 3)

    def test_rejects_action_chunk_that_does_not_start_with_immediate_action(self):
        payload = valid_payload()
        payload["action_chunk"][0][0] = 0.3
        with self.assertRaisesRegex(ProtocolError, "first row"):
            parse_command(payload)

    def test_rejects_action_chunk_over_onboard_horizon_limit(self):
        payload = valid_payload()
        payload["action_chunk"] = [payload["action_local_delta"][0]] * 11
        with self.assertRaisesRegex(ProtocolError, "shape"):
            parse_command(payload, max_action_chunk_steps=10)

    def test_integrates_incremental_actions_from_capture_pose(self):
        capture, targets = integrate_action_chunk(
            (0.1, 0.0, 0.0, 0.0),
            (1.1, 2.0, 1.0, 0.0),
            ((0.1, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.0)),
        )
        self.assertEqual(capture, (1.0, 2.0, 1.0, 0.0))
        for actual, expected in zip(targets[-1], (1.2, 2.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected)

    def test_lookahead_discards_waypoints_passed_during_inference(self):
        result = select_action_chunk_target(
            (0.1, 0.0, 0.0, 0.0),
            (0.1, 0.0, 1.0, 0.0),
            (
                (0.1, 0.0, 0.0, 0.0),
                (0.1, 0.0, 0.0, 0.0),
                (0.1, 0.0, 0.0, 0.0),
            ),
            (0.16, 0.0, 1.0, 0.0),
            lookahead_distance_m=0.05,
            max_cross_track_m=1.0,
        )
        self.assertEqual(result.selected_index, 2)
        self.assertEqual(result.skipped_count, 2)
        self.assertAlmostEqual(result.target[0], 0.3)

    def test_lookahead_suppresses_fully_traversed_chunk(self):
        result = select_action_chunk_target(
            (0.1, 0.0, 0.0, 0.0),
            (0.1, 0.0, 1.0, 0.0),
            ((0.1, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.0)),
            (0.25, 0.0, 1.0, 0.0),
            lookahead_distance_m=0.05,
            max_cross_track_m=1.0,
        )
        self.assertIsNone(result.target)
        self.assertIn("already traversed", result.reason)

    def test_ten_step_chunk_can_be_sampled_to_six_or_eight_original_waypoints(self):
        actions = tuple((0.1, 0.0, 0.0, 0.0) for _ in range(10))
        for sample_count in (6, 8):
            plan = build_action_chunk_plan(
                actions[0],
                (0.1, 0.0, 1.0, 0.0),
                actions,
                sample_count=sample_count,
            )
            self.assertEqual(len(plan.sampled_indices), sample_count)
            self.assertEqual(plan.sampled_indices[0], 0)
            self.assertEqual(plan.sampled_indices[-1], 9)
            self.assertEqual(tuple(sorted(set(plan.sampled_indices))), plan.sampled_indices)

    def test_sampled_chunk_advances_in_order_and_prunes_passed_waypoints(self):
        actions = tuple((0.1, 0.0, 0.0, 0.0) for _ in range(10))
        plan = build_action_chunk_plan(
            actions[0],
            (0.1, 0.0, 1.0, 0.0),
            actions,
            sample_count=6,
        )
        first = select_action_chunk_plan_target(
            plan, (0.0, 0.0, 1.0, 0.0), 0.05, 0.5
        )
        advanced = select_action_chunk_plan_target(
            plan, (0.55, 0.0, 1.0, 0.0), 0.05, 0.5
        )
        self.assertEqual(first.selected_sample_index, 0)
        self.assertGreater(advanced.selected_sample_index, first.selected_sample_index)
        self.assertGreater(advanced.skipped_count, 0)
        self.assertLess(advanced.selected_index, 9)

        complete = select_action_chunk_plan_target(
            plan, (1.01, 0.0, 1.0, 0.0), 0.05, 0.5
        )
        self.assertIsNone(complete.target)
        self.assertEqual(complete.skipped_count, 6)

    def test_sampled_chunk_rejects_cross_track_deviation(self):
        actions = tuple((0.1, 0.0, 0.0, 0.0) for _ in range(10))
        plan = build_action_chunk_plan(
            actions[0],
            (0.1, 0.0, 1.0, 0.0),
            actions,
            sample_count=8,
        )
        result = select_action_chunk_plan_target(
            plan, (0.2, 1.0, 1.0, 0.0), 0.05, 0.5
        )
        self.assertIsNone(result.target)
        self.assertIn("outside", result.reason)

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
