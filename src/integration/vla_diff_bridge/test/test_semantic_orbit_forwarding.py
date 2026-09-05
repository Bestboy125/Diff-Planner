"""ROS-free check that a validated semantic request is only forwarded onboard."""
import json
import time
import unittest
from unittest.mock import Mock

from test_waypoint_execution import BRIDGE
from vla_diff_bridge.protocol import parse_command


def semantic_payload():
    return {
        "schema_version": 3,
        "type": "operator_task",
        "task_id": "semantic-test",
        "sequence": 1,
        "sent_at_unix_ms": int(time.time() * 1000),
        "ttl_ms": 500,
        "command": "SEMANTIC_ORBIT",
        "frame_id": "world",
        "body_frame_id": "base_link",
        "magnitude": 1.5,
        "magnitude_unit": "m",
        "semantic_orbit": {
            "target_label": "Chair",
            "radius_m": 1.5,
            "laps": 1,
            "direction": "clockwise",
            "yaw_mode": "face_center",
            "keep_current_altitude": True,
        },
    }


class SemanticOrbitForwardingTest(unittest.TestCase):
    def setUp(self):
        class FakeString(object):
            def __init__(self, data=""):
                self.data = data

        BRIDGE.String = FakeString
        self.bridge = bridge = BRIDGE.VlaDiffBridge.__new__(BRIDGE.VlaDiffBridge)
        bridge._last_sequence = {}
        bridge.world_frame, bridge.body_frame = "world", "base_link"
        bridge.max_operator_step_m = 2.0
        bridge.max_operator_yaw_rad = 1.5708
        bridge.takeoff_height_m = 0.8
        bridge.preview_only_mode = False
        bridge.live_publish_enabled = bridge.operator_task_enabled = True
        bridge.min_goal_z_m, bridge.max_goal_z_m = 0.1, 2.0
        bridge._require_fresh_odom = Mock(
            return_value=(0.0, 0.0, 1.1, 0.0, int(time.time() * 1000))
        )
        bridge.semantic_orbit_pub = Mock()
        bridge.semantic_orbit_pub.get_num_connections.return_value = 1
        bridge.semantic_orbit_cancel_pub = Mock()
        bridge._active_chunk = None
        bridge._active_orbit = None
        bridge._last_track_monotonic = None

    def test_forwards_sanitized_request_without_publishing_a_goal(self):
        command = parse_command(semantic_payload())
        status, detail = self.bridge._apply_operator_task(command)
        self.assertEqual(status, "accepted")
        self.assertIn("no takeoff", detail)
        published = json.loads(self.bridge.semantic_orbit_pub.publish.call_args.args[0].data)
        self.assertEqual(published["target_label"], "chair")
        self.assertEqual(published["radius_m"], 1.5)
        self.assertEqual(published["laps"], 1.0)
        self.assertTrue(published["keep_current_altitude"])

    def test_rejects_when_executor_is_not_connected(self):
        self.bridge.semantic_orbit_pub.get_num_connections.return_value = 0
        with self.assertRaisesRegex(Exception, "executor is unavailable"):
            self.bridge._apply_operator_task(parse_command(semantic_payload()))


if __name__ == "__main__":
    unittest.main()
