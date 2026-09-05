"""ROS-free deterministic world-frame orbit generation checks."""
import math
import time
import unittest
from unittest.mock import Mock

from test_waypoint_execution import BRIDGE
from vla_diff_bridge.protocol import BridgeCommand, OPERATOR_MESSAGE_TYPE, parse_command


def orbit_payload():
    return {
        "schema_version": 3, "type": "operator_task", "task_id": "orbit-test",
        "sequence": 1, "sent_at_unix_ms": int(time.time() * 1000), "ttl_ms": 500,
        "command": "ORBIT_WORLD", "frame_id": "world", "body_frame_id": "base_link",
        "magnitude": 1.0, "magnitude_unit": "m",
        "orbit": {"center": [0.0, 0.0, 1.0], "radius_m": 1.0, "laps": 1.0,
                  "direction": "counterclockwise", "yaw_mode": "face_center"},
    }


class OperatorOrbitTest(unittest.TestCase):
    def setUp(self):
        self.bridge = b = BRIDGE.VlaDiffBridge.__new__(BRIDGE.VlaDiffBridge)
        b.max_operator_step_m = 2.0
        b.min_goal_z_m, b.max_goal_z_m = 0.1, 2.0
        b.orbit_waypoint_spacing_m = 0.35
        b.orbit_arrival_tolerance_m = 0.20
        b.max_orbit_waypoints = 720

    def test_protocol_and_geometry_are_deterministic(self):
        command = parse_command(orbit_payload())
        points = self.bridge._build_orbit_waypoints(command, (1.0, 0.0, 1.0, 0.0, 0))
        self.assertGreater(len(points), 20)
        self.assertAlmostEqual(points[0][0], 1.0)
        self.assertAlmostEqual(points[0][1], 0.0)
        self.assertAlmostEqual(points[-1][0], 1.0, places=6)
        self.assertAlmostEqual(points[-1][1], 0.0, places=6)
        for x, y, z, yaw in points:
            self.assertAlmostEqual(math.hypot(x, y), 1.0, places=6)
            self.assertEqual(z, 1.0)
            expected_yaw = math.atan2(-y, -x)
            self.assertAlmostEqual(math.sin(yaw), math.sin(expected_yaw), places=6)
            self.assertAlmostEqual(math.cos(yaw), math.cos(expected_yaw), places=6)

    def test_timer_advances_only_after_reaching_current_waypoint(self):
        command = parse_command(orbit_payload())
        points = self.bridge._build_orbit_waypoints(command, (1.0, 0.0, 1.0, 0.0, 0))
        b = self.bridge
        b._active_orbit = BRIDGE.ActiveOrbitExecution(command, points, 0)
        b._publish_goal, b._publish_status, b.hover_pub = Mock(), Mock(), Mock()
        b._require_fresh_odom = Mock(return_value=(2.0, 0.0, 1.0, 0.0, 0))
        b._advance_active_orbit()
        b._publish_goal.assert_not_called()
        b._require_fresh_odom.return_value = (1.0, 0.0, 1.0, 0.0, 0)
        b._advance_active_orbit()
        b._publish_goal.assert_called_once_with(points[1], hold_heading=True)
        self.assertEqual(b._active_orbit.waypoint_index, 1)


if __name__ == "__main__":
    unittest.main()
