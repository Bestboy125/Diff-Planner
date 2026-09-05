"""ROS-free operator distance boundary checks; all publications are mocked."""
import time
import unittest
from unittest.mock import Mock

from test_waypoint_execution import BRIDGE
from vla_diff_bridge.protocol import BridgeCommand, OPERATOR_MESSAGE_TYPE, ProtocolError


class OperatorDistanceTest(unittest.TestCase):
    def setUp(self):
        self.bridge = b = BRIDGE.VlaDiffBridge.__new__(BRIDGE.VlaDiffBridge)
        b._last_sequence = {}
        b.world_frame, b.body_frame = 'world', 'base_link'
        b.max_goal_step_m, b.max_operator_step_m = 1.0, 2.0
        b.preview_only_mode = False
        b.live_publish_enabled = b.operator_task_enabled = True
        b.min_goal_z_m, b.max_goal_z_m = 0.1, 2.0
        b._require_fresh_odom = Mock(return_value=(0.0, 0.0, 1.0, 0.0, int(time.time()*1000)))
        b._publish_goal = Mock()
        b.operator_yaw_hold_pub = Mock()
        b.operator_yaw_hold_pub.get_num_connections.return_value = 1
        b._active_chunk = None
        b._last_track_monotonic = None

    def request(self, action, magnitude):
        return BridgeCommand('mission', 1, int(time.time()*1000), 500, 'openvla', action,
                             'world', None, None, body_frame_id='base_link',
                             message_type=OPERATOR_MESSAGE_TYPE, task_id=action, magnitude=magnitude)

    def test_two_metre_horizontal_moves_allowed(self):
        for action in ('MOVE_FORWARD', 'MOVE_BACKWARD', 'MOVE_LEFT', 'MOVE_RIGHT'):
            status, _ = self.bridge._apply_operator_task(self.request(action, 2.0))
            self.assertEqual(status, 'accepted')
        self.assertEqual(self.bridge._publish_goal.call_count, 4)

    def test_above_two_metres_rejected_for_all_six_directions(self):
        for action in ('MOVE_FORWARD', 'MOVE_BACKWARD', 'MOVE_LEFT', 'MOVE_RIGHT', 'MOVE_UP', 'MOVE_DOWN'):
            with self.assertRaisesRegex(ProtocolError, 'max_operator_step_m'):
                self.bridge._apply_operator_task(self.request(action, 2.001))
        self.bridge._publish_goal.assert_not_called()

    def test_vertical_moves_still_obey_altitude_bounds(self):
        for action in ('MOVE_UP', 'MOVE_DOWN'):
            with self.assertRaisesRegex(ProtocolError, 'altitude'):
                self.bridge._apply_operator_task(self.request(action, 2.0))
        self.bridge._publish_goal.assert_not_called()

    def test_vla_goal_limit_remains_one_metre(self):
        with self.assertRaisesRegex(ProtocolError, 'max_goal_step_m'):
            self.bridge._validate_goal((1.5, 0.0, 1.0, 0.0))
