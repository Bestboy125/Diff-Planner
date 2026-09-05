import math
import unittest
from unittest.mock import Mock, patch

import test_operator_distance as distance_tests
from test_waypoint_execution import BRIDGE
from vla_diff_bridge.protocol import ProtocolError


class OperatorHeadingTest(unittest.TestCase):
    def setUp(self):
        self.fixture = distance_tests.OperatorDistanceTest()
        self.fixture.setUp()
        self.bridge = self.fixture.bridge

    def test_all_six_translations_hold_captured_heading(self):
        b = self.bridge
        for yaw in (0.7, 3.13, -3.13):
            b._last_sequence = {}
            b._require_fresh_odom.return_value = (0, 0, 1, yaw, 0)
            for action in ('MOVE_FORWARD', 'MOVE_BACKWARD', 'MOVE_LEFT', 'MOVE_RIGHT', 'MOVE_UP', 'MOVE_DOWN'):
                b._apply_operator_task(self.fixture.request(action, 0.2))
                args, kwargs = b._publish_goal.call_args
                self.assertEqual(args[0][3], yaw)
                self.assertTrue(kwargs['hold_heading'])

    def test_explicit_rotations_do_not_lock_heading(self):
        b = self.bridge
        b.max_operator_yaw_rad = 1.5708
        b._require_fresh_odom.return_value = (0, 0, 1, 0.7, 0)
        for action, expected in [('YAW_LEFT', 1.0), ('YAW_RIGHT', 0.4)]:
            b._apply_operator_task(self.fixture.request(action, 0.3))
            args, kwargs = b._publish_goal.call_args
            self.assertAlmostEqual(args[0][3], expected)
            self.assertFalse(kwargs['hold_heading'])

    def test_old_trajectory_server_is_rejected_before_goal_publication(self):
        b = self.bridge
        b.operator_yaw_hold_pub.get_num_connections.return_value = 0
        with self.assertRaisesRegex(ProtocolError, 'updated traj_server'):
            b._apply_operator_task(self.fixture.request('MOVE_LEFT', 0.2))
        b._publish_goal.assert_not_called()

    def test_publication_uses_only_the_selected_yaw_channel(self):
        b = self.bridge
        b.yaw_pub, b.goal_pub = Mock(), Mock()
        with patch.object(BRIDGE.rospy, 'Time', Mock(), create=True), \
             patch.object(BRIDGE, 'quaternion_from_euler', return_value=(0,0,0,1)):
            BRIDGE.VlaDiffBridge._publish_goal(b, (0, 0.2, 1, 0.7), hold_heading=True)
            b.operator_yaw_hold_pub.publish.assert_called_once()
            b.yaw_pub.publish.assert_not_called()
            self.assertEqual(b.operator_yaw_hold_pub.publish.call_args[0][0].yaw_dot, 0)
            BRIDGE.VlaDiffBridge._publish_goal(b, (0, 0.2, 1, 1.0))
            b.yaw_pub.publish.assert_called_once()
            self.assertEqual(b.goal_pub.publish.call_count, 2)
