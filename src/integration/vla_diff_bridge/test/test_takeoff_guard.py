"""No ROS master, service calls, or real publishers are used in these tests."""
import time
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

from test_waypoint_execution import BRIDGE, PACKAGE
from vla_diff_bridge.protocol import BridgeCommand, OPERATOR_MESSAGE_TYPE, ProtocolError


class TakeoffGuardTest(unittest.TestCase):
    def setUp(self):
        self.bridge = b = BRIDGE.VlaDiffBridge.__new__(BRIDGE.VlaDiffBridge)
        b._last_sequence = {}
        b.world_frame, b.body_frame = 'world', 'base_link'
        b.takeoff_height_m = 0.8
        b.preview_only_mode = False
        b.live_publish_enabled = b.operator_task_enabled = True
        b._latest_fcu_state = (True, False, time.monotonic())
        b._require_fresh_odom = Mock()
        b.takeoff_land_pub = Mock()
        b._active_chunk = object()
        b._last_track_monotonic = 123
        self.params = {
            '/px4ctrl/auto_takeoff_land/enable_auto_arm': True,
            '/px4ctrl/auto_takeoff_land/no_RC': False,
            '/px4ctrl/auto_takeoff_land/takeoff_height': 0.8,
        }
        self.request = BridgeCommand(
            'mission', 1, int(time.time()*1000), 500, 'openvla', 'TAKEOFF',
            'world', None, None, body_frame_id='base_link',
            message_type=OPERATOR_MESSAGE_TYPE, task_id='operator-test', magnitude=0.8)

    def apply(self):
        with patch.object(BRIDGE.rospy, 'get_param', self.params.get, create=True):
            return self.bridge._apply_operator_task(self.request)

    def test_missing_stale_disconnected_all_reject_without_mutation(self):
        b = self.bridge
        original = b._active_chunk
        for state in (None, (False, False, time.monotonic()), (True, False, time.monotonic()-3)):
            b._latest_fcu_state = state
            with self.assertRaises(ProtocolError):
                self.apply()
            b.takeoff_land_pub.publish.assert_not_called()
            self.assertIs(b._active_chunk, original)
            self.assertEqual(b._last_sequence, {})

    def test_bad_or_missing_parameters_reject(self):
        for key, bad in (('enable_auto_arm', False), ('no_RC', True),
                         ('takeoff_height', 1.0), ('takeoff_height', float('nan'))):
            full = '/px4ctrl/auto_takeoff_land/' + key
            old = self.params[full]
            for value in (bad, None):
                self.params[full] = value
                with self.assertRaises(ProtocolError):
                    self.apply()
            self.params[full] = old
        self.bridge.takeoff_land_pub.publish.assert_not_called()

    def test_disarmed_auto_takeoff_only_publishes_mock_request(self):
        status, detail = self.apply()
        self.assertEqual(status, 'accepted')
        self.assertIn('arm and climb', detail)
        self.bridge.takeoff_land_pub.publish.assert_called_once()
        self.bridge._require_fresh_odom.assert_called_once()

    def test_locked_mode_never_publishes_takeoff(self):
        self.bridge.live_publish_enabled = False
        status, _ = self.apply()
        self.assertEqual(status, 'operator_locked')
        self.bridge.takeoff_land_pub.publish.assert_not_called()

    def test_launch_overrides_after_yaml_and_wrapper_uses_it(self):
        node = ET.parse(PACKAGE / 'launch/px4ctrl_vla.launch').getroot().find('node')
        tags = list(node)
        load_index = next(i for i, child in enumerate(tags) if child.tag == 'rosparam')
        params = {child.attrib['name']: child for child in tags if child.tag == 'param'}
        for name, value in (('enable_auto_arm', 'true'), ('no_RC', 'false'), ('takeoff_height', '0.8')):
            child = params['auto_takeoff_land/' + name]
            self.assertEqual(child.attrib['value'], value)
            self.assertGreater(tags.index(child), load_index)
        source = (PACKAGE.parents[2] / 'sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh').read_text()
        self.assertIn('start_launch px4ctrl vla_diff_bridge px4ctrl_vla.launch', source)


if __name__ == '__main__':
    unittest.main()
