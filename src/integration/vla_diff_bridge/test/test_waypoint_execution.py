"""Exercise the real bridge state machine with ROS fully mocked (no runtime)."""
import importlib.util
import pathlib
import sys
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
from vla_diff_bridge.protocol import BridgeCommand, HOLD, TRACK, ProtocolError


def load_bridge():
    modules = {}
    for name in ("rospy", "geometry_msgs", "geometry_msgs.msg", "nav_msgs",
                 "nav_msgs.msg", "mavros_msgs", "mavros_msgs.msg", "quadrotor_msgs", "quadrotor_msgs.msg",
                 "std_msgs", "std_msgs.msg", "tf", "tf.transformations"):
        modules[name] = types.ModuleType(name)
    for name, attributes in {
        "geometry_msgs.msg": ("PoseStamped",),
        "nav_msgs.msg": ("Odometry",),
        "mavros_msgs.msg": ("State",),
        "quadrotor_msgs.msg": ("PositionCommand", "TakeoffLand"),
        "std_msgs.msg": ("Empty", "String"),
    }.items():
        for attribute in attributes:
            setattr(modules[name], attribute, Mock())
    for attribute in ("euler_from_quaternion", "quaternion_from_euler"):
        setattr(modules["tf.transformations"], attribute, Mock())
    spec = importlib.util.spec_from_file_location(
        "mocked_waypoint_bridge", PACKAGE / "scripts" / "vla_diff_bridge_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
    return module


BRIDGE = load_bridge()


def command(sequence=1, x=0.1, action=TRACK):
    return BridgeCommand(
        "mission", sequence, int(time.time() * 1000), 500, "pi05", action,
        "world", (0.1, 0.0, 0.0, 0.0) if action == TRACK else None,
        (x, 0.0, 1.0, 0.0) if action == TRACK else None,
        tuple((0.1, 0.0, 0.0, 0.0) for _ in range(10)) if action == TRACK else None,
    )


class WaypointExecutionTest(unittest.TestCase):
    def setUp(self):
        self.bridge = b = BRIDGE.VlaDiffBridge.__new__(BRIDGE.VlaDiffBridge)
        b._lock = threading.RLock()
        b._last_sequence = {}
        b._terminal_missions = set()
        b._active_mission_id = None
        b._active_chunk = None
        b._last_track_monotonic = None
        b._watchdog_latched = False
        b.live_publish_enabled = True
        b.preview_only_mode = False
        b.planning_preview_enabled = True
        b.action_chunk_lookahead_enabled = True
        b.action_chunk_sample_count = 8
        b.action_chunk_lookahead_distance_m = 0.05
        b.action_chunk_max_cross_track_m = 0.5
        b.max_goal_step_m = 1.0
        b.min_goal_z_m = 0.1
        b.max_goal_z_m = 2.0
        b.max_odom_age_ms = 250
        b.watchdog_timeout_ms = 1000
        b.world_frame = "world"
        b._publish_goal = Mock()
        b._publish_status = Mock()
        b.hover_pub = Mock()
        b.stop_pub = Mock()
        self.odom(0.0)

    def odom(self, x, y=0.0, age_ms=0):
        self.bridge._latest_odom = (x, y, 1.0, 0.0, int(time.time() * 1000) - age_ms)

    def test_buffers_eight_and_publishes_one_at_a_time_not_terminal_only(self):
        b = self.bridge
        b._apply(command())
        self.assertEqual(len(b._active_chunk.plan.sampled_indices), 8)
        self.assertEqual(b._publish_goal.call_count, 1)
        self.assertAlmostEqual(b._publish_goal.call_args[0][0][0], 0.1)
        receipt_time = b._last_track_monotonic
        self.odom(0.25)
        b._watchdog_callback(None)
        self.assertEqual(b._publish_goal.call_count, 2)
        self.assertLess(b._publish_goal.call_args[0][0][0], 1.0)
        self.assertGreater(b._active_chunk.sample_index, 0)
        self.assertEqual(b._last_track_monotonic, receipt_time)
        b._watchdog_callback(None)
        self.assertEqual(b._publish_goal.call_count, 2)
        self.odom(0.0)  # noisy regression cannot reissue an earlier sample
        b._watchdog_callback(None)
        self.assertEqual(b._publish_goal.call_count, 2)

    def test_hold_cancels_buffer(self):
        b = self.bridge
        b._apply(command())
        b._apply(command(sequence=2, action=HOLD))
        self.odom(0.5)
        b._watchdog_callback(None)
        self.assertIsNone(b._active_chunk)
        self.assertEqual(b._publish_goal.call_count, 1)
        b.hover_pub.publish.assert_called_once()

    def test_all_six_or_eight_waypoints_dispatch_in_order(self):
        for count in (6, 8):
            self.setUp()
            b = self.bridge
            b.action_chunk_sample_count = count
            b._apply(command())
            expected = b._active_chunk.plan.sampled_waypoints
            for target in expected:
                self.odom(target[0])
                b._watchdog_callback(None)
            actual = tuple(call[0][0] for call in b._publish_goal.call_args_list)
            self.assertEqual(actual, expected)
            self.assertIsNone(b._active_chunk)

    def test_single_action_replaces_buffer_without_sequential_replay(self):
        b = self.bridge
        b._apply(command())
        single = BridgeCommand("mission", 2, int(time.time() * 1000), 500,
                               "openvla", TRACK, "world", (0.1, 0., 0., 0.),
                               (0.1, 0., 1., 0.))
        b._apply(single)
        self.assertIsNone(b._active_chunk)
        self.odom(0.3)
        b._watchdog_callback(None)
        self.assertEqual(b._publish_goal.call_count, 2)

    def test_new_chunk_replaces_old_chunk_and_duplicate_does_not(self):
        b = self.bridge
        b._apply(command())
        self.odom(0.25)
        b._apply(command(sequence=2, x=0.35))
        self.assertEqual(b._active_chunk.command.sequence, 2)
        with self.assertRaises(ProtocolError):
            b._apply(command(sequence=1))
        self.assertEqual(b._active_chunk.command.sequence, 2)

    def test_watchdog_cancels_buffer_before_any_late_publication(self):
        b = self.bridge
        b._apply(command())
        b._last_track_monotonic -= 2.0
        self.odom(0.5)
        b._watchdog_callback(None)
        self.assertIsNone(b._active_chunk)
        self.assertEqual(b._publish_goal.call_count, 1)
        b.hover_pub.publish.assert_called_once()

    def test_stale_odom_and_corridor_departure_abort_buffer(self):
        for age, y in ((1000, 0.0), (0, 1.0)):
            self.setUp()
            b = self.bridge
            b._apply(command())
            self.odom(0.3, y=y, age_ms=age)
            b._watchdog_callback(None)
            self.assertIsNone(b._active_chunk)
            self.assertEqual(b._publish_goal.call_count, 1)
            b.hover_pub.publish.assert_called_once()


if __name__ == "__main__":
    unittest.main()
