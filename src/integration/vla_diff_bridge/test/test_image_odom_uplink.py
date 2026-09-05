"""Exercise the real payload builder with fake ROS messages, never start ROS."""
import importlib.util
import pathlib
import threading
import types
import unittest
from unittest.mock import Mock, patch


def load_uplink():
    modules = {}
    for name in ("rospy", "nav_msgs", "nav_msgs.msg", "quadrotor_msgs", "quadrotor_msgs.msg",
                 "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "tf2_ros"):
        modules[name] = types.ModuleType(name)
    for module, names in {
        "rospy": ("Time", "Duration"), "nav_msgs.msg": ("Odometry",),
        "quadrotor_msgs.msg": ("PositionCommand",),
        "sensor_msgs.msg": ("CameraInfo", "CompressedImage", "Image"),
        "std_msgs.msg": ("String",),
    }.items():
        for name in names:
            setattr(modules[module], name, Mock())
    path = pathlib.Path(__file__).resolve().parents[1] / "scripts/onboard_observation_uplink_node.py"
    spec = importlib.util.spec_from_file_location("uplink_under_test", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", modules):
        spec.loader.exec_module(module)
    return module.ObservationUplink


class ImageOdomUplinkTest(unittest.TestCase):
    def setUp(self):
        ns = types.SimpleNamespace
        node_class = load_uplink()
        self.node = node_class.__new__(node_class)
        self.node._lock = threading.RLock()
        self.node._sequence = 0
        self.node.observation_mode = "image_odom"
        self.node._tf_buffer = Mock()
        self.node._tf_buffer.lookup_transform.side_effect = RuntimeError("no camera TF")
        self.node.max_pair_age_ms = 80
        self.node.world_frame = "world"
        self.node.body_frame = "base_link"
        self.node.camera_frame = "usb"
        self.node.allow_empty_odom_child_frame = True
        self.node.vehicle_id = "uav0"
        self.node.calibration_id = "usb-config-v1"
        self.node.calibration_validated = True
        self.stamp = ns(to_sec=lambda: 1000.0)
        xyz = ns(x=0.0, y=0.0, z=1.0)
        quat = ns(x=0.0, y=0.0, z=0.0, w=1.0)
        self.odom = ns(header=ns(stamp=self.stamp, frame_id="world"), child_frame_id="base_link",
                       pose=ns(pose=ns(position=xyz, orientation=quat)),
                       twist=ns(twist=ns(linear=xyz, angular=xyz)))
        self.camera = ns(header=ns(frame_id="usb"), width=640, height=480,
                         distortion_model="plumb_bob", K=[1.0]*9, D=[])

    def build(self):
        return self.node._build_payload(self.stamp, "usb", b"\xff\xd8test\xff\xd9",
                                        self.odom, self.camera, None)

    def test_no_tf_lookup_or_fake_calibration(self):
        payload = self.build()
        self.assertIsNone(payload["body_from_camera"])
        self.assertFalse(payload["calibration_validated"])
        self.assertEqual(payload["observation_mode"], "image_odom")
        self.assertEqual(payload["odometry"]["child_frame_id"], "base_link")
        self.node._tf_buffer.lookup_transform.assert_not_called()

    def test_calibrated_mode_still_requires_tf(self):
        self.node.observation_mode = "calibrated"
        with self.assertRaisesRegex(RuntimeError, "TF unavailable"):
            self.build()

    def test_missing_odometry_rejected(self):
        self.odom = None
        with self.assertRaisesRegex(RuntimeError, "odometry unavailable"):
            self.build()

    def test_unsynchronized_odometry_rejected(self):
        self.odom.header.stamp = types.SimpleNamespace(to_sec=lambda: 999.9)
        with self.assertRaisesRegex(RuntimeError, "not synchronized"):
            self.build()

    def test_wrong_odom_frame_rejected(self):
        self.odom.header.frame_id = "wrong"
        with self.assertRaisesRegex(RuntimeError, "world frame"):
            self.build()

    def test_wrong_camera_frame_rejected(self):
        self.camera.header.frame_id = "wrong"
        with self.assertRaisesRegex(RuntimeError, "optical frame"):
            self.build()


if __name__ == "__main__":
    unittest.main()
