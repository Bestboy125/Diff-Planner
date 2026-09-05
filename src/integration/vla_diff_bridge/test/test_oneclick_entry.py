import pathlib
import unittest


WORKSPACE = pathlib.Path(__file__).resolve().parents[4]


class OneClickEntryTest(unittest.TestCase):
    def test_usb_only_no_fabricated_tf(self):
        source = (WORKSPACE / "sh_files/start_onboard_vla_full_preview.sh").read_text(encoding="utf-8")
        self.assertIn("vla_usb_camera.launch", source)
        self.assertNotIn("realsense2_camera", source)
        self.assertNotIn("static_transform_publisher", source)
        self.assertIn('== calibrated', source)
        self.assertIn('unset _CATKIN_SETUP_DIR', source)

    def test_cli_live_opt_in_and_isolated_default(self):
        source = (WORKSPACE / "sh_files/start_onboard_vla_full_preview.sh").read_text(encoding="utf-8")
        self.assertIn("MODE=preview", source)
        self.assertIn('"${2:-}" == \'I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION\'', source)
        self.assertIn('export VLA_BRIDGE_MODE="${MODE}"', source)
        self.assertIn('if [[ "${MODE}" == live ]]; then', source)
        self.assertIn('start_owned preview_stack bash', source)
        self.assertIn('wait_mavros_connected_disarmed 5', source)

    def test_autonomous_multipoint_stays_disabled(self):
        source = (WORKSPACE / "sh_files/start_onboard_vla_full_preview.sh").read_text(encoding="utf-8")
        for setting in ("START_PLAN", "BACK_PLAN", "AUTO_PLANNING", "AUTO_LANDING"):
            self.assertIn('${MULTIPOINT_' + setting + ':-0}', source)
        self.assertIn("monitor_owned", source)
        self.assertIn("trap 'exit 143' TERM", source)

    def test_vla_stack_selects_heading_hold_binary(self):
        source = (WORKSPACE / "sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh").read_text(encoding="utf-8")
        self.assertIn('traj_server_executable:=traj_server_heading_hold', source)
        launch = (WORKSPACE / "src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch").read_text()
        self.assertIn('type="$(arg traj_server_executable)"', launch)


if __name__ == "__main__":
    unittest.main()
