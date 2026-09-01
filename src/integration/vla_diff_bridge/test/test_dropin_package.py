import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class DropInPackageTest(unittest.TestCase):
    def test_stack_defaults_every_runtime_group_off(self):
        launch = (PACKAGE_ROOT / "launch" / "vla_fastlio_diff_preview_stack.launch").read_text(
            encoding="utf-8"
        )
        for argument in (
            "start_diff_planner_preview",
            "start_network_bridge",
            "start_observation_uplink",
        ):
            self.assertIn('name="{}" default="false"'.format(argument), launch)

    def test_unmodified_diff_planner_topics_are_isolated_by_absolute_remap(self):
        launch = (PACKAGE_ROOT / "launch" / "vla_fastlio_diff_preview_stack.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('from="/goal" to="$(arg preview_goal_topic)"', launch)
        self.assertIn('from="/planning/yaw" to="$(arg preview_yaw_topic)"', launch)
        self.assertIn('from="/position_cmd" to="$(arg optimized_preview_topic)"', launch)
        self.assertNotIn("/setpoints_cmd", launch)

    def test_package_contains_no_flight_stack_process(self):
        launch = (PACKAGE_ROOT / "launch" / "vla_fastlio_diff_preview_stack.launch").read_text(
            encoding="utf-8"
        ).lower()
        for forbidden in ("mavros", "px4ctrl", "arming", "takeoff", "offboard"):
            self.assertNotIn(forbidden, launch)

    def test_dropin_package_has_no_compiled_source_of_its_own(self):
        extensions = {path.suffix for path in PACKAGE_ROOT.rglob("*") if path.is_file()}
        self.assertFalse(extensions.intersection({".cpp", ".cc", ".cxx", ".h", ".hpp"}))

    def test_all_launch_files_are_well_formed_xml(self):
        for launch_path in (PACKAGE_ROOT / "launch").glob("*.launch"):
            ET.parse(launch_path)

    def test_diff_planner_include_receives_only_supported_arguments(self):
        workspace_src = PACKAGE_ROOT.parents[1]
        include_path = (
            workspace_src
            / "diff_planner"
            / "plan_manage"
            / "launch"
            / "include"
            / "advanced_param_exp.xml"
        )
        supported = {
            node.attrib["name"]
            for node in ET.parse(include_path).getroot().findall("arg")
        }
        stack_root = ET.parse(
            PACKAGE_ROOT / "launch" / "vla_fastlio_diff_preview_stack.launch"
        ).getroot()
        include = next(
            node
            for node in stack_root.iter("include")
            if "advanced_param_exp.xml" in node.attrib.get("file", "")
        )
        supplied = {node.attrib["name"] for node in include.findall("arg")}
        self.assertFalse(supplied - supported)


if __name__ == "__main__":
    unittest.main()
