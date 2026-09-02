#!/usr/bin/env bash
set -euo pipefail

# Static/build verification only. This script never starts roscore, roslaunch,
# MAVROS, PX4, a controller, a planner node, or a publisher.
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if pgrep -af 'roscore|rosmaster|roslaunch|mavros|px4ctrl|traj_server|offboard' >/tmp/vla_control_processes.txt; then
  echo "Refusing verification while ROS/control processes are active:" >&2
  sed -n '1,40p' /tmp/vla_control_processes.txt >&2
  exit 20
fi

python3 -m unittest discover -s "${package_dir}/test" -p 'test_*.py'
python3 -m compileall -q "${package_dir}/src" "${package_dir}/scripts"

if grep -RInE '/setpoints_cmd|mavros|px4ctrl|arming|takeoff|offboard' "${package_dir}/launch"; then
  echo "Forbidden flight-control reference found in integration launch files" >&2
  exit 21
fi

echo "PASS: package tests and static no-motion audit completed; no runtime was started"
