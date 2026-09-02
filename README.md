# Diff-Planner Simulation Integration Fork

This branch records the Diff-Planner side of the validated UAV simulation
workflow. It is based on Diff-Planner and EGO-Planner-v2 and adds the ROS 1
`vla_diff_bridge` package used to preview remote VLA goals against onboard
localization and planning interfaces.

## Purpose

The simulation architecture separates low-rate semantic reasoning from
high-rate local planning:

- OpenVLA produces a bounded body-frame motion proposal from an RGB frame,
  natural-language instruction, and vehicle state;
- the host converts the proposal to a short world-frame target;
- `vla_diff_bridge` validates authentication, sequence, TTL, calibration,
  coordinate frames, odometry age, and goal bounds; and
- Diff-Planner uses local map and odometry data to optimize a collision-aware
  trajectory.

The checked-in configuration is preview-only. It does not arm PX4, switch flight
mode, take off, or publish actuator commands.

## Integration package

The incremental package is located at
`src/integration/vla_diff_bridge` and includes:

- a TCP/NDJSON command bridge;
- an HTTP observation uplink for RGB, CameraInfo, odometry, TF, and planner
  feedback;
- isolated preview goal and yaw topics;
- coordinate and protocol tests; and
- launch wrappers for FAST-LIO/EKF and Diff-Planner preview validation.

## Build

```bash
git clone --recursive <repository-url>
cd Diff-Planner
catkin_make
source devel/setup.bash
```

The validated deployment path uses Ubuntu 20.04 and ROS Noetic. Revalidate all
dependencies when using another platform.

## Safe preview configuration

Provide network addresses and credentials through a private environment file:

```bash
export VLA_BACKEND_URL=http://<HOST_ONBOARD_IP>:8080
export VLA_HOST_IP=<HOST_ONBOARD_IP>
export VLA_BRIDGE_TOKEN=<RANDOM_BRIDGE_TOKEN>
export VLA_OBSERVATION_TOKEN=<RANDOM_OBSERVATION_TOKEN>
export VLA_CALIBRATION_ID=<VALIDATED_CALIBRATION_ID>
export VLA_CALIBRATION_VALIDATED=I_VALIDATED_CAMERA_INFO_AND_TF
```

Tracked defaults use loopback or `REQUIRED` placeholders. Never commit actual
addresses, tokens, camera serial numbers, or machine-specific configuration.

After providing odometry, point cloud, and camera topics, review and run:

```bash
./sh_files/run_vla_fastlio_diff_preview.sh
```

The wrapper launches an isolated planning preview and contains no arming,
takeoff, landing, PX4 mode-switch, or actuator command.

## Coordinate contract

- world: right-handed local frame with Z up;
- body: ROS FLU, with X forward, Y left, and Z up;
- optical camera: X right, Y down, and Z forward;
- policy action: `[dx_body, dy_body, dz_body, d_yaw]`;
- units: metres and radians.

The host performs body-to-world conversion. The onboard bridge independently
checks target distance, altitude, frame names, calibration identity, observation
freshness, and odometry freshness before publishing a preview.

## Documentation

- `docs/VLA_BRIDGE_MODIFICATIONS.md`
- `src/integration/vla_diff_bridge/docs/01_ARCHITECTURE.md`
- `src/integration/vla_diff_bridge/docs/02_PROTOCOL.md`
- `src/integration/vla_diff_bridge/docs/03_LAUNCH_AND_CONFIG.md`
- `src/integration/vla_diff_bridge/docs/04_SAFETY_AND_TESTING.md`

## Upstream and license

This fork retains the work of the Diff-Planner maintainers and derives from
[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2). It is
distributed under the GNU General Public License v3.0; see `LICENSE`.
