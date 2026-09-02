# 模块 9：真机增量 integration 包

## 部署单元

真机只需要复制此目录：

```text
src/integration/vla_diff_bridge/
```

目标机载工作空间示例：

```text
<onboard_diff_planner_ws>/
├── src/
│   ├── diff_planner/...
│   ├── Utils/quadrotor_msgs/...
│   └── integration/
│       └── vla_diff_bridge/
├── build/
└── devel/
```

该包不替换、不覆盖 Diff-Planner、FAST-LIO、EKF、相机驱动、MAVROS 或控制器源码。主机 FastAPI/OpenVLA/π0.5 代码也不放入机载 catkin 工作空间。

## 为什么不改 Diff-Planner 核心

原版 Diff-Planner 使用绝对话题：

- FSM 输入 `/goal`
- traj_server yaw 输入 `/planning/yaw`
- traj_server 输出 `/position_cmd`

`vla_fastlio_diff_preview_stack.launch` 在节点作用域内进行绝对名称 remap：

```text
/goal          -> /vla/preview_goal
/planning/yaw  -> /vla/preview_yaw
/position_cmd  -> /vla/optimized_trajectory_preview
```

因此真机适配无需修改 `diff_replan_fsm.cpp` 和 `traj_server.cpp`。机载电脑以后更新 Diff-Planner 时，只需保持上述原始接口存在。

## catkin 依赖

包清单声明 ROS1 依赖：`rospy`、`roslaunch`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`std_msgs`、`tf`、`tf2_ros`、`cv_bridge`、`quadrotor_msgs` 和运行时 `diff_planner`。

飞机充满后，先只读核对目标工作空间确实包含这些包。未来预定的增量编译命令如下，本轮没有执行：

```bash
cd <onboard_diff_planner_ws>
catkin build vla_diff_bridge
source devel/setup.bash
rospack find vla_diff_bridge
```

如果机载工作空间使用 `catkin_make`，则它不能真正只编译一个包；届时优先确认现有工程使用 catkin_tools 还是 catkin_make，再选择命令。

## 实机编译前必须采集的只读信息

```bash
rostopic type /ekf/ekf_odom
rostopic type /laserMapping/cloud_registered
rostopic type /camera/color/image_raw/compressed
rostopic type /camera/color/camera_info
rostopic echo -n 1 /ekf/ekf_odom/header
rostopic echo -n 1 /camera/color/camera_info
rosrun tf tf_echo base_link camera_color_optical_frame
```

需要记录：实际话题名、消息类型、world/body/camera frame、图像分辨率、K/D、`T_body_camera`、时间戳来源和各话题频率。所有数值确认后才生成 calibration ID，并将 `calibration_validated` 改为 `true`。

## 一体化 launch 的失败关闭设计

`vla_fastlio_diff_preview_stack.launch` 的三个运行组默认均为 `false`：

- `start_diff_planner_preview`
- `start_network_bridge`
- `start_observation_uplink`

网络 token 与 calibration ID 默认为 `REQUIRED`。即使有人直接执行 launch，也不会启动节点；即使只打开网络桥，旧控制输出仍强制 `live_publish_enabled=false`，并被重映射到 `/vla/disabled/*`。

## 充满电后的分阶段验证顺序

1. 只做工作空间、依赖和 Python/ROS 包发现检查。
2. 只启动观测上行，核对图像、FAST-LIO 状态、CameraInfo 与 TF，不启动 Diff-Planner。
3. 只启动网络桥，发送协议测试消息，确认 ACK 为 preview。
4. 启动 Diff-Planner 预览，确认优化结果只出现在 `/vla/optimized_trajectory_preview`。
5. 用 `rostopic info /setpoints_cmd` 确认本 integration 包及预览 traj_server 均不是 publisher。

以上阶段均不允许连接解锁、起飞、OFFBOARD、控制器或执行机构流程。

## 后续实机阶段待定项

- 机载实际工作空间绝对路径与构建工具。
- FAST-LIO 原始 odom 是否直接为 `world -> base_link`，还是必须继续使用 `/ekf/ekf_odom`。
- 相机是否提供 compressed JPEG；若只有 raw Image，需把 launch 切换到 raw transport。
- 相机和 FAST-LIO 是否共用 ROS wall clock，能否满足 80 ms 同步阈值。
- 点云、odom 和 Diff-Planner 地图的 world 原点是否一致。
