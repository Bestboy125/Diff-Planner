# FAST-LIO → K-frame VLA → Diff-Planner 代码适配说明

## 安全边界

本次新增的是“感知上行 + 规划预览”链路，不是飞行控制链路。默认配置不会解锁、起飞、切换飞控模式，也不会向 `/setpoints_cmd`、MAVROS、PX4 或 px4ctrl 发布消息。`CONTROL_OUTPUT_ENABLED` 和 `live_publish_enabled` 继续保持 `false`。

Diff-Planner 的预览输出固定隔离到 `/vla/optimized_trajectory_preview`。该话题只被观测上行节点读取并返回网页，不能连接到控制器。

## 数据流

1. `onboard_observation_uplink_node.py` 订阅相机 JPEG、`CameraInfo`、FAST-LIO/EKF `Odometry`、TF 外参和隔离的规划器预览输出。
2. 图像回调只向长度为 2 的队列写入；网络拥塞时丢弃最旧帧，不阻塞相机流。
3. 主机 `ObservationPipeline` 校验时间同步、消息序号、坐标系、内参、外参、标定 ID 和 JPEG 完整性。
4. 每累计 K 个新图像序号选取最新一帧，调用当前任务指定的 OpenVLA 或 π0.5。
5. VLA 输出的 FLU 机体系增量按 FAST-LIO yaw 转到 ENU `world` 目标。
6. 主机发送 schema v2 `planning_preview`。机载桥只把目标送到 `/vla/preview_goal` 和 `/vla/preview_yaw`。
7. 预览模式 Diff-Planner 使用点云避障和轨迹优化，结果只写入 `/vla/optimized_trajectory_preview`，再随下一帧状态上行。

## 坐标系约定

- 世界系：`world`，ROS ENU，x/y/z 分别为前（或东）/左（或北）/上。
- 机体系：`base_link`，ROS FLU，x 前、y 左、z 上。
- 相机系：`camera_color_optical_frame`，ROS optical，x 右、y 下、z 前。
- `body_from_camera` 表示 `T_base_link_camera_optical`，即把相机系点变换到机体系。
- VLA 动作语义固定为 `[dx_body, dy_body, dz_body, d_yaw]`，单位 `[m,m,m,rad]`。

主机变换公式：

```text
x_world = x + cos(yaw)*dx_body - sin(yaw)*dy_body
y_world = y + sin(yaw)*dx_body + cos(yaw)*dy_body
z_world = z + dz_body
```

相机 optical 外参不参与动作向量旋转；它用于校验图像来源和后续视觉几何处理。这样不会把“相机向右”误当成“机体向前”。

## 内参与外参

内参来自 `sensor_msgs/CameraInfo` 的宽高、畸变模型、K(3×3)、D。外参来自 TF 查询 `base_link <- camera_color_optical_frame`。

真实值未硬编码。以下任一条件不满足，主机会拒绝该帧且不会调用 VLA：

- `calibration_validated=true`；
- 主机与机载端 `calibration_id` 完全一致；
- CameraInfo 和图像 frame 一致；
- TF 的 parent/child 与配置一致；
- 图像与 odom 时间差不超过默认 80 ms；
- 帧龄不超过默认 750 ms。

在机载环境只读核对：

```bash
rostopic echo -n 1 /camera/color/camera_info
rostopic echo -n 1 /ekf/ekf_odom
rosrun tf tf_echo base_link camera_color_optical_frame
```

核对后生成带日期/设备序列号的 ID，例如 `front-rgb-SN1234-2026-09-01`，同时配置主机 `EXPECTED_CALIBRATION_ID` 与机载 `VLA_CALIBRATION_ID`。

## 主机模块改动

- `ground_station/backend/app/schemas.py`：新增观测、CameraInfo、TF、局部状态、规划预览数据模型。
- `observation_pipeline.py`：K 帧计数、时间/标定/坐标检查、FLU→ENU 转换、异步 VLA 推理和预览发送。
- `onboard_bridge.py`：新增不可作为控制命令解释的 schema v2 `planning_preview`。
- `main.py`：新增观测 POST 和最新图像 GET；状态 WebSocket 返回 K 帧进度、FAST-LIO 状态和优化预览。
- 前端：显示实时图像、K 帧计数、坐标系、标定状态和只读位置/yaw。

主机必须配置：

```powershell
$env:ONBOARD_OBSERVATION_TOKEN="<random-token>"
$env:ONBOARD_BRIDGE_TOKEN="<different-random-token>"
$env:OBSERVATION_K_FRAMES="5"
$env:EXPECTED_WORLD_FRAME="world"
$env:EXPECTED_BODY_FRAME="base_link"
$env:EXPECTED_CAMERA_FRAME="camera_color_optical_frame"
$env:EXPECTED_CALIBRATION_ID="front-rgb-SN1234-2026-09-01"
$env:CONTROL_OUTPUT_ENABLED="false"
```

## 机载与 Diff-Planner 模块改动

- `onboard_observation_uplink_node.py`：只读订阅与非阻塞 HTTP 上行；代码中没有控制 Publisher。
- `vla_diff_bridge_node.py`：区分旧控制协议和新预览协议；预览只发布到独立话题。
- `protocol.py`：schema v2 强制携带观测序号、时间、world/body/camera frame 和 calibration ID。
- Diff-Planner 核心源码保持不变；集成包 launch 对原始绝对话题做作用域内 remap。
- `vla_fastlio_diff_preview_stack.launch`：在 integration 包内组装规划器、网络桥和观测上行，三个运行组默认关闭。
- `scripts/run_vla_fastlio_diff_preview.sh`：包内启动包装器，只组装上述预览链路，不包含飞控和运动步骤。
- `docs/09_REAL_DEPLOYMENT_PACKAGE.md`：增量复制、未来单包编译和充满电后的分阶段核查流程。

## 运行前静态检查

当前阶段不要执行启动脚本。部署后先在未解锁、螺旋桨拆除或硬件隔离条件下检查 ROS 图：

```bash
rostopic info /vla/optimized_trajectory_preview
rostopic info /setpoints_cmd
```

前者应只有预览链路；后者不能出现本适配器或预览 traj_server 作为 publisher。
