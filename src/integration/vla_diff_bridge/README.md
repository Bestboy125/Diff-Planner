# vla_diff_bridge

ROS1/catkin 增量集成包，用于把机载 FAST-LIO、RGB CameraInfo/TF、主机 K 帧 VLA 推理与 Diff-Planner 规划预览连接起来。

此目录是实机部署的最小代码单元。将整个 `vla_diff_bridge` 目录放入机载 Diff-Planner catkin 工作空间的 `src/integration/`（或 `src/`）即可；不需要覆盖 Diff-Planner 核心源码。

## 安全默认值

- `live_publish_enabled=false`
- `preview_only_mode=true`，schema v1 控制命令直接拒绝
- `planning_preview_enabled=false`
- 一体化 launch 的所有 `start_*` 参数均为 `false`
- `calibration_validated=false`
- token 与 calibration ID 默认都是 `REQUIRED`
- 优化输出只允许 `/vla/optimized_trajectory_preview`

## 包内模块

可显式使用 `observation_mode=image_odom`（启动脚本环境变量 `VLA_OBSERVATION_MODE=image_odom`），只上报图像和飞机里程计，不查询或伪造相机外参。此模式发送 `body_from_camera=null`、`calibration_validated=false`，需要主机同样启用；仍检查 CameraInfo、图像/odom 对齐及 frame。公共默认 `calibrated` 继续要求真实相机 TF。两种模式均不移除 Diff-Planner 对实时 odom 的依赖。

- `scripts/onboard_observation_uplink_node.py`：只读订阅图像、CameraInfo、TF、odom 和预览轨迹，并通过 HTTP 上行。
- `launch/vla_usb_camera.launch`：启动 KINGSEN 单目 USB 相机，默认输出 `/vla_usb_camera/*`，不包含任何飞控节点。
- `scripts/vla_diff_bridge_node.py`：接收完整 VLA 动作块，在机载端做世界系累计，采样 6/8 个航点，并基于最新 odom 持续剔除过期航点；按进度逐个发布 goal/yaw，预览模式使用隔离话题。不是只发送终点，也不是向 Diff-Planner 一次提交整条路径进行联合优化。
- `src/vla_diff_bridge/protocol.py`：纯 Python 协议、动作块几何和时效/字段校验。
- `launch/vla_fastlio_diff_preview_stack.launch`：在不修改 Diff-Planner 核心的前提下组装规划预览链路。
- `config/`：失败关闭的默认参数。
- `docs/09_REAL_DEPLOYMENT_PACKAGE.md`：充电完成后实机编译、只读核查和接线顺序。

USB 相机默认使用稳定设备路径
`/dev/v4l/by-id/usb-KINGSEN_KS2A418-2.0-video-index0`、640×480@30 FPS 和 MJPEG。
`/home/nv/.ros/camera_info/head_camera.yaml` 只能作为已发现的内参候选；必须确认它确实属于
当前相机和分辨率，并重新验证 `base_link -> vla_usb_camera_optical_frame` 安装外参，旧 D435
calibration ID 不得复用。
