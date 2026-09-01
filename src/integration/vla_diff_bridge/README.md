# vla_diff_bridge

ROS1/catkin 增量集成包，用于把机载 FAST-LIO、RGB CameraInfo/TF、主机 K 帧 VLA 推理与 Diff-Planner 规划预览连接起来。

此目录是实机部署的最小代码单元。将整个 `vla_diff_bridge` 目录放入机载 Diff-Planner catkin 工作空间的 `src/integration/`（或 `src/`）即可；不需要覆盖 Diff-Planner 核心源码。

## 安全默认值

- `live_publish_enabled=false`
- `preview_only_mode=true`，schema v1 控制命令直接拒绝
- `planning_preview_enabled=false`
- 一体化 launch 的三个 `start_*` 参数均为 `false`
- `calibration_validated=false`
- token 与 calibration ID 默认都是 `REQUIRED`
- 优化输出只允许 `/vla/optimized_trajectory_preview`

## 包内模块

- `scripts/onboard_observation_uplink_node.py`：只读订阅图像、CameraInfo、TF、odom 和预览轨迹，并通过 HTTP 上行。
- `scripts/vla_diff_bridge_node.py`：接收主机 schema v2 规划预览并发布隔离 goal/yaw。
- `src/vla_diff_bridge/protocol.py`：纯 Python 协议和时效/字段校验。
- `launch/vla_fastlio_diff_preview_stack.launch`：在不修改 Diff-Planner 核心的前提下组装规划预览链路。
- `config/`：失败关闭的默认参数。
- `docs/09_REAL_DEPLOYMENT_PACKAGE.md`：充电完成后实机编译、只读核查和接线顺序。

本阶段不要启动一体化 launch；等待飞机充满并完成现场话题、内外参与 TF 核查后再进行机载编译。
