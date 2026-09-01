# Branch profile: real-deployment

当前分支面向真实机载接口的代码适配，但只允许规划预览：FAST-LIO/CameraInfo/TF 上行、主机每 K 帧 VLA 推理、目标坐标转换、Diff-Planner 隔离优化与本地状态回传。

禁止在本阶段把 `/vla/optimized_trajectory_preview` 重映射到 `/setpoints_cmd`，并保持 `live_publish_enabled=false`。`simulation-validated` 分支用于保留此前仿真闭环基线。

真机代码必须以独立 `src/integration/vla_diff_bridge` catkin 包增量加入机载工作空间，不依赖覆盖 Diff-Planner 核心 C++ 文件。
