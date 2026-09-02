# VLA 与 Diff-Planner 衔接改动总览

本次改动没有把网络代码写进轨迹优化器，也没有修改 `planner_manager`、`traj_opt` 或 PX4 控制器。新增的 `vla_diff_bridge` 是独立 ROS1 包，负责把主机后端的 VLA 推理结果变成 Diff-Planner 已支持的目标点和航向输入。

数据链路如下：

`OpenVLA / π0.5 -> ground_station 后端 -> TCP/NDJSON:50051 -> vla_diff_bridge -> /vla/preview_goal + /vla/preview_yaw -> Diff-Planner -> /vla/optimized_trajectory_preview`

安全状态链路如下：

真实部署分支启用 `preview_only_mode=true`，拒绝 schema v1 的 TRACK/HOLD/COMPLETE/EMERGENCY_STOP 控制协议，只接受 schema v2 `PLAN_PREVIEW`。

默认参数 `live_publish_enabled: false`。在这个状态下网络命令会完成格式、时间和安全检查，但不会发布 `/goal`、`/planning/yaw` 或强制停止消息。

## 改动模块索引

1. [协议校验模块](../src/integration/vla_diff_bridge/docs/01_PROTOCOL.md)：主机与机载端的字段、单位、TTL、顺序号和 ACK。
2. [ROS 桥接节点](../src/integration/vla_diff_bridge/docs/02_ROS_BRIDGE_NODE.md)：话题映射、目标检查、悬停和 watchdog。
3. [ROS 启动与配置](../src/integration/vla_diff_bridge/docs/03_LAUNCH_AND_CONFIG.md)：launch 参数、YAML 安全边界及现场待填写项。
4. [Shell 启动脚本](../src/integration/vla_diff_bridge/docs/04_START_SCRIPT.md)：包内规划预览入口。
5. [主机后端发送端](../src/integration/vla_diff_bridge/docs/05_HOST_BACKEND.md)：如何把两个模型的共同输出封装并发送。
6. [验证说明](../src/integration/vla_diff_bridge/docs/06_VERIFICATION.md)：Windows 可执行验证、Ubuntu/ROS 待执行验证及实机前门槛。
7. [Diff-Planner FSM 改动](../src/integration/vla_diff_bridge/docs/07_DIFF_PLANNER_FSM.md)：可恢复悬停入口和原 mandatory-stop 标志修复。
8. [Headless 仿真](../src/integration/vla_diff_bridge/docs/08_HEADLESS_SIMULATION.md)：不启动实机模块的笔记本闭环仿真入口。
9. [真机增量包](../src/integration/vla_diff_bridge/docs/09_REAL_DEPLOYMENT_PACKAGE.md)：单包复制、未来增量编译与实机只读核查顺序。

## Diff-Planner 核心保持不变

真实部署不修改 `diff_replan_fsm.cpp`、`traj_server.cpp`、地图、搜索或优化器。integration launch 在节点作用域中把原始 `/goal`、`/planning/yaw`、`/position_cmd` 绝对名称重映射到 `/vla/*` 预览命名空间。早期 FSM hover-stop 改动仅保留在 `simulation-validated` 分支。

## 现场仍需确认

- `VLA_HOST_ONBOARD_IP`：主机在机载局域网中的实际地址，必须通过未提交的部署环境传入。
- `VLA_ODOM_TOPIC`：实际里程计话题，LIO 示例使用 `/ekf/ekf_odom`。
- `world_frame`：当前为 `world`，必须与里程计和地图坐标系一致。
- 高度边界和单步最大距离：当前分别为 `0.1~2.0 m` 与 `1.0 m`，必须根据机体、场地和训练动作尺度重新标定。
- 主机与机载电脑需要 NTP/chrony 对时，否则 500 ms TTL 会造成过期拒绝。
