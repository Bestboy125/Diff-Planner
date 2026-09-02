# 模块 2：ROS 桥接节点

对应代码：`scripts/vla_diff_bridge_node.py`。

## ROS 输入输出

- 订阅 `~odom`，launch 默认映射到 `/ekf/ekf_odom`。
- 发布 `~goal`，映射到 `/goal`，消息类型 `geometry_msgs/PoseStamped`。
- 发布 `~yaw`，映射到 `/planning/yaw`，消息类型 `quadrotor_msgs/PositionCommand`，使用其 `yaw` 字段。
- 发布 `~hover_stop`，映射到 `/vla_hover_stop_to_planner`，用于 HOLD、COMPLETE 和 watchdog。
- 发布 `~mandatory_stop`，映射到 `/mandatory_stop_to_planner`。
- 发布 `/vla_bridge/status`，消息类型 `std_msgs/String`，内容是 JSON 状态。

## TRACK 处理

节点在持锁状态下检查任务是否冲突、sequence 是否递增、目标坐标系是否一致。随后要求最新 odom 不超过 `max_odom_age_ms`，计算当前位置到 `target_mission` 的三维距离，并检查单步距离和高度边界。通过后先发布 yaw，再发布 `/goal`，Diff-Planner 状态机将新目标交给 `planNextWaypoint()` 进行全局/局部轨迹更新。

## HOLD 与 COMPLETE

两者都不是直接发零速度，也不把当前位置当成零长度普通目标。桥接节点发布 `hover_stop`，由 Diff-Planner 在当前 odom 位置生成满足动力学约束的紧急减速轨迹；速度降至阈值后 FSM 回到 `WAIT_TARGET`。`COMPLETE` 额外终结 bridge mission，防止延迟旧包再次触发运动。

## Watchdog

收到 live `TRACK` 后开始计时。超过 `watchdog_timeout_ms` 没有新命令时：

节点发布一次可恢复 `hover_stop` 并锁存 watchdog，直到收到下一条合法 TRACK。停止轨迹使用规划器自身的 odom，不依赖桥接节点缓存的 odom 新鲜度；TRACK 的目标边界检查仍要求桥接节点 odom 新鲜。

这套 watchdog 监控的是“主机轨迹意图流”，不替代 PX4 控制器、遥控器急停或飞控 failsafe。

## 并发模型

TCP 服务每个连接使用独立线程；所有 mission、sequence、odom 和 watchdog 状态都由同一个可重入锁保护。ROS 回调线程只更新最新 odom，网络线程负责命令校验和发布。单条消息上限默认为 64 KiB，避免网络输入造成无界内存使用。

## 安全锁

`live_publish_enabled=false` 时仍完成协议和目标安全检查，但返回 `preview`，不发布控制话题。这是开发机、网络联调和首次上机检查应使用的模式。
