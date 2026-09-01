# 模块 3：ROS launch 与 YAML 配置

对应文件：`launch/vla_diff_bridge.launch`、`config/vla_bridge.yaml`、`CMakeLists.txt`、`package.xml`、`setup.py`。

## launch 的职责

launch 加载安全参数并完成话题映射。可覆盖参数：

- `auth_token`：必填；默认 `REQUIRED` 会让节点拒绝启动。
- `live_publish_enabled`：默认 `false`。
- `allowed_host_ip`：默认 `192.168.14.250`。
- `odom_topic`：默认 `/ekf/ekf_odom`。
- `goal_topic`、`yaw_topic`、`hover_stop_topic` 和 `mandatory_stop_topic`：默认对接当前 Diff-Planner 单机实飞配置。

## YAML 安全边界

- `max_ttl_ms: 2000`：机载允许的 TTL 硬上限；主机实际默认使用 500 ms。
- `max_goal_step_m: 1.0`：单条 VLA 命令允许的最大三维目标位移。
- `min_goal_z_m/max_goal_z_m: 0.1/2.0`：任务坐标系高度边界。
- `max_odom_age_ms: 250`：做目标距离检查和悬停时允许的最大 odom 年龄。
- `watchdog_timeout_ms: 1000`：命令流断开后的悬停触发时间。
- `max_message_bytes: 65536`：单条网络消息上限。

## catkin 集成

这是独立包，依赖 `rospy`、`geometry_msgs`、`nav_msgs`、`std_msgs`、`tf` 和仓库已有的 `quadrotor_msgs`。`catkin_python_setup()` 安装纯 Python 协议包，`catkin_install_python()` 安装 ROS 节点。没有增加 pip 或 apt 第三方网络库，降低机载环境部署负担。

## 部署前调整

不能直接把示例边界用于飞行。应先根据训练数据 action 分布统计 P99 位移，再把 `max_goal_step_m` 设置为略高于正常 P99、低于危险跃迁的值。高度边界需要与 Diff-Planner virtual wall、场地净空和飞控限制取最严格交集。
