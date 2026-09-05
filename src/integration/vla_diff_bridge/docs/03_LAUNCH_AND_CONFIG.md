# 模块 3：ROS launch 与 YAML 配置

对应文件：`launch/vla_diff_bridge.launch`、`config/vla_bridge.yaml`、`CMakeLists.txt`、`package.xml`、`setup.py`。

## launch 的职责

launch 加载安全参数并完成话题映射。可覆盖参数：

- `auth_token`：必填；默认 `REQUIRED` 会让节点拒绝启动。
- `live_publish_enabled`：默认 `false`。
- `allowed_host_ip`：必须通过私有部署配置传入 Windows 主机在机载网段的地址；仓库默认值仅为 loopback。
- `odom_topic`：默认 `/ekf/ekf_odom`。
- `goal_topic`、`yaw_topic`、`hover_stop_topic` 和 `mandatory_stop_topic`：默认对接当前 Diff-Planner 单机实飞配置。
- `camera_frame`：默认 `vla_usb_camera_optical_frame`。
- `camera_info_topic` / `image_compressed_topic`：默认
  `/vla_usb_camera/camera_info` 和 `/vla_usb_camera/image_raw/compressed`。

`launch/vla_usb_camera.launch` 仅启动 USB 图像源。默认选用 KS2A418 的稳定 by-id 路径、
640×480@30 FPS、MJPEG；可通过 launch 参数覆盖。它不会发布虚假的机体外参，
`base_link -> vla_usb_camera_optical_frame` 必须来自经过确认的真实标定。

## YAML 安全边界

- `max_ttl_ms: 2000`：机载允许的 TTL 硬上限；主机实际默认使用 500 ms。
- `action_chunk_lookahead_enabled: true`：在机载端启用动作块世界系对齐和过期航点剔除。
- `max_action_chunk_steps: 10`：允许接收的 π0.5 动作块最大长度。
- `action_chunk_sample_count: 8`：10 步中选取 8 点，可设为 6；不足该数量时保留全部点。
  启动脚本支持 `VLA_ACTION_CHUNK_SAMPLE_COUNT=6` 或 `8`。目标逐个发布，后续点由最新 odom
  进度驱动，不一次发布全部点；通信 watchdog 仍为 1000 ms，不会被本地推进延长。
- `action_chunk_lookahead_distance_m: 0.10`：在实际轨迹进度之前额外保留的前视距离。
- `action_chunk_max_cross_track_m: 1.0`：实际位置偏离预测轨迹走廊的拒绝阈值。
- `max_goal_step_m: 1.0`：单条 VLA 命令允许的最大三维目标位移。
- `max_operator_step_m: 2.0`：人工六向移动单次距离上限，独立于 VLA 限制；仍检查目标高度边界。
- `~operator_yaw_hold` 映射到 `/drone_0_traj_server/operator_yaw_hold`，传递六向平移的固定航向。需要配套 `traj_server_heading_hold`；无订阅者则拒绝移动。后续更新的普通航向命令解除保持，过期/无时间戳提示不会覆盖已锁定航向。
- `min_goal_z_m/max_goal_z_m: 0.1/2.0`：任务坐标系高度边界。
- `max_odom_age_ms: 250`：做目标距离检查和悬停时允许的最大 odom 年龄。
- `watchdog_timeout_ms: 1000`：命令流断开后的悬停触发时间。
- `max_message_bytes: 65536`：单条网络消息上限。

## catkin 集成

这是独立包，依赖 `rospy`、`geometry_msgs`、`nav_msgs`、`std_msgs`、`tf` 和仓库已有的 `quadrotor_msgs`。`catkin_python_setup()` 安装纯 Python 协议包，`catkin_install_python()` 安装 ROS 节点。没有增加 pip 或 apt 第三方网络库，降低机载环境部署负担。

## 部署前调整

不能直接把示例边界用于飞行。应先根据训练数据 action 分布统计 P99 位移，再把 `max_goal_step_m` 设置为略高于正常 P99、低于危险跃迁的值。高度边界需要与 Diff-Planner virtual wall、场地净空和飞控限制取最严格交集。
