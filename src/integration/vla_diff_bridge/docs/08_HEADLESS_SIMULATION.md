# 模块 8：VLA Headless 仿真

对应文件：`diff_planner/plan_manage/launch/sim/run_sim_vla_headless.launch`。

该 launch 用于笔记本和 CI 风格的闭环验证，保留 Diff-Planner 原仿真的随机地图、`poscmd_2_odom` 运动学无人机、局部点云感知、轨迹规划和轨迹服务器，但不启动 RViz、multipoint、MAVROS、PX4 或实机控制器。

仿真关键话题：

- odom：`/drone_0_visual_slam/odom`
- 规划目标：`/goal`
- yaw：`/planning/yaw`
- 规划输出：`/drone_0_planning/pos_cmd`
- 仿真点云：`/drone_0_pcl_render_node/cloud`
- 可恢复停止：`/vla_hover_stop_to_planner`

启动方式：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch diff_planner run_sim_vla_headless.launch
```

桥接节点另行启动，仿真 odom 参数必须使用：

```bash
roslaunch vla_diff_bridge vla_diff_bridge.launch \
  auth_token:='测试密钥' \
  allowed_host_ip:='发送端 IP' \
  live_publish_enabled:=true \
  odom_topic:=/drone_0_visual_slam/odom
```

当前仓库将 `local_sensing/CMakeLists.txt` 中的 `ENABLE_CUDA` 设为 `false`，因此该启动方式只发布点云，不发布 RGB、`depth` 或 `colordepth`。它能验证轨迹控制闭环，但不能验证“仿真相机图像 → VLA”的视觉闭环。

CUDA 渲染实现会提供 `depth` 和伪彩 `colordepth`，但笔记本当前没有 `nvcc`，未编译该路径；而且伪彩深度也不等同于 `uav-flow-real` 的真实 RGB 分布。视觉闭环应后续接入 Gazebo/Isaac/UE 相机或真实机载 RGB 话题后单独验收。
