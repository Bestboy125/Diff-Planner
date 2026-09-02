# 笔记本 VLA–Diff-Planner 仿真验证报告

验证日期：2026-08-27  
笔记本：`<SIM_USER>@<SIM_HOST>`
远端分支：`vla-sim-integration`

## 1. 验证结论

以下链路已经实际跑通：

```text
Windows 主机真实 OpenVLA 推理
  -> TCP/NDJSON 局域网轨迹命令
  -> 笔记本 vla_diff_bridge
  -> /goal + /planning/yaw
  -> Diff-Planner 在线规划/重规划
  -> /drone_0_planning/pos_cmd
  -> poscmd_2_odom 运动学仿真
  -> /drone_0_visual_slam/odom
  -> COMPLETE 可恢复悬停并回到 WAIT_TARGET
```

编译、协议、安全状态机、局域网传输、真实 OpenVLA 输出接入和最终停止均通过。当前仿真没有 RGB 相机发布器，因此尚未验证“仿真 RGB 图像流 → VLA”的视觉闭环，也未连接 PX4、MAVROS 或真实飞控。

## 2. 环境与代码同步

- 系统：Ubuntu 20.04.6 / Kylin V10 SP1，ROS Noetic。
- GPU：NVIDIA RTX 3060 Laptop 6 GB。
- 远端仓库：`/home/oem/vla-project/Diff-Planner`。
- 在现有仓库上创建分支 `vla-sim-integration`，保留原有 Git 文件和未跟踪的 `build/`、`devel/`、`.vscode/`。
- 只同步 VLA 桥接、FSM 衔接、启动文件、文档和必要构建修复，没有覆盖整仓库。
- 用 `catkin_make -DCMAKE_BUILD_TYPE=Release -j2` 完整编译到 100%。

原 `src/user_command/multipoint/CMakeLists.txt` 没有查找 Eigen，导致 `Eigen/Dense` 不可见。本次加入 `find_package(Eigen3 REQUIRED)` 和 `${EIGEN3_INCLUDE_DIRS}` 后编译通过。

## 3. 仿真配置

使用 `run_sim_vla_headless.launch`，只启动 Diff-Planner 软件仿真所需节点：随机地图、点云渲染、里程计可视化数据、Diff-Planner、轨迹服务器和 `poscmd_2_odom`。未启动 RViz、multipoint、MAVROS、PX4 或实机控制器。

专用 headless 地图使用 40 个柱状障碍和 20 个圆形障碍，以降低笔记本 CPU 负载并保持软件验收稳定。桥接服务监听 TCP 50051，启用 token、发送端 IP 白名单、序列号、TTL、有限数值、步长和高度约束。

Windows 主机通过路由连接仿真机时，仿真机看到的来源地址可能不同于操作网页所在接口，
因此仿真白名单必须按运行时实际来源地址配置，不能把本地实测地址写入仓库。

## 4. 验证结果

### 4.1 编译与自动化测试

- 远端 Catkin 工作空间：编译成功。
- 远端桥接协议测试：5 项通过。
- 主机 FastAPI 后端测试：11 项通过。
- React/Vite 前端：生产构建成功。
- FastAPI `/api/health`：后端在线、OpenVLA 在线、安全锁开启；π0.5 本轮未启动。

### 4.2 协议和安全行为

- 连续 TRACK：接受并触发规划/重规划。
- HOLD 后恢复 TRACK：通过。
- 1 秒无命令：触发 watchdog 可恢复悬停。
- COMPLETE：停止并回到 `WAIT_TARGET`。
- COMPLETE 后同任务 TRACK：按终态规则拒绝。
- 错误 token：拒绝。
- 错误来源 IP：拒绝并返回实际来源地址。
- 过期 TTL：拒绝。
- TCP 往返时延约 9.5 ms。

两台机器的 Unix 时间存在约 812 ms 偏差，500 ms TTL 会把正常命令判断为过期。本次纯仿真使用 2000 ms TTL。实机部署不应长期放宽 TTL，应先用 chrony/PTP 校时，再恢复 500 ms 或根据实测时延确定更严格值。

### 4.3 预设轨迹闭环

主机发送 6 个 TRACK 和 1 个 COMPLETE，所有 ACK 均为 `accepted`。仿真无人机最终到达 `(-9.5, 0, 1)`，线速度和角速度均为 0。日志确认状态变化：

```text
WAIT_TARGET -> SEQUENTIAL_START -> EXEC_TRAJ
EXEC_TRAJ <-> REPLAN_TRAJ
EXEC_TRAJ -> WAIT_TARGET
WAIT_TARGET -> EMERGENCY_STOP -> WAIT_TARGET
```

### 4.4 真实 OpenVLA 输出闭环

主机加载训练后的 OpenVLA 3-epoch 权重，输入 `UAV-Flow-Eval/debug.jpg`、指令 `Fly forward while keeping a safe distance from obstacles.` 和当前状态 `[-9.5, 0, 1, 0 deg]`。

模型输出：

```text
action_local_delta = [0.0005088888, 0.0002282505, -0.0001587937, 0.0003670572]
target_mission     = [-9.4994911112, 0.0002282505, 0.9998412063, 0.0003670572]
```

动作通过有限值、1 m 最大步长和 0.1–2.0 m 高度预检。4 个 TRACK 和 1 个 COMPLETE 均被笔记本接受。最终里程计为：

```text
position = [-9.4994907379, 0.0002282505, 0.9998412132]
linear velocity  = [0, 0, 0]
angular velocity = [0, 0, 0]
```

这证明真实模型输出的坐标语义、桥接协议、Diff-Planner 输入和仿真执行结果一致。

## 5. 尚未覆盖的范围

1. 当前 CPU 点云渲染节点只发布 `/drone_0_pcl_render_node/cloud`，没有 RGB、`depth` 或 `colordepth`；笔记本也没有 `nvcc`，无法直接编译仓库的 CUDA 深度渲染路径。
2. 本轮真实 OpenVLA 使用数据集 RGB 图片，而不是与仿真位姿同步的相机帧，因此证明的是“真实模型输出 → 控制闭环”，不是视觉反馈闭环。
3. π0.5 本轮没有启动；其协议适配已有主机端测试，但仍需用同一仿真流程做一次单模型验收。
4. 没有验证飞控解锁、PX4 offboard、MAVROS、真实 SLAM、真实网络丢包或急停硬件。

## 6. 后续验收顺序

1. 接入 Gazebo/Isaac/UE 的同步 RGB 相机和时间戳，先保持控制输出锁定做图像流压力测试。
2. 在仿真里分别运行 OpenVLA 与 π0.5 的连续 2 Hz 推理，记录端到端帧龄、推理延迟、命令丢包和 watchdog 次数。
3. 校准两机时钟，把命令 TTL 恢复到安全基线。
4. 做软件在环故障注入：断图像、断 Wi-Fi、模型超时、乱序、重复序列、异常数值和规划失败。
5. 通过上述验收后，再进入有桨保护/拆桨条件下的 PX4/MAVROS 硬件在环测试。

## 7. 可复现命令

笔记本：

```bash
cd /home/oem/vla-project/Diff-Planner
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch diff_planner run_sim_vla_headless.launch

roslaunch vla_diff_bridge vla_diff_bridge.launch \
  auth_token:='<test-token>' \
  allowed_host_ip:='<HOST_SIMULATION_IP>' \
  live_publish_enabled:=true \
  odom_topic:=/drone_0_visual_slam/odom
```

主机的可复现工具：

- `ground_station/backend/tools/sim_lan_smoke_test.py`：预设轨迹协议闭环。
- `ground_station/backend/tools/openvla_sim_closed_loop.py`：真实 OpenVLA 单步安全预检与仿真闭环；默认 dry-run，只有显式 `--live` 才发送控制命令。
