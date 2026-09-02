# 机载部署记录（无运动验证）

## 安全边界

本轮只允许源码部署、协议测试、XML/Python 静态检查和 catkin 增量编译。没有启动
`roscore`、`roslaunch`、MAVROS、px4ctrl、Diff-Planner 节点或 traj_server，也没有发送
解锁、起飞、Offboard、目标点、速度、姿态或轨迹命令。

## Git 基线

部署前在 `/home/nv/Diff-planner` 建立了本地 Git 基线：

- 基线提交：`e487543 Baseline onboard Diff-Planner before VLA integration`
- 不可变回退分支：`onboard-baseline`
- 部署分支：`real-deployment`
- `build/`、`devel/`、bag、图像与运行产物被 `.gitignore` 排除，但原文件仍保留在磁盘。

## 已核对的机载接口

- ROS：Noetic，catkin_make 工作区。
- Diff-Planner odom：`/ekf/ekf_odom`。
- FAST-LIO 点云：`/laserMapping/cloud_registered`。
- Diff-Planner 目标输入：源码中的绝对话题 `/goal`。
- 实机 traj_server 输出：`run_exp_single_lio.launch` 将 `position_cmd` 映射到
  `/setpoints_cmd`。本包预览实例不使用该 launch，并将自己的输出隔离到
  `/vla/optimized_trajectory_preview`。
- RealSense 默认彩色话题：`/camera/color/image_raw`、
  `/camera/color/image_raw/compressed` 和 `/camera/color/camera_info`；实际消息频率、
  CameraInfo 与 TF 仍须在后续只读传感器检查中确认。

## 本轮细化

### 预览 launch

删除了机载 `advanced_param_exp.xml` 不支持的 `init_x/init_y/init_z` 参数，并加入测试，
自动比较 include 的实参和被包含文件声明的形参。三个 `start_*` 开关仍默认为 false。

### 图像上行

同时支持 compressed 与 raw 图像话题，默认继续使用 JPEG compressed，避免在 ROS 回调
线程里做图像编码。HTTP 工作线程使用容量为 2 的覆盖队列，后端变慢时丢旧帧而不阻塞
相机流。

### EKF frame 兼容

当前 `ekf_node_vio_timesync_with_acc_pub.cpp` 发布 `/ekf/ekf_odom` 时将 world frame 写为
`world`，但没有填写 `child_frame_id`。配置项 `allow_empty_odom_child_frame=true` 允许将空值
显式归一化为协议机体系 `base_link`。非空且不是 `base_link` 的消息仍被拒绝。由于
`calibration_validated=false`，在机体—IMU—相机外参人工确认前，主机继续拒绝该观测进入
VLA 推理。

### 网络

部署时应通过私有环境文件分别提供 `<HOST_ONBOARD_IP>` 和 `<ONBOARD_IP>`。上行后端、
来源 IP 白名单和主机端桥地址均不得硬编码进仓库。token 仍为 `REQUIRED`，不允许无认证启动。

### 无运动验证器

`verify_onboard_no_motion.sh` 先检查是否存在 ROS/control 进程；若存在立即失败。随后只运行
Python 单元测试、语法编译、launch XML 检查和禁用话题扫描，不启动任何 ROS runtime。

## 后续传感器只读确认

飞机保持未解锁状态时，可由操作员单独启动传感器/定位链，再只读记录：实际图像与
CameraInfo frame、K/D、图像频率、`world -> base_link` 语义、
`base_link -> camera_color_optical_frame` TF 和各消息时间差。确认后生成固定
calibration ID；在此之前不得把 `calibration_validated` 改为 true。
