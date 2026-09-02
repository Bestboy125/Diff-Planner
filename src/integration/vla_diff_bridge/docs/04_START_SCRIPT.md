# 模块 4：启动 Shell 脚本

> `Diff-Planner/sh_files/run_vla_diff_planner_lio.sh` 是早期整机方案，仅保留给模拟分支回溯。`real-deployment` 使用包内 `scripts/run_vla_fastlio_diff_preview.sh` 和 `launch/vla_fastlio_diff_preview_stack.launch`；它们可随单个 integration 包安装，且不启动飞控栈。详细流程见 `09_REAL_DEPLOYMENT_PACKAGE.md`。

对应文件：`Diff-Planner/sh_files/run_vla_diff_planner_lio.sh`。

脚本使用 bash 和 `set -euo pipefail`，通过绝对解析得到工作空间路径，并在退出时向所有由脚本启动的子进程发送 SIGINT。它没有复制原脚本中的明文 sudo 密码；串口权限应通过 udev 规则预先配置。

## 仅桥接模式

```bash
cd ~/Diff-Planner
export VLA_BRIDGE_AUTH_TOKEN='替换为随机长密钥'
./sh_files/run_vla_diff_planner_lio.sh --bridge-only
```

此模式假设 roscore、odom 和 Diff-Planner 已由其他终端启动，只启动桥接节点。默认 `VLA_BRIDGE_LIVE_OUTPUT_ENABLED=false`，适合网络和协议验证。

首次复制到 Ubuntu 后先补执行权限：

```bash
chmod +x sh_files/run_vla_diff_planner_lio.sh
chmod +x src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py
```

## 完整 LIO 栈模式

```bash
export VLA_BRIDGE_AUTH_TOKEN='替换为随机长密钥'
export VLA_ALLOW_HARDWARE_STACK=I_UNDERSTAND_HARDWARE_STACK
./sh_files/run_vla_diff_planner_lio.sh --full-lio
```

该模式仿照现有 `run_single_lio.sh`，依次启动 MAVROS、MAVLink 消息频率、faster-lio、EKF、Diff-Planner、px4ctrl 和桥接节点，但不发送起飞命令，也不启动原 multipoint 节点，因为 VLA 桥接层直接提供 `/goal`。

## 打开 live 输出

只有完成仿真、悬桨和现场急停验证后才可同时设置：

```bash
export VLA_BRIDGE_LIVE_OUTPUT_ENABLED=true
export VLA_ALLOW_LIVE_OUTPUT=I_UNDERSTAND_LIVE_OUTPUT
```

主机后端还必须独立设置 `CONTROL_OUTPUT_ENABLED=true`。双端锁的目的是防止只改一台电脑的配置就产生运动命令。

可选变量包括 `VLA_HOST_ONBOARD_IP`、`VLA_ODOM_TOPIC` 和 `VLA_START_RVIZ=true`。
