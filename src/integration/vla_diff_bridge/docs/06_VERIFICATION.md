# 模块 6：验证范围与步骤

## 已可在 Windows 主机执行

1. 纯 Python 协议单元测试：合法 OpenVLA/π0.5 共同格式、过期、NaN、单位错误和 HOLD。
2. ground_station API 测试：安全锁下生成匹配的机载 command，且不会发送网络数据。
3. Python 语法编译检查。
4. 通过 WSL 的 `bash -n` 检查启动脚本语法；此步骤不启动 ROS 或硬件。

本次实际结果：ground_station `11 passed`；机载纯协议 `5 passed`；主机生成 command 后由机载 parser 读取的交叉契约检查通过；TCP ACK 往返、Python 编译、XML、YAML 和 shell 语法检查通过。

## 必须在 Ubuntu 20.04 + ROS Noetic 执行

```bash
cd ~/Diff-Planner
catkin_make
source devel/setup.bash
export VLA_BRIDGE_AUTH_TOKEN='测试密钥'
./sh_files/run_vla_diff_planner_lio.sh --bridge-only
```

随后使用模拟 odom，并保持 `live_publish_enabled=false`，从主机发送命令，确认 ACK 为 `preview`。再在仿真中打开 live，检查：

- `/goal` 位置等于 `target_mission[0][0:3]`。
- `/planning/yaw.yaw` 等于 `target_mission[0][3]`。
- 重复 sequence、过期 TTL、错误 token、超步长和陈旧 odom 均返回 `rejected`。
- 断开主机命令流 1 秒后只触发一次 `/vla_hover_stop_to_planner`，规划器减速后进入 `WAIT_TARGET`。
- `COMPLETE` 后同 mission 的延迟命令被拒绝。

## 实机前最低门槛

实机前还需要完成仿真、去桨台架、限高限速、小范围系留、遥控器优先级和断网/断图像/模型超时演练。本次在 Windows 上没有编译 ROS 工程，也没有连接、启动或控制无人机。
