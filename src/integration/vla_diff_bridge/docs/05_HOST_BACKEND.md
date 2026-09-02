# 模块 5：主机后端发送端

对应主机文件：`ground_station/backend/app/onboard_bridge.py`、`schemas.py`、`config.py`、`main.py`。

## 新增 API

`POST /api/bridge/commands` 接收模型归一化结果，例如：

```json
{
  "mission_id": "任务 UUID",
  "sequence": 0,
  "policy": "openvla",
  "command": "TRACK",
  "action_local_delta": [[0.2, 0.0, 0.0, 0.01]],
  "target_mission": [[1.2, 2.0, 1.0, 0.51]]
}
```

后端补齐 schema、时间戳、500 ms TTL、世界坐标系、动作语义和单位。浏览器不接触 `auth_token`；token 只在后端 TCP 客户端写入线路消息。

发送前还会把命令绑定到当前 mission：mission_id 和 policy 必须相同，`TRACK` 只允许在 `RUNNING` 状态；`HOLD`、`COMPLETE` 和 `EMERGENCY_STOP` 各自有明确的允许状态。即使全局控制开关被打开，`dry_run` 任务也不会发送到机载端。

## 安全锁行为

当 `CONTROL_OUTPUT_ENABLED=false` 时，API 返回 `delivery.status=safety_locked` 和完整的非敏感 command，便于前端检查，但不建立机载连接。开启后才调用 TCP 客户端，等待并校验 ACK 的消息类型、mission_id 和 sequence。

## 环境变量

- `ONBOARD_BRIDGE_HOST`：机载电脑 IP；当前默认 `127.0.0.1`，避免开发机误连。
- `ONBOARD_BRIDGE_PORT`：默认 `50051`。
- `ONBOARD_BRIDGE_TOKEN`：必须与机载脚本的 `VLA_BRIDGE_AUTH_TOKEN` 相同。
- `ONBOARD_COMMAND_TTL_MS`：默认 `500`。
- `CONTROL_OUTPUT_ENABLED`：默认 `false`。

## 闭环调用顺序

后续图像循环每一拍应执行：读取最新图像和 odom → 调用选定模型推理接口 → 递增 sequence → 调用 `/api/bridge/commands`。只有收到 `accepted` ACK 才把该拍记为成功下发；`rejected`、超时或断链都不得重发同一 sequence，而应进入 HOLD/故障状态并生成新 sequence。
