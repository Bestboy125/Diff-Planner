# 模块 1：主机—机载轨迹协议

> FAST-LIO/K-frame 真机适配使用 schema v2 `planning_preview`。它与 schema v1 控制消息严格区分，只能进入隔离的 Diff-Planner 预览话题。完整坐标、标定和安全说明见 `Diff-Planner/docs/FAST_LIO_VLA_DIFF_ADAPTER.md`。

对应代码：`src/vla_diff_bridge/protocol.py`。

## 输入格式

传输层是 TCP 长连接或短连接，每条消息为一行 UTF-8 JSON，端口默认 `50051`。一条 `TRACK` 示例：

```json
{
  "schema_version": 1,
  "type": "trajectory_command",
  "mission_id": "b4e0...",
  "sequence": 12,
  "sent_at_unix_ms": 1787625000000,
  "ttl_ms": 500,
  "policy": "pi05",
  "command": "TRACK",
  "frame_id": "world",
  "action_semantic": ["dx_body", "dy_body", "dz_body", "d_yaw"],
  "action_units": ["m", "m", "m", "rad"],
  "action_local_delta": [[0.2, 0.0, 0.0, 0.03]],
  "target_mission": [[1.2, 2.0, 1.0, 0.53]],
  "auth_token": "仅在线路上传输，不回显到浏览器"
}
```

`action_local_delta` 与当前主机模型输出完全对应，含义是机体系 FLU 下的 `[dx_body, dy_body, dz_body, d_yaw]`。`target_mission` 是主机根据当前状态和机体 yaw 转换出的任务坐标系绝对目标 `[x, y, z, yaw]`。机载端实际向规划器发布 `target_mission`，同时保留并校验原始 action，便于审计模型输出。

## 命令类型

- `TRACK`：必须包含两个 `[1,4]` 数组，发布新目标点与 yaw。
- `HOLD`：不要求动作数组，触发 Diff-Planner 可恢复 hover-stop；之后可用同 mission 的更大 sequence 恢复 TRACK。
- `COMPLETE`：触发可恢复 hover-stop，并把该任务标记为终态，后续同一任务命令会被拒绝。
- `EMERGENCY_STOP`：发布 Diff-Planner 的 mandatory-stop，只用于紧急故障。

## 拒绝条件

- schema、消息类型、策略名、动作语义或单位不匹配。
- 数组形状不是 `[1,4]`，或存在 NaN/Inf。
- 命令已过 TTL、时间戳异常超前、TTL 大于机载上限。
- sequence 重复或倒序。
- 来源 IP 不在 allowlist，或 token 不一致。
- frame_id 与机载 `world_frame` 不同。

## 输出 ACK

每条命令返回一行 `trajectory_ack`，状态为 `preview`、`accepted` 或 `rejected`。ACK 带回相同 `mission_id` 和 `sequence`，主机据此防止把上一条命令的响应错配给当前命令。

## 与模型输出的关系

OpenVLA 与 π0.5 都已归一成同一动作语义。π0.5 的 `action_chunk` 不直接发给机载规划器；主机每个闭环周期只选当前第一步形成一个 `TRACK`，下一帧图像和最新里程计到达后重新推理。这保持“边执行边优化”，也避免旧 action chunk 在环境变化后继续执行。
