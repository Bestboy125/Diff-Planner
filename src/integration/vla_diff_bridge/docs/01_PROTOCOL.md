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
  "action_chunk": [
    [0.2, 0.0, 0.0, 0.03],
    [0.2, 0.0, 0.0, 0.00]
  ],
  "auth_token": "仅在线路上传输，不回显到浏览器"
}
```

`action_local_delta` 与动作块第一行完全一致，含义是机体系 FLU 下的 `[dx_body, dy_body, dz_body, d_yaw]`。`target_mission` 是这第一行对应的任务坐标系目标；两者使机载端可恢复推理输入时刻的位姿。`action_chunk` 是可选的完整动作块，默认最多 10 行。主机只负责原样传输，不选择将要执行的行。

机载端从第一行和第一目标恢复捕获位姿，将全部机体系增量依次累计为固定世界系轨迹。
从 10 步中按弧长近似均匀选取 8 个原始航点（可配置为 6 个），保留首末航点与原顺序；
不足采样数量时保留全部点。剔除计算仍在完整的 10 步折线上投影最新 FAST-LIO/EKF 位姿，
而不是在稀疏采样路径上投影。位于实际进度加前视距离之前的采样点会被跳过。

机载保留剩余航点队列，每个 100 ms 定时周期最多向 Diff-Planner 发布一个新的未来目标，
只有进度进入下一采样点时才发布，索引不倒退。绝不一次循环发布全部目标（`/goal` 队列只有 1，
那样最终只留下终点）。新的已验证动作块替换旧块；HOLD/COMPLETE/EMERGENCY_STOP、操作员
接管、看门狗超时会清空队列；执行中里程计失效或越出走廊会中止队列，live 下发 hover-stop。
内部航点推进不续期通信看门狗。纯原地偏航仍使用即时单目标，不以空间进度猜测偏航完成。

## 命令类型

- `TRACK`：必须包含两个 `[1,4]` 数组；可附带 `[1..10,4]` 动作块，由机载端选择有效未来目标。
- `HOLD`：不要求动作数组，触发 Diff-Planner 可恢复 hover-stop；之后可用同 mission 的更大 sequence 恢复 TRACK。
- `COMPLETE`：触发可恢复 hover-stop，并把该任务标记为终态，后续同一任务命令会被拒绝。
- `EMERGENCY_STOP`：发布 Diff-Planner 的 mandatory-stop，只用于紧急故障。

## 拒绝条件

- schema、消息类型、策略名、动作语义或单位不匹配。
- 单动作/目标形状不是 `[1,4]`，动作块不是 `[1..max_action_chunk_steps,4]`，第一行不一致，或存在 NaN/Inf。
- 命令已过 TTL、时间戳异常超前、TTL 大于机载上限。
- sequence 重复或倒序。
- 来源 IP 不在 allowlist，或 token 不一致。
- frame_id 与机载 `world_frame` 不同。

## 输出 ACK

每条命令返回一行 `trajectory_ack`，状态包括 `preview`、`accepted`、`stale_chunk` 或 `rejected`。ACK 带回相同 `mission_id` 和 `sequence`，主机据此防止把上一条命令的响应错配给当前命令。

## 与模型输出的关系

OpenVLA 的单动作保持原路径；π0.5 的完整 `action_chunk` 随 `TRACK`/预览消息传到机载桥。
机载桥按上述队列逐个发布采样目标供 Diff-Planner 逐段优化，不是只给最后一个航点，也不是
让 Diff-Planner 一次联合优化整条多点轨迹。新观测推理、过期剔除和安全门禁仍保持闭环。
