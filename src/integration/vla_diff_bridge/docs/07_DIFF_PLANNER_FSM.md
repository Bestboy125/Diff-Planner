# 模块 7：Diff-Planner FSM 最小改动

对应文件：`plan_manage/include/plan_manage/diff_replan_fsm.h`、`plan_manage/src/diff_replan_fsm.cpp`、`advanced_param_exp.xml`、`advanced_param_sim.xml`。

## 新增可恢复 hover-stop

FSM 新增私有订阅 `hover_stop`。回调把 `need_hover_stop_` 和 `flag_escape_emergency_` 置为 true，然后进入 `EMERGENCY_STOP`。下一拍状态机调用已有 `callEmergencyStop(odom_pos_)` 生成停止轨迹；当速度小于原有 0.1 m/s 阈值后，已有 fail-safe 分支清除目标并回到 `WAIT_TARGET`。因此临时 HOLD 或网络 watchdog 停止后，后续新目标仍能恢复任务。

实验和仿真参数文件都把该私有话题映射为 `/vla_hover_stop_to_planner`，保证单机 LIO、VIO 和仿真使用同一接口。

## 修复 mandatory-stop

原 `mandatoryStopCallback()` 只切换 `EMERGENCY_STOP` 并关闭 fail-safe，却没有确保 `flag_escape_emergency_` 为 true。该标志在普通 FSM 周期末会被清 false，因而运行后的 mandatory-stop 存在无法调用一次 `callEmergencyStop()` 的风险。

现在回调同时设置：

- `need_hover_stop_ = true`
- `flag_escape_emergency_ = true`
- `enable_fail_safe_ = false`

这样强停一定尝试生成一次停止轨迹，并保持在紧急状态；它与可恢复 hover-stop 的语义明确分离。

## 未改动部分

没有改动 A*、栅格地图、全局/局部轨迹优化、轨迹服务器输出格式、px4ctrl 或 MAVROS。新增回调复用现有 `callEmergencyStop()`，减少对原算法动力学行为的影响。
