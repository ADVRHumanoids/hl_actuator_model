# hl_actuator_model

## `trj_chirp`：不带速度参考的扫频正弦（试验中）

复制自 `trj`（`chirp_trajectory_plugin.cpp/.h`），轨迹、参数和用法完全相同，只有一处区别：位置模式下 `set_vel_ref(0.0)`，不给速度前馈，与 RL 策略在机器人上只发目标位置的用法一致。`trj` 的文件没有改动。

- 日志：`v_ref` 是实际下发的速度参考（恒为 0），新增 `v_traj` 记录正弦的解析速度；其余变量与 `trj` 相同。
- 在线调参与 `trj` 相同，参数名前缀换成 `/trj_chirp/`，例如 `/trj_chirp/j_motor/amplitude`。
- 不要和 `trj` 同时打开。

```bash
xbot2 plugin switch homing 1     # 回零结束后
xbot2 plugin switch trj_chirp 1
```
