# hl_actuator_model

## 策略式目标插件 `trj_policy`（试验中）

`trj` 发的是平滑的扫频参考；而在机器人上，RL 策略按控制频率（如 50 Hz）发出**零阶保持、会跳变**的目标位置，且不给速度前馈。`trj_policy`（`policy_trajectory_plugin_haotian`）模拟后者，是一个独立插件，**不改 `trj` 的任何文件**，测试通过后再决定是否合并。

- 生成：在 q0 ± A 内随机取航点，以阶跃（概率 `step_probability`）或线性斜坡到达，再停留一段时间；目标每 `1/policy_rate_hz` 秒更新一次，相邻两次最多变化 `max_step`。开头和结尾 `warmup_time` 秒内幅值渐入、渐出，首尾都等于 q0。
- 只下发目标位置，速度参考恒为 0。默认在实验期间关闭 joint_safety 参考滤波，结束后恢复（`disable_reference_filter`，与 `trj` 相同）。
- 同一 `seed` 在 Windows 预览和台架上生成完全相同的目标序列；每次实验换一个 seed。
- 启动前必须在 `config/xbot2_config.yaml` 填 `max_velocity`、`max_link_torque`、`max_tracking_error`（为 0 时拒绝启动，但不影响 xbot2-core 和 `trj`）；还要求 q0 ± A 在位置限内，且 `max_step × policy_rate_hz ≤ max_velocity`。
- 日志名不要带点（如 `A0.1` 写成 `A010`）：MatLogger2 会把点后面当扩展名，只接受 `.mat` 或无扩展名。
- 日志（默认 `/tmp/policy_targets`）的变量与 `trj` 相同，另有：`segment`（0 停留、1 斜坡、2 阶跃）、`policy_index`，以及只记一次的 `cfg_*`（种子、幅值、频率、`max_step`、q0、是否关滤波）。

台架上运行（编译后，xbot2-core 用同一个 `xbot2_config.yaml` 启动；不要和 `trj` 同时打开）：

```bash
xbot2 plugin switch trj_policy 1
# 两次实验之间可在线改 amplitude / seed / stop_time / enable_log / log_file，下次启动生效
```

离线预览（不连硬件；参数依次为 幅值A、seed、max_step、策略频率，其余用默认值 T=60 s、R=5 s、q0=0）：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic actuator_data_collection/src/preview_policy.cpp -o preview_policy
./preview_policy 0.3 0 0.05 50 > policy.csv   # 列：time,q_ref,segment，1 ms 间隔
```
