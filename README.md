# hl_actuator_model

本分支 `bench-test` 同时包含三个轨迹插件，编译一次即可：`trj`（原有扫频，带速度参考）、`trj_chirp`（同样的扫频，速度参考置 0）、`trj_policy`（策略式目标）。**同一时间只打开一个轨迹插件。**

实验计划见 [docs/experiment_plan.md](docs/experiment_plan.md)。

## 运行脚本 `run_trj.sh`

`actuator_data_collection/scripts/run_trj.sh` 通过 ROS 2 服务设置参数，确认后启动插件，并在日志旁写一份实验条件文件 `<log_file>_meta.yaml`（执行器、配重、增益、git 版本、本次参数），转换数据时会自动读取。

```bash
export ACTUATOR_ID=A0368 LOAD_MASS=2.0 LOAD_RADIUS=0.30 KP=80 KD=8.5   # 空载时 LOAD_MASS=0 LOAD_RADIUS=0
./run_trj.sh trj_chirp  j_motor/amplitude=0.3 log_file=/tmp/chirp_A030
./run_trj.sh trj_policy amplitude=0.1 seed=1.0 log_file=/tmp/policy_A010_seed1
DRY_RUN=1 ./run_trj.sh trj_chirp j_motor/amplitude=0.3 log_file=/tmp/x     # 只打印，不执行
```

- 每次实验用新的 `log_file`；名字不要带点（`A0.1` 写成 `A010`），MatLogger2 只接受 `.mat` 或无扩展名。
- `ACTUATOR_ID`、`LOAD_MASS`、`LOAD_RADIUS` 必填（不想写条件文件时设 `META=0`）。
- 拷数据时把 `.mat` 和 `_meta.yaml` 一起拷走。

## 策略式目标插件 `trj_policy`（试验中）

`trj` 发的是平滑的扫频参考；而在机器人上，RL 策略按控制频率（如 50 Hz）发出**零阶保持、会跳变**的目标位置，且不给速度前馈。`trj_policy`（`policy_trajectory_plugin_haotian`）模拟后者，是一个独立插件，不改 `trj` 的任何文件。

- 生成：在 q0 ± A 内随机取航点，以阶跃（概率 `step_probability`）或线性斜坡到达，再停留一段时间；目标每 `1/policy_rate_hz` 秒更新一次，相邻两次最多变化 `max_step`。开头和结尾 `warmup_time` 秒内幅值渐入、渐出，首尾都等于 q0。
- XBot2 HAL 要求 d(pos_ref)/dt ≤ 20 rad/s，并按实际周期计算。所以实际下发的参考以 `max_ref_rate`（默认 8 rad/s，给周期抖动留余量）向策略目标移动：0.05 rad 的跳变约 6 ms 走完，仍在下一次策略更新（20 ms）之前到达。日志里 `q_target` 是策略目标，`q_ref` 是限速后实际下发的参考。
- 只下发目标位置，速度参考恒为 0。默认在实验期间关闭 joint_safety 参考滤波，结束后恢复（`disable_reference_filter`，与 `trj` 相同）。
- 同一 `seed` 在 Windows 预览和台架上生成完全相同的目标序列；每次实验换一个 seed。
- 运行中越限时插件打印 `SAFETY STOP trj_policy ...`（哪一项、实际值、阈值），写出已采数据后停止；参考保持上一周期的值，xbot2-core 不受影响。
- 启动前必须在 `config/xbot2_config.yaml` 填 `max_velocity`、`max_link_torque`、`max_tracking_error`（为 0 时拒绝启动，但不影响 xbot2-core 和 `trj`）；还要求 q0 ± A 在位置限内，`max_step × policy_rate_hz ≤ max_velocity`，`max_step ≤ max_ref_rate / policy_rate_hz`。
- 日志（默认 `/tmp/policy_targets`）的变量与 `trj` 相同，另有：`time`、`valid`、`q_target`、`segment`（0 停留、1 斜坡、2 阶跃）、`policy_index`，以及只记一次的 `cfg_*`（种子、幅值、频率、`max_step`、q0、是否关滤波）。

离线预览（不连硬件；参数依次为 幅值A、seed、max_step、策略频率，其余用默认值 T=60 s、R=5 s、q0=0）：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic actuator_data_collection/src/preview_policy.cpp -o preview_policy
./preview_policy 0.3 0 0.05 50 > policy.csv   # 列：time,q_ref,segment，1 ms 间隔
```

## `trj_chirp`：不带速度参考的扫频正弦（试验中）

复制自 `trj`（`chirp_trajectory_plugin.cpp/.h`），轨迹、参数和用法相同，区别：

- 位置模式下 `set_vel_ref(0.0)`，不给速度前馈，与 RL 策略在机器人上只发目标位置的用法一致。
- 安全检查（与 `trj_policy` 相同，每项阈值 > 0 时生效）：`position_min/max`、`max_velocity`、`max_link_torque`、`max_tracking_error`。越限时打印 `SAFETY STOP trj_chirp ...` 并停止插件。
- 日志：`v_ref` 是实际下发的速度参考（恒为 0），新增 `v_traj` 记录正弦的解析速度；其余变量与 `trj` 相同。
- 在线调参与 `trj` 相同，参数名前缀换成 `/trj_chirp/`，例如 `/trj_chirp/j_motor/amplitude`。

`trj` 的文件没有改动。
