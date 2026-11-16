# hl_actuator_model

本分支 `bench-test` 同时包含三个轨迹插件，编译一次即可：`trj`（原有扫频，带速度参考）、`trj_chirp`（同样的扫频，速度参考置 0）、`trj_policy`（策略式目标）。**同一时间只打开一个轨迹插件。**

实验计划见 [docs/experiment_plan.md](docs/experiment_plan.md)。

## 运行脚本 `run_trj.sh`

`actuator_data_collection/scripts/run_trj.sh` 通过 ROS 2 服务设置参数，确认后启动插件，并在日志旁写一份实验条件文件 `<log_file>_meta.yaml`（执行器、配重、增益、git 版本、本次参数），转换数据时会自动读取。

```bash
export ACTUATOR_ID=A0368 LOAD_MASS=2.0 LOAD_RADIUS=0.30 KP=80 KD=8.5   # 空载时 LOAD_MASS=0 LOAD_RADIUS=0
./run_trj.sh trj_chirp  j_motor/amplitude=0.3 log_file=/tmp/chirp_A030
./run_trj.sh trj_policy amplitude=0.1 seed=1.0 log_file=/tmp/policy_A010_seed1
AMBIENT_TEMP=22 ./run_trj.sh trj_policy amplitude=0.4 stop_time=7200 probe_duration=30 log_file=/tmp/thermal_L150   # 温升实验
DRY_RUN=1 ./run_trj.sh trj_chirp j_motor/amplitude=0.3 log_file=/tmp/x     # 只打印，不执行
```

- 每次实验用新的 `log_file`；名字不要带点（`A0.1` 写成 `A010`），MatLogger2 只接受 `.mat` 或无扩展名。
- `ACTUATOR_ID`、`LOAD_MASS`、`LOAD_RADIUS` 必填（不想写条件文件时设 `META=0`）。
- 拷数据时把 `.mat` 和 `_meta.yaml` 一起拷走。
- 温升实验加 `AMBIENT_TEMP`（环境温度 °C），写进条件文件。

## 策略式目标插件 `trj_policy`（试验中）

`trj` 发的是平滑的扫频参考；而在机器人上，RL 策略按控制频率（如 50 Hz）发出**零阶保持、会跳变**的目标位置，且不给速度前馈。`trj_policy`（`policy_trajectory_plugin_haotian`）模拟后者，是一个独立插件，不改 `trj` 的任何文件。

- 生成：在 q0 ± A 内随机取航点，以阶跃（概率 `step_probability`）或线性斜坡到达，再停留一段时间；目标每 `1/policy_rate_hz` 秒更新一次，相邻两次最多变化 `max_step`。开头和结尾 `warmup_time` 秒内幅值渐入、渐出，首尾都等于 q0。
- XBot2 HAL 要求 d(pos_ref)/dt ≤ 20 rad/s，并按实际周期计算。所以实际下发的参考以 `max_ref_rate`（默认 8 rad/s，给周期抖动留余量）向策略目标移动：0.05 rad 的跳变约 6 ms 走完，仍在下一次策略更新（20 ms）之前到达。日志里 `q_target` 是策略目标，`q_ref` 是限速后实际下发的参考。
- 只下发目标位置，速度参考恒为 0。默认在实验期间关闭 joint_safety 参考滤波，结束后恢复（`disable_reference_filter`，与 `trj` 相同）。
- 同一 `seed` 在 Windows 预览和台架上生成完全相同的目标序列；每次实验换一个 seed。
- 运行中越限时插件打印 `SAFETY STOP trj_policy ...`（哪一项、实际值、阈值），写出已采数据后停止；参考保持上一周期的值，xbot2-core 不受影响。
- 启动前必须在 `config/xbot2_config.yaml` 填 `max_velocity`、`max_link_torque`、`max_tracking_error`，以及数据手册上的 `rated_torque`、`peak_torque`（输出端 N·m）和 `stop_motor_temperature`（°C）（为 0 时拒绝启动，但不影响 xbot2-core 和 `trj`）；`max_link_torque` 不能超过 `peak_torque`；还要求 q0 ± A 在位置限内，`max_step × policy_rate_hz ≤ max_velocity`，`max_step ≤ max_ref_rate / policy_rate_hz`。
- 日志（默认 `/tmp/policy_targets`）的变量与 `trj` 相同，另有：`time`、`valid`、`q_target`、`segment`（0 停留、1 斜坡、2 阶跃）、`policy_index`、`probe`、`chunk`，只记一次的 `cfg_*`（种子、幅值、频率、`max_step`、q0、是否关滤波、温升参数、启动时电机温度），以及结束时写的 `stop_code`（0 跑完、1 电机温度、2 驱动器温度、3 安全检查、4 外部停止）、`stop_elapsed`、`dropped_samples`。日志在运行中由后台线程持续写盘（MatAppender），长时间实验不占满内存。
- 温升 / 过载实验（见实验计划第 3 节）：
  - 电机温度连续 0.5 s 达到 `stop_motor_temperature` 时打印 `THERMAL STOP trj_policy ...` 并停止；可选 `stop_driver_temperature`。
  - `max_start_motor_temperature` > 0 时，电机温度高于它就拒绝启动，保证每档从冷态开始。
  - `probe_duration` > 0 时目标序列为 探测段、加热段 交替（P H P H ...），每个探测段完全相同（`probe_seed`），第一个是冷态基准；加热段长 `heat_duration`，种子依次为 seed、seed+1……；总时长按整段向上取整。
  - 每 `status_period` 秒打印一行：电机 / 驱动器温度、`tau_mot` RMS（最近一段和从开始）、超过额定力矩的时间比例。用它在试跑时调整幅值和 q0，让 RMS 落在目标档位。

离线预览（不连硬件；参数依次为 幅值A、seed、max_step、策略频率、总时长、探测段时长、加热段时长、探测段种子，其余用默认值 R=5 s、q0=0）：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic actuator_data_collection/src/preview_policy.cpp -o preview_policy
./preview_policy 0.3 0 0.05 50 > policy.csv   # 列：time,q_ref,segment,probe，1 ms 间隔
./preview_policy 0.3 0 0.05 50 1200 30 300 1000 > thermal.csv   # 插探测段
```

## `trj_chirp`：不带速度参考的扫频正弦（试验中）

复制自 `trj`（`chirp_trajectory_plugin.cpp/.h`），轨迹、参数和用法相同，区别：

- 位置模式下 `set_vel_ref(0.0)`，不给速度前馈，与 RL 策略在机器人上只发目标位置的用法一致。
- 安全检查（与 `trj_policy` 相同，每项阈值 > 0 时生效）：`position_min/max`、`max_velocity`、`max_link_torque`、`max_tracking_error`。越限时打印 `SAFETY STOP trj_chirp ...` 并停止插件。
- 日志：`v_ref` 是实际下发的速度参考（恒为 0），新增 `v_traj` 记录正弦的解析速度；其余变量与 `trj` 相同。
- 在线调参与 `trj` 相同，参数名前缀换成 `/trj_chirp/`，例如 `/trj_chirp/j_motor/amplitude`。
- **长时间循环扫频（温升实验）**：频率本来就以 `period` 为周期在 `freq_min` → `freq_max` → `freq_min` 之间往复，所以把 `stop_time` 设大（或 0 = 一直跑，手动停）就是循环扫频，每个周期都覆盖从低到高的速度，可以比较不同温度下各速度的表现。不需要改 yaml：

  ```bash
  AMBIENT_TEMP=22 ./run_trj.sh trj_chirp stop_time=7200 log_file=/tmp/chirp_long_m2   # 换配重后换 log_file 和 LOAD_MASS
  ```

  - 日志由后台线程持续写盘，几小时也不会占满内存或被覆盖（原来 `stop_time` 为 0 时只保留最后 1000 s）。
  - 新增日志变量：`cycle`（第几个扫频周期）、`freq`（当前频率 Hz）、`trajectory_time`，结束时写 `stop_code`（0 跑完、1 电机温度、3 安全检查、4 外部停止）、`stop_elapsed`、`dropped_samples`。
  - 每 10 s 打印一行：周期、频率、电机 / 驱动器温度、`tau_mot` RMS，换配重时用它判断力矩档位。
  - 温度停机线 `stop_motor_temperature` 是可选项，yaml 里没写就不检查（启动时会提示），要靠人看状态行；驱动器自身的过温 / I²t 保护仍然有效。
  - 配重加重后若 `|tau_link|` 超过 `max_link_torque`（现为 30 N·m）会 `SAFETY STOP`；这一项要在 yaml 里调（不超过峰值力矩），不能在线改。

`trj` 的文件没有改动。
