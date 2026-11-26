# hl_actuator_model

本分支 `bench-test` 同时包含六个插件，编译一次即可：`trj`（原有扫频，带速度参考）、`trj_chirp`（同样的扫频，速度参考置 0）、`trj_policy`（策略式目标）、`trj_static`（静态带载检查）、`goto`（移动到指定角度）、`trj_heat`（pin 固定输出，温度 PID 堵转升温保温），以及 `trj_horizontal`（`trj_chirp` 的水平扫频实例）。**同一时间只打开一个轨迹插件。**

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
- 拷数据时把 `.mat`、`_meta.yaml` 和 `_hal*.yaml` 一起拷走。
- 启动时脚本把驱动器配置（默认是 `xbot2_config.yaml` 里 `ec_pos` 指向的 HAL 配置，含位置环增益）复制成 `<log_file>_hal.yaml`，meta 里记下原路径和 sha256；`HAL_CONFIG=<文件>` 覆盖路径，`ECAT_CONFIG=<文件>` 额外复制 repl 的配置为 `_ecat.yaml`。文件不存在时不启动。HAL 配置里 `value: $PWD/...yaml` 引用的文件（位置环增益在 `joint_config/position.yaml`）也复制成 `<log_file>_hal_<名字>.yaml`，`$PWD` 按 HAL 配置所在目录展开；引用的文件不存在时只提示。
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

## `trj_static`：静态带载检查（自动逐点）

`static_hold_plugin_haotian`，独立插件。从启动位置 q0 出发，依次平滑移动到 `q0 + offsets[i]`，每个位置保持 `hold_time` 秒并记录，全部测完回到 q0 后自动停止，不用在 GUI 里手动换位置。

```bash
export ACTUATOR_ID=... LOAD_MASS=2.0 LOAD_RADIUS=0.30
./run_trj.sh trj_static log_file=/tmp/static_m2          # 配重自然下垂时启动
LOAD_MASS=0 LOAD_RADIUS=0 ./run_trj.sh trj_static log_file=/tmp/static_m0   # 空载（只有摆臂）
```

- 默认 `offsets`：0 → +0.3 → … → +1.2 → … → 0 → −0.3 → … → −1.2 → … → 0（17 个点，约 3 min）。同一角度从两侧各到一次，用来看静摩擦造成的滞回。可在线改，例如 `offsets=[0.0,0.5,1.0,0.5,0.0]`。
- 起始位置不必正好下垂：分析脚本会拟合出下垂角度（重力矩为零的位置），只要起始位置离下垂不超过 90°。
- 移动段是五次多项式，峰值速度 `move_velocity`（0.3 rad/s），起止速度和加速度为 0；保持段前 `settle_time`（2 s）不计入平均。
- 每个点保持结束时打印 `point i done: q_link ..., tau_link ..., tau_mot ...`，现场就能看出力矩是否随角度按 sin 变化。
- 安全检查与 `trj_policy` 相同（位置、速度、`max_link_torque` 30 N·m、跟踪误差、70 °C）；q0 + offsets 超出位置限时拒绝启动。配重的 m·g·r 要明显小于 `max_link_torque`。
- 日志变量：与其他插件相同的实测量，加 `phase`（0 移动、1 稳定、2 保持）、`point`、`v_ref`、`cfg_offsets`、`cfg_initial_value` 和 `stop_code` 等。

### `goto`：移动到指定角度

和 `trj_static` 同一个插件类型（`mode: goto`），平滑移动到绝对角度 `target` 后停住，不回原位：

```bash
META=0 ./run_trj.sh goto target=-0.25     # 例如移到 analyze_static.py 算出的下垂角度
```

峰值速度 0.2 rad/s，起止平滑；到位保持 1 s 后插件停止，参考留在 `target`。安全检查与 `trj_static` 相同，`target` 超出位置限时拒绝启动。

分析（Actuator_Net 仓库）：`python scripts/analyze_static.py <日志目录或文件> --out runs/static.png`，输出每个点的平均值、拟合的 `tau_link = A·sin(q − q_hang) + c`（q_hang 是下垂角度，A 的符号即力矩方向，c 即零偏）、移到下垂位置的 `goto` 命令、建议的 `--torque_sign` / `--torque_offset`、`tau_link` 与 `tau_mot` 的比例、静摩擦、传动柔度；多个配重时还给出 A 对 m·g·r 的斜率（传感器比例）和截距（摆臂自重）。

## `trj_horizontal`：配重保持在水平位置附近小幅扫频（试验中）

和 `trj_chirp` 同一个插件（`chirp_trajectory_plugin_haotian`），yaml 里多一个实例，用 `side` 选择水平位置在哪一侧。重力矩在水平位置最大，所以这个测试让执行器在大的静态负载上叠加小幅动态运动。

1. 从当前位置（配重自然下垂）按五次多项式平滑移动到水平位置 `hang_position + side·π/2`，峰值速度 `transfer_velocity`（默认 0.3 rad/s，约 10 s）；
2. 停稳并修正中心：驱动器位置环没有积分，水平托住配重时实际角度会比参考差 重力矩/Kp + 静摩擦（7 kg 约 12–20°）。停稳阶段按编码器实测角度慢慢积分出参考偏移（`correction_gain`），实际角度在 `correction_tolerance`（0.01 rad）内保持 1 s 后才开始扫频，终端打印到位角度和偏移；`max_settle_time`（30 s）内没到位也开始，并打印 `NOT settled`。`center_correction=false` 关掉；
3. 扫频前检查力矩余量：停稳末尾 |tau_link|（0.5 s 滑动平均）超过 `max_hold_torque`（yaml 75 N·m，0 不检查），或参考偏移已顶到 `max_correction`，就不扫频，直接平滑回起点，`stop_code` 5，终端打印 `NO TORQUE HEADROOM`。驱动器约 80–85 N·m 饱和，11-23 托举 77–82 N·m 的 run 扫频时关节完全不动。每次都会打印托举力矩和 托举 + 98.7×幅值；
4. 绕修正后的参考扫频 `stop_time` 秒（实际中心在水平位置），幅值 `j_motor/amplitude`（rad），频率在 `freq_min`–`freq_max` 之间往复；最后 `fade_time` 秒幅值平滑降到 0。扫频中继续慢速修正中心（`sweep_correction_gain`，yaml 0.03 1/s，0 关闭）：(中心 − 实测角度) 按最低扫频频率的周期低通后再积分，修正停稳偏移里单向静摩擦造成的平均角度偏差（11-23 为 0.07–0.14 rad），不吃掉扫频本身；增益不超过 0.5 × `freq_min` 时不超调。偏移记在日志 `center_offset`，状态行也打印；
5. 平滑回到起点，插件自动停止（`stop_code` 0）。

```bash
cd ~/xbot2_ws/src/hl_actuator_model/actuator_data_collection/scripts
export ACTUATOR_ID=0 LOAD_MASS=7 LOAD_RADIUS=0.30 KP=98.7 KD=9.6
META=0 ./run_trj.sh goto target=0.0                       # 先回到下垂位置
./run_trj.sh trj_horizontal side=1  j_motor/amplitude=0.26 stop_time=60 log_file=/tmp/chirp_hpos_m7_A026   # 正方向水平，±15°
./run_trj.sh trj_horizontal side=-1 j_motor/amplitude=0.17 stop_time=60 log_file=/tmp/chirp_hneg_m7_A017   # 负方向水平，±10°
```

- `side`：`1` 或 `-1`（`0` 等于普通的 `trj_chirp`，绕起点扫频）。哪边是左、哪边是右按台架安装看，第一次用小配重确认方向。
- 幅值：10° = 0.17 rad，15° = 0.26 rad，20° = 0.35 rad。频率用 `j_motor/freq_min`、`j_motor/freq_max`，周期 `j_motor/period`。
- 位置范围放大到 ±2.2 rad（约 ±126°）。启动时检查起点和 中心 ± 幅值 都在范围内，超出就拒绝启动；机械限位和 URDF 的关节限位也要允许。
- 中途停止（`xbot2 plugin switch trj_horizontal 0` 或安全停机）时摆臂停在水平附近、仍然带着负载：用 `META=0 ./run_trj.sh goto target=0.0` 回到下垂（`goto` 的范围和跟踪误差已相应放宽）。
- 日志里 `chirp_phase`：0 移到水平、1 停稳、2 扫频、3 回起点；`center_offset` 是参考修正量，`cfg_center`、`cfg_start`、`cfg_side` 记录中心和起点。过渡段也是有效的训练数据。
- 文件名建议 `chirp_hpos_m7_A026` / `chirp_hneg_m7_A026`（`hpos`/`hneg` 是标签，`m7` 配重 kg，`A026` 幅值 0.26 rad），处理脚本能从文件名读出配重和幅值。
- `trj_chirp` 本身不受影响：扫频计算移到 `src/chirp_schedule.h`，`fade_time = 0` 时与原来逐位相同（外层仓库 `tests/cpp/test_chirp_schedule.cpp`）。日志的 `trajectory_time` 改为插件启动后的时间，`side = 0` 时与原来相同。

## `trj_heat`：pin 固定输出，堵转升温并保温（试验中）

带载测试前把电机加热到指定温度并维持一段时间，然后释放力矩，拔 pin 后马上测。控制逻辑在 `src/thermal_soak.h`（外层仓库 `tests/cpp/test_thermal_soak.cpp` 用两节点热模型做了闭环离线测试），插件在 `src/thermal_soak_plugin.cpp`。

- 原理：输出端固定，位置模式下参考偏离固定位置 δ，堵转力矩 ≈ 驱动器刚度 98.7 N·m/rad × δ。温度 PID 输出 u ∈ [0, 1] 是加热功率比例，力矩 = `max_torque`·√u（铜损 ∝ 力矩²）。
- 阶段：Heat（升温）→ Soak（进入 `setpoint ± tolerance` 后保温 `soak_time`）→ Release（力矩按 `torque_rate` 降到 0，参考回到固定位置）→ 打印 `trj_heat DONE`，插件自动停止。**看到 DONE 才能拔 pin。**
- 安全：输出端离开固定位置超过 `max_position_drift`（pin 松了或没装）、速度、`|tau_link − torque_offset|`、电机 / 驱动器温度停机线、温度读数无效 → 先把参考设到当前位置（卸掉力矩）再停止。
- 升温超时（`max_heat_time` 内没到设定温度，说明力矩不够）→ 自动释放，`stop_code` 5。
- 加热功率：(力矩/50/Km)²，Km = 0.3634 N·m/√W。30 N·m ≈ 2.7 W，60 N·m ≈ 11 W，88 N·m（额定）≈ 23.5 W。默认 `max_torque: 30` 多半升不到 50 °C，确认力矩传感器、pin、减速器量程后再调（堵转建议不超过额定的 60–70 %，一两相电流集中，发热不均匀）。
- 热源是绕组：电机温度维持在 setpoint 时，减速器的稳态温度更低。需要减速器温度时另用测温枪 / 热电偶读。

```bash
# 摆臂自然下垂，插 pin，然后
./run_trj.sh trj_heat release_only=false setpoint=50 soak_time=1800 log_file=/tmp/heat_50C
# 提前结束（会先释放力矩再停）
ros2 service call /xbotcore/set_parameter_values xbot_msgs/srv/SetString "{request: {/trj_heat/release: true}}"
# 运行中改设定温度
ros2 service call /xbotcore/set_parameter_values xbot_msgs/srv/SetString "{request: {/trj_heat/setpoint: 55}}"
# 被 plugin switch 0 等外部停止后参考还带偏移（结束时会打印 WARNING）：先释放，再拔 pin
META=0 ./run_trj.sh trj_heat release_only=true
```

日志每 `log_period`（0.1 s）一行：`heat_phase`、`temp_mot`、`temp_dri`、`temp_filtered`、`setpoint`、`u`、`pid_p/i/d`、`torque_cmd`、`soak_elapsed`、`q_ref`、`q_link`、`q_mot`、`tau_link`、`tau_mot`，以及 `cfg_*`、`stop_code`。它不是训练数据，转换脚本会跳过。

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
  - 温度停机线 `stop_motor_temperature` 是可选项，yaml 里现为 70 °C（暂定，与 `trj_policy` 相同）；删掉这一行就不检查（启动时会提示）。驱动器自身的过温 / I²t 保护仍然有效。
  - 配重加重后若 `|tau_link|` 超过 `max_link_torque`（现为 30 N·m）会 `SAFETY STOP`；这一项要在 yaml 里调（不超过峰值力矩），不能在线改。

`trj` 的文件没有改动。
