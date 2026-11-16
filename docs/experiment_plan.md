# Actuator identification on the bench: experiment plan

Goal: identify an actuator network for sim-to-real. In simulation the network replaces the ideal PD
actuator: from the history of the position error `e = q - q_des` and the measured velocity `v` it
predicts the output torque `tau_link`, every simulation step. On the robot nothing changes: the policy
sends position targets to the drives. Secondary studies: temperature and (emulated) faults.

All numbers below are proposals to discuss; thresholds must be confirmed against the actuator ratings.

## Actuator data

Motor datasheet (motor side), bus voltage 48 V, gear ratio 50 (output values below ignore gearbox efficiency,
like the drive's current-based `tau_mot`):

| | Motor | Output (x50) |
|---|---|---|
| Rated torque / current | 1.76 Nm / 13.6 A | 88 Nm |
| Peak torque / current | 4.95 Nm / 38.6 A | 247.5 Nm |
| Rated / no-load speed at 48 V | 2825 / 4368 rpm | 5.9 / 9.1 rad/s |
| Kt, Ke, Kv | 0.129 Nm/A, 10.47 V/krpm, 105 rpm/V | |
| Km | 0.3634 Nm/sqrt(W): copper loss (tau/Km)^2 = 23 W rated, 186 W peak (cold) | |
| Line resistance / inductance | 126 mOhm / 366.7 uH | |
| Rotor inertia | 215.5 g cm^2 | 0.054 kg m^2 at the output (x2500) |
| Electrical / mechanical time constant | 2.91 ms / 0.16 ms | |
| Ambient operating range | -20 to 50 C | |

Not in the datasheet: maximum winding temperature (insulation class; 70 C is used as a provisional stop line),
location of the `temp_mot` sensor, gearbox and torque-sensor ratings. The calibration data (same 50:1 type)
show `tau_link` saturating at about 100 to 115 Nm, far below the 247.5 Nm motor peak, so the usable overload
range at the output is probably set by the gearbox / sensor, not the motor. Hot motor: copper resistance about +0.39 %/C (more loss at the same current),
magnet flux and Kt down roughly 0.1 %/C, so the current-based `tau_mot` overestimates the real torque.

## 0. Open questions (need answers before Phase 1)

| Question | Why it matters |
|---|---|
| How does the robot deployment set the velocity reference (`vel_ref`)? 0, a finite difference of the targets, or something else? | The network is only valid for the same command interface. `trj` sends a velocity reference, `trj_chirp` / `trj_policy` send 0. |
| Which XBot2 HAL safety check fired `on_safety_triggered()` for `trj_chirp` / `trj_policy`? Where are its thresholds (e.g. `ec_all.yaml`)? | Both plugins with zero velocity reference triggered it; `trj` did not. |
| How are jumping policy targets smoothed or rate-limited on the robot (joint_safety filter, interpolation)? | The HAL rejects d(pos_ref)/dt > 20 rad/s, so the robot pipeline must smooth policy targets. The bench should use the same smoothing. |
| Drive position-loop gains (kp, kd) used on the robot? Can they be read through XBot2? | The network is valid only for these gains; they go into every run's metadata. |
| Available loads (mass, radius) and the actuator's rated / peak torque and speed. | Defines the load levels and the safety thresholds. |
| Gearbox type of the bench actuator (harmonic drive? planetary?). | Determines realistic fault modes (Phase 4). |
| Maximum winding temperature (insulation class) and where `temp_mot` is measured (winding or housing); gearbox rated / peak torque. Rated / peak motor torque are known (see Actuator data). | `trj_policy` refuses to start without `stop_motor_temperature`; the sensor location sets the safety margin; the gearbox / torque sensor may limit below 88 / 247.5 Nm. |
| Drive current / I²t limits and their derating with temperature. | In the overload phase the drive may limit current before the temperature stop. |

## 1. Phase 0: verification after the encoder repair (~30 min)

1. Low-speed `trj` run, no load: `q_mot` and `q_link` must agree (output-referred) up to the transmission
   deflection; `v_mot` consistent with the derivative of `q_mot`.
2. Static gravity test with a load: hold several angles (e.g. `trj_policy` with long dwell) and check
   `tau_link ~ m g r sin(q - q0) + offset`. This validates the torque sensor, its sign and zero, and the
   encoder zero.
3. A/B command-interface test, same parameters, small amplitude (0.3 rad):
   `trj` vs `trj_chirp`. If tracking error and `tau_mot` are the same, the drive ignores `vel_ref`.
4. Confirm the HAL safety cause (Phase 0 question) before using larger amplitudes.

## 2. Phase 1: core identification data

Same drive gains everywhere. Per run: `run_trj.sh` with `ACTUATOR_ID`, `LOAD_MASS`, `LOAD_RADIUS`,
`KP`, `KD` set, so every log has a `_meta.yaml`.

Loads (3 levels): none; light (peak gravity torque about 25 % of the rated torque); heavy (about 50 %).

| Trajectory | Settings per load | Runs per load | Duration |
|---|---|---|---|
| `trj_chirp` (zero `vel_ref`) | amplitude 0.2 / 0.5 / 0.8 rad, 0.1 to 1 Hz; plus 0.1 rad, 0.5 to 3 Hz | 4 | 60 s each |
| `trj_policy` | amplitude 0.1 / 0.3 / 0.5 rad, 5 seeds each; one extra set with larger `max_step` for higher speed | 15 + 3 | 60 s each |
| `trj` (with `vel_ref`) | 2 runs, as reference for the command-interface question | 2 | 60 s each |

About 24 min of motion per load, about 45 min including set-up; about 2.5 h for three loads.

What each trajectory contributes:

- Chirp: high speeds (up to about 6 rad/s), systematic frequency coverage, friction at speed.
- Policy-style targets: decoupled `e` and `v` (target jumps while the joint is slow), static loaded holds
  (stiffness, stiction), step transients of the drive, the zero-order-hold rhythm the network sees in
  simulation.

Locked-output torque-mode runs (as in the existing calibration data) cover the high-torque saturation
region; reuse those or repeat them once per actuator.

## 3. Phase 2: temperature and overload

Goals: (a) how long the actuator can run between rated and peak torque before reaching its temperature
limit; (b) how the actuator behaves as it heats up (torque constant, friction, tracking, current limits);
(c) temperature-labelled identification data. Temperature changes the torque constant (magnet), winding
resistance (voltage headroom), gearbox friction (lubricant viscosity) and the torque-sensor offset.

Heating comes from the heavy load running dynamic `trj_policy` targets (no locked output, no torque mode).

Plugin support (`trj_policy`):

- Required: `rated_torque`, `peak_torque` (output N·m) and `stop_motor_temperature` (datasheet limit minus a
  margin; larger margin if `temp_mot` is a housing sensor). `max_link_torque` must not exceed `peak_torque`.
- Temperature stop: `THERMAL STOP` after `temp_mot` stays at or above the stop line for 0.5 s (optional
  `stop_driver_temperature`). Logged `stop_code`: 0 completed, 1 motor temperature, 2 driver temperature,
  3 safety limit, 4 stopped externally; `stop_elapsed` is the run time at the stop.
- `max_start_motor_temperature` (e.g. ambient + 3 °C) refuses to start a warm motor, so every level starts cold.
- Probes: with `probe_duration > 0` the targets alternate probe and heating segments, P H P H ...; every probe
  is the same sequence (`probe_seed`), the first one is the cold baseline. Logged as `probe` / `chunk`.
- Every `status_period` s the plugin prints motor / driver temperature, RMS `tau_mot` (current-based, the
  heat source) over the last period and since start, and the share of time above `rated_torque`.
- The log is streamed to disk during the run, so hour-long runs do not accumulate in memory.

Procedure, per overload level:

1. Choose the level: target RMS `tau_mot` of about 1.0, 1.25, 1.5 and 1.75 x rated (not above peak). With a
   dynamic trajectory the level is set by the load (m, r), the centre angle `q0` (near horizontal for more
   gravity torque) and `amplitude` / `max_step`. Tune in a short trial run by watching the status line, then
   keep the settings fixed.
2. Start cold (`max_start_motor_temperature`), record the ambient temperature (`AMBIENT_TEMP` in `run_trj.sh`).
3. Run with `stop_time: 7200`, `probe_duration: 30`, `heat_duration: 300` until `THERMAL STOP`.
4. Cooling curve: move the load to the hanging position (gravity torque about 0) and run `trj_policy` with
   `amplitude: 0` (and `max_start_motor_temperature: 0`) for 30 to 60 min; it starts once the motor is below
   the stop line. Then let the motor reach ambient before the next level.

Expect one or two levels per session because of the cool-down.

Alternative heating run, speed coverage per temperature: `trj_chirp` with a large `stop_time` (or 0, stopped by
hand) repeats the frequency sweep `freq_min -> freq_max -> freq_min` every `period`, so every cycle covers the
whole speed range and the cycles are directly comparable across temperatures (`cycle`, `freq` are logged). The
torque level is set by changing the load by hand, one log file per load. The temperature stop is optional
(`stop_motor_temperature` in the yaml); without it the operator watches the status line.

Safety: mechanical end stops on the load, nobody in its swing range, an operator watching the status line.
If the drive limits current (I²t) first, the tracking error grows and the plugin stops with `SAFETY STOP`, or
XBot2 fires `on_safety_triggered()`; note which one happened.

Analysis:

- Time to the stop line versus RMS `tau_mot`: the overload capability curve between rated and peak.
- Thermal model from heating and cooling curves: `C dT/dt = k tau_mot^2 (1 + alpha (T - T_ref)) - (T - T_amb) / R`
  (two nodes, winding and housing, if one time constant does not fit). It predicts the allowed time for
  other duty cycles and gives a temperature state for simulation.
- Behaviour versus temperature from the identical probes: tracking error, `tau_link / tau_mot` (torque
  constant and efficiency), friction at speed, current saturation.
- Network: train on the cold part, check whether the prediction error correlates with temperature; only if
  it does, add temperature (from the thermal model in simulation) as an input or randomize the affected
  parameters.

## 4. Phase 3: faults (only reversible emulation, after lab approval)

Primary route: synthetic faults on top of the identified healthy model, in simulation (backlash growth as
a dead zone, local gear damage as a position-periodic torque disturbance, wear as rising friction, sensor
faults as offset / noise / dropouts). Uses: robust policies, fault detection from the residual between
predicted and measured torque, fault-conditioned actuator models.

Optional bench validation, reversible only: e.g. a coupling with known play between output and load to
emulate backlash. No hardware damage (broken teeth) on lab actuators.

## 5. Data handling

- One `.mat` per run plus `<log_file>_meta.yaml`; copy both off `/tmp` after each session.
- Conversion (Actuator_Net repo): `scripts/convert_bench_log.py` reads the three plugin formats and the
  metadata, aligns each measurement with the command in effect, and optionally decimates to the
  simulation step (e.g. 1 kHz to 200 Hz, one file per phase offset).
- Inputs: `q_link` (or `q_mot`), `v_mot` (finer resolution than `v_link`), command `q_target`
  (policy) / `q_ref` (chirp). Target: `tau_link`. History about 40 to 100 ms.
- Splits by whole runs, never by random samples: test on held-out seeds within each load, plus one
  held-out load as an out-of-distribution test. Coverage of `(e, v)` per trajectory type is checked with
  `scripts/plot_coverage.py`.
