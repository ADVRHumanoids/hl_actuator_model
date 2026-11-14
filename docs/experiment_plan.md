# Actuator identification on the bench: experiment plan

Goal: identify an actuator network for sim-to-real. In simulation the network replaces the ideal PD
actuator: from the history of the position error `e = q - q_des` and the measured velocity `v` it
predicts the output torque `tau_link`, every simulation step. On the robot nothing changes: the policy
sends position targets to the drives. Secondary studies: temperature and (emulated) faults.

All numbers below are proposals to discuss; thresholds must be confirmed against the actuator ratings.

## 0. Open questions (need answers before Phase 1)

| Question | Why it matters |
|---|---|
| How does the robot deployment set the velocity reference (`vel_ref`)? 0, a finite difference of the targets, or something else? | The network is only valid for the same command interface. `trj` sends a velocity reference, `trj_chirp` / `trj_policy` send 0. |
| Which XBot2 HAL safety check fired `on_safety_triggered()` for `trj_chirp` / `trj_policy`? Where are its thresholds (e.g. `ec_all.yaml`)? | Both plugins with zero velocity reference triggered it; `trj` did not. |
| How are jumping policy targets smoothed or rate-limited on the robot (joint_safety filter, interpolation)? | The HAL rejects d(pos_ref)/dt > 20 rad/s, so the robot pipeline must smooth policy targets. The bench should use the same smoothing. |
| Drive position-loop gains (kp, kd) used on the robot? Can they be read through XBot2? | The network is valid only for these gains; they go into every run's metadata. |
| Available loads (mass, radius) and the actuator's rated / peak torque and speed. | Defines the load levels and the safety thresholds. |
| Gearbox type of the bench actuator (harmonic drive? planetary?). | Determines realistic fault modes (Phase 4). |

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

## 3. Phase 2: temperature (~1.5 h)

Temperature changes the torque constant (magnet), winding resistance (voltage headroom), gearbox
friction (lubricant viscosity) and the torque-sensor offset. `temp_mot` and `temp_dri` are already logged.

1. Cold start: first runs of the day, before anything else (one chirp + one policy run, heavy load).
2. Heating: repeated policy runs at the heavy load until the motor temperature stops rising.
3. Hot: repeat the cold-start runs.
4. Record the ambient temperature in `NOTE`.

Analysis: train on normal-temperature data, then check whether the prediction error correlates with
temperature. Only if it does, add temperature as a network input (with a first-order thermal model in
simulation) or randomize the affected parameters.

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
