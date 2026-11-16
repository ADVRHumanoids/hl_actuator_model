#include "policy_trajectory_plugin.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using namespace XBot;

// 1. 插件加载时调用：只读参数和分配内存，不做硬件阈值检查，
//    这样即使阈值还没填，xbot2-core 也能正常启动，trj 等其他插件不受影响。
bool PolicyTrajectoryPlugin::on_initialize()
{
    _ros = std::make_unique<Ros2Support>(Ros2Support::get_main_node()->create_sub_node(getName()));
    _progress_pub = _ros->advertise<std_msgs::msg::Float32>("progress", 1);
    getParam("~joint", _joint_name);
    getParam("~amplitude", _gen.amplitude);
    getParam("~policy_rate_hz", _gen.policy_rate_hz);
    getParam("~warmup_time", _gen.warmup_time);
    getParam("~dwell_min", _gen.dwell_min);
    getParam("~dwell_max", _gen.dwell_max);
    getParam("~ramp_min", _gen.ramp_min);
    getParam("~ramp_max", _gen.ramp_max);
    getParam("~step_probability", _gen.step_probability);
    getParam("~max_step", _gen.max_step);
    getParam("~seed", _seed);
    getParam("~stop_time", _stop_time);
    getParam("~position_min", _q_min);
    getParam("~position_max", _q_max);
    getParam("~max_velocity", _max_velocity);
    getParam("~max_link_torque", _max_link_torque);
    getParam("~max_tracking_error", _max_tracking_error);
    getParam("~max_ref_rate", _max_ref_rate);
    getParam("~rated_torque", _rated_torque);
    getParam("~peak_torque", _peak_torque);
    getParam("~stop_motor_temperature", _stop_motor_temperature);
    getParam("~stop_driver_temperature", _stop_driver_temperature);
    getParam("~max_start_motor_temperature", _max_start_motor_temperature);
    getParam("~probe_duration", _probe_duration);
    getParam("~heat_duration", _heat_duration);
    getParam("~probe_seed", _probe_seed);
    getParam("~status_period", _status_period);
    getParam("~disable_reference_filter", _disable_reference_filter);
    getParam("~enable_log", _enable_log);
    getParam("~log_file", _log_file);
    // 这几项可在两次实验之间从 GUI / ros2 service 修改，在下一次 on_start 生效；
    // 其余生成参数在初始化时读入，修改需重新加载插件。
    createTunableParam("~amplitude", &_gen.amplitude, Validators::InRange(0.0, 3.2), &_param_upd_queue);
    createTunableParam("~seed", &_seed, Validators::InRange(0.0, 1e9), &_param_upd_queue);
    createTunableParam("~stop_time", &_stop_time, Validators::IsPositive<double>(), &_param_upd_queue);
    createTunableParam("~enable_log", &_enable_log, ParameterValidator<bool>(), &_param_upd_queue);
    createTunableParam("~log_file", &_log_file, ParameterValidator<std::string>(), &_param_upd_queue);
    createTunableParam("~probe_duration", &_probe_duration, Validators::InRange(0.0, 600.0), &_param_upd_queue);
    createTunableParam("~heat_duration", &_heat_duration, Validators::InRange(0.0, 3600.0), &_param_upd_queue);
    _model = ModelInterface::getModel(_robot->getConfigOptions());
    _robot->getJointPositionMinimal(_q);
    _robot->getMotorPosition(_qmot);
    _robot->getJointVelocity(_v);
    _robot->getJointEffort(_tau);
    _robot->getMotorTorque(_tau_mot);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    _q_ref = _q;
    _active = Eigen::VectorXd::Zero(_q.size());
    return true;
}

// 2. 每次启动实验调用：先完成所有检查，再关滤波、切控制模式，失败时不改变台架状态。
void PolicyTrajectoryPlugin::on_start()
{
    _param_upd_queue.run();
    for(double limit : {_q_min, _q_max, _max_velocity, _max_link_torque, _max_tracking_error, _max_ref_rate,
                        _stop_time, _seed})
        if(!std::isfinite(limit)) throw std::invalid_argument("nonfinite policy experiment parameter");
    if(_q_min >= _q_max || _max_velocity <= 0 || _max_link_torque <= 0 || _max_tracking_error <= 0)
        throw std::invalid_argument("configure output-side policy experiment limits first");
    for(double value : {_rated_torque, _peak_torque, _stop_motor_temperature, _stop_driver_temperature,
                        _max_start_motor_temperature, _probe_duration, _heat_duration, _probe_seed, _status_period})
        if(!std::isfinite(value)) throw std::invalid_argument("nonfinite thermal experiment parameter");
    // 过载实验的依据：额定、峰值力矩和电机温度停机线都来自数据手册，没有就不跑。
    if(_rated_torque <= 0 || _peak_torque <= _rated_torque || _stop_motor_temperature <= 0)
        throw std::invalid_argument("configure rated_torque < peak_torque and stop_motor_temperature first (motor datasheet)");
    if(_max_link_torque > _peak_torque)
        throw std::invalid_argument("max_link_torque must not exceed peak_torque");
    if(_stop_driver_temperature < 0 || _max_start_motor_temperature < 0 || _status_period <= 0 || _probe_seed < 0)
        throw std::invalid_argument("invalid thermal experiment parameter");
    if(_max_ref_rate <= 0 || _max_ref_rate >= 20.0)
        throw std::invalid_argument("max_ref_rate must be in (0, 20) rad/s, the XBot2 HAL d(pos_ref)/dt limit");
    _joint = _robot->getDeviceInstance<Hal::JointBase>(_joint_name);
    if(!_joint || _joint->readonly()) throw std::invalid_argument("joint must name a writable joint");
    _idx = _model->getDofIndex(_joint_name);
    if(_idx < 0 || _idx >= _q_ref.size()) throw std::invalid_argument("joint has no scalar DOF");

    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    _joint->sense();
    // 温度检查：已经到停机线就不启动；设了启动上限时，要等电机冷却到上限以下，保证每档都从冷态开始计时。
    _start_motor_temperature = _temp_mot(_idx);
    if(!std::isfinite(_start_motor_temperature))
        throw std::invalid_argument("motor temperature reading is not finite");
    if(_start_motor_temperature >= _stop_motor_temperature)
        throw std::invalid_argument(fmt::format("motor at {} C, already at stop_motor_temperature {} C",
                                                _start_motor_temperature, _stop_motor_temperature));
    if(_max_start_motor_temperature > 0 && _start_motor_temperature > _max_start_motor_temperature)
        throw std::invalid_argument(fmt::format("motor at {} C, above max_start_motor_temperature {} C; let it cool",
                                                _start_motor_temperature, _max_start_motor_temperature));
    if(_stop_driver_temperature > 0 && _temp_dri(_idx) >= _stop_driver_temperature)
        throw std::invalid_argument(fmt::format("driver at {} C, already at stop_driver_temperature {} C",
                                                _temp_dri(_idx), _stop_driver_temperature));
    _gen.seed = static_cast<std::uint64_t>(_seed);
    // q0 来自关节现有位置参考；实验前先按台架流程完成 homing。
    _gen.initial_value = _joint->get_pos_ref();
    // 参数非法或 max_step 太小时在这里抛出。不插探测段时与原来的单段序列完全相同。
    _schedule_time = actuator_collection::generate_schedule(_gen, _stop_time, _heat_duration, _probe_duration,
        static_cast<std::uint64_t>(_probe_seed), _targets, _segments, _probe, _chunk);
    // 目标始终在 q0 ± A 内；目标每帧最多变 max_step，相当于参考斜率上限 max_step*rate。
    if(_gen.initial_value - _gen.amplitude < _q_min || _gen.initial_value + _gen.amplitude > _q_max)
        throw std::invalid_argument("q0 +- amplitude exceeds position limits");
    if(_gen.max_step*_gen.policy_rate_hz > _max_velocity)
        throw std::invalid_argument("max_step * policy_rate_hz exceeds max_velocity");
    // 每次目标变化都要在下一次策略更新前走完，否则就不是零阶保持了。
    if(_gen.max_step > _max_ref_rate/_gen.policy_rate_hz)
        throw std::invalid_argument("max_step exceeds max_ref_rate / policy_rate_hz");
    jinfo("policy targets for joint {}: q0 {}, amplitude {}, {} updates at {} Hz ({} s), seed {}, probes {} s every {} s",
          _joint_name, _gen.initial_value, _gen.amplitude, _targets.size(), _gen.policy_rate_hz, _schedule_time,
          _gen.seed, _probe_duration, _probe_duration > 0 ? _heat_duration : 0.0);
    jinfo("motor {} C, driver {} C at start; stop at motor {} C{}", _start_motor_temperature, _temp_dri(_idx),
          _stop_motor_temperature, _stop_driver_temperature > 0 ? fmt::format(" / driver {} C", _stop_driver_temperature) : "");

    _q_ref = _q;
    _q_ref(_idx) = _gen.initial_value;
    _q_target = _gen.initial_value;
    _k_cmd = 0;
    _active.setZero();
    _active(_idx) = 1;
    // 先建日志：MakeLogger 会因文件名不合法抛出（例如名字里带点 "A0.1"，MatLogger2 只接受 .mat 或无扩展名）。
    // 放在切换控制模式、关滤波之前，启动失败时台架状态不变。
    _appender.reset();
    _logger.reset();
    if(_enable_log) {
        if(_log_file.empty()) _log_file = "/tmp/policy_targets";
        jinfo("logging enabled, log file: {}", _log_file);
        // producer_consumer + MatAppender：rt 线程只往缓冲里放数据，后台线程持续写盘。
        // 缓冲按 60 s 设，写盘线程短暂卡顿也不丢数据；丢了会在状态行和日志 dropped_samples 里看到。
        MatLogger2::Options opt;
        opt.default_buffer_size = std::ceil(60.0/getPeriodSec());
        _logger = MatLogger2::MakeLogger(_log_file, opt);
        _logger->set_buffer_mode(VariableBuffer::Mode::producer_consumer);
        for(const auto& name : {"q_ref", "q_link", "q_mot", "v_link", "v_mot", "tau_link", "tau_mot",
                                "tau_ref", "temp_mot", "temp_dri", "active"})
            _logger->create(name, _q.size());
        for(const auto& name : {"time", "trajectory_time", "valid", "segment", "policy_index", "q_target",
                                "probe", "chunk"})
            _logger->create(name, 1);
        // 结束时各写一次：停止原因、停止时的实验时间、丢掉的周期数。
        for(const auto& name : {"stop_code", "stop_elapsed", "dropped_samples"})
            _logger->create(name, 1);
        // 本次实验的生成参数各记一次，便于离线用同一种子复现目标序列。
        for(const auto& [name, value] : std::initializer_list<std::pair<const char*, double>>{
                {"cfg_seed", _seed}, {"cfg_amplitude", _gen.amplitude},
                {"cfg_policy_rate_hz", _gen.policy_rate_hz}, {"cfg_max_step", _gen.max_step},
                {"cfg_initial_value", _gen.initial_value},
                {"cfg_reference_filter_disabled", _disable_reference_filter ? 1.0 : 0.0},
                {"cfg_probe_duration", _probe_duration}, {"cfg_heat_duration", _heat_duration},
                {"cfg_probe_seed", _probe_seed}, {"cfg_rated_torque", _rated_torque},
                {"cfg_peak_torque", _peak_torque}, {"cfg_stop_motor_temperature", _stop_motor_temperature},
                {"cfg_stop_driver_temperature", _stop_driver_temperature},
                {"cfg_start_motor_temperature", _start_motor_temperature}}) {
            _logger->create(name, 1);
            _logger->add(name, value);
        }
        _appender = MatAppender::MakeInstance();
        _appender->add_logger(_logger);
        _appender->start_flush_thread();
    }
    std::map<std::string, ControlMode> modes;
    modes[_joint_name] = ControlMode::Position();
    if(!_robot->setControlMode(modes))
        throw std::runtime_error("failed to set position control mode");
    // 阶跃目标经过 2 Hz 参考滤波后就不是阶跃了；默认像上游 trj 一样在实验期间关掉滤波，结束后恢复。
    // 若机器人部署时也开着这个滤波，应设 disable_reference_filter: false，让网络把滤波一起辨识进去。
    if(_disable_reference_filter) {
        Hal::JointSafety::enable_filter(false, -1);
        _filter_disabled = true;
    }
    _elapsed = 0;
    _have_command = false;
    _win_sq = _win_over = _win_n = _tot_sq = _tot_over = _tot_n = 0;
    _next_status = _status_period;
    _hot_motor_time = _hot_driver_time = 0;
    _dropped = 0;
    _stop_code = 4;
    _clock_start = std::chrono::steady_clock::now();
}
void PolicyTrajectoryPlugin::starting() { start_completed(); }

// 3. rt_main 线程周期调用（1 kHz）：读实测 → 检查 → 记录 → 按时间查表下发目标。
void PolicyTrajectoryPlugin::run()
{
    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getJointVelocity(_v);
    _robot->getJointEffort(_tau);
    _robot->getMotorTorque(_tau_mot);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    const double dt = getPeriodSec();
    const double q = _q(_idx), v = _v(_idx), tau = _tau(_idx), e = q - _q_ref(_idx);
    const double tau_mot = _tau_mot(_idx), temp_mot = _temp_mot(_idx), temp_dri = _temp_dri(_idx);
    // 安全检查。越限时绝不能 throw：rt 线程里未捕获的异常会让整个 xbot2-core terminate（core dumped）。
    // 这里只记下原因，记录完本周期数据后停止插件；参考保持上一周期的值，不会触发 HAL 的变化率检查。
    // 跟踪误差包含目标阶跃本身（最多 max_step）和执行器的跟踪滞后，阈值要留出余量。
    std::string violation;
    if(!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(tau) || !std::isfinite(tau_mot) ||
       !std::isfinite(temp_mot) || (_stop_driver_temperature > 0 && !std::isfinite(temp_dri)))
        violation = fmt::format("nonfinite telemetry: q {} v {} tau_link {} tau_mot {} temp_mot {} temp_dri {}",
                                q, v, tau, tau_mot, temp_mot, temp_dri);
    else if(q < _q_min || q > _q_max)
        violation = fmt::format("position {} outside [{}, {}] rad", q, _q_min, _q_max);
    else if(std::abs(v) > _max_velocity)
        violation = fmt::format("|velocity| {} > max_velocity {} rad/s", std::abs(v), _max_velocity);
    else if(std::abs(tau) > _max_link_torque)
        violation = fmt::format("|tau_link| {} > max_link_torque {} Nm", std::abs(tau), _max_link_torque);
    else if(std::abs(e) > _max_tracking_error)
        violation = fmt::format("|q - q_ref| {} > max_tracking_error {} rad (q {}, q_ref {})",
                                std::abs(e), _max_tracking_error, q, _q_ref(_idx));
    // 温度停机线：连续 0.5 s 超过才停，避免单个读数跳变误停。这是温升实验的正常终点，不算安全违规。
    _hot_motor_time = temp_mot >= _stop_motor_temperature ? _hot_motor_time + dt : 0.0;
    _hot_driver_time = _stop_driver_temperature > 0 && temp_dri >= _stop_driver_temperature ? _hot_driver_time + dt : 0.0;
    // 力矩统计用 tau_mot（由电流估计）：发热来自电流，输出端力矩传感器还含减速器摩擦的影响。
    if(violation.empty()) {
        const double over = std::abs(tau_mot) > _rated_torque ? 1.0 : 0.0;
        _win_sq += tau_mot*tau_mot; _win_over += over; _win_n += 1;
        _tot_sq += tau_mot*tau_mot; _tot_over += over; _tot_n += 1;
    }
    // 先记录当前反馈和上一周期已下发的指令（q_ref 限速后、q_target 限速前）；第一行 valid=0。
    if(_logger) {
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - _clock_start).count();
        if(!_logger->add("time", wall)) ++_dropped;  // producer_consumer 缓冲满时 add 返回 false
        _logger->add("trajectory_time", _elapsed);
        _logger->add("valid", _have_command ? 1.0 : 0.0);
        _logger->add("segment", double(_segments[_k_cmd]));
        _logger->add("policy_index", double(_k_cmd));
        _logger->add("probe", double(_probe[_k_cmd]));
        _logger->add("chunk", double(_chunk[_k_cmd]));
        _logger->add("q_target", _q_target);
        _logger->add("active", _active);
        _logger->add("q_ref", _q_ref);
        _logger->add("q_link", _q);
        _logger->add("v_link", _v);
        _logger->add("tau_link", _tau);
        _logger->add("tau_mot", _tau_mot);
        _logger->add("temp_mot", _temp_mot);
        _logger->add("temp_dri", _temp_dri);
        _robot->getMotorPosition(_qmot);
        _robot->positionToMinimal(_qmot, _q);
        _logger->add("q_mot", _q);
        _robot->getMotorVelocity(_v);
        _logger->add("v_mot", _v);
        _robot->getEffortReferenceFeedback(_tau);
        _logger->add("tau_ref", _tau);
    }
    // 定期打印温度和力矩统计，便于在线判断这一档是否落在额定和峰值之间。
    if(_elapsed >= _next_status && _win_n > 0) {
        jinfo("t {:.0f} s{} | motor {:.1f} C, driver {:.1f} C | tau_mot rms {:.1f} Nm (last {:.0f} s), {:.1f} Nm "
              "since start | above rated {:.0f} % (last), {:.0f} % (since start){}",
              _elapsed, _probe[_k_cmd] ? " [probe]" : "", temp_mot, temp_dri, std::sqrt(_win_sq/_win_n),
              _status_period, std::sqrt(_tot_sq/_tot_n), 100*_win_over/_win_n, 100*_tot_over/_tot_n,
              _dropped ? fmt::format(" | DROPPED {} log samples", _dropped) : std::string());
        _win_sq = _win_over = _win_n = 0;
        _next_status += _status_period;
    }
    if(!violation.empty()) {
        jinfo("SAFETY STOP trj_policy at t = {} s: {}", _elapsed, violation);
        _stop_code = 3;
        stop();
        return;
    }
    if(_hot_motor_time >= 0.5 || _hot_driver_time >= 0.5) {
        const bool motor = _hot_motor_time >= 0.5;
        jinfo("THERMAL STOP trj_policy at t = {} s: {} {} C >= {} C (motor started at {} C); "
              "tau_mot rms {} Nm, {} % of the time above rated {} Nm",
              _elapsed, motor ? "motor" : "driver", motor ? temp_mot : temp_dri,
              motor ? _stop_motor_temperature : _stop_driver_temperature, _start_motor_temperature,
              _tot_n > 0 ? std::sqrt(_tot_sq/_tot_n) : 0.0, _tot_n > 0 ? 100*_tot_over/_tot_n : 0.0, _rated_torque);
        _stop_code = motor ? 1 : 2;
        stop();
        return;
    }
    // 发出终点目标（= q0）后再多记录一个周期。
    if(_elapsed > _schedule_time + dt/2) { _stop_code = 0; stop(); return; }
    // 策略目标按零阶保持查表；实际下发的参考向它移动，每周期最多 max_ref_rate*dt，
    // 满足 HAL 的 d(pos_ref)/dt 限制。preflight 已保证每次目标变化在下一次策略更新前走完。
    _k_cmd = std::clamp(int(std::floor(_elapsed*_gen.policy_rate_hz + 1e-9)), 0, int(_targets.size()) - 1);
    _q_target = _targets[_k_cmd];
    const double dmax = _max_ref_rate*dt;
    _q_ref(_idx) += std::clamp(_q_target - _q_ref(_idx), -dmax, dmax);
    _joint->set_pos_ref(_q_ref(_idx));
    _joint->set_vel_ref(0.0);  // 策略只给目标位置，不给速度前馈
    _joint->move();
    _have_command = true;
    _elapsed += dt;
    std_msgs::msg::Float32 msg;
    msg.data = std::min(1.0, _elapsed/_schedule_time);
    _progress_pub->publish(msg);
}

// 正常停止或异常退出：恢复参考滤波，写入停止原因，停掉写盘线程，销毁 logger 写出剩余数据。
void PolicyTrajectoryPlugin::finish()
{
    if(_filter_disabled) {
        Hal::JointSafety::enable_filter(true, -1);
        _filter_disabled = false;
    }
    jinfo("trj_policy finished: stop_code {} at t = {} s, {} log samples dropped", _stop_code, _elapsed, _dropped);
    if(_logger) {
        _logger->add("stop_code", double(_stop_code));
        _logger->add("stop_elapsed", _elapsed);
        _logger->add("dropped_samples", double(_dropped));
    }
    _appender.reset();
    _logger.reset();
}
void PolicyTrajectoryPlugin::on_stop() { finish(); }
void PolicyTrajectoryPlugin::on_abort() { finish(); }

// 注册名必须与 CMake 目标和 YAML 中的 type 一致。
XBOT2_REGISTER_PLUGIN(PolicyTrajectoryPlugin, policy_trajectory_plugin_haotian)
