#include "static_hold_plugin.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using namespace XBot;

// 1. 插件加载时调用：只读参数和分配内存，不做阈值检查，这样阈值没填时 xbot2-core 和其他插件照常加载。
bool StaticHoldPlugin::on_initialize()
{
    _ros = std::make_unique<Ros2Support>(Ros2Support::get_main_node()->create_sub_node(getName()));
    _progress_pub = _ros->advertise<std_msgs::msg::Float32>("progress", 1);
    getParam("~joint", _joint_name);
    getParam("~offsets", _offsets);
    getParam("~hold_time", _schedule.hold_time);
    getParam("~settle_time", _schedule.settle_time);
    getParam("~move_velocity", _schedule.move_velocity);
    getParam("~min_move_time", _schedule.min_move_time);
    getParam("~position_min", _q_min);
    getParam("~position_max", _q_max);
    getParam("~max_velocity", _max_velocity);
    getParam("~max_link_torque", _max_link_torque);
    getParam("~max_tracking_error", _max_tracking_error);
    getParam("~stop_motor_temperature", _stop_motor_temperature);
    getParam("~velocity_feedforward", _velocity_feedforward);
    getParam("~disable_reference_filter", _disable_reference_filter);
    getParam("~enable_log", _enable_log);
    getParam("~log_file", _log_file);
    // 这几项可在两次实验之间从 GUI / ros2 service（run_trj.sh）修改，在下一次 on_start 生效。
    createTunableParam("~offsets", &_offsets, ParameterValidator<std::vector<double>>(), &_param_upd_queue);
    createTunableParam("~hold_time", &_schedule.hold_time, Validators::IsPositive<double>(), &_param_upd_queue);
    createTunableParam("~move_velocity", &_schedule.move_velocity, Validators::InRange(0.01, 2.0), &_param_upd_queue);
    createTunableParam("~enable_log", &_enable_log, ParameterValidator<bool>(), &_param_upd_queue);
    createTunableParam("~log_file", &_log_file, ParameterValidator<std::string>(), &_param_upd_queue);
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

// 2. 每次启动调用：先完成所有检查，再建日志、切控制模式、关滤波，失败时不改变台架状态。
void StaticHoldPlugin::on_start()
{
    _param_upd_queue.run();
    for(double limit : {_q_min, _q_max, _max_velocity, _max_link_torque, _max_tracking_error, _stop_motor_temperature})
        if(!std::isfinite(limit)) throw std::invalid_argument("nonfinite static test limit");
    if(_q_min >= _q_max || _max_velocity <= 0 || _max_link_torque <= 0 || _max_tracking_error <= 0)
        throw std::invalid_argument("configure output-side static test limits first");
    if(_schedule.move_velocity >= _max_velocity)
        throw std::invalid_argument("move_velocity must be below max_velocity");
    _joint = _robot->getDeviceInstance<Hal::JointBase>(_joint_name);
    if(!_joint || _joint->readonly()) throw std::invalid_argument("joint must name a writable joint");
    _idx = _model->getDofIndex(_joint_name);
    if(_idx < 0 || _idx >= _q_ref.size()) throw std::invalid_argument("joint has no scalar DOF");

    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getTemperatureMotor(_temp_mot);
    _joint->sense();
    if(_stop_motor_temperature > 0 && !(_temp_mot(_idx) < _stop_motor_temperature))
        throw std::invalid_argument(fmt::format("motor at {} C, not below stop_motor_temperature {} C",
                                                _temp_mot(_idx), _stop_motor_temperature));
    // q0 来自关节现有位置参考。建议从配重自然下垂的位置开始，分析脚本把 q0 当作重力矩为零的角度来判断力矩方向。
    _schedule.offsets = _offsets;
    _schedule.initial_value = _joint->get_pos_ref();
    _schedule.build();  // 参数非法时在这里抛出
    const auto [lo, hi] = _schedule.range();
    if(lo < _q_min || hi > _q_max)
        throw std::invalid_argument(fmt::format("targets [{}, {}] exceed position limits [{}, {}]", lo, hi, _q_min, _q_max));
    _cursor = 0;
    _q_ref = _q;
    _q_ref(_idx) = _schedule.initial_value;
    _q_cmd = _schedule.initial_value;
    _v_cmd = 0;
    _phase_cmd = actuator_collection::StaticSchedule::Settle;
    _point_cmd = -1;
    _active.setZero();
    _active(_idx) = 1;

    _appender.reset();
    _logger.reset();
    if(_enable_log) {
        if(_log_file.empty()) _log_file = "/tmp/static_hold";
        jinfo("logging enabled, log file: {}", _log_file);
        MatLogger2::Options opt;
        opt.default_buffer_size = std::ceil(60.0/getPeriodSec());
        _logger = MatLogger2::MakeLogger(_log_file, opt);  // 文件名不合法时在这里抛出，台架状态未变
        _logger->set_buffer_mode(VariableBuffer::Mode::producer_consumer);
        for(const auto& name : {"q_ref", "q_link", "q_mot", "v_link", "v_mot", "tau_link", "tau_mot",
                                "tau_ref", "temp_mot", "temp_dri", "active"})
            _logger->create(name, _q.size());
        // phase：0 移动、1 稳定、2 保持；point：第几个位置（回到 q0 的最后一段为 -1）。
        for(const auto& name : {"time", "trajectory_time", "valid", "phase", "point", "v_ref"})
            _logger->create(name, 1);
        for(const auto& name : {"stop_code", "stop_elapsed", "dropped_samples"})
            _logger->create(name, 1);
        for(const auto& [name, value] : std::initializer_list<std::pair<const char*, double>>{
                {"cfg_initial_value", _schedule.initial_value}, {"cfg_hold_time", _schedule.hold_time},
                {"cfg_settle_time", _schedule.settle_time}, {"cfg_move_velocity", _schedule.move_velocity},
                {"cfg_velocity_feedforward", _velocity_feedforward ? 1.0 : 0.0},
                {"cfg_reference_filter_disabled", _disable_reference_filter ? 1.0 : 0.0}}) {
            _logger->create(name, 1);
            _logger->add(name, value);
        }
        _logger->create("cfg_offsets", int(_offsets.size()));
        Eigen::VectorXd offsets = Eigen::VectorXd::Zero(_offsets.size());
        for(std::size_t i = 0; i < _offsets.size(); ++i) offsets(i) = _offsets[i];
        _logger->add("cfg_offsets", offsets);
        _appender = MatAppender::MakeInstance();
        _appender->add_logger(_logger);
        _appender->start_flush_thread();
    }
    std::map<std::string, ControlMode> modes;
    modes[_joint_name] = ControlMode::Position();
    if(!_robot->setControlMode(modes))
        throw std::runtime_error("failed to set position control mode");
    if(_disable_reference_filter) {
        Hal::JointSafety::enable_filter(false, -1);
        _filter_disabled = true;
    }
    _elapsed = 0;
    _hot_time = 0;
    _have_command = false;
    _acc_point = -1;
    _acc_n = _acc_q = _acc_tau = _acc_tau_mot = 0;
    _dropped = 0;
    _stop_code = 4;
    _clock_start = std::chrono::steady_clock::now();
    jinfo("static test on joint {}: q0 {}, {} positions, hold {} s (first {} s settling), {:.0f} s in total",
          _joint_name, _schedule.initial_value, _offsets.size(), _schedule.hold_time, _schedule.settle_time,
          _schedule.duration);
}
void StaticHoldPlugin::starting() { start_completed(); }

// 一个位置的保持段结束时打印平均值，现场就能看出力矩是否随角度按 sin 变化。
void StaticHoldPlugin::print_point()
{
    if(_acc_n <= 0) return;
    jinfo("point {} done: q_link {:.4f} rad (q - q0 {:.4f}), tau_link {:.2f} Nm, tau_mot {:.2f} Nm",
          _acc_point, _acc_q/_acc_n, _acc_q/_acc_n - _schedule.initial_value, _acc_tau/_acc_n, _acc_tau_mot/_acc_n);
    _acc_n = _acc_q = _acc_tau = _acc_tau_mot = 0;
}

// 3. rt_main 线程周期调用（1 kHz）：读实测 → 检查 → 记录 → 按时间查表下发参考。
void StaticHoldPlugin::run()
{
    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getJointVelocity(_v);
    _robot->getJointEffort(_tau);
    _robot->getMotorTorque(_tau_mot);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    const double dt = getPeriodSec();
    const double q = _q(_idx), v = _v(_idx), tau = _tau(_idx), e = q - _q_cmd;
    // 越限时不 throw（rt 线程里的异常会让 xbot2-core terminate），记录本周期后停止，参考保持上一周期的值。
    std::string violation;
    if(!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(tau))
        violation = fmt::format("nonfinite telemetry: q {} v {} tau_link {}", q, v, tau);
    else if(q < _q_min || q > _q_max)
        violation = fmt::format("position {} outside [{}, {}] rad", q, _q_min, _q_max);
    else if(std::abs(v) > _max_velocity)
        violation = fmt::format("|velocity| {} > max_velocity {} rad/s", std::abs(v), _max_velocity);
    else if(std::abs(tau) > _max_link_torque)
        violation = fmt::format("|tau_link| {} > max_link_torque {} Nm", std::abs(tau), _max_link_torque);
    else if(std::abs(e) > _max_tracking_error)
        violation = fmt::format("|q - q_ref| {} > max_tracking_error {} rad (q {}, q_ref {})",
                                std::abs(e), _max_tracking_error, q, _q_cmd);
    _hot_time = _stop_motor_temperature > 0 && !(_temp_mot(_idx) < _stop_motor_temperature) ? _hot_time + dt : 0.0;

    // 记录当前反馈和上一周期已下发的指令；第一行 valid=0。
    if(_logger) {
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - _clock_start).count();
        if(!_logger->add("time", wall)) ++_dropped;
        _logger->add("trajectory_time", _elapsed);
        _logger->add("valid", _have_command ? 1.0 : 0.0);
        _logger->add("phase", double(_phase_cmd));
        _logger->add("point", double(_point_cmd));
        _logger->add("v_ref", _v_cmd);
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
    if(_phase_cmd == actuator_collection::StaticSchedule::Hold && _point_cmd >= 0) {
        _acc_point = _point_cmd;
        _acc_n += 1;
        _acc_q += q;
        _acc_tau += tau;
        _acc_tau_mot += _tau_mot(_idx);
    }
    if(!violation.empty()) {
        jinfo("SAFETY STOP trj_static at t = {} s: {}", _elapsed, violation);
        _stop_code = 3;
        stop();
        return;
    }
    if(_hot_time >= 0.5) {
        jinfo("THERMAL STOP trj_static at t = {} s: motor {} C >= {} C", _elapsed, _temp_mot(_idx), _stop_motor_temperature);
        _stop_code = 1;
        stop();
        return;
    }
    if(_elapsed > _schedule.duration + dt/2) {
        print_point();
        _stop_code = 0;
        stop();
        return;
    }
    const auto s = _schedule.sample(_elapsed, _cursor);
    if(s.point != _point_cmd) print_point();  // 换到下一个位置
    _q_cmd = s.q;
    _v_cmd = _velocity_feedforward ? s.v : 0.0;
    _phase_cmd = s.phase;
    _point_cmd = s.point;
    _q_ref(_idx) = _q_cmd;
    _joint->set_pos_ref(_q_cmd);
    _joint->set_vel_ref(_v_cmd);
    _joint->move();
    _have_command = true;
    _elapsed += dt;
    std_msgs::msg::Float32 msg;
    msg.data = std::min(1.0, _elapsed/_schedule.duration);
    _progress_pub->publish(msg);
}

// 正常停止或异常退出：恢复参考滤波，写入停止原因，停掉写盘线程，销毁 logger 写出剩余数据。
void StaticHoldPlugin::finish()
{
    if(_filter_disabled) {
        Hal::JointSafety::enable_filter(true, -1);
        _filter_disabled = false;
    }
    jinfo("trj_static finished: stop_code {} at t = {} s, {} log samples dropped", _stop_code, _elapsed, _dropped);
    if(_logger) {
        _logger->add("stop_code", double(_stop_code));
        _logger->add("stop_elapsed", _elapsed);
        _logger->add("dropped_samples", double(_dropped));
    }
    _appender.reset();
    _logger.reset();
}
void StaticHoldPlugin::on_stop() { finish(); }
void StaticHoldPlugin::on_abort() { finish(); }

// 注册名必须与 CMake 目标和 YAML 中的 type 一致。
XBOT2_REGISTER_PLUGIN(StaticHoldPlugin, static_hold_plugin_haotian)
