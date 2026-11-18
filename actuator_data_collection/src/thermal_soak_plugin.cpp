#include "thermal_soak_plugin.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using namespace XBot;
using actuator_collection::ThermalSoak;

namespace {
const char* phase_name(int phase)
{
    switch(phase) {
    case ThermalSoak::Heat: return "Heat";
    case ThermalSoak::Soak: return "Soak";
    case ThermalSoak::Release: return "Release";
    default: return "Done";
    }
}
}

// 1. 插件加载时调用：只读参数，不做检查，这样阈值没填时 xbot2-core 和其他插件照常加载。
bool ThermalSoakPlugin::on_initialize()
{
    getParam("~joint", _joint_name);
    getParam("~setpoint", _ctrl.setpoint);
    getParam("~tolerance", _ctrl.tolerance);
    getParam("~soak_time", _ctrl.soak_time);
    getParam("~max_heat_time", _ctrl.max_heat_time);
    getParam("~max_torque", _ctrl.max_torque);
    getParam("~drive_stiffness", _ctrl.drive_stiffness);
    getParam("~torque_rate", _ctrl.torque_rate);
    getParam("~kp", _ctrl.kp);
    getParam("~ki", _ctrl.ki);
    getParam("~kd", _ctrl.kd);
    getParam("~temperature_filter", _ctrl.temperature_filter);
    getParam("~direction", _direction);
    getParam("~torque_offset", _torque_offset);
    getParam("~max_link_torque", _max_link_torque);
    getParam("~max_position_drift", _max_position_drift);
    getParam("~max_velocity", _max_velocity);
    getParam("~stop_motor_temperature", _stop_motor_temperature);
    getParam("~stop_driver_temperature", _stop_driver_temperature);
    getParam("~release_only", _release_only);
    getParam("~status_period", _status_period);
    getParam("~log_period", _log_period);
    getParam("~disable_reference_filter", _disable_reference_filter);
    getParam("~enable_log", _enable_log);
    getParam("~log_file", _log_file);
    // setpoint、release 运行中生效（run() 里处理 _live_queue）；其余在下一次启动时生效。
    createTunableParam("~setpoint", &_ctrl.setpoint, Validators::InRange(20.0, 90.0), &_live_queue);
    createTunableParam("~release", &_release, ParameterValidator<bool>(), &_live_queue);
    createTunableParam("~soak_time", &_ctrl.soak_time, Validators::InRange(0.0, 36000.0), &_param_upd_queue);
    createTunableParam("~max_torque", &_ctrl.max_torque, Validators::InRange(0.0, 300.0), &_param_upd_queue);
    createTunableParam("~max_link_torque", &_max_link_torque, Validators::InRange(0.0, 300.0), &_param_upd_queue);
    createTunableParam("~tolerance", &_ctrl.tolerance, Validators::InRange(0.1, 10.0), &_param_upd_queue);
    createTunableParam("~max_heat_time", &_ctrl.max_heat_time, Validators::InRange(0.0, 36000.0), &_param_upd_queue);
    createTunableParam("~torque_rate", &_ctrl.torque_rate, Validators::InRange(0.1, 20.0), &_param_upd_queue);
    createTunableParam("~kp", &_ctrl.kp, Validators::InRange(0.0, 10.0), &_param_upd_queue);
    createTunableParam("~ki", &_ctrl.ki, Validators::InRange(0.0, 1.0), &_param_upd_queue);
    createTunableParam("~kd", &_ctrl.kd, Validators::InRange(0.0, 1000.0), &_param_upd_queue);
    createTunableParam("~release_only", &_release_only, ParameterValidator<bool>(), &_param_upd_queue);
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
    return true;
}

// 2. 每次启动调用：先完成所有检查，再建日志、切控制模式、关滤波，失败时不改变台架状态。
void ThermalSoakPlugin::on_start()
{
    _param_upd_queue.run();
    _live_queue.run();
    _release = false;
    const std::string err = _ctrl.validate();
    if(!err.empty()) throw std::invalid_argument(err);
    if(_direction != 1.0 && _direction != -1.0) throw std::invalid_argument("direction must be 1 or -1");
    for(double limit : {_max_link_torque, _max_position_drift, _max_velocity, _stop_motor_temperature,
                        _stop_driver_temperature, _torque_offset, _status_period, _log_period})
        if(!std::isfinite(limit)) throw std::invalid_argument("nonfinite trj_heat limit");
    if(_max_link_torque <= 0 || _max_position_drift <= 0 || _max_velocity <= 0 || _stop_motor_temperature <= 0)
        throw std::invalid_argument("configure max_link_torque, max_position_drift, max_velocity, stop_motor_temperature first");
    if(_ctrl.max_torque >= _max_link_torque)
        throw std::invalid_argument(fmt::format("max_torque {} must be below max_link_torque {}", _ctrl.max_torque, _max_link_torque));
    if(_ctrl.setpoint + _ctrl.tolerance >= _stop_motor_temperature)
        throw std::invalid_argument(fmt::format("setpoint {} + tolerance {} must be below stop_motor_temperature {}",
                                                _ctrl.setpoint, _ctrl.tolerance, _stop_motor_temperature));
    if(_status_period <= 0 || _log_period <= 0) throw std::invalid_argument("status_period and log_period must be > 0");
    _joint = _robot->getDeviceInstance<Hal::JointBase>(_joint_name);
    if(!_joint || _joint->readonly()) throw std::invalid_argument("joint must name a writable joint");
    _idx = _model->getDofIndex(_joint_name);
    if(_idx < 0 || _idx >= _q.size()) throw std::invalid_argument("joint has no scalar DOF");

    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    _joint->sense();
    const double temp = _temp_mot(_idx);
    if(!std::isfinite(temp) || temp <= 0)
        throw std::invalid_argument(fmt::format("motor temperature reading {} C is not valid", temp));
    if(!(temp < _stop_motor_temperature))
        throw std::invalid_argument(fmt::format("motor at {} C, not below stop_motor_temperature {} C", temp, _stop_motor_temperature));
    // 固定位置取启动时的实测位置；参考从现有位置参考无扰接管（偏移按 torque_rate 衰减），
    // 上次被外部停止留下的偏移也这样释放。
    _q_pin = _q(_idx);
    const double pos_ref = _joint->get_pos_ref();
    if(!std::isfinite(pos_ref)) throw std::invalid_argument("position reference is not finite");
    _ctrl.reset(_direction*(pos_ref - _q_pin), _release_only);
    _q_cmd = pos_ref;

    _appender.reset();
    _logger.reset();
    if(_enable_log) {
        if(_log_file.empty()) _log_file = "/tmp/heat";
        jinfo("logging enabled, log file: {}", _log_file);
        MatLogger2::Options opt;
        opt.default_buffer_size = std::ceil(120.0/_log_period);
        _logger = MatLogger2::MakeLogger(_log_file, opt);  // 文件名不合法时在这里抛出，台架状态未变
        _logger->set_buffer_mode(VariableBuffer::Mode::producer_consumer);
        // heat_phase：0 升温、1 保温、2 释放、3 完成（转换脚本据此识别并跳过这类日志）。每 log_period 记一行。
        for(const auto& name : {"time", "trajectory_time", "heat_phase", "temp_mot", "temp_dri", "temp_filtered",
                                "setpoint", "u", "pid_p", "pid_i", "pid_d", "torque_cmd", "soak_elapsed",
                                "q_ref", "q_link", "q_mot", "v_link", "tau_link", "tau_mot"})
            _logger->create(name, 1);
        for(const auto& name : {"stop_code", "stop_elapsed", "dropped_samples"})
            _logger->create(name, 1);
        for(const auto& [name, value] : std::initializer_list<std::pair<const char*, double>>{
                {"cfg_setpoint", _ctrl.setpoint}, {"cfg_tolerance", _ctrl.tolerance}, {"cfg_soak_time", _ctrl.soak_time},
                {"cfg_max_heat_time", _ctrl.max_heat_time}, {"cfg_max_torque", _ctrl.max_torque},
                {"cfg_drive_stiffness", _ctrl.drive_stiffness}, {"cfg_torque_rate", _ctrl.torque_rate},
                {"cfg_kp", _ctrl.kp}, {"cfg_ki", _ctrl.ki}, {"cfg_kd", _ctrl.kd}, {"cfg_direction", _direction},
                {"cfg_q_pin", _q_pin}, {"cfg_release_only", _release_only ? 1.0 : 0.0},
                {"cfg_start_motor_temperature", temp}}) {
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
    if(_disable_reference_filter) {
        Hal::JointSafety::enable_filter(false, -1);
        _filter_disabled = true;
    }
    _elapsed = 0;
    _next_status = 0;
    _next_log = 0;
    _hot_mot = _hot_dri = 0;
    _dropped = 0;
    _stop_code = 4;
    _clock_start = std::chrono::steady_clock::now();
    if(_release_only)
        jinfo("trj_heat RELEASE ONLY on joint {}: reference offset {:.3f} rad -> 0 at {} Nm/s (set release_only=false for heating)",
              _joint_name, pos_ref - _q_pin, _ctrl.torque_rate);
    else
        jinfo("trj_heat on joint {}: pinned at {:.4f} rad, motor {:.1f} C -> setpoint {} C, max stall torque {} Nm "
              "(direction {:+.0f}), soak {} s; release early: ros2 service call /xbotcore/set_parameter_values "
              "xbot_msgs/srv/SetString \"{{request: {{/{}/release: true}}}}\"",
              _joint_name, _q_pin, temp, _ctrl.setpoint, _ctrl.max_torque, _direction, _ctrl.soak_time, getName());
}
void ThermalSoakPlugin::starting() { start_completed(); }

void ThermalSoakPlugin::log_row(double q, double v, double tau)
{
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - _clock_start).count();
    if(!_logger->add("time", wall)) ++_dropped;
    _logger->add("trajectory_time", _elapsed);
    _logger->add("heat_phase", double(_ctrl.phase));
    _logger->add("temp_mot", _temp_mot(_idx));
    _logger->add("temp_dri", _temp_dri(_idx));
    _logger->add("temp_filtered", _ctrl.temp_filtered);
    _logger->add("setpoint", _ctrl.setpoint);
    _logger->add("u", _ctrl.u);
    _logger->add("pid_p", _ctrl.p);
    _logger->add("pid_i", _ctrl.i);
    _logger->add("pid_d", _ctrl.d);
    _logger->add("torque_cmd", _direction*_ctrl.torque_cmd);
    _logger->add("soak_elapsed", _ctrl.soak_elapsed);
    _logger->add("q_ref", _q_cmd);
    _logger->add("q_link", q);
    _logger->add("v_link", v);
    _logger->add("tau_link", tau);
    _logger->add("tau_mot", _tau_mot(_idx));
    _robot->getMotorPosition(_qmot);
    _robot->positionToMinimal(_qmot, _q);
    _logger->add("q_mot", _q(_idx));
}

// 越限或温度停机：参考设到当前实测位置，力矩立即卸掉（pin 断了时摆臂也不会被拉向偏移后的参考），然后停止。
void ThermalSoakPlugin::halt(int code)
{
    _robot->getJointPositionMinimal(_q);
    const double q = _q(_idx);
    if(std::isfinite(q)) {
        _q_cmd = q;
        _joint->set_pos_ref(q);
        _joint->set_vel_ref(0.0);
        _joint->move();
    }
    _stop_code = code;
    stop();
}

// 3. rt_main 线程周期调用（1 kHz）：读实测 → 检查 → 温度 PID → 下发参考。
void ThermalSoakPlugin::run()
{
    _live_queue.run();  // 只有 setpoint、release 运行中生效；max_torque 等要到下次启动，避免绕过 on_start 的检查
    if(_release) { _ctrl.request_release(); _release = false; }
    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getJointVelocity(_v);
    _robot->getJointEffort(_tau);
    _robot->getMotorTorque(_tau_mot);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    const double dt = getPeriodSec();
    const double q = _q(_idx), v = _v(_idx), tau = _tau(_idx);
    const double temp = _temp_mot(_idx), temp_dri = _temp_dri(_idx);

    // 越限时不 throw（rt 线程里的异常会让 xbot2-core terminate），打印原因后卸力矩并停止。
    std::string violation;
    if(!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(tau))
        violation = fmt::format("nonfinite telemetry: q {} v {} tau_link {}", q, v, tau);
    else if(!std::isfinite(temp) || temp <= 0)
        violation = fmt::format("motor temperature reading {} C is not valid", temp);
    else if(std::abs(q - _q_pin) > _max_position_drift)
        violation = fmt::format("output moved {:.4f} rad from the pinned position (> max_position_drift {}): pin loose or missing?",
                                q - _q_pin, _max_position_drift);
    else if(std::abs(v) > _max_velocity)
        violation = fmt::format("|velocity| {} > max_velocity {} rad/s: pin loose or missing?", std::abs(v), _max_velocity);
    else if(std::abs(tau - _torque_offset) > _max_link_torque)
        violation = fmt::format("|tau_link - torque_offset| {} > max_link_torque {} Nm", std::abs(tau - _torque_offset), _max_link_torque);
    _hot_mot = !(temp < _stop_motor_temperature) ? _hot_mot + dt : 0.0;
    _hot_dri = _stop_driver_temperature > 0 && !(temp_dri < _stop_driver_temperature) ? _hot_dri + dt : 0.0;

    if(_logger && _elapsed >= _next_log) {
        log_row(q, v, tau);
        _next_log += _log_period;
    }
    if(!violation.empty()) {
        jinfo("SAFETY STOP trj_heat at t = {:.1f} s: {}", _elapsed, violation);
        halt(3);
        return;
    }
    if(_hot_mot >= 0.5) {
        jinfo("THERMAL STOP trj_heat at t = {:.1f} s: motor {} C >= {} C", _elapsed, temp, _stop_motor_temperature);
        halt(1);
        return;
    }
    if(_hot_dri >= 0.5) {
        jinfo("THERMAL STOP trj_heat at t = {:.1f} s: driver {} C >= {} C", _elapsed, temp_dri, _stop_driver_temperature);
        halt(2);
        return;
    }

    const int before = _ctrl.phase;
    _ctrl.step(temp, dt);
    if(_ctrl.phase != before) {
        if(_ctrl.phase == ThermalSoak::Soak && !_release_only)
            jinfo("trj_heat: reached {:.1f} C at t = {:.0f} s, soaking{}", _ctrl.temp_filtered, _elapsed,
                  _ctrl.soak_time > 0 ? fmt::format(" for {:.0f} s", _ctrl.soak_time) : std::string(" until release"));
        else if(_ctrl.phase == ThermalSoak::Release)
            jinfo("trj_heat: {} at t = {:.0f} s, releasing torque", _ctrl.timed_out
                  ? fmt::format("setpoint not reached within max_heat_time {} s (raise max_torque?)", _ctrl.max_heat_time)
                  : std::string("soak finished"), _elapsed);
    }
    if(_ctrl.phase == ThermalSoak::Done) {
        jinfo("trj_heat DONE: torque released (tau_link {:.1f} Nm, motor {:.1f} C). Remove the pin and start the loaded test now.",
              tau - _torque_offset, temp);
        _stop_code = _ctrl.timed_out ? 5 : 0;
        stop();
        return;
    }
    _q_cmd = _q_pin + _direction*_ctrl.offset();
    _joint->set_pos_ref(_q_cmd);
    _joint->set_vel_ref(0.0);
    _joint->move();

    if(_elapsed >= _next_status) {
        const std::string soak = _ctrl.phase == ThermalSoak::Soak
            ? fmt::format(" {:.1f}/{} min", _ctrl.soak_elapsed/60, _ctrl.soak_time > 0 ? fmt::format("{:.0f}", _ctrl.soak_time/60) : std::string("inf"))
            : std::string();
        jinfo("trj_heat [{}{}] t {:.0f} s: motor {:.1f} C (filtered {:.1f}, setpoint {}), driver {:.1f} C, u {:.2f}, "
              "torque cmd {:.1f} Nm, tau_link {:.1f} Nm, tau_mot {:.1f} Nm",
              phase_name(_ctrl.phase), soak, _elapsed, temp, _ctrl.temp_filtered, _ctrl.setpoint, temp_dri, _ctrl.u,
              _direction*_ctrl.torque_cmd, tau - _torque_offset, _tau_mot(_idx));
        _next_status += _status_period;
    }
    _elapsed += dt;
}

// 停止：恢复参考滤波，写停止原因，销毁 logger 写出剩余数据。外部停止时参考可能还带着偏移，提示先释放再拔 pin。
void ThermalSoakPlugin::finish()
{
    if(_filter_disabled) {
        Hal::JointSafety::enable_filter(true, -1);
        _filter_disabled = false;
    }
    const double left = _q_cmd - _q_pin;
    jinfo("trj_heat finished: stop_code {} at t = {:.0f} s, {} log samples dropped", _stop_code, _elapsed, _dropped);
    if(std::abs(left)*_ctrl.drive_stiffness > 2.0)
        jinfo("WARNING trj_heat: position reference is still {:.3f} rad (~{:.0f} Nm) away from the pinned position. "
              "Do NOT remove the pin; release first: ./run_trj.sh trj_heat release_only=true ... (META=0)",
              left, left*_ctrl.drive_stiffness);
    if(_logger) {
        _logger->add("stop_code", double(_stop_code));
        _logger->add("stop_elapsed", _elapsed);
        _logger->add("dropped_samples", double(_dropped));
    }
    _appender.reset();
    _logger.reset();
}
void ThermalSoakPlugin::on_stop() { finish(); }
void ThermalSoakPlugin::on_abort() { finish(); }

// 注册名必须与 CMake 目标和 YAML 中的 type 一致。
XBOT2_REGISTER_PLUGIN(ThermalSoakPlugin, thermal_soak_plugin_haotian)
