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
    _model = ModelInterface::getModel(_robot->getConfigOptions());
    _robot->getJointPositionMinimal(_q);
    _robot->getMotorPosition(_qmot);
    _robot->getJointVelocity(_v);
    _robot->getJointEffort(_tau);
    _q_ref = _q;
    _active = Eigen::VectorXd::Zero(_q.size());
    return true;
}

// 2. 每次启动实验调用：先完成所有检查，再关滤波、切控制模式，失败时不改变台架状态。
void PolicyTrajectoryPlugin::on_start()
{
    _param_upd_queue.run();
    for(double limit : {_q_min, _q_max, _max_velocity, _max_link_torque, _max_tracking_error, _stop_time, _seed})
        if(!std::isfinite(limit)) throw std::invalid_argument("nonfinite policy experiment parameter");
    if(_q_min >= _q_max || _max_velocity <= 0 || _max_link_torque <= 0 || _max_tracking_error <= 0)
        throw std::invalid_argument("configure output-side policy experiment limits first");
    _joint = _robot->getDeviceInstance<Hal::JointBase>(_joint_name);
    if(!_joint || _joint->readonly()) throw std::invalid_argument("joint must name a writable joint");
    _idx = _model->getDofIndex(_joint_name);
    if(_idx < 0 || _idx >= _q_ref.size()) throw std::invalid_argument("joint has no scalar DOF");

    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _joint->sense();
    _gen.duration = _stop_time;
    _gen.seed = static_cast<std::uint64_t>(_seed);
    // q0 来自关节现有位置参考；实验前先按台架流程完成 homing。
    _gen.initial_value = _joint->get_pos_ref();
    _gen.generate(_targets, _segments);  // 参数非法或 max_step 太小时在这里抛出
    // 目标始终在 q0 ± A 内；目标每帧最多变 max_step，相当于参考斜率上限 max_step*rate。
    if(_gen.initial_value - _gen.amplitude < _q_min || _gen.initial_value + _gen.amplitude > _q_max)
        throw std::invalid_argument("q0 +- amplitude exceeds position limits");
    if(_gen.max_step*_gen.policy_rate_hz > _max_velocity)
        throw std::invalid_argument("max_step * policy_rate_hz exceeds max_velocity");
    jinfo("policy targets for joint {}: q0 {}, amplitude {}, {} updates at {} Hz, seed {}",
          _joint_name, _gen.initial_value, _gen.amplitude, _targets.size(), _gen.policy_rate_hz, _gen.seed);

    _q_ref = _q;
    _q_ref(_idx) = _gen.initial_value;
    _active.setZero();
    _active(_idx) = 1;
    // 先建日志：MakeLogger 会因文件名不合法抛出（例如名字里带点 "A0.1"，MatLogger2 只接受 .mat 或无扩展名）。
    // 放在切换控制模式、关滤波之前，启动失败时台架状态不变。
    _logger.reset();
    if(_enable_log) {
        if(_log_file.empty()) _log_file = "/tmp/policy_targets";
        jinfo("logging enabled, log file: {}", _log_file);
        MatLogger2::Options opt;
        opt.default_buffer_size = std::ceil(_stop_time/getPeriodSec()) + 3;
        _logger = MatLogger2::MakeLogger(_log_file, opt);
        _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);
        for(const auto& name : {"q_ref", "q_link", "q_mot", "v_link", "v_mot", "tau_link", "tau_mot",
                                "tau_ref", "temp_mot", "temp_dri", "active"})
            _logger->create(name, _q.size());
        for(const auto& name : {"time", "trajectory_time", "valid", "segment", "policy_index"})
            _logger->create(name, 1);
        // 本次实验的生成参数各记一次，便于离线用同一种子复现目标序列。
        for(const auto& [name, value] : std::initializer_list<std::pair<const char*, double>>{
                {"cfg_seed", _seed}, {"cfg_amplitude", _gen.amplitude},
                {"cfg_policy_rate_hz", _gen.policy_rate_hz}, {"cfg_max_step", _gen.max_step},
                {"cfg_initial_value", _gen.initial_value},
                {"cfg_reference_filter_disabled", _disable_reference_filter ? 1.0 : 0.0}}) {
            _logger->create(name, 1);
            _logger->add(name, value);
        }
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
    _clock_start = std::chrono::steady_clock::now();
}
void PolicyTrajectoryPlugin::starting() { start_completed(); }

// 3. rt_main 线程周期调用（1 kHz）：检查实测 → 记录 → 按时间查表下发目标。
void PolicyTrajectoryPlugin::run()
{
    _robot->sense(false);
    _robot->getJointPositionMinimal(_q);
    _robot->getJointVelocity(_v);
    _robot->getJointEffort(_tau);
    // 跟踪误差包含目标阶跃本身（最多 max_step）和执行器的跟踪滞后，阈值要留出余量。
    if(!std::isfinite(_q(_idx)) || !std::isfinite(_v(_idx)) || !std::isfinite(_tau(_idx)) ||
       _q(_idx) < _q_min || _q(_idx) > _q_max || std::abs(_v(_idx)) > _max_velocity ||
       std::abs(_tau(_idx)) > _max_link_torque || std::abs(_q(_idx) - _q_ref(_idx)) > _max_tracking_error)
        throw std::runtime_error("policy experiment telemetry exceeded configured limits");
    const int k = _gen.index(_elapsed);
    // 先记录当前反馈和上一周期已下发的目标；第一行 valid=0（尚无本插件指令）。
    if(_logger) {
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - _clock_start).count();
        _logger->add("time", wall);
        _logger->add("trajectory_time", _elapsed);
        _logger->add("valid", _have_command ? 1.0 : 0.0);
        _logger->add("segment", double(_segments[k]));
        _logger->add("policy_index", double(k));
        _logger->add("active", _active);
        _logger->add("q_ref", _q_ref);
        _logger->add("q_link", _q);
        _logger->add("v_link", _v);
        _logger->add("tau_link", _tau);
        _robot->getMotorPosition(_qmot);
        _robot->positionToMinimal(_qmot, _q);
        _logger->add("q_mot", _q);
        _robot->getMotorVelocity(_v);
        _logger->add("v_mot", _v);
        _robot->getMotorTorque(_tau);
        _logger->add("tau_mot", _tau);
        _robot->getEffortReferenceFeedback(_tau);
        _logger->add("tau_ref", _tau);
        _robot->getTemperatureMotor(_tau);
        _logger->add("temp_mot", _tau);
        _robot->getTemperatureBoard(_tau);
        _logger->add("temp_dri", _tau);
    }
    // 发出终点目标（= q0）后再多记录一个周期。
    if(_elapsed > _stop_time + getPeriodSec()/2) { stop(); return; }
    _q_ref(_idx) = _targets[k];
    _joint->set_pos_ref(_targets[k]);
    _joint->set_vel_ref(0.0);  // 策略只给目标位置，不给速度前馈
    _joint->move();
    _have_command = true;
    _elapsed += getPeriodSec();
    std_msgs::msg::Float32 msg;
    msg.data = std::min(1.0, _elapsed/_stop_time);
    _progress_pub->publish(msg);
}

// 正常停止或异常退出：恢复参考滤波，销毁 logger 写出 MAT 文件。
void PolicyTrajectoryPlugin::finish()
{
    if(_filter_disabled) {
        Hal::JointSafety::enable_filter(true, -1);
        _filter_disabled = false;
    }
    _logger.reset();
}
void PolicyTrajectoryPlugin::on_stop() { finish(); }
void PolicyTrajectoryPlugin::on_abort() { finish(); }

// 注册名必须与 CMake 目标和 YAML 中的 type 一致。
XBOT2_REGISTER_PLUGIN(PolicyTrajectoryPlugin, policy_trajectory_plugin_haotian)
