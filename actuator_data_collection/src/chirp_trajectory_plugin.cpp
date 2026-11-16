#include "chirp_trajectory_plugin.h"

using namespace XBot;

bool ChirpTrajectoryPlugin::on_initialize()
{
    // ros support from rt
    _ros = std::make_unique<Ros2Support>(Ros2Support::get_main_node()->create_sub_node(getName()));
    _progress_pub = _ros->advertise<std_msgs::msg::Float32>("progress", 1);

    // create trj from the mandatory amplitude param
    auto dev_map = _robot->getDevices<Hal::JointBase>().get_device_name_map();
    for(auto [jname, jptr] : dev_map)
    {
        if(jptr->readonly())
        {
            jinfo("skipping read-only joint {} \n", jname);
            continue;
        }

        Chirp trj;
        _trj_map[jptr.get()] = trj;
        _enabled_joints.push_back(jname);
    }

    // set optional default values from params, map version
    std::map<std::string, double> amplitude, freq_min, freq_max, period, phase;
    getParam("~amplitude", amplitude);
    getParam("~freq_min", freq_min);
    getParam("~freq_max", freq_max);
    getParam("~period", period);
    getParam("~phase_offset", phase);
    getParam("~position_min", _q_min);
    getParam("~position_max", _q_max);
    getParam("~max_velocity", _max_velocity);
    getParam("~max_link_torque", _max_link_torque);
    getParam("~max_tracking_error", _max_tracking_error);
    getParam("~stop_motor_temperature", _stop_motor_temperature);  // 可选，yaml 里没有时为 0（不检查）
    getParam("~status_period", _status_period);                    // 可选，默认 10 s

    for(auto [jname, a] : amplitude)
    {
        auto dev = _robot->getDeviceInstance<Hal::JointBase>(jname);

        _trj_map.at(dev).amplitude = a;

        _enabled_joints.push_back(jname);
    }

    for(auto [jname, f] : freq_min)
    {
        auto dev = _robot->getDeviceInstance<Hal::JointBase>(jname);

        _trj_map.at(dev).freq_min_hz = f;
    }
    
    for(auto [jname, f] : freq_max)
    {
        _trj_map.at(_robot->getDeviceInstance<Hal::JointBase>(jname)).freq_max_hz = f;
    }

    for(auto [jname, t] : period)
    {
        _trj_map.at(_robot->getDeviceInstance<Hal::JointBase>(jname)).period = t;
    }

    for(auto [jname, p] : phase)
    {
        _trj_map.at(_robot->getDeviceInstance<Hal::JointBase>(jname)).phase_offset = p;
    }

    // same, scalar version
    double x = 0;
    if(getParam("~amplitude", x))
    {
        for(auto& [dev, trj] : _trj_map)
        {
            trj.amplitude = x;
        }
    }

    if(getParam("~freq_min", x))
    {
        for(auto& [dev, trj] : _trj_map)
        {
            trj.freq_min_hz = x;
        }
    }

    if(getParam("~freq_max", x))
    {
        for(auto& [dev, trj] : _trj_map)
        {
            trj.freq_max_hz = x;
        }
    }

    if(getParam("~period", x))
    {
        for(auto& [dev, trj] : _trj_map)
        {
            trj.period = x;
        }
    }

    // create tunable parameters
    for(auto& [dev, trj] : _trj_map)
    {

        jinfo("creating tunable parameters for joint {} \n",
              dev->get_name());

        createTunableParam(
            fmt::format("~{}/freq_min", dev->get_name()),
            &trj.freq_min_hz,
            Validators::InRange(0.0, 100.0),
            &_param_upd_queue);

        createTunableParam(
             fmt::format("~{}/freq_max", dev->get_name()),
             &trj.freq_max_hz,
             Validators::InRange(0.0, 100.0),
             &_param_upd_queue);

        createTunableParam(
            fmt::format("~{}/amplitude", dev->get_name()),
            &trj.amplitude,
            Validators::InRange(-300.0, 300.0),
            &_param_upd_queue);

        createTunableParam(
            fmt::format("~{}/period", dev->get_name()),
            &trj.period,
            Validators::IsPositive<double>(),
            &_param_upd_queue);

    }

    createTunableParam(
        "~stop_time",
        &_stop_time,
        Validators::IsPositive<double>(false),
        &_param_upd_queue);

    createTunableParam(
        "~enable_log",
        &_enable_log,
        ParameterValidator<bool>(),
        &_param_upd_queue);

    createTunableParam(
        "~log_file",
        &_log_file,
        ParameterValidator<std::string>(),
        &_param_upd_queue);

    createTunableParam(
        "~trj_out",
        &_trj_out_str,
        Validators::DiscreteValues<std::string>({"Position", "Torque"}),
        &_param_upd_queue);

    createTunableParam(
        "~enabled_joints",
        &_enabled_joints,
        ParameterValidator<std::vector<std::string>>(),
        &_param_upd_queue);

    // set param with all supported joints
    setParam("~supported_joints", _enabled_joints);

    // allocate
    _model = ModelInterface::getModel(_robot->getConfigOptions());
    _model->getJointPosition(_qmot);
    _model->getJointPositionMinimal(_q);
    _model->getJointVelocity(_v);
    _model->getJointVelocity(_v_sent);
    _v_sent.setZero();
    _model->getJointPositionMinimal(_q_cmd);
    _model->getJointPositionMinimal(_q_meas);
    _model->getJointVelocity(_v_meas);
    _model->getJointVelocity(_tau_meas);
    _model->getJointAcceleration(_a);
    _model->computeInverseDynamics(_tauid);
    _model->getJointVelocity(_temp_mot);
    _model->getJointVelocity(_temp_dri);
    _model->getJointVelocity(_tau_mot);
    _model->getJointVelocity(_cycle);
    _model->getJointVelocity(_freq);
    _cycle.setZero();
    _freq.setZero();

    return true;

}

void ChirpTrajectoryPlugin::on_start()
{
    _param_upd_queue.run();

    if(_trj_out_str == "Position")
    {
        jinfo("trajectory output set to Position");
        _trj_out = Position;
    }
    else if(_trj_out_str == "Torque")
    {
        jinfo("trajectory output set to Torque");
        _trj_out = Torque;
    }

    if(_enable_log)
    {
        if(_log_file.empty())
        {
            _log_file = "/tmp/data_acquisition_plugin_log";
        }

        jinfo("logging enabled, log file: {}", _log_file);

        // producer_consumer + MatAppender：rt 线程只往缓冲里放数据，后台线程持续写盘；缓冲按 60 s 设。
        MatLogger2::Options opt;
        opt.default_buffer_size = std::ceil(60.0 / getPeriodSec());

        _logger = MatLogger2::MakeLogger(_log_file,
                                         opt);

        _logger->set_buffer_mode(VariableBuffer::Mode::producer_consumer);

        const int nv = _robot->getNv();

        _logger->create("q_ref", nv);
        _logger->create("q_link", nv);
        _logger->create("q_mot", nv);
        _logger->create("v_ref", nv);
        _logger->create("v_traj", nv);
        _logger->create("v_link", nv);
        _logger->create("v_mot", nv);
        _logger->create("tau_link", nv);
        _logger->create("tau_mot", nv);
        _logger->create("tau_id", nv);
        _logger->create("temp_mot", nv);
        _logger->create("temp_dri", nv);
        _logger->create("cycle", nv);            // 第几个扫频周期
        _logger->create("freq", nv);             // 当前频率 Hz
        _logger->create("trajectory_time", 1);
        for(const auto& name : {"stop_code", "stop_elapsed", "dropped_samples"})
        {
            _logger->create(name, 1);            // 结束时各写一次
        }

        _appender = MatAppender::MakeInstance();
        _appender->add_logger(_logger);
        _appender->start_flush_thread();
    }


    for(auto& [j, t] : _trj_map)
    {
        j->sense();
        t.reset();
        t.initial_value = j->get_pos_ref();

        jinfo("initial value for joint {} set to {}", j->get_name(), t.initial_value);
        _q_cmd(_model->getDofIndex(j->get_name())) = t.initial_value;

        auto it = std::find(_enabled_joints.begin(), _enabled_joints.end(), j->get_name());

        if(it == _enabled_joints.end())
        {
            jinfo("joint {} not in enabled_joints list, setting amplitude to 0", j->get_name());
            t.amplitude = 0.0;
        }

        std::map<std::string, ControlMode> ctrl_map;
        ctrl_map[j->get_name()] = _trj_out == Position ? ControlMode::Position() : ControlMode::Effort();
        
        bool ctrl_ok = _robot->setControlMode(ctrl_map);

        if(!ctrl_ok)
        {
            throw std::runtime_error(fmt::format("failed to set control mode for joint '{}'", j->get_name()));
        }
    }

    Hal::JointSafety::enable_filter(false, -1);

    _elapsed = 0;
    _next_status = _status_period;
    _dropped = 0;
    _stop_code = 4;
    for(auto& [j, t] : _trj_map)
    {
        t.win_sq = t.win_n = t.hot_time = 0;
    }
    if(_stop_motor_temperature > 0)
    {
        jinfo("temperature stop at motor {} C", _stop_motor_temperature);
    }
    else
    {
        jinfo("no temperature stop configured (stop_motor_temperature); watch the status line");
    }
}

void ChirpTrajectoryPlugin::starting()
{
    start_completed();
}

void ChirpTrajectoryPlugin::run()
{
    // update telemetry
    _robot->sense(false);

    // 安全检查：只检查有幅值的关节；跟踪误差只在位置模式下检查（和上一周期下发的参考比）。
    _robot->getJointPositionMinimal(_q_meas);
    _robot->getJointVelocity(_v_meas);
    _robot->getJointEffort(_tau_meas);
    _robot->getMotorTorque(_tau_mot);
    _robot->getTemperatureMotor(_temp_mot);
    _robot->getTemperatureBoard(_temp_dri);
    const double dt = getPeriodSec();
    std::string violation, thermal;
    for(auto& [j, t] : _trj_map)
    {
        if(t.amplitude == 0.0 || !violation.empty())
        {
            continue;
        }
        const int i = _model->getDofIndex(j->get_name());
        const double q = _q_meas(i), v = _v_meas(i), tau = _tau_meas(i), e = q - _q_cmd(i);
        if(!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(tau))
            violation = fmt::format("{}: nonfinite telemetry q {} v {} tau_link {}", j->get_name(), q, v, tau);
        else if(_q_min < _q_max && (q < _q_min || q > _q_max))
            violation = fmt::format("{}: position {} outside [{}, {}] rad", j->get_name(), q, _q_min, _q_max);
        else if(_max_velocity > 0 && std::abs(v) > _max_velocity)
            violation = fmt::format("{}: |velocity| {} > max_velocity {} rad/s", j->get_name(), std::abs(v), _max_velocity);
        else if(_max_link_torque > 0 && std::abs(tau) > _max_link_torque)
            violation = fmt::format("{}: |tau_link| {} > max_link_torque {} Nm", j->get_name(), std::abs(tau), _max_link_torque);
        else if(_trj_out == Position && _max_tracking_error > 0 && std::abs(e) > _max_tracking_error)
            violation = fmt::format("{}: |q - q_ref| {} > max_tracking_error {} rad", j->get_name(), std::abs(e), _max_tracking_error);
        else if(_stop_motor_temperature > 0 && !std::isfinite(_temp_mot(i)))
            violation = fmt::format("{}: nonfinite motor temperature", j->get_name());
        // 温度停机线：连续 0.5 s 超过才停，避免单个读数跳变误停。
        t.hot_time = _stop_motor_temperature > 0 && _temp_mot(i) >= _stop_motor_temperature ? t.hot_time + dt : 0.0;
        if(t.hot_time >= 0.5 && thermal.empty())
            thermal = fmt::format("{}: motor {} C >= {} C", j->get_name(), _temp_mot(i), _stop_motor_temperature);
        if(std::isfinite(_tau_mot(i)))
        {
            t.win_sq += _tau_mot(i)*_tau_mot(i);
            t.win_n += 1;
        }
    }
    // 定期打印状态行：每个有幅值的关节的温度、tau_mot RMS、当前频率和扫频周期，便于手动换配重时判断力矩档位。
    if(_elapsed >= _next_status)
    {
        for(auto& [j, t] : _trj_map)
        {
            if(t.amplitude == 0.0 || t.win_n == 0) continue;
            const int i = _model->getDofIndex(j->get_name());
            jinfo("t {:.0f} s | {} cycle {:.0f}, {:.2f} Hz | motor {:.1f} C, driver {:.1f} C | tau_mot rms {:.1f} Nm (last {:.0f} s){}",
                  _elapsed, j->get_name(), t.get_cycle(), t.get_freq(), _temp_mot(i), _temp_dri(i),
                  std::sqrt(t.win_sq/t.win_n), _status_period,
                  _dropped ? fmt::format(" | DROPPED {} log samples", _dropped) : std::string());
            t.win_sq = t.win_n = 0;
        }
        _next_status += _status_period;
    }
    if(!violation.empty())
    {
        jinfo("SAFETY STOP trj_chirp at t = {} s: {}", _elapsed, violation);
        _stop_code = 3;
        stop();
        return;
    }
    if(!thermal.empty())
    {
        jinfo("THERMAL STOP trj_chirp at t = {} s: {}", _elapsed, thermal);
        _stop_code = 1;
        stop();
        return;
    }

    // execute trajectories
    bool trj_ended = false;
    double trj_time = 0;

    for(auto& [j, t] : _trj_map)
    {
        auto [q, v, a] = t.step(getPeriodSec());

        int jidx = _model->getDofIndex(j->get_name());
        _cycle(jidx) = t.get_cycle();
        _freq(jidx) = t.get_freq();

        if(_trj_out == Position)
        {
            _q(jidx) = q;
            _v(jidx) = v;
            _a(jidx) = a;

            j->set_pos_ref(q);
            _q_cmd(jidx) = q;
            // 与 trj 唯一的不同：不给速度前馈，速度参考置 0。
            j->set_vel_ref(0.0);
        }
        else
        {
            j->set_tor_ref(q);
        }

        j->move();

        trj_time = t.get_time();

        if(_stop_time > 0 && trj_time > _stop_time)
        {
            trj_ended = true;
        }
    }

    _elapsed += dt;

    // publish progress
    if(_stop_time > 0)
    {
        std_msgs::msg::Float32 msg;
        msg.data = trj_time/_stop_time;
        _progress_pub->publish(msg);
    }

    // stop if needed
    if(trj_ended)
    {
        jinfo("stopping after {} s", _stop_time);
        _stop_code = 0;

        std_msgs::msg::Float32 msg;
        msg.data = 1.0;
        _progress_pub->publish(msg);

        stop();
    }

    // exit if we don't have a logger
    if(!_logger)
    {
        return;
    }

    // log data

    if(!_logger->add("q_ref", _q))
    {
        ++_dropped;   // producer_consumer 缓冲满时 add 返回 false
    }
    _logger->add("trajectory_time", trj_time);
    _logger->add("cycle", _cycle);
    _logger->add("freq", _freq);
    _logger->add("v_ref", _v_sent);   // 实际下发的速度参考，恒为 0
    _logger->add("v_traj", _v);       // 正弦的解析速度（未下发）
    _logger->add("a_ref", _a);

    _robot->getJointPositionMinimal(_q);
    _logger->add("q_link", _q);

    _robot->getMotorPosition(_qmot);
    _robot->positionToMinimal(_qmot, _q);
    _logger->add("q_mot", _q);

    _robot->getJointVelocity(_v);
    _logger->add("v_link", _v);

    _robot->getMotorVelocity(_v);
    _logger->add("v_mot", _v);

    _robot->getJointEffort(_tauid);
    _logger->add("tau_link", _tauid);

    _robot->getMotorTorque(_tauid);
    _logger->add("tau_mot", _tauid);

    _robot->getEffortReferenceFeedback(_tauid);
    _logger->add("tau_ref", _tauid);

    _robot->getTemperatureMotor(_tauid);
    _logger->add("temp_mot", _tauid);

    _robot->getTemperatureBoard(_tauid);
    _logger->add("temp_dri", _tauid);

}

// 正常停止或异常退出：恢复参考滤波，写入停止原因，停掉写盘线程，销毁 logger 写出剩余数据。
void ChirpTrajectoryPlugin::finish()
{
    Hal::JointSafety::enable_filter(true, -1);
    jinfo("trj_chirp finished: stop_code {} at t = {} s, {} log samples dropped", _stop_code, _elapsed, _dropped);
    if(_logger)
    {
        _logger->add("stop_code", double(_stop_code));
        _logger->add("stop_elapsed", _elapsed);
        _logger->add("dropped_samples", double(_dropped));
    }
    _appender.reset();
    _logger.reset();
}

void ChirpTrajectoryPlugin::on_stop()
{
    finish();
}

void ChirpTrajectoryPlugin::on_abort()
{
    finish();
}

std::tuple<double, double, double> ChirpTrajectoryPlugin::Chirp::step(double dt)
{
    double dt_scaling = 1.0;
    double dt_scaling_der = 0;

    if(_time <= warmup_time)
    {
        double tau = _time/warmup_time;
        dt_scaling = ((6*tau - 15)*tau + 10)*tau*tau*tau;
    }

    _freq_hz = freq_min_hz + (1 - std::cos(2*M_PI/period*_time))/2.0*(freq_max_hz - freq_min_hz);

    _phase += _freq_hz*2*M_PI*dt_scaling*dt;

    _time += dt;

    double qref = initial_value + amplitude*std::sin(dt_scaling*phase_offset + _phase);

    double vref = amplitude*std::cos(dt_scaling*phase_offset + _phase)*(_freq_hz*2*M_PI*dt_scaling);

    double dfreq = std::sin(2*M_PI/period*_time)*(freq_max_hz - freq_min_hz)*M_PI/period;

    double aref = amplitude*std::cos(dt_scaling*phase_offset + _phase)*(dfreq*2*M_PI*dt_scaling) +
                  -amplitude*std::pow(_freq_hz*2*M_PI*dt_scaling, 2)*std::sin(dt_scaling*phase_offset + _phase);

    return {qref, vref, aref};
}

void ChirpTrajectoryPlugin::Chirp::reset()
{
    _phase = 0;
    _time = 0;
}

XBOT2_REGISTER_PLUGIN(ChirpTrajectoryPlugin, chirp_trajectory_plugin_haotian)
