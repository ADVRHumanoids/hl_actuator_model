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
    // 水平扫频模式（可选，trj_horizontal 用）；yaml 里没有时 side = 0，与原来的 trj_chirp 相同。
    getParam("~side", _side);
    getParam("~hang_position", _hang_position);
    getParam("~transfer_velocity", _transfer_velocity);
    getParam("~min_transfer_time", _min_transfer_time);
    getParam("~settle_time", _settle_time);
    getParam("~fade_time", _fade_time);
    getParam("~center_correction", _center_correction);
    getParam("~correction_gain", _correction_gain);
    getParam("~correction_tolerance", _correction_tolerance);
    getParam("~correction_hold", _correction_hold);
    getParam("~max_correction", _max_correction);
    getParam("~max_settle_time", _max_settle_time);
    getParam("~sweep_correction_gain", _sweep_correction_gain);   // 可选，默认 0（扫频中不修正）
    getParam("~max_hold_torque", _max_hold_torque);               // 可选，默认 0（不检查）

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

    // 水平扫频模式的参数，下次启动生效。side 只能是 -1、0、+1（在 on_start 检查）。
    createTunableParam("~side", &_side, Validators::InRange(-1.0, 1.0), &_param_upd_queue);
    createTunableParam("~hang_position", &_hang_position, Validators::InRange(-3.2, 3.2), &_param_upd_queue);
    createTunableParam("~transfer_velocity", &_transfer_velocity, Validators::InRange(0.01, 2.0), &_param_upd_queue);
    createTunableParam("~settle_time", &_settle_time, Validators::InRange(0.0, 600.0), &_param_upd_queue);
    createTunableParam("~fade_time", &_fade_time, Validators::InRange(0.0, 60.0), &_param_upd_queue);
    createTunableParam("~center_correction", &_center_correction, ParameterValidator<bool>(), &_param_upd_queue);
    createTunableParam("~correction_gain", &_correction_gain, Validators::InRange(0.01, 5.0), &_param_upd_queue);
    createTunableParam("~correction_tolerance", &_correction_tolerance, Validators::InRange(0.001, 0.2), &_param_upd_queue);
    createTunableParam("~max_correction", &_max_correction, Validators::InRange(0.0, 1.5), &_param_upd_queue);
    createTunableParam("~max_settle_time", &_max_settle_time, Validators::InRange(0.0, 600.0), &_param_upd_queue);
    createTunableParam("~sweep_correction_gain", &_sweep_correction_gain, Validators::InRange(0.0, 1.0), &_param_upd_queue);
    createTunableParam("~max_hold_torque", &_max_hold_torque, Validators::InRange(0.0, 300.0), &_param_upd_queue);

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
    _model->getJointVelocity(_offset_log);
    _model->getJointVelocity(_hold_tau);
    _cycle.setZero();
    _freq.setZero();
    _offset_log.setZero();
    _hold_tau.setZero();

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

    // 先算起点、中心、过渡时长并做检查（可能 throw），通过后再建日志、切控制模式，失败时不改变台架状态。
    if(_side != 0.0 && _side != 1.0 && _side != -1.0)
    {
        throw std::invalid_argument(fmt::format("side must be -1, 0 or 1, got {}", _side));
    }
    const bool horizontal = _side != 0.0;
    if(horizontal && _trj_out != Position)
    {
        throw std::invalid_argument("side = +1/-1 (horizontal sweep) needs trj_out Position");
    }
    if(horizontal && _stop_time <= 0)
    {
        throw std::invalid_argument("side = +1/-1 needs stop_time > 0 (sweep duration before returning)");
    }
    for(double x : {_hang_position, _transfer_velocity, _min_transfer_time, _settle_time, _fade_time,
                     _correction_gain, _correction_tolerance, _correction_hold, _max_correction, _max_settle_time,
                     _sweep_correction_gain, _max_hold_torque})
    {
        if(!std::isfinite(x)) throw std::invalid_argument("nonfinite horizontal-sweep parameter");
    }
    if(_transfer_velocity <= 0 || _settle_time < 0 || _fade_time < 0 || _min_transfer_time < 0)
    {
        throw std::invalid_argument("transfer_velocity must be > 0; settle_time, fade_time, min_transfer_time >= 0");
    }
    if(_stop_time > 0 && _fade_time >= _stop_time)
    {
        throw std::invalid_argument(fmt::format("fade_time {} must be shorter than stop_time {}", _fade_time, _stop_time));
    }
    if(_correction_gain <= 0 || _correction_tolerance <= 0 || _correction_hold < 0 || _max_correction < 0 ||
       _max_settle_time < _settle_time)
    {
        throw std::invalid_argument("center correction: gain, tolerance > 0; hold, max_correction >= 0; max_settle_time >= settle_time");
    }
    if(_sweep_correction_gain < 0 || _max_hold_torque < 0)
    {
        throw std::invalid_argument("sweep_correction_gain and max_hold_torque must be >= 0");
    }
    _approach.clear();
    _return.clear();
    _corr.clear();
    _sweep_track.clear();
    _offset_log.setZero();
    _hold_tau.setZero();
    _end_code = 0;
    _approach_time = _return_time = 0;
    for(auto& [j, t] : _trj_map)
    {
        j->sense();
        t.reset();

        auto it = std::find(_enabled_joints.begin(), _enabled_joints.end(), j->get_name());

        if(it == _enabled_joints.end())
        {
            jinfo("joint {} not in enabled_joints list, setting amplitude to 0", j->get_name());
            t.amplitude = 0.0;
        }

        const double start = j->get_pos_ref();
        // 只移动有幅值的关节；水平中心 = 下垂角度 + side·π/2。
        const double center = horizontal && t.amplitude != 0.0 ? _hang_position + _side*actuator_collection::kPi/2 : start;
        if(!std::isfinite(start))
        {
            throw std::invalid_argument(fmt::format("joint {}: position reference is not finite", j->get_name()));
        }
        if(t.amplitude != 0.0 && _q_min < _q_max)
        {
            const double lo = std::min(start, center - std::abs(t.amplitude)), hi = std::max(start, center + std::abs(t.amplitude));
            if(lo < _q_min || hi > _q_max)
            {
                throw std::invalid_argument(fmt::format("joint {}: motion [{:.3f}, {:.3f}] rad exceeds position limits [{}, {}]",
                                                        j->get_name(), lo, hi, _q_min, _q_max));
            }
        }
        const double T = Transfer::duration(center - start, _transfer_velocity, _min_transfer_time);
        _approach[j] = Transfer{start, center, T};
        CenterCorrector c;
        c.gain = _correction_gain;
        c.tolerance = _correction_tolerance;
        c.hold_time = _correction_hold;
        c.max_offset = _max_correction;
        _corr[j] = c;
        SweepCenterTracker s;
        s.gain = _sweep_correction_gain;
        // 低通时间常数 = 最低扫频频率的周期（滤掉扫频本身），至少 1 s
        s.filter_time = std::max(1.0, t.freq_min_hz > 0 ? 1.0/t.freq_min_hz : t.period);
        s.max_offset = _max_correction;
        _sweep_track[j] = s;
        if(horizontal && _center_correction && t.amplitude != 0.0 && s.gain > 0.5/s.filter_time)
        {
            jinfo("note: joint {}: sweep_correction_gain {} > 0.5/filter_time = {:.3f}; the center tracking may overshoot",
                  j->get_name(), s.gain, 0.5/s.filter_time);
        }
        // 修正后扫频中心可能偏离 center 最多 max_correction：一起检查位置范围
        if(horizontal && _center_correction && t.amplitude != 0.0 && _q_min < _q_max &&
           (center - std::abs(t.amplitude) - _max_correction < _q_min || center + std::abs(t.amplitude) + _max_correction > _q_max))
        {
            jinfo("note: joint {}: center +- (amplitude + max_correction) reaches outside [{}, {}]; the position check stops the run "
                  "if the corrected reference gets there", j->get_name(), _q_min, _q_max);
        }
        _return[j] = Transfer{center, start, T};
        _approach_time = std::max(_approach_time, T);
        _return_time = std::max(_return_time, T);
        t.initial_value = center;
        t.fade_time = _fade_time;
        t.duration = _stop_time;

        jinfo("joint {}: start {:.4f} rad, sweep center {:.4f} rad, amplitude {} rad", j->get_name(), start, center, t.amplitude);
        _q_cmd(_model->getDofIndex(j->get_name())) = start;
    }
    _stage = _approach_time > 0 ? Approach : (_settle_time > 0 ? Settle : Sweep);
    _stage_time = 0;
    if(horizontal)
    {
        jinfo("horizontal sweep (side {:+.0f}): move {:.1f} s, settle {} s, sweep {} s (fade {} s), return {:.1f} s",
              _side, _approach_time, _settle_time, _stop_time, _fade_time, _return_time);
        jinfo("center correction {}, sweep_correction_gain {} 1/s, max_correction {} rad, max_hold_torque {} Nm (0 = no check)",
              _center_correction ? "on" : "off", _sweep_correction_gain, _max_correction, _max_hold_torque);
    }
    else
    {
        jinfo("sweep around the start: stop_time {} s (0 = until stopped)", _stop_time);
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
        _logger->create("trajectory_time", 1);   // 插件启动后的时间（含过渡和保持），每周期 + dt
        _logger->create("chirp_phase", 1);       // 0 移到中心、1 停稳、2 扫频、3 回起点
        _logger->create("cfg_center", nv);       // 每个关节的扫频中心
        _logger->create("cfg_start", nv);        // 每个关节的起点
        _logger->create("center_offset", nv);    // 参考修正 rad（停稳阶段积分，扫频阶段慢速跟踪；扫频绕 中心 + 修正 进行）
        for(const auto& [name, value] : std::initializer_list<std::pair<const char*, double>>{
                {"cfg_side", _side}, {"cfg_hang_position", _hang_position}, {"cfg_transfer_velocity", _transfer_velocity},
                {"cfg_settle_time", _settle_time}, {"cfg_fade_time", _fade_time}, {"cfg_stop_time", _stop_time},
                {"cfg_sweep_correction_gain", _sweep_correction_gain}, {"cfg_max_hold_torque", _max_hold_torque},
                {"cfg_max_correction", _max_correction}})
        {
            _logger->create(name, 1);
            _logger->add(name, value);
        }
        Eigen::VectorXd center = _q_cmd, start = _q_cmd;
        for(auto& [j, t] : _trj_map)
        {
            const int i = _model->getDofIndex(j->get_name());
            center(i) = _approach.at(j).to;
            start(i) = _approach.at(j).from;
        }
        _logger->add("cfg_center", center);
        _logger->add("cfg_start", start);
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
            jinfo("t {:.0f} s | {} cycle {:.0f}, {:.2f} Hz | motor {:.1f} C, driver {:.1f} C | tau_mot rms {:.1f} Nm (last {:.0f} s){}{}",
                  _elapsed, j->get_name(), t.get_cycle(), t.get_freq(), _temp_mot(i), _temp_dri(i),
                  std::sqrt(t.win_sq/t.win_n), _status_period,
                  _side != 0.0 ? fmt::format(" | center offset {:+.3f} rad", _offset_log(i)) : std::string(),
                  _dropped ? fmt::format(" | DROPPED {} log samples", _dropped) : std::string());
            t.win_sq = t.win_n = 0;
        }
        _next_status += _status_period;
    }
    if(!violation.empty())
    {
        jinfo("SAFETY STOP {} at t = {} s: {}", getName(), _elapsed, violation);
        _stop_code = 3;
        stop();
        return;
    }
    if(!thermal.empty())
    {
        jinfo("THERMAL STOP {} at t = {} s: {}", getName(), _elapsed, thermal);
        _stop_code = 1;
        stop();
        return;
    }

    // execute trajectories：按阶段给出参考。side = 0 时一直是 Sweep，与原来的 trj_chirp 相同。
    bool trj_ended = false, done = false;
    double trj_time = 0;
    const int stage_cmd = _stage;   // 本周期参考所属的阶段（记入日志）
    _stage_time += dt;
    const bool correcting = _side != 0.0 && _center_correction;
    bool settled = true;

    for(auto& [j, t] : _trj_map)
    {
        int jidx = _model->getDofIndex(j->get_name());
        double q = 0, v = 0, a = 0;
        if(_stage == Approach)
        {
            std::tie(q, v) = _approach.at(j).at(_stage_time);
        }
        else if(_stage == Settle)
        {
            auto& c = _corr.at(j);
            if(correcting && t.amplitude != 0.0)
            {
                settled = c.step(_approach.at(j).to - _q_meas(jidx), dt) && settled;
            }
            // 托举力矩的滑动平均（时间常数 0.5 s），扫频前的余量检查用
            if(std::isfinite(_tau_meas(jidx)))
            {
                _hold_tau(jidx) += (_tau_meas(jidx) - _hold_tau(jidx))*std::min(1.0, dt/0.5);
            }
            q = _approach.at(j).to + c.offset;
            _offset_log(jidx) = c.offset;
        }
        else if(_stage == Sweep)
        {
            // 慢速中心跟踪：扫频中心 = 中心 + 偏移，偏移按低通后的 (中心 − 实测角度) 慢慢积分
            if(correcting && t.amplitude != 0.0 && _sweep_correction_gain > 0)
            {
                auto& c = _corr.at(j);
                c.offset = _sweep_track.at(j).step(c.offset, _approach.at(j).to - _q_meas(jidx), dt);
                t.initial_value = _approach.at(j).to + c.offset;
                _offset_log(jidx) = c.offset;
            }
            std::tie(q, v, a) = t.step(dt);
            trj_time = t.get_time();
            if(_stop_time > 0 && trj_time > _stop_time)
            {
                trj_ended = true;
            }
        }
        else
        {
            std::tie(q, v) = _return.at(j).at(_stage_time);
        }

        _cycle(jidx) = t.get_cycle();
        _freq(jidx) = _stage == Sweep ? t.get_freq() : 0.0;

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
    }

    _elapsed += dt;

    // 阶段切换（在下一周期生效）
    if(_stage == Approach && _stage_time >= _approach_time)
    {
        _stage = Settle;
        _stage_time = 0;
        jinfo("t {:.1f} s: at the sweep center, settling {} s", _elapsed, _settle_time);
    }
    if(_stage == Settle && _stage_time >= _settle_time && (!correcting || settled || _stage_time >= _max_settle_time))
    {
        std::string no_headroom;
        for(auto& [j, t] : _trj_map)
        {
            const int i = _model->getDofIndex(j->get_name());
            const double center = _approach.at(j).to, offset = _corr.at(j).offset;
            t.initial_value = center + offset;   // 扫频绕修正后的参考进行，实际中心在水平位置
            if(t.amplitude == 0.0)
            {
                continue;
            }
            const double hold = _hold_tau(i), peak = std::abs(hold) + 98.7*std::abs(t.amplitude);
            if(correcting)
            {
                jinfo("t {:.1f} s: {} {} at {:.4f} rad (target {:.4f}, error {:.4f} rad = {:.1f} deg), reference offset {:.3f} rad (~{:.1f} Nm at 98.7 Nm/rad)",
                      _elapsed, j->get_name(), settled ? "settled" : "NOT settled within max_settle_time,", _q_meas(i), center,
                      center - _q_meas(i), (center - _q_meas(i))*180/actuator_collection::kPi, offset, offset*98.7);
                _sweep_track.at(j).reset(center - _q_meas(i));
            }
            // 驱动器约 80–85 N·m 饱和：托举力矩 + Kp·幅值 是关节不动时扫频要求的最大力矩
            jinfo("t {:.1f} s: {} holding tau_link {:.1f} Nm; hold + 98.7 x amplitude = {:.1f} Nm (drive saturates around 80-85 Nm)",
                  _elapsed, j->get_name(), hold, peak);
            if(!no_headroom.empty())
            {
                continue;
            }
            if(_max_hold_torque > 0 && std::abs(hold) > _max_hold_torque)
            {
                no_headroom = fmt::format("{}: holding |tau_link| {:.1f} Nm > max_hold_torque {} Nm", j->get_name(), std::abs(hold), _max_hold_torque);
            }
            else if(_max_hold_torque > 0 && correcting && _max_correction > 0 && std::abs(offset) >= _max_correction - 1e-6)
            {
                no_headroom = fmt::format("{}: reference offset {:.3f} rad reached max_correction {} rad", j->get_name(), offset, _max_correction);
            }
        }
        if(no_headroom.empty())
        {
            _stage = Sweep;
            _stage_time = 0;
            jinfo("t {:.1f} s: sweep starts ({} s)", _elapsed, _stop_time);
        }
        else
        {
            // 余量不够：不扫频，从当前参考平滑回起点，回到后以 stop_code 5 停止
            for(auto& [j, t] : _trj_map)
            {
                _return.at(j).from = _q_cmd(_model->getDofIndex(j->get_name()));
            }
            _end_code = 5;
            _stage = Return;
            _stage_time = 0;
            jinfo("NO TORQUE HEADROOM at t = {:.1f} s: {}; not sweeping, returning to the start ({:.1f} s). "
                  "Use a lighter load / shorter radius or a center further below horizontal.", _elapsed, no_headroom, _return_time);
        }
    }
    if(trj_ended)
    {
        if(_side != 0.0)
        {
            // 从最后下发的参考平滑回到起点（有 fade_time 时最后的参考就是中心）
            for(auto& [j, t] : _trj_map)
            {
                _return.at(j).from = _q_cmd(_model->getDofIndex(j->get_name()));
            }
            _stage = Return;
            _stage_time = 0;
            jinfo("t {:.1f} s: sweep finished after {} s, returning to the start ({:.1f} s)", _elapsed, _stop_time, _return_time);
        }
        else
        {
            done = true;
        }
    }
    if(_stage == Return && _stage_time >= _return_time)
    {
        done = true;
    }

    // publish progress
    if(_stop_time > 0)
    {
        std_msgs::msg::Float32 msg;
        msg.data = _stage == Sweep ? trj_time/_stop_time : (_stage == Return ? 1.0 : 0.0);
        _progress_pub->publish(msg);
    }

    // stop if needed
    if(done)
    {
        jinfo(_side != 0.0 ? "back at the start, stopping" : "stopping after {} s", _stop_time);
        _stop_code = _end_code;

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
    _logger->add("trajectory_time", _elapsed);   // 原来记扫频时间；side = 0 时两者相同
    _logger->add("chirp_phase", double(stage_cmd));
    _logger->add("center_offset", _offset_log);
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
    // t 是插件启动后的总时间（水平模式含移动、停稳、返回），不是 stop_time。
    static const char* reasons[] = {"finished normally", "THERMAL STOP", "?", "SAFETY STOP", "stopped externally (plugin switch 0 / XBot2 safety)",
                                    "NO TORQUE HEADROOM, returned without sweeping"};
    jinfo("{} finished: stop_code {} ({}) at t = {:.1f} s (stop_time {} s), {} log samples dropped", getName(), _stop_code,
          _stop_code >= 0 && _stop_code <= 5 ? reasons[_stop_code] : "?", _elapsed, _stop_time, _dropped);
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

XBOT2_REGISTER_PLUGIN(ChirpTrajectoryPlugin, chirp_trajectory_plugin_haotian)
