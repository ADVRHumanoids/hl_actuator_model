#include "trajectory_plugin.h"

using namespace XBot;

bool TrajectoryPlugin::on_initialize()
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

    for(auto [jname, a] : amplitude)
    {
        auto dev = _robot->getDeviceInstance<Hal::JointBase>(jname);

        _trj_map.at(dev).amplitude = a;
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
    _enabled_joints.clear();

    // allocate
    _model = ModelInterface::getModel(_robot->getConfigOptions());
    _model->getJointPosition(_qmot);
    _model->getJointPositionMinimal(_q);
    _model->getJointVelocity(_v);
    _model->getJointAcceleration(_a);
    _model->computeInverseDynamics(_tauid);

    return true;

}

void TrajectoryPlugin::on_start()
{
    _param_upd_queue.run();

    if(_trj_out_str == "Position")
    {
        _trj_out = Position;
    }
    else if(_trj_out_str == "Torque")
    {
        _trj_out = Torque;
    }

    if(_enable_log)
    {
        if(_log_file.empty())
        {
            _log_file = "/tmp/data_acquisition_plugin_log";
        }

        MatLogger2::Options opt;
        opt.default_buffer_size = _stop_time > 0 ? std::ceil(_stop_time / getPeriodSec()) : 1e6;

        _logger = MatLogger2::MakeLogger(_log_file,
                                         opt);

        _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);

        const int nv = _robot->getNv();

        _logger->create("q_ref", nv);
        _logger->create("q_link", nv);
        _logger->create("q_mot", nv);
        _logger->create("v_ref", nv);
        _logger->create("v_link", nv);
        _logger->create("v_mot", nv);
        _logger->create("tau_link", nv);
        _logger->create("tau_mot", nv);
        _logger->create("tau_id", nv);
    }


    for(auto& [j, t] : _trj_map)
    {
        j->sense();
        t.reset();
        t.initial_value = j->get_pos_ref();

        auto it = std::find(_enabled_joints.begin(), _enabled_joints.end(), j->get_name());

        if(it == _enabled_joints.end())
        {
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

    _enabled_joints.clear();

    Hal::JointSafety::enable_filter(false, -1);
}

void TrajectoryPlugin::starting()
{
    start_completed();
}

void TrajectoryPlugin::run()
{
    // update telemetry
    _robot->sense(false);

    // execute trajectories
    bool trj_ended = false;
    double trj_time = 0;

    for(auto& [j, t] : _trj_map)
    {
        auto [q, v, a] = t.step(getPeriodSec());

        int jidx = _model->getDofIndex(j->get_name());

        if(_trj_out == Position)
        {
            _q(jidx) = q;
            _v(jidx) = v;
            _a(jidx) = a;

            j->set_pos_ref(q);
            j->set_vel_ref(v);
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

    _logger->add("q_ref", _q);
    _logger->add("v_ref", _v);
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

}

void TrajectoryPlugin::on_stop()
{
    Hal::JointSafety::enable_filter(true, -1);
    _logger.reset();
}

void TrajectoryPlugin::on_abort()
{
    Hal::JointSafety::enable_filter(true, -1);
    _logger.reset();
}

std::tuple<double, double, double> TrajectoryPlugin::Chirp::step(double dt)
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

void TrajectoryPlugin::Chirp::reset()
{
    _phase = 0;
    _time = 0;
}

XBOT2_REGISTER_PLUGIN(TrajectoryPlugin, data_collection_plugin_tianhao)
