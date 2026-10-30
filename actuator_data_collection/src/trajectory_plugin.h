#ifndef TRAJECTORY_PLUGIN_H
#define TRAJECTORY_PLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
#include <xbot2/ros/ros2_support.h>
#include <std_msgs/msg/float32.hpp>

namespace XBot {

class TrajectoryPlugin : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void starting() override;
    void run() override;
    void on_stop() override;
    void on_abort() override;

private:

    double _stop_time = 0;

    bool _enable_log = false;

    std::string _log_file;

    enum TrajectoryOutput {
        Position,
        Torque
    };

    Ros2Support::UniquePtr _ros;

    Publisher<std_msgs::msg::Float32>::Ptr _progress_pub;

    TrajectoryOutput _trj_out = Position;

    std::string _trj_out_str = "Position";

    MatLogger2::Ptr _logger;

    Eigen::VectorXd _tau_id;

    ModelInterface::Ptr _model;

    Eigen::VectorXd _qmot, _q, _v, _a, _tauid;

    struct Chirp
    {
        std::tuple<double, double, double> step(double dt);

        void reset();

        double get_time() const { return _time; }

        double warmup_time = 1.0;
        double freq_min_hz = 0.1;
        double freq_max_hz = 1.0;
        double period = 5.0;
        double amplitude = 0.0;
        double phase_offset = 0.0;
        double initial_value = 0.0;

    private:

        double _phase = 0;
        double _freq_hz = 0;
        double _time = 0;


    };

    CallbackQueue _param_upd_queue;

    std::map<Hal::JointBase*, Chirp> _trj_map;

    std::vector<std::string> _enabled_joints;
};

}

#endif // TRAJECTORY_PLUGIN_H
