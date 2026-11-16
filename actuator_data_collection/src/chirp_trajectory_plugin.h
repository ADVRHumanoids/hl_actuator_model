#ifndef CHIRP_TRAJECTORY_PLUGIN_H
#define CHIRP_TRAJECTORY_PLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <xbot2/ros/ros2_support.h>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

namespace XBot {

// 与 trj（TrajectoryPlugin）相同的扫频正弦，区别：位置模式下速度参考恒为 0，不给速度前馈，
// 与 RL 策略在机器人上只发目标位置的用法一致；另有可选的输出端安全检查。类型名 chirp_trajectory_plugin_haotian。
class ChirpTrajectoryPlugin : public ControlPlugin
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
    // 日志由后台线程持续写盘：长时间循环扫频（stop_time 很大或为 0）时不会攒在内存里，也不会被循环缓冲覆盖。
    MatAppender::Ptr _appender;
    long _dropped = 0;                      // 日志缓冲满而丢掉的周期数，应为 0。

    Eigen::VectorXd _tau_id;

    ModelInterface::Ptr _model;

    Eigen::VectorXd _qmot, _q, _v, _a, _tauid;

    // 实际下发的速度参考（恒为 0）；_v 仍是正弦的解析速度，记为 v_traj。
    Eigen::VectorXd _v_sent;

    // 安全检查（与 trj_policy 相同）：每项阈值 > 0 时生效，位置检查在 position_min < position_max 时生效。
    // 越限时打印原因并停止插件，不 throw（rt 线程里的异常会让 xbot2-core terminate）。
    double _q_min = 0, _q_max = 0, _max_velocity = 0, _max_link_torque = 0, _max_tracking_error = 0;
    Eigen::VectorXd _q_cmd;                 // 上一周期下发的位置参考，用于跟踪误差检查
    Eigen::VectorXd _q_meas, _v_meas, _tau_meas;

    // 长时间运行：温度和力矩监测。stop_motor_temperature > 0 时电机温度连续 0.5 s 达到它就停（THERMAL STOP），
    // 只在 yaml 里写了这一项时生效；status_period 秒打印一次状态行。
    double _stop_motor_temperature = 0, _status_period = 10, _next_status = 0, _elapsed = 0;
    Eigen::VectorXd _temp_mot, _temp_dri, _tau_mot, _cycle, _freq;
    // 停止原因，写进日志 stop_code：0 跑完；1 电机温度；3 安全检查；4 外部停止或中止（与 trj_policy 相同）。
    int _stop_code = 4;
    void finish();

    struct Chirp
    {
        std::tuple<double, double, double> step(double dt);

        void reset();

        double get_time() const { return _time; }

        double get_freq() const { return _freq_hz; }

        // 第几个扫频周期（频率 freq_min → freq_max → freq_min 为一个周期），从 0 开始。
        double get_cycle() const { return std::floor(_time/period); }

        // 每个关节自己的统计：tau_mot 平方和、样本数（最近一个状态周期），温度超线持续时间。
        double win_sq = 0, win_n = 0, hot_time = 0;

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

#endif // CHIRP_TRAJECTORY_PLUGIN_H
