#ifndef CHIRP_TRAJECTORY_PLUGIN_H
#define CHIRP_TRAJECTORY_PLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <xbot2/ros/ros2_support.h>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include "chirp_schedule.h"

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

    // 扫频和过渡移动的计算在 chirp_schedule.h（可离线测试）。
    using Chirp = actuator_collection::Chirp;
    using Transfer = actuator_collection::Transfer;

    // 水平扫频模式（side = +1 / -1）：起点 → 水平中心 hang_position + side·π/2（Approach）→ 停稳 settle_time（Settle）
    // → 扫频 stop_time 秒，最后 fade_time 秒幅值衰减到 0（Sweep）→ 回到起点（Return）→ 自动停止。
    // side = 0 时绕起点扫频，与原来的 trj_chirp 完全相同（settle_time、fade_time 默认 0）。
    enum Stage { Approach = 0, Settle = 1, Sweep = 2, Return = 3 };
    double _side = 0, _hang_position = 0, _transfer_velocity = 0.3, _min_transfer_time = 2.0;
    double _settle_time = 0, _fade_time = 0;
    int _stage = Sweep;
    double _stage_time = 0, _approach_time = 0, _return_time = 0;
    std::map<Hal::JointBase*, Transfer> _approach, _return;
    Eigen::VectorXd _stage_log;

    CallbackQueue _param_upd_queue;

    std::map<Hal::JointBase*, Chirp> _trj_map;

    std::vector<std::string> _enabled_joints;
};

}

#endif // CHIRP_TRAJECTORY_PLUGIN_H
