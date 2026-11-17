#ifndef STATIC_HOLD_PLUGIN_H
#define STATIC_HOLD_PLUGIN_H
#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <xbot2/ros/ros2_support.h>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include "static_schedule.h"
namespace XBot {
// 静态带载检查插件：单关节、位置模式，依次移动到 q0 + offsets[i] 并保持，全部测完回到 q0 后自动停止。
// 用来核对力矩传感器的方向、零点、比例，编码器零位，以及静态时 tau_link 与 tau_mot 的关系。
// 与其他轨迹插件独立，类型名 static_hold_plugin_haotian，yaml 里叫 trj_static。
class StaticHoldPlugin : public ControlPlugin {
public:
    using ControlPlugin::ControlPlugin;
    bool on_initialize() override;
    void on_start() override;
    void starting() override;
    void run() override;
    void on_stop() override;
    void on_abort() override;
private:
    void finish();
    void print_point();
    actuator_collection::StaticSchedule _schedule;
    std::size_t _cursor = 0;
    Ros2Support::UniquePtr _ros;
    Publisher<std_msgs::msg::Float32>::Ptr _progress_pub;
    CallbackQueue _param_upd_queue;
    ModelInterface::Ptr _model;
    MatLogger2::Ptr _logger;
    MatAppender::Ptr _appender;
    Hal::JointBase* _joint = nullptr;
    int _idx = -1;
    std::string _joint_name, _log_file;
    std::vector<double> _offsets;
    // mode: "static" 按 offsets 逐点测试后回到 q0；"goto" 平滑移动到绝对位置 target 后停住（不回 q0），
    // 用来在测试前把关节放到指定角度（例如配重自然下垂的位置）。同一个插件类型在 yaml 里配两个实例。
    bool _goto = false;
    double _target = 0;
    // 安全阈值：位置限和前三项为 0 表示未配置，不能启动；stop_motor_temperature 为 0 表示不检查温度。
    double _q_min = 0, _q_max = 0, _max_velocity = 0, _max_link_torque = 0, _max_tracking_error = 0;
    double _stop_motor_temperature = 0, _hot_time = 0;
    double _elapsed = 0;
    bool _enable_log = true, _disable_reference_filter = true, _filter_disabled = false;
    bool _velocity_feedforward = true, _have_command = false;
    // 上一周期下发的指令（与实测同一行记录）。
    double _q_cmd = 0, _v_cmd = 0;
    int _phase_cmd = 0, _point_cmd = 0;
    // 当前位置 Hold 段的累计量，换到下一个位置时打印平均值。
    int _acc_point = -1;
    double _acc_n = 0, _acc_q = 0, _acc_tau = 0, _acc_tau_mot = 0;
    long _dropped = 0;
    int _stop_code = 4;   // 0 跑完；1 电机温度；3 安全检查；4 外部停止或中止（与其他插件相同）
    Eigen::VectorXd _q_ref, _active, _q, _qmot, _v, _tau, _tau_mot, _temp_mot, _temp_dri;
    std::chrono::steady_clock::time_point _clock_start;
};
}
#endif
