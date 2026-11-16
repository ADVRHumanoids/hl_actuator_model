#ifndef POLICY_TRAJECTORY_PLUGIN_H
#define POLICY_TRAJECTORY_PLUGIN_H
#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <xbot2/ros/ros2_support.h>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include "policy_targets.h"
namespace XBot {
// 策略式目标采集插件：和 TrajectoryPlugin（trj）完全独立，类型名 policy_trajectory_plugin_haotian。
// 单关节、位置模式，只下发零阶保持的目标位置，速度参考恒为 0，与 RL 策略在机器人上的用法一致。
class PolicyTrajectoryPlugin : public ControlPlugin {
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
    actuator_collection::PolicyTargets _gen;
    Ros2Support::UniquePtr _ros;
    Publisher<std_msgs::msg::Float32>::Ptr _progress_pub;
    CallbackQueue _param_upd_queue;
    ModelInterface::Ptr _model;
    MatLogger2::Ptr _logger;
    // 日志由后台线程持续写盘，长时间实验不会把整段数据攒在内存里，xbot2-core 异常退出时也只丢最后几十秒。
    MatAppender::Ptr _appender;
    Hal::JointBase* _joint = nullptr;
    int _idx = -1;                       // 关节在模型 DOF 向量里的下标。
    std::string _joint_name, _log_file;
    std::vector<double> _targets;        // on_start 里一次生成；run 只查表。
    std::vector<int> _segments;
    std::vector<int> _probe, _chunk;     // 探测段标记和段号，见 policy_targets.h 的 generate_schedule。
    double _stop_time = 60, _elapsed = 0, _seed = 0;
    // 输出端安全阈值；为 0 表示未配置，不能启动。
    double _q_min = 0, _q_max = 0, _max_velocity = 0, _max_link_torque = 0, _max_tracking_error = 0;
    // 下发参考的最大变化率 rad/s：XBot2 HAL 会拒绝 d(pos_ref)/dt > 20 rad/s，阶跃目标按此速率在几毫秒内走完。
    double _max_ref_rate = 15.0;
    // 温升 / 过载实验。额定、峰值力矩（输出端 N·m）和电机温度停机线必须配置，否则不能启动。
    double _rated_torque = 0, _peak_torque = 0, _stop_motor_temperature = 0;
    double _stop_driver_temperature = 0;      // 驱动器温度停机线 °C，0 表示不检查。
    double _max_start_motor_temperature = 0;  // 启动时电机温度上限 °C（保证每档从冷态开始），0 表示不检查。
    double _probe_duration = 0, _heat_duration = 300, _probe_seed = 1000;  // probe_duration = 0 时不插探测段。
    double _status_period = 10;               // 每隔多少秒打印一次温度和力矩统计。
    double _schedule_time = 0;                // 目标序列实际总时长（按整段取整）。
    // 运行统计：tau_mot 的平方和、超过额定的周期数，分"最近一个打印周期"和"从开始"两组。
    double _win_sq = 0, _win_over = 0, _win_n = 0, _tot_sq = 0, _tot_over = 0, _tot_n = 0, _next_status = 0;
    double _hot_motor_time = 0, _hot_driver_time = 0, _start_motor_temperature = 0;
    long _dropped = 0;                        // 日志缓冲满而丢掉的周期数，应为 0。
    // 停止原因，写进日志 stop_code：0 跑完；1 电机温度；2 驱动器温度；3 安全检查；4 外部停止或中止。
    int _stop_code = 4;
    double _q_target = 0;                // 当前策略目标（限速前），单独记日志。
    int _k_cmd = 0;                      // 上一条指令对应的策略目标下标。
    bool _enable_log = true, _disable_reference_filter = true, _filter_disabled = false, _have_command = false;
    // 参考缓存与实测临时缓存分开，避免记录时混用。
    Eigen::VectorXd _q_ref, _active;
    Eigen::VectorXd _q, _qmot, _v, _tau, _temp_mot, _temp_dri, _tau_mot;
    std::chrono::steady_clock::time_point _clock_start;
};
}
#endif
