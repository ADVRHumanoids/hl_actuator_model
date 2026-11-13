#ifndef POLICY_TRAJECTORY_PLUGIN_H
#define POLICY_TRAJECTORY_PLUGIN_H
#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
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
    Hal::JointBase* _joint = nullptr;
    int _idx = -1;                       // 关节在模型 DOF 向量里的下标。
    std::string _joint_name, _log_file;
    std::vector<double> _targets;        // on_start 里一次生成；run 只查表。
    std::vector<int> _segments;
    double _stop_time = 60, _elapsed = 0, _seed = 0;
    // 输出端安全阈值；为 0 表示未配置，不能启动。
    double _q_min = 0, _q_max = 0, _max_velocity = 0, _max_link_torque = 0, _max_tracking_error = 0;
    bool _enable_log = true, _disable_reference_filter = true, _filter_disabled = false, _have_command = false;
    // 参考缓存与实测临时缓存分开，避免记录时混用。
    Eigen::VectorXd _q_ref, _active;
    Eigen::VectorXd _q, _qmot, _v, _tau;
    std::chrono::steady_clock::time_point _clock_start;
};
}
#endif
