#ifndef THERMAL_SOAK_PLUGIN_H
#define THERMAL_SOAK_PLUGIN_H
#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <chrono>
#include <string>
#include "thermal_soak.h"
namespace XBot {
// 升温保温插件（yaml 里叫 trj_heat，类型名 thermal_soak_plugin_haotian）：输出端用 pin 固定，
// 位置模式下让参考偏离固定位置产生堵转力矩，温度 PID 调力矩，把电机维持在 setpoint 一段时间（热量传到减速器），
// 然后力矩斜坡降到 0、参考回到固定位置，自动停止并提示可以拔 pin。控制逻辑在 thermal_soak.h。
class ThermalSoakPlugin : public ControlPlugin {
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
    void halt(int code);   // 参考设到当前实测位置（卸掉力矩）后停止
    void log_row(double q, double v, double tau);
    actuator_collection::ThermalSoak _ctrl;
    CallbackQueue _param_upd_queue, _live_queue;
    ModelInterface::Ptr _model;
    MatLogger2::Ptr _logger;
    MatAppender::Ptr _appender;
    Hal::JointBase* _joint = nullptr;
    int _idx = -1;
    std::string _joint_name, _log_file;
    double _direction = 1.0;          // 力矩方向 +1 / -1
    double _torque_offset = 0.0;      // 力矩传感器零偏 N·m，只用于安全检查和打印
    // 安全阈值（必填，0 表示未配置不能启动；stop_driver_temperature 为 0 表示不检查）
    double _max_link_torque = 0, _max_position_drift = 0, _max_velocity = 0;
    double _stop_motor_temperature = 0, _stop_driver_temperature = 0;
    double _status_period = 10.0, _log_period = 0.1;
    bool _release_only = false, _release = false;
    bool _enable_log = true, _disable_reference_filter = true, _filter_disabled = false;
    double _q_pin = 0, _q_cmd = 0, _elapsed = 0, _next_status = 0, _next_log = 0;
    double _hot_mot = 0, _hot_dri = 0;
    long _dropped = 0;
    int _stop_code = 4;   // 0 完成；1 电机温度；2 驱动器温度；3 安全检查；4 外部停止；5 升温超时（已释放）
    Eigen::VectorXd _q, _qmot, _v, _tau, _tau_mot, _temp_mot, _temp_dri;
    std::chrono::steady_clock::time_point _clock_start;
};
}
#endif
