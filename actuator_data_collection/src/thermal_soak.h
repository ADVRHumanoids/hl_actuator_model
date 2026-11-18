#ifndef THERMAL_SOAK_H
#define THERMAL_SOAK_H
// 升温保温控制器（不依赖 XBot2，可离线测试）：输出端用 pin 固定后，在位置模式下让参考偏离固定位置，
// 堵转力矩约等于 驱动器刚度 × 偏移。温度 PID 的输出 u ∈ [0, 1] 是加热功率的比例（铜损 ∝ 电流² ∝ 力矩²），
// 所以力矩 = max_torque · sqrt(u)，这样温度环近似线性。
// 流程：Heat（升温，u 多半饱和在 1）→ Soak（第一次进入 setpoint ± tolerance 后开始计时，继续 PID 保温，
// 让热量传到减速器）→ Release（力矩按 torque_rate 降到 0，参考回到固定位置）→ Done。
#include <algorithm>
#include <cmath>
#include <string>

namespace actuator_collection {

struct ThermalSoak {
    enum Phase { Heat = 0, Soak = 1, Release = 2, Done = 3 };

    // 参数（含义见 xbot2_config.yaml 的 trj_heat）
    double setpoint = 50.0;          // 目标电机温度 °C
    double tolerance = 1.0;          // 进入 setpoint ± tolerance 开始计保温时间 °C
    double soak_time = 1800.0;       // 保温时长 s；0 表示一直保温，直到 request_release()
    double max_heat_time = 3600.0;   // 升温阶段最长 s，超时说明力矩不够，直接释放；0 表示不限
    double max_torque = 30.0;        // 堵转力矩上限 N·m（输出端）
    double drive_stiffness = 98.7;   // 驱动器位置环刚度 N·m/rad（输出端），力矩 → 参考偏移
    double torque_rate = 2.0;        // 力矩变化率上限 N·m/s（加载和释放都按它）
    double kp = 0.2;                 // PID：u 每 °C
    double ki = 0.0003;              // 1/(°C·s)
    double kd = 0.0;                 // s/°C，作用于滤波后温度的变化率
    double temperature_filter = 2.0; // 温度一阶低通时间常数 s
    double release_hold = 1.0;       // 力矩降到 0 后再等多久算完成 s

    // 状态
    Phase phase = Heat;
    double t = 0, soak_elapsed = 0, done_elapsed = 0;
    double temp_filtered = NAN, temp_prev = NAN;
    double p = 0, i = 0, d = 0, u = 0;
    double torque_cmd = 0;  // 斜坡后的力矩指令 N·m（≥ 0，方向在插件里乘）
    double offset0 = 0;     // 启动时参考相对固定位置的偏移 rad（无扰启动），按斜坡衰减到 0
    bool soak_started = false, timed_out = false;

    std::string validate() const
    {
        auto finite = [](double x) { return std::isfinite(x); };
        if(!finite(setpoint) || setpoint <= 0) return "setpoint must be > 0";
        if(!finite(tolerance) || tolerance <= 0) return "tolerance must be > 0";
        if(!finite(soak_time) || soak_time < 0) return "soak_time must be >= 0";
        if(!finite(max_heat_time) || max_heat_time < 0) return "max_heat_time must be >= 0";
        if(!finite(max_torque) || max_torque <= 0) return "max_torque must be > 0";
        if(!finite(drive_stiffness) || drive_stiffness <= 0) return "drive_stiffness must be > 0";
        if(!finite(torque_rate) || torque_rate <= 0) return "torque_rate must be > 0";
        if(!finite(kp) || !finite(ki) || !finite(kd) || kp < 0 || ki < 0 || kd < 0) return "kp, ki, kd must be >= 0";
        if(!finite(temperature_filter) || temperature_filter < 0) return "temperature_filter must be >= 0";
        if(!finite(release_hold) || release_hold < 0) return "release_hold must be >= 0";
        return "";
    }

    // initial_offset：启动时 (参考 − 实测位置)·方向，用来无扰接管；release_only 时直接进入释放。
    void reset(double initial_offset, bool release_only)
    {
        phase = release_only ? Release : Heat;
        t = soak_elapsed = done_elapsed = 0;
        temp_filtered = temp_prev = NAN;
        p = i = d = u = 0;
        offset0 = initial_offset;
        torque_cmd = 0;
        soak_started = timed_out = false;
    }

    void request_release()
    {
        if(phase == Heat || phase == Soak) phase = Release;
    }

    // 参考相对固定位置的偏移 rad（乘方向前）
    double offset() const { return torque_cmd/drive_stiffness + offset0; }

    // 每个控制周期调用一次；temperature 是电机温度 °C。
    void step(double temperature, double dt)
    {
        t += dt;
        // 温度低通 + 微分（对测量值，避免设定值跳变引起冲击）
        if(!std::isfinite(temp_filtered)) temp_filtered = temperature;
        else temp_filtered += (temperature - temp_filtered)*dt/(temperature_filter + dt);
        const double rate = std::isfinite(temp_prev) ? (temp_filtered - temp_prev)/dt : 0.0;
        temp_prev = temp_filtered;

        double target = 0;
        if(phase == Heat || phase == Soak) {
            const double e = setpoint - temp_filtered;
            p = kp*e;
            d = -kd*rate;
            // 条件积分抗饱和：输出已饱和且误差会让它更饱和时不积分
            const double u_free = p + i + d;
            if(!((u_free >= 1 && e > 0) || (u_free <= 0 && e < 0))) i += ki*e*dt;
            i = std::clamp(i, -1.0, 1.0);
            u = std::clamp(p + i + d, 0.0, 1.0);
            target = max_torque*std::sqrt(u);

            if(phase == Heat) {
                if(std::abs(e) <= tolerance) { phase = Soak; soak_started = true; }
                else if(max_heat_time > 0 && t >= max_heat_time) { timed_out = true; phase = Release; target = 0; }
            }
            if(phase == Soak) {
                soak_elapsed += dt;
                if(soak_time > 0 && soak_elapsed >= soak_time) { phase = Release; target = 0; }
            }
        }
        if(phase == Release || phase == Done) { u = 0; target = 0; }

        // 力矩斜坡；启动偏移按同样的速率衰减
        const double step_tau = torque_rate*dt;
        torque_cmd += std::clamp(target - torque_cmd, -step_tau, step_tau);
        const double step_off = step_tau/drive_stiffness;
        offset0 -= std::clamp(offset0, -step_off, step_off);

        if(phase == Release && torque_cmd <= 0 && offset0 == 0) {
            done_elapsed += dt;
            if(done_elapsed >= release_hold) phase = Done;
        }
    }
};

}  // namespace actuator_collection
#endif
