#ifndef CHIRP_SCHEDULE_H
#define CHIRP_SCHEDULE_H
// 扫频正弦和过渡移动（不依赖 XBot2，可离线测试）。chirp_trajectory_plugin 用它们：
// Chirp 与原先插件里的实现逐行相同，只多了可选的结尾幅值衰减（fade_time = 0 时与原来完全一样）；
// Transfer 是五次多项式移动，用于水平扫频模式里 起点 → 水平中心 和 中心 → 起点。
#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>

namespace actuator_collection {

constexpr double kPi = 3.14159265358979323846;

// 五次多项式 0 → 1，起止速度、加速度为 0；quintic_d 是对 s 的导数。
inline double quintic(double s)
{
    s = std::clamp(s, 0.0, 1.0);
    return s*s*s*(10 + s*(-15 + 6*s));
}
inline double quintic_d(double s)
{
    if(s <= 0 || s >= 1) return 0;
    return 30*s*s*(1 - s)*(1 - s);
}

struct Chirp
{
    // 返回 (q, v, a)：位置参考、解析速度、解析加速度（后两者只记录，不下发）。
    std::tuple<double, double, double> step(double dt)
    {
        double dt_scaling = 1.0;

        if(_time <= warmup_time)
        {
            double tau = _time/warmup_time;
            dt_scaling = ((6*tau - 15)*tau + 10)*tau*tau*tau;
        }

        _freq_hz = freq_min_hz + (1 - std::cos(2*kPi/period*_time))/2.0*(freq_max_hz - freq_min_hz);

        _phase += _freq_hz*2*kPi*dt_scaling*dt;

        _time += dt;

        // 结尾幅值衰减：最后 fade_time 秒内按五次多项式从 1 降到 0，扫频结束时正好回到中心、速度为 0。
        double scale = 1.0, scale_d = 0.0;
        if(fade_time > 0 && duration > 0 && _time > duration - fade_time)
        {
            const double s = (_time - (duration - fade_time))/fade_time;
            scale = 1.0 - quintic(s);
            scale_d = -quintic_d(s)/fade_time;
        }

        const double arg = dt_scaling*phase_offset + _phase;
        const double w = _freq_hz*2*kPi*dt_scaling;

        double qref = initial_value + scale*amplitude*std::sin(arg);

        double vref = scale*amplitude*std::cos(arg)*w + scale_d*amplitude*std::sin(arg);

        double dfreq = std::sin(2*kPi/period*_time)*(freq_max_hz - freq_min_hz)*kPi/period;

        double aref = scale*(amplitude*std::cos(arg)*(dfreq*2*kPi*dt_scaling) +
                             -amplitude*std::pow(w, 2)*std::sin(arg));

        return {qref, vref, aref};
    }

    void reset()
    {
        _phase = 0;
        _time = 0;
    }

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
    double initial_value = 0.0;   // 扫频中心
    double fade_time = 0.0;       // 结尾衰减时长 s，0 表示不衰减（原来的行为）
    double duration = 0.0;        // 扫频总时长 s（stop_time），0 表示不限，此时不衰减

private:

    double _phase = 0;
    double _freq_hz = 0;
    double _time = 0;
};

// 五次多项式移动 from → to，时长 T（T = 0 表示已在目标）。
struct Transfer
{
    double from = 0, to = 0, T = 0;

    // 峰值速度不超过 max_velocity（五次多项式峰值速度 = 1.875·d/T），且不短于 min_time。
    static double duration(double distance, double max_velocity, double min_time)
    {
        if(distance == 0) return 0;
        return std::max(min_time, 1.875*std::abs(distance)/max_velocity);
    }

    std::pair<double, double> at(double t) const
    {
        if(T <= 0) return {to, 0.0};
        const double s = t/T;
        return {from + (to - from)*quintic(s), (to - from)*quintic_d(s)/T};
    }
};

// 停稳阶段的中心修正：驱动器只有 PD 位置环（无积分），水平位置托住配重时实际角度比参考差
// 重力矩 / Kp（7 kg@0.3 m 约 0.22 rad）再加静摩擦。这里用实测角度慢慢积分出参考偏移，
// 直到实际角度在 tolerance 内保持 hold_time，扫频就绕"修正后的参考"进行，实际中心在水平位置。
struct CenterCorrector
{
    double gain = 0.5;          // 1/s：参考偏移的变化率 = gain × 误差
    double tolerance = 0.01;    // rad
    double hold_time = 1.0;     // 误差连续在 tolerance 内多久算到位 s
    double max_offset = 0.6;    // |参考偏移| 上限 rad

    double offset = 0, within = 0;

    void reset() { offset = within = 0; }

    // error = 目标中心 − 实测角度；返回是否已到位。
    bool step(double error, double dt)
    {
        offset = std::clamp(offset + gain*error*dt, -max_offset, max_offset);
        within = std::abs(error) <= tolerance ? within + dt : 0.0;
        return within >= hold_time;
    }
};

// 扫频阶段的慢速中心跟踪：停稳阶段积分出的偏移里含有单向的静摩擦，扫频时摩擦正负抵消，
// 实际平均角度会偏离中心（11-23 日志中 0.07–0.14 rad）；饱和时上下不对称也会让平均角度漂走。
// 这里把 (中心 − 实测角度) 先低通（时间常数 filter_time，取最低扫频频率的周期，滤掉扫频本身），
// 再以很小的增益积分进参考偏移。低通 + 积分的闭环：ζ = 1/(2·sqrt(filter_time·gain))，
// gain ≤ 0.5/filter_time 时不超调（filter_time 10 s 时 gain ≤ 0.05）。gain = 0 关闭。
struct SweepCenterTracker
{
    double gain = 0.0;          // 1/s
    double filter_time = 10.0;  // s
    double max_offset = 0.6;    // 与停稳阶段共用的 |参考偏移| 上限 rad

    double filtered = 0;

    void reset(double error) { filtered = error; }

    // error = 目标中心 − 实测角度；返回更新后的参考偏移。
    double step(double offset, double error, double dt)
    {
        if(gain <= 0) return offset;
        filtered += (error - filtered)*std::min(1.0, dt/filter_time);
        return std::clamp(offset + gain*filtered*dt, -max_offset, max_offset);
    }
};

}  // namespace actuator_collection
#endif
