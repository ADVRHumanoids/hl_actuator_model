#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

namespace actuator_collection {

// 策略式目标位置：模拟 RL 策略按控制频率发出的零阶保持目标，而不是平滑参考轨迹。
// 纯数学实现，不访问电机或 ROS；离线预览和实时插件共用。
// 生成方式：在 [-1, 1] 里随机取航点，每个航点用"阶跃"或"线性斜坡"到达，然后停留一段时间；
// 乘以幅值 A 和起止包络后，再限制相邻两次目标之间的变化不超过 max_step。
// 这样数据里既有"误差大、速度小"（阶跃后、停留时带载保持），也有不同速度的跟踪段。
struct PolicyTargets
{
    double amplitude = 0.3;        // A：目标相对 q0 的最大偏移，输出端 rad。
    double initial_value = 0.0;    // q0：插件启动时的位置参考。
    double policy_rate_hz = 50.0;  // 目标更新频率 Hz；两次更新之间目标保持不变。
    double duration = 60.0;        // T：整段时长 s，含两端包络过渡。
    double warmup_time = 5.0;      // R：开头渐入、结尾渐出的时长 s，要求 T >= 2R。
    double dwell_min = 0.2;        // 每个航点停留时间范围 s。
    double dwell_max = 1.5;
    double ramp_min = 0.1;         // 斜坡过渡用时范围 s。
    double ramp_max = 1.0;
    double step_probability = 0.4; // 以阶跃方式到达下一个航点的概率，其余用斜坡。
    double max_step = 0.05;        // 相邻两次目标的最大变化 rad；阶跃会被拆成每帧不超过此值。
    std::uint64_t seed = 0;        // 随机种子；同一种子在 Windows 预览和台架上生成相同序列。

    // 每个策略周期所处的阶段，记录到日志里方便分析。
    enum Segment { Dwell = 0, Ramp = 1, Step = 2 };

    void validate() const
    {
        const double values[] = {amplitude, initial_value, policy_rate_hz, duration, warmup_time,
                                 dwell_min, dwell_max, ramp_min, ramp_max, step_probability, max_step};
        for(double v : values)
            if(!std::isfinite(v)) throw std::invalid_argument("policy targets: nonfinite parameter");
        if(amplitude < 0 || policy_rate_hz <= 0 || warmup_time <= 0 || duration < 2*warmup_time ||
           dwell_min <= 0 || dwell_max < dwell_min || ramp_min <= 0 || ramp_max < ramp_min ||
           step_probability < 0 || step_probability > 1 || max_step <= 0)
            throw std::invalid_argument("policy targets: invalid parameter");
    }

    // 目标个数：t = 0, 1/rate, ..., T，共 floor(T*rate)+1 个。
    int count() const { return int(std::floor(duration*policy_rate_hz + 1e-9)) + 1; }

    // 当前时间 t 对应的目标下标（零阶保持）；t 超出 [0, T] 时取首尾。
    int index(double t) const
    {
        const int k = int(std::floor(t*policy_rate_hz + 1e-9));
        return std::clamp(k, 0, count() - 1);
    }

    // 一次性生成全部目标 q 和阶段 segment（长度 count()）。插件在 on_start 调用，
    // 实时循环只查表，不做随机数和内存分配。首尾目标都等于 q0。
    void generate(std::vector<double>& q, std::vector<int>& segment) const
    {
        validate();
        const int n = count();
        const double dt = 1.0/policy_rate_hz;
        // mt19937_64 的输出由 C++ 标准规定；自己换算成 [0,1)，不用各标准库实现不同的 distribution。
        std::mt19937_64 rng(seed);
        auto uniform = [&](double lo, double hi) { return lo + (hi - lo)*double(rng() >> 11)*0x1.0p-53; };
        auto ticks = [&](double seconds) { return std::max(1, int(std::lround(seconds/dt))); };

        // 1. 归一化航点序列 u ∈ [-1, 1]。
        std::vector<double> u(n, 0.0);
        segment.assign(n, Dwell);
        double current = 0;
        for(int k = 1; k < n; )
        {
            const double next = uniform(-1, 1);
            if(uniform(0, 1) < step_probability)
            {
                u[k] = next;
                segment[k] = Step;
                ++k;
            }
            else
            {
                const int m = ticks(uniform(ramp_min, ramp_max));
                for(int j = 1; j <= m && k < n; ++j, ++k)
                {
                    u[k] = current + (next - current)*j/m;
                    segment[k] = Ramp;
                }
            }
            current = next;
            const int d = ticks(uniform(dwell_min, dwell_max));
            for(int j = 0; j < d && k < n; ++j, ++k)
            {
                u[k] = current;
                segment[k] = Dwell;
            }
        }

        // 2. 乘幅值和起止包络，再限制每帧变化量。目标始终在 q0 ± A 内：
        //    每帧只向本帧期望值移动，而期望值和上一帧目标都在该区间内。
        q.assign(n, initial_value);
        for(int k = 1; k < n; ++k)
        {
            const double desired = initial_value + amplitude*envelope(k*dt)*u[k];
            const double change = std::clamp(desired - q[k-1], -max_step, max_step);
            q[k] = q[k-1] + change;
            // 阶跃被拆成多帧时，后续还在追赶的帧也标为 Step，而不是停留。
            if(segment[k] == Dwell && segment[k-1] == Step && change != desired - q[k-1])
                segment[k] = Step;
        }
        // 结尾包络已回到 0；若 max_step 太小、来不及回到 q0，直接报错而不是悄悄截断。
        if(std::abs(q.back() - initial_value) > 1e-9)
            throw std::invalid_argument("policy targets: max_step too small to return to q0 within warmup_time");
        q.back() = initial_value;
    }

    // 起止包络：五次平滑阶跃 S(u)=10u^3-15u^4+6u^5，开头 0→1，结尾 1→0。
    double envelope(double t) const
    {
        auto smooth = [](double x) { x = std::clamp(x, 0.0, 1.0); return x*x*x*(10 + x*(-15 + 6*x)); };
        return std::min(smooth(t/warmup_time), smooth((duration - t)/warmup_time));
    }
};

// 长时间（温升）实验的目标序列：探测段 P 和加热段 H 交替，P H0 P H1 P H2 ...，直到总时长不少于 total。
// 探测段每次都用 probe_seed，目标序列完全相同，用来比较同一输入在不同温度下的响应；
// 第一段就是探测段，作为冷态基准。加热段 k 用种子 base.seed + k。
// 每段都从 q0 出发、回到 q0，所以首尾相接是连续的；段间的过渡由各段自己的包络完成。
// probe_duration <= 0 时只有一段 duration = total、种子 base.seed，与 base.generate() 完全相同。
// 输出：q、segment 同 generate()；probe 为 1 表示探测段；chunk 为段号 0, 1, 2, ...
// 返回实际总时长 s（按整段向上取整，可能略大于 total）。
inline double generate_schedule(const PolicyTargets& base, double total, double heat_duration,
                                double probe_duration, std::uint64_t probe_seed,
                                std::vector<double>& q, std::vector<int>& segment,
                                std::vector<int>& probe, std::vector<int>& chunk)
{
    if(!std::isfinite(total) || total <= 0 || !std::isfinite(heat_duration) || !std::isfinite(probe_duration))
        throw std::invalid_argument("policy schedule: invalid durations");
    const bool probes = probe_duration > 0;
    if(probes && heat_duration <= 0)
        throw std::invalid_argument("policy schedule: heat_duration must be > 0 when probes are enabled");
    q.clear(); segment.clear(); probe.clear(); chunk.clear();
    std::vector<double> qc;
    std::vector<int> sc;
    double covered = 0;
    for(int c = 0, heat = 0; covered < total - 1e-9; ++c)
    {
        PolicyTargets g = base;
        const bool is_probe = probes && c % 2 == 0;
        g.duration = !probes ? total : is_probe ? probe_duration : heat_duration;
        g.seed = is_probe ? probe_seed : base.seed + std::uint64_t(heat++);
        g.generate(qc, sc);  // 每段各自检查 duration >= 2*warmup_time
        if(qc.size() < 2) throw std::invalid_argument("policy schedule: segment shorter than one policy period");
        const std::size_t first = q.empty() ? 0 : 1;  // 段首 = 上一段段尾 = q0，不重复
        q.insert(q.end(), qc.begin() + first, qc.end());
        segment.insert(segment.end(), sc.begin() + first, sc.end());
        probe.insert(probe.end(), qc.size() - first, is_probe ? 1 : 0);
        chunk.insert(chunk.end(), qc.size() - first, c);
        covered += double(qc.size() - 1)/base.policy_rate_hz;
    }
    return covered;
}

} // namespace actuator_collection
