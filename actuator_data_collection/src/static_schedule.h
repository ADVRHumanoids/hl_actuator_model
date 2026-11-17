#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace actuator_collection {

// 静态带载检查的位置序列：从 q0 出发，依次平滑移动到 q0 + offsets[i]，每个位置保持 hold_time，
// 最后回到 q0（return_to_start = false 时停在最后一个位置，用于"移动到指定角度"）。纯数学实现，离线测试和实时插件共用。
// 移动用五次多项式（最小加加速度），起止速度、加速度都为 0；时长按峰值速度 move_velocity 计算，
// 不短于 min_move_time。保持段的前 settle_time 秒标为 Settle（等振荡衰减），之后标为 Hold。
struct StaticSchedule
{
    std::vector<double> offsets;   // 相对 q0 的目标偏移，输出端 rad，按顺序访问。
    double initial_value = 0.0;    // q0：插件启动时的位置参考。
    double hold_time = 10.0;       // 每个位置的保持时长 s（含 settle_time）。
    double settle_time = 2.0;      // 保持段开头不计入统计的时长 s。
    double move_velocity = 0.3;    // 移动段的峰值速度 rad/s。
    double min_move_time = 1.0;    // 移动段最短时长 s，小距离也不会太快。
    bool return_to_start = true;   // 最后是否回到 q0。

    enum Phase { Move = 0, Settle = 1, Hold = 2 };

    struct Leg { double start, move_time, hold, from, to; int point; };
    struct Sample { double q, v; int phase, point; };

    std::vector<Leg> legs;
    double duration = 0;

    void validate() const
    {
        if(offsets.empty()) throw std::invalid_argument("static schedule: offsets is empty");
        for(double x : offsets)
            if(!std::isfinite(x)) throw std::invalid_argument("static schedule: nonfinite offset");
        const double values[] = {initial_value, hold_time, settle_time, move_velocity, min_move_time};
        for(double v : values)
            if(!std::isfinite(v)) throw std::invalid_argument("static schedule: nonfinite parameter");
        if(hold_time <= settle_time || settle_time < 0 || move_velocity <= 0 || min_move_time <= 0)
            throw std::invalid_argument("static schedule: need hold_time > settle_time >= 0, move_velocity > 0, min_move_time > 0");
    }

    // 生成全部移动/保持段；最后一段回到 q0、不保持，point = -1。插件在 on_start 调用，实时循环只查表。
    void build()
    {
        validate();
        legs.clear();
        double t = 0, q = initial_value;
        auto add = [&](double target, double hold, int point) {
            const double d = std::abs(target - q);
            // 五次多项式的峰值速度 = 1.875 * 距离 / 时长
            const double T = d > 0 ? std::max(min_move_time, 1.875*d/move_velocity) : 0.0;
            legs.push_back({t, T, hold, q, target, point});
            t += T + hold;
            q = target;
        };
        for(std::size_t i = 0; i < offsets.size(); ++i)
            add(initial_value + offsets[i], hold_time, int(i));
        if(return_to_start)
            add(initial_value, 0.0, -1);
        duration = t;
    }

    // 时间 t 处的参考位置、速度、阶段和位置序号。cursor 由调用方保存，t 单调增加时只向前走，不分配内存。
    Sample sample(double t, std::size_t& cursor) const
    {
        while(cursor + 1 < legs.size() && t >= legs[cursor + 1].start) ++cursor;
        const Leg& leg = legs[cursor];
        const double tau = t - leg.start;
        if(tau < leg.move_time)
        {
            const double s = std::clamp(tau/leg.move_time, 0.0, 1.0);
            const double p = s*s*s*(10 + s*(-15 + 6*s));
            const double dp = 30*s*s*(1 - s)*(1 - s)/leg.move_time;
            return {leg.from + (leg.to - leg.from)*p, (leg.to - leg.from)*dp, Move, leg.point};
        }
        return {leg.to, 0.0, tau - leg.move_time < settle_time ? Settle : Hold, leg.point};
    }

    // 所有目标位置的最小、最大值（含 q0），用于启动前检查位置限。
    std::pair<double, double> range() const
    {
        double lo = initial_value, hi = initial_value;
        for(double x : offsets) { lo = std::min(lo, initial_value + x); hi = std::max(hi, initial_value + x); }
        return {lo, hi};
    }
};

} // namespace actuator_collection
