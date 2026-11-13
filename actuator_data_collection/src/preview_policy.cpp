#include "policy_targets.h"
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// 离线工具：输出策略式目标的 CSV，不连接电机，也不读取 YAML。
// 参数依次为：幅值 A [rad]、种子、最大每帧变化 max_step [rad]、策略频率 [Hz]。
// 其余参数用 PolicyTargets 的默认值（T=60 s、R=5 s、q0=0），要和 YAML 保持一致需手动修改。
int main(int argc, char** argv)
{
    actuator_collection::PolicyTargets gen;
    gen.amplitude = argc > 1 ? std::stod(argv[1]) : gen.amplitude;
    gen.seed = argc > 2 ? std::stoull(argv[2]) : gen.seed;
    gen.max_step = argc > 3 ? std::stod(argv[3]) : gen.max_step;
    gen.policy_rate_hz = argc > 4 ? std::stod(argv[4]) : gen.policy_rate_hz;

    std::vector<double> q;
    std::vector<int> segment;
    gen.generate(q, segment);

    // 与插件相同：1 kHz 循环里按时间查表，目标在两次策略更新之间保持不变。
    const double dt = 0.001;
    const int n = int(std::lround(gen.duration/dt));
    std::cerr << "policy targets: " << q.size() << " updates at " << gen.policy_rate_hz
              << " Hz, |q - q0| <= " << gen.amplitude << " rad, max change per update "
              << gen.max_step << " rad\n";
    std::cout << "time,q_ref,segment\n" << std::setprecision(15);
    for(int i = 0; i <= n; ++i)
    {
        const int k = gen.index(i*dt);
        std::cout << i*dt << ',' << q[k] << ',' << segment[k] << '\n';
    }
}
