#include "dsp_resampler.h"
#include <cmath>
#include <numbers>

namespace sstv::dsp {

Resampler::Resampler(double input_rate, double target_rate, int num_phases)
    : m_input_rate(input_rate),
      m_target_rate(target_rate),
      m_num_phases(num_phases),
      m_output_index_frac(0.0)
{
    m_ratio = input_rate / target_rate;

    // 质量因子：降采样时需要更长的滤波器来保证陡峭的过渡带
    // 12-24 之间权衡性能与质量。SSTV 建议 16 以上。
    m_taps_per_phase = 16;

    design_filter();
    reset();
}

void Resampler::design_filter() {
    int total_taps = m_num_phases * m_taps_per_phase;
    std::vector<double> proto_filter(total_taps);

    // 关键：抗混叠截止频率
    // 如果是降采样 (input > target)，cutoff 必须是 target_rate/2
    double Fs_internal = m_input_rate * m_num_phases;
    double cutoff = std::min(m_input_rate, m_target_rate) * 0.45;
    double omega_c = 2.0 * std::numbers::pi * cutoff / Fs_internal;

    double center = (total_taps - 1) / 2.0;
    double sum_all = 0.0;

    for (int i = 0; i < total_taps; ++i) {
        double n = i - center;
        // Sinc
        if (std::abs(n) < 1e-9) {
            proto_filter[i] = omega_c / std::numbers::pi;
        } else {
            proto_filter[i] = std::sin(omega_c * n) / (std::numbers::pi * n);
        }
        // Blackman 窗 (比 Hamming 更好的阻带抑制，适合音频采样率转换)
        double a0 = 0.42, a1 = 0.5, a2 = 0.08;
        proto_filter[i] *= (a0 - a1 * std::cos(2.0 * std::numbers::pi * i / (total_taps - 1))
                               + a2 * std::cos(4.0 * std::numbers::pi * i / (total_taps - 1)));
        sum_all += proto_filter[i];
    }

    // 分解到相位库
    m_filter_bank.assign(m_num_phases, std::vector<float>(m_taps_per_phase));
    for (int p = 0; p < m_num_phases; ++p) {
        float phase_sum = 0.0f;
        for (int t = 0; t < m_taps_per_phase; ++t) {
            // 采样原型滤波器
            m_filter_bank[p][t] = static_cast<float>(proto_filter[p + t * m_num_phases]);
            phase_sum += m_filter_bank[p][t];
        }
        // 归一化每一相，保证增益为 1
        for (int t = 0; t < m_taps_per_phase; ++t) {
            m_filter_bank[p][t] /= phase_sum;
        }
    }
}

void Resampler::reset() {
    // 预填充历史缓冲区（初始全 0）
    m_history.assign(m_taps_per_phase, 0.0f);
    m_output_index_frac = 0.0;
}

std::vector<float> Resampler::process_block(const float* input, size_t count) {
    // 1. 将输入合并到历史数据中
    std::vector<float> work_buffer;
    work_buffer.reserve(m_history.size() + count);
    work_buffer.insert(work_buffer.end(), m_history.begin(), m_history.end());
    work_buffer.insert(work_buffer.end(), input, input + count);

    std::vector<float> output;
    output.reserve(static_cast<size_t>(count / m_ratio) + 1);

    // 2. 遍历可以产生的输出样本
    // 我们需要 m_taps_per_phase 个样本来计算一个卷积
    size_t total_samples = work_buffer.size();

    while (true) {
        size_t base_idx = static_cast<size_t>(m_output_index_frac);
        if (base_idx + m_taps_per_phase > total_samples) break;

        // 计算相位索引 (0 到 m_num_phases-1)
        double fract = m_output_index_frac - std::floor(m_output_index_frac);
        int phase_idx = static_cast<int>(fract * m_num_phases);
        phase_idx = std::clamp(phase_idx, 0, m_num_phases - 1);

        // 执行卷积
        float sum = 0.0f;
        const auto& coeffs = m_filter_bank[phase_idx];
        for (int t = 0; t < m_taps_per_phase; ++t) {
            sum += work_buffer[base_idx + t] * coeffs[t];
        }
        output.push_back(sum);

        // 步进：移动 ratio (输入采样点的跨度)
        m_output_index_frac += m_ratio;
    }

    // 3. 更新历史记录
    // 减去已经完全跨过的整数部分，保留末尾不足一个窗口的数据
    size_t processed_int = static_cast<size_t>(m_output_index_frac);
    m_output_index_frac -= processed_int;

    if (processed_int < total_samples) {
        m_history.assign(work_buffer.begin() + processed_int, work_buffer.end());
    } else {
        m_history.assign(m_taps_per_phase, 0.0f); // 防御性处理
    }

    return output;
}

} // namespace sstv::dsp
