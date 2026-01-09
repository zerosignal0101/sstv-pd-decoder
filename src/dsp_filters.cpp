// src/dsp_filters.cpp
#include "dsp_filters.h"
#include <numbers>   // C++20: For std::numbers::pi
#include <algorithm> // For std::fill

namespace sstv::dsp {

// 辅助函数：计算 sinc(x) = sin(pi*x) / (pi*x)
static double sinc(double x) {
    // 使用一个小的阈值处理 x 接近 0 的情况，避免除以零，并返回极限值 1.0
    if (std::abs(x) < 1e-9) return 1.0;
    return std::sin(std::numbers::pi * x) / (std::numbers::pi * x);
}

FilterCoefficients make_fir_coeffs(
    int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high,
    double attenuation_db, double gain) // attenuation_db 参数目前未使用
{
    // 参数有效性检查
    if (tap_count <= 0) return {}; // 抽头数必须为正
    if (sample_rate <= 0) return {}; // 采样率必须为正
    if (cutoff_freq_low < 0 || cutoff_freq_high < 0) return {}; // 截止频率不能为负

    // 确保截止频率不超过奈奎斯特频率
    double nyquist_freq = sample_rate / 2.0;
    if (cutoff_freq_high > nyquist_freq) {
        cutoff_freq_high = nyquist_freq;
    }

    // 对于带通滤波器，低截止频率通常应小于高截止频率
    // 如果 cutoff_freq_low >= cutoff_freq_high，这可能表示一个无效的带通范围
    // 或一个零带宽的滤波器 (LPF(fc) - LPF(fc) = 0)，此时系数将全部为零。
    // 如果 cutoff_freq_low == 0 且 cutoff_freq_high > 0，则此设计等效于一个低通滤波器。
    // 我们允许这种情况发生，并生成相应的系数。

    FilterCoefficients coeffs(tap_count);

    // 计算归一化截止频率 (相对于采样率)
    // 注意：sinc 函数参数中的 2.0 * fc_norm 是为了使通带增益近似为 1
    double fc1_norm = cutoff_freq_low / sample_rate;
    double fc2_norm = cutoff_freq_high / sample_rate;

    // M 是滤波器阶数 (抽头数 - 1)
    double M = tap_count - 1;

    // 生成滤波器系数
    for (int i = 0; i < tap_count; ++i) {
        // 'n' 是关于滤波器中心对称的时间索引
        double n = static_cast<double>(i) - M / 2.0;

        // 带通滤波器系数通过 LPF(fc_high) - LPF(fc_low) 方式设计
        // 这个公式在理想情况下已提供了接近1的通带增益。
        coeffs[i] = 2.0 * fc2_norm * sinc(2.0 * fc2_norm * n) -
                    2.0 * fc1_norm * sinc(2.0 * fc1_norm * n);

        // 应用 Hamming 窗以减少旁瓣和吉布斯现象
        double window = 0.54 - 0.46 * std::cos(2.0 * std::numbers::pi * static_cast<double>(i) / M);
        coeffs[i] *= window;
    }

    // 应用最终的用户指定增益
    for (double& c : coeffs) {
        c *= gain;
    }

    return coeffs;
}


FIRFilter::FIRFilter(int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high)
    : m_current_pos(0) // 初始化写入指针为0
{
    // 调用 make_fir_coeffs 生成系数
    m_coeffs = make_fir_coeffs(tap_count, sample_rate, cutoff_freq_low, cutoff_freq_high);

    // 根据实际生成的系数数量更新 m_tap_count
    // 这样可以处理 make_fir_coeffs 在 tap_count <= 0 时返回空向量的情况
    m_tap_count = m_coeffs.size();

    // 根据实际抽头数调整延迟线大小并清零
    m_delay_line.resize(m_tap_count, 0.0f);
}

void FIRFilter::clear() {
    // 将延迟线中的所有样本置零，清除滤波器历史状态
    std::fill(m_delay_line.begin(), m_delay_line.end(), 0.0f);
    m_current_pos = 0; // 重置写入指针
}

float FIRFilter::process_sample(float input_sample) {
    // 如果没有抽头（例如，滤波器未成功初始化），则不进行滤波，直接返回0
    if (m_tap_count == 0) {
        return 0.0f; // 或者根据需求返回 input_sample 实现直通
    }

    // 将新输入的样本放入循环缓冲区中的当前写入位置
    m_delay_line[m_current_pos] = input_sample;

    float output_sample = 0.0f;

    // 执行卷积运算: y[n] = sum_{k=0 to N-1} (h[k] * x[n-k])
    // m_coeffs[k] 对应 h[k]
    // m_delay_line[(m_current_pos + m_tap_count - k) % m_tap_count] 对应 x[n-k]
    for (size_t k = 0; k < m_tap_count; ++k) {
        // 计算延迟线中对应 x[n-k] 的索引
        // (m_current_pos - k + m_tap_count) % m_tap_count 确保在循环缓冲区中正确访问历史样本
        // k=0 时访问最新样本 (m_delay_line[m_current_pos])，k=1 时访问倒数第二个样本，以此类推。
        size_t delay_idx = (m_current_pos + m_tap_count - k) % m_tap_count;
        output_sample += m_delay_line[delay_idx] * static_cast<float>(m_coeffs[k]);
    }

    // 移动写入指针到下一个位置（循环）
    m_current_pos = (m_current_pos + 1) % m_tap_count;

    return output_sample;
}

void FIRFilter::process_block(const float* input_samples, float* output_samples, size_t count) {
    // 循环处理样本块中的每个样本
    for (size_t i = 0; i < count; ++i) {
        output_samples[i] = process_sample(input_samples[i]);
    }
}

} // namespace sstv::dsp
