// include/dsp_filters.h
#pragma once

#include "sstv_types.h" // 包含 FilterCoefficients 和 FilterDelayLine
#include <vector>
#include <numbers>
#include <cmath>

namespace sstv::dsp {

// 函数：生成FIR滤波器系数（Hamming窗加Sinc函数法）
// 适用于生成带通滤波器系数 (LPF(fc_high) - LPF(fc_low))。
// tap_count: 滤波器抽头数量（即系数数量）
// sample_rate: 采样率
// cutoff_freq_low: 低截止频率
// cutoff_freq_high: 高截止频率
// attenuation_db: 衰减量（目前在此简化实现中未直接使用，但可用于更高级的滤波器设计或抽头数选择）
// gain: 最终系数的整体增益因子
FilterCoefficients make_fir_coeffs(
    int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high,
    double attenuation_db = 60.0, double gain = 1.0);

class FIRFilter {
public:
    // 构造函数：根据参数初始化滤波器
    FIRFilter(int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high);

    // 清空内部延迟线（滤波器状态），将其所有样本置零
    void clear();

    // 处理单个样本，返回滤波后的输出样本
    float process_sample(float input_sample);

    // 处理一个样本块
    // input_samples: 输入样本数组
    // output_samples: 输出样本数组 (与输入数组大小相同)
    // count: 样本数量
    void process_block(const float* input_samples, float* output_samples, size_t count);

private:
    FilterCoefficients m_coeffs;    // 滤波器系数
    FilterDelayLine m_delay_line;   // 延迟线（循环缓冲区），存储历史输入样本
    size_t m_tap_count;             // 滤波器抽头数量 (m_coeffs.size())
    size_t m_current_pos;           // 延迟线中的写入指针，指向最新样本的位置
};

class Biquad {
public:
    // 构造一个二阶 IIR 谐振器 (Bandpass)
    void setup_bandpass(double frequency, double bandwidth, double sample_rate) {
        double omega = 2.0 * std::numbers::pi * frequency / sample_rate;
        double bw_rad = 2.0 * std::numbers::pi * bandwidth / sample_rate;

        m_b2 = -std::exp(-bw_rad);
        m_b1 = 2.0 * std::exp(-bw_rad / 2.0) * std::cos(omega);
        // a0 归一化增益，使中心频率增益为 1
        m_a0 = 1.0 - std::sqrt(m_b1*m_b1 / (4.0 * -m_b2)) * (1.0 + m_b2);
        // 简化版 a0
        m_a0 = (1.0 + m_b2) * std::sin(omega) * 0.5;
        clear();
    }

    // 构造一个低通滤波器 (Butterworth 2nd order)
    void setup_lowpass(double cutoff, double sample_rate) {
        double ff = cutoff / sample_rate;
        double ita = 1.0 / std::tan(std::numbers::pi * ff);
        double q = std::sqrt(2.0);
        double den = 1.0 + q * ita + ita * ita;
        m_a0 = 1.0 / den;
        m_a1 = 2.0 / den;
        m_a2 = 1.0 / den;
        m_b1 = 2.0 * (ita * ita - 1.0) / den;
        m_b2 = -(1.0 - q * ita + ita * ita) / den;
        m_is_lowpass = true;
        clear();
    }

    float process(float in) {
        float out;
        if (m_is_lowpass) {
            out = m_a0 * in + m_a1 * m_z1 + m_a2 * m_z2 + m_b1 * m_y1 + m_b2 * m_y2;
        } else {
            // 谐振器形式: y[n] = a0*x[n] + b1*y[n-1] + b2*y[n-2]
            out = m_a0 * in + m_b1 * m_y1 + m_b2 * m_y2;
        }
        m_z2 = m_z1; m_z1 = in;
        m_y2 = m_y1; m_y1 = out;
        return out;
    }

    void clear() { m_z1 = m_z2 = m_y1 = m_y2 = 0; }

private:
    double m_a0 = 0, m_a1 = 0, m_a2 = 0, m_b1 = 0, m_b2 = 0;
    double m_z1 = 0, m_z2 = 0, m_y1 = 0, m_y2 = 0;
    bool m_is_lowpass = false;
};

} // namespace sstv::dsp
