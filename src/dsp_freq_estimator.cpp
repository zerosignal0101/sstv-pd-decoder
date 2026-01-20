#include "dsp_freq_estimator.h"
#include <numbers>
#include <cmath>
#include <algorithm>

namespace sstv::dsp {

// 增加抽头数可以提高希尔伯特变换的精度，但会增加延迟
// 对于11025Hz采样率，63或127是比较平衡的选择
constexpr int DEFAULT_HILBERT_TAPS = 63;

FrequencyEstimator::FrequencyEstimator(double sample_rate)
    : m_sample_rate(sample_rate),
      m_last_freq(0.0),
      m_write_pos(0),
      m_prev_i(0.0f),
      m_prev_q(0.0f),
      m_samples_processed(0)
{
    m_agc = std::make_unique<AGC>();

    m_buffer_size = DEFAULT_HILBERT_TAPS;
    m_group_delay = m_buffer_size / 2;

    generate_hilbert_coeffs();
    m_buffer.resize(m_buffer_size, 0.0f);
}

void FrequencyEstimator::clear() {
    std::fill(m_buffer.begin(), m_buffer.end(), 0.0f);
    m_write_pos = 0;
    m_last_freq = 0.0;
    m_prev_i = 0.0f;
    m_prev_q = 0.0f;
    m_samples_processed = 0;
}

void FrequencyEstimator::generate_hilbert_coeffs() {
    m_coeffs.resize(m_buffer_size);
    int M = m_buffer_size - 1;
    for (int i = 0; i < m_buffer_size; ++i) {
        int n = i - M / 2;
        if (n == 0) {
            m_coeffs[i] = 0.0f;
        } else if (n % 2 == 0) {
            m_coeffs[i] = 0.0f; // 理想希尔伯特偶数项为0
        } else {
            // 理想脉冲响应: 2 / (pi * n)
            double val = 2.0 / (std::numbers::pi * n);
            // 使用 Blackman 窗减少吉布斯效应
            double window = 0.42 - 0.5 * std::cos(2.0 * std::numbers::pi * i / M) +
                            0.08 * std::cos(4.0 * std::numbers::pi * i / M);
            m_coeffs[i] = static_cast<float>(val * window);
        }
    }
}

// IIR DC blocker
float dc_blocker(float input) {
    static float prev_input = 0.0f;
    static float prev_output = 0.0f;
    const float alpha = 0.995f; // 截止频率越小，alpha 越接近 1

    float output = input - prev_input + alpha * prev_output;
    prev_input = input;
    prev_output = output;
    return output;
}

double FrequencyEstimator::process_sample(float input_sample) {
    // DC Blocker (必须在 AGC 之前，否则 DC 会被放大)
    float sample_no_dc = dc_blocker(input_sample);

    // AGC (确保信号进入希尔伯特变换时幅度适中)
    float sample_normalized = m_agc->process(sample_no_dc);

    // 1. 更新循环缓冲区
    m_buffer[m_write_pos] = sample_normalized;

    // 2. 卷积计算 Q 路 (正交分量)
    float q = 0.0f;
    for (size_t i = 0; i < m_buffer_size; ++i) {
        // 访问缓冲区：从最新样本向后回溯
        size_t tap_idx = (m_write_pos + m_buffer_size - i) % m_buffer_size;
        q += m_buffer[tap_idx] * m_coeffs[i];
    }

    // 3. 获取 I 路 (通过组延迟对齐同相分量)
    size_t read_idx = (m_write_pos + m_buffer_size - m_group_delay) % m_buffer_size;
    float i_val = m_buffer[read_idx];

    // 更新指针
    m_write_pos = (m_write_pos + 1) % m_buffer_size;
    m_samples_processed++;

    // 4. 启动过渡期检查 (确保滤波器填满)
    if (m_samples_processed <= m_buffer_size) {
        m_prev_i = i_val;
        m_prev_q = q;
        return 0.0;
    }

    // 5. 噪声门限检查 (防止在静音期间产生随机频率)
    float mag_sq = i_val * i_val + q * q;
    if (mag_sq < 1e-7f) {
        m_prev_i = i_val;
        m_prev_q = q;
        return m_last_freq; // 或者返回 0.0
    }

    // 6. 复数差分鉴频核心算法
    // 设当前复数 Z(n) = I + jQ, 前一时刻复数 Z(n-1) = Ip + jQp
    // 相位差 delta_phi = angle( Z(n) * conj(Z(n-1)) )
    // Z(n) * conj(Z(n-1)) = (I + jQ) * (Ip - jQp)
    //                     = (I*Ip + Q*Qp) + j(Q*Ip - I*Qp)
    // 实部 Re (点积): I*Ip + Q*Qp
    // 虚部 Im (叉积): Q*Ip - I*Qp

    double dot   = static_cast<double>(i_val) * m_prev_i + static_cast<double>(q) * m_prev_q;
    double cross = static_cast<double>(q) * m_prev_i - static_cast<double>(i_val) * m_prev_q;

    // 直接得到相邻样本间的相位变化量，无需进行传统的 Unwrap 操作
    double delta_phase = std::atan2(cross, dot);

    // 7. 更新状态
    m_prev_i = i_val;
    m_prev_q = q;

    // 8. 转换为频率 (Hz)
    // f = (delta_phase * Fs) / (2 * PI)
    double raw_freq = (delta_phase * m_sample_rate) / (2.0 * std::numbers::pi);
    m_last_freq = raw_freq;

    return m_last_freq;
}

std::vector<double> FrequencyEstimator::process_block(const float* input_samples, size_t count) {
    std::vector<double> frequencies;
    frequencies.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        frequencies.push_back(process_sample(input_samples[i]));
    }
    return frequencies;
}

// 映射函数保持不变
uint8_t freq_to_pixel_value(double frequency) {
    if (frequency < BLACK_FREQ) return 0;
    if (frequency > WHITE_FREQ) return 255;
    double normalized = (frequency - BLACK_FREQ) / FREQ_RANGE;
    return static_cast<uint8_t>(std::clamp(normalized * 255.0, 0.0, 255.0));
}

} // namespace sstv::dsp
