#pragma once

#include "sstv_types.h"
#include "dsp_agc.h"
#include <vector>
#include <cmath>
#include <numeric>
#include <memory>
#include <iostream>
#include <algorithm>

namespace sstv::dsp {

    class FrequencyEstimator {
    public:
        explicit FrequencyEstimator(double sample_rate);

        std::vector<double> process_block(const float* input_samples, size_t count);
        double process_sample(float input_sample);

        [[nodiscard]] double get_last_frequency() const { return m_last_freq; }
        void clear();

    private:
        void generate_hilbert_coeffs();

        double m_sample_rate;
        double m_last_freq;

        // AGC
        std::unique_ptr<AGC> m_agc;

        // 滤波器状态
        std::vector<float> m_buffer;
        std::vector<float> m_coeffs;
        size_t m_buffer_size;
        size_t m_write_pos;
        size_t m_group_delay;

        // --- 核心状态：用于差分鉴频 ---
        float m_prev_i; // 上一时刻的同相分量
        float m_prev_q; // 上一时刻的正交分量

        size_t m_samples_processed;
    };

    uint8_t freq_to_pixel_value(double frequency);

} // namespace sstv::dsp
