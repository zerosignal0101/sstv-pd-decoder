#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>

namespace sstv::dsp {

    class Resampler {
    public:
        /**
         * @param input_rate 原始输入采样率 (如 48000)
         * @param target_rate 目标采样率 (11025)
         * @param num_phases 相位密度，越高抗混叠越好，建议 32-64
         */
        Resampler(double input_rate, double target_rate, int num_phases = 64);

        /**
         * 处理音频块。支持流式处理，内部维护历史状态。
         */
        std::vector<float> process_block(const float* input, size_t count);

        void reset();

    private:
        double m_input_rate;
        double m_target_rate;
        double m_ratio;             // input_rate / target_rate

        int m_num_phases;
        int m_taps_per_phase;       // 每个相位的抽头数

        // 多相滤波器组 [phase][tap]
        std::vector<std::vector<float>> m_filter_bank;

        // 状态维护
        std::vector<float> m_history;
        double m_output_index_frac; // 指向输入流的浮点索引

        void design_filter();
    };

} // namespace sstv::dsp
