#pragma once

#include <vector>
#include <cstddef>
#include <samplerate.h> // libsamplerate 头文件

namespace sstv::dsp {

    class Resampler {
    public:
        /**
         * @param input_rate 原始采样率 (48000)
         * @param target_rate 目标采样率 (11025)
         * @param quality 质量等级：
         *        SRC_SINC_BEST_QUALITY (最慢, 97dB SNR)
         *        SRC_SINC_MEDIUM_QUALITY (推荐)
         *        SRC_SINC_FASTEST (最快, 适用于性能受限环境)
         */
        Resampler(double input_rate, double target_rate, int quality = SRC_SINC_MEDIUM_QUALITY);
        ~Resampler();

        // 禁止拷贝，因为 SRC_STATE 是指针
        Resampler(const Resampler&) = delete;
        Resampler& operator=(const Resampler&) = delete;

        std::vector<float> process_block(const float* input, size_t count);
        void reset();

    private:
        SRC_STATE* m_src_state;
        double m_ratio;
        int m_error;
    };

} // namespace sstv::dsp
