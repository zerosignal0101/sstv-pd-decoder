#include "dsp_resampler.h"
#include <stdexcept>
#include <string>

namespace sstv::dsp {

    Resampler::Resampler(double input_rate, double target_rate, int quality) {
        m_ratio = target_rate / input_rate; // 注意：libsamplerate 的 ratio 是 out_rate / in_rate

        int channels = 1; // SSTV 是单声道
        m_src_state = src_new(quality, channels, &m_error);

        if (!m_src_state) {
            throw std::runtime_error("SRC Error: " + std::string(src_strerror(m_error)));
        }
    }

    Resampler::~Resampler() {
        if (m_src_state) {
            src_delete(m_src_state);
        }
    }

    void Resampler::reset() {
        src_reset(m_src_state);
    }

    std::vector<float> Resampler::process_block(const float* input, size_t count) {
        if (count == 0) return {};

        // 1. 预估输出大小（考虑到 ratio 和内部缓存，稍微多分配一点空间）
        size_t expected_output_count = static_cast<size_t>(count * m_ratio) + 128;
        std::vector<float> output(expected_output_count);

        // 2. 填充数据结构
        SRC_DATA data;
        data.data_in = input;
        data.data_out = output.data();
        data.input_frames = static_cast<long>(count);
        data.output_frames = static_cast<long>(expected_output_count);
        data.src_ratio = m_ratio;
        data.end_of_input = 0; // 流处理模式，设为 0

        // 3. 执行转换
        m_error = src_process(m_src_state, &data);
        if (m_error != 0) {
            throw std::runtime_error("SRC Process Error: " + std::string(src_strerror(m_error)));
        }

        // 4. 根据实际产生的样本数调整 vector 大小
        // data.output_frames_gen 是库实际写入 data_out 的样本数
        output.resize(data.output_frames_gen);

        return output;
    }

} // namespace sstv::dsp
