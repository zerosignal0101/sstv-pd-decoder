#pragma once

#include "sstv_types.h"
#include <vector>
#include <memory>
#include <cmath>

namespace sstv {

    class VISDecoder {
    public:
        VISDecoder(double sample_rate, ModeDetectedCallback on_mode_detected_cb);

        // 处理频率序列，返回是否检测到完整 VIS
        bool process_frequencies(const std::vector<double>& frequencies);
        void reset();

    private:
        enum class State {
            IDLE,                   // 等待信号
            PREAMBLE,               // 8个前导音 (1900/1500/2300...)
            LEADER_BURST_1,         // 300ms 1900Hz
            BREAK_1200,             // 10ms 1200Hz
            LEADER_BURST_2,         // 300ms 1900Hz
            START_BIT,              // 30ms 1200Hz
            DATA_BITS,              // 7 bits
            PARITY_BIT,             // Even parity
            STOP_BIT,               // 30ms 1200Hz
            COMPLETE
        };

        State m_state;
        double m_sample_rate;
        double m_samples_per_ms;
        ModeDetectedCallback m_on_mode_detected;

        // 状态追踪
        double m_state_timer_samples;   // 当前状态已持续的采样数
        size_t m_preamble_step;         // 当前处于第几个前导音
        int    m_error_count;           // 连续频率错误计数，用于鲁棒性

        // 位解码辅助
        int    m_decoded_vis_bits;
        int    m_bit_count;
        double m_bit_freq_accumulator;  // 用于计算位周期内的平均频率
        int    m_bit_sample_count;

        // 内部逻辑
        void transition_to(State new_state);
        bool is_freq_near(double freq, double target, double tolerance = 60.0);
        void handle_bit_decoding(double avg_freq);

        // 常量：允许连续错误的时间（毫秒），超过此时间则重置
        const double MAX_ERROR_TIME_MS = 15.0;
    };

} // namespace sstv
