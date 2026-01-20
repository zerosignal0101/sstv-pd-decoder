#include "sstv_vis_decoder.h"
#include <iostream>

namespace sstv {

VISDecoder::VISDecoder(double sample_rate, ModeDetectedCallback on_mode_detected_cb)
    : m_sample_rate(sample_rate),
      m_samples_per_ms(sample_rate / 1000.0),
      m_on_mode_detected(on_mode_detected_cb) {
    reset();
}

double VISDecoder::get_smoothed_freq(double raw_freq) {
    m_median_buffer.push_back(raw_freq);
    if (m_median_buffer.size() > MEDIAN_WINDOW) m_median_buffer.pop_front();

    std::vector<double> sorted = {m_median_buffer.begin(), m_median_buffer.end()};
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
}

void VISDecoder::reset() {
    m_state = State::IDLE;
    m_state_timer_samples = 0;
    m_preamble_step = 0;
    m_error_count = 0;
    m_decoded_vis_bits = 0;
    m_bit_count = 0;
    m_bit_freq_accumulator = 0;
    m_bit_sample_count = 0;
}

void VISDecoder::transition_to(State new_state) {
    m_state = new_state;
    m_state_timer_samples = 0;
    m_error_count = 0;
    m_bit_freq_accumulator = 0;
    m_bit_sample_count = 0;
}

bool VISDecoder::is_freq_near(double freq, double target, double tolerance) {
    return std::abs(freq - target) < tolerance;
}

bool VISDecoder::process_frequency(const double& raw_freq) {
    // 中值滤波预处理
    double freq = get_smoothed_freq(raw_freq);

    m_state_timer_samples += 1.0;

    // 鲁棒性检查：如果频率完全丢失（0），快速重置
    if (freq < 100.0) {
        reset();
        return false;
    }

    switch (m_state) {
        case State::IDLE:
            // 寻找第一个前导音 (1900Hz)
            if (is_freq_near(freq, DEFAULT_PREAMBLE_TONES[0].frequency)) {
                if (m_state_timer_samples >= ((DEFAULT_PREAMBLE_TONES[0].duration_ms - 5.0) * m_samples_per_ms)) {
                    m_preamble_step = 1; // 已经完成第0个音
                    transition_to(State::PREAMBLE);
                }
            } else {
                m_state_timer_samples = 0; // 频率不对，重置计时
            }
            break;

        case State::PREAMBLE: {
            const auto& target_tone = DEFAULT_PREAMBLE_TONES[m_preamble_step];
            if (is_freq_near(freq, target_tone.frequency)) {
                if (m_state_timer_samples >= (target_tone.duration_ms * m_samples_per_ms)) {
                    m_preamble_step++;
                    // 进入下一个前缀检测，误差归零，计数器归零
                    m_error_count = 0;
                    m_state_timer_samples = 0;
                    if (m_preamble_step >= DEFAULT_PREAMBLE_TONES.size()) {
                        transition_to(State::LEADER_BURST_1);
                    }
                }
            } else {
                // 允许短时间误差
                if (++m_error_count > (MAX_ERROR_TIME_MS * m_samples_per_ms)) {
                    reset();
                };
            }
            break;
        }

        case State::LEADER_BURST_1:
            if (is_freq_near(freq, VIS_LEADER_BURST_FREQ)) {
                if (m_state_timer_samples >= (VIS_LEADER_BURST_DURATION_MS * m_samples_per_ms)) {
                    transition_to(State::BREAK_1200);
                }
            } else {
                if (++m_error_count > (MAX_ERROR_TIME_MS * m_samples_per_ms)) reset();
            }
            break;

        case State::BREAK_1200:
            if (is_freq_near(freq, VIS_BREAK_FREQ)) {
                if (m_state_timer_samples >= (VIS_BREAK_DURATION_MS * m_samples_per_ms)) {
                    transition_to(State::LEADER_BURST_2);
                }
            } else {
                if (++m_error_count > (MAX_ERROR_TIME_MS * m_samples_per_ms)) reset();
            }
            break;

        case State::LEADER_BURST_2:
            if (is_freq_near(freq, VIS_LEADER_BURST_FREQ)) {
                if (m_state_timer_samples >= (VIS_LEADER_BURST_DURATION_MS * m_samples_per_ms)) {
                    // // Debug info
                    // std::cout << "Transition to START_BIT" << std::endl;
                    transition_to(State::START_BIT);
                }
            } else {
                if (++m_error_count > (MAX_ERROR_TIME_MS * m_samples_per_ms)) reset();
            }
            break;

        case State::START_BIT:
            if (is_freq_near(freq, VIS_START_STOP_FREQ)) {
                if (m_state_timer_samples >= (VIS_BIT_DURATION_MS * m_samples_per_ms)) {
                    transition_to(State::DATA_BITS);
                }
            } else {
                if (++m_error_count > (MAX_ERROR_TIME_MS * m_samples_per_ms)) reset();
            }
            break;

        case State::DATA_BITS:
            // 累加整个位周期内的频率
            m_bit_freq_accumulator += freq;
            m_bit_sample_count++;

            if (m_state_timer_samples >= (VIS_BIT_DURATION_MS * m_samples_per_ms)) {
                double avg_f = m_bit_freq_accumulator / m_bit_sample_count;

                // --- 核心修改：动态判定门限 ---
                // 逻辑 1 是 1100，逻辑 0 是 1300，中间点是 1200
                // 我们不看绝对频率，看它在 1200 的哪一边
                int bit = (avg_f < 1200.0) ? 1 : 0;

                m_decoded_vis_bits |= (bit << m_bit_count);
                m_bit_count++;

                // 重置位累加器
                m_state_timer_samples = 0;
                m_bit_freq_accumulator = 0;
                m_bit_sample_count = 0;

                if (m_bit_count >= 7) transition_to(State::PARITY_BIT);
            }
            break;

        case State::PARITY_BIT:
            m_bit_freq_accumulator += freq;
            m_bit_sample_count++;

            if (m_state_timer_samples >= (VIS_BIT_DURATION_MS * m_samples_per_ms)) {
                double avg_f = m_bit_freq_accumulator / m_bit_sample_count;
                int p_bit = is_freq_near(avg_f, VIS_LOGIC_1_FREQ, 80.0) ? 1 : 0;

                // 偶校验检查
                int ones = 0;
                for(int i=0; i<7; ++i) if((m_decoded_vis_bits >> i) & 1) ones++;
                bool parity_ok = ((ones + p_bit) % 2 == 0);

                if (parity_ok) {
                    // // Debug info
                    // std::cout << "Parity OK: " << ones << std::endl;
                    transition_to(State::STOP_BIT);
                }
                else {
                    std::cerr << "VIS: Parity Error" << std::endl;
                    reset();
                }
            }
            break;

        case State::STOP_BIT:
            if (is_freq_near(freq, VIS_START_STOP_FREQ)) {
                if (m_state_timer_samples >= (VIS_BIT_DURATION_MS * m_samples_per_ms)) {
                    auto it = GLOBAL_VIS_MAP.find(m_decoded_vis_bits);
                    if (it != GLOBAL_VIS_MAP.end()) {
                        m_on_mode_detected(it->second);
                        m_state = State::COMPLETE;
                        return true;
                    } else {
                        m_on_mode_detected(SSTVMode("Unknown", m_decoded_vis_bits, 0, 0, 0, SSTVFamily::UNKNOWN));
                        m_state = State::COMPLETE;
                        return true;
                    }
                }
            } else {
                if (++m_error_count > (MAX_ERROR_TIME_MS * m_samples_per_ms)) reset();
            }
            break;

        case State::COMPLETE:
            return true;
    }
    return (m_state == State::COMPLETE);
}

} // namespace sstv
