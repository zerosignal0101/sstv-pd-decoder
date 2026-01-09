#include "sstv_vis_decoder.h"
#include <iostream>
#include <algorithm> // For std::clamp

namespace sstv {

// Frequency comparison tolerance
constexpr double FREQ_TOLERANCE_HZ = 50.0; // +/- 50 Hz

VISDecoder::VISDecoder(double sample_rate, ModeDetectedCallback on_mode_detected_cb)
    : m_state(State::IDLE),
      m_sample_rate(sample_rate),
      m_samples_per_ms(sample_rate / 1000.0),
      m_on_mode_detected(on_mode_detected_cb)
{
    reset();
}

void VISDecoder::reset() {
    m_state = State::IDLE;
    m_current_tone_samples = 0.0;
    m_expected_duration_samples = 0.0;
    m_expected_freq = 0.0;
    m_preamble_step = 0;
    m_decoded_vis_bits = 0;
    m_bit_count = 0;
    m_parity_sum = 0;
}

bool VISDecoder::check_duration_and_freq(double freq, double expected_freq, double duration_ms, double tolerance_ms) {
    // Check frequency
    if (std::abs(freq - expected_freq) > FREQ_TOLERANCE_HZ) {
        m_current_tone_samples = 0.0; // Reset counter on frequency deviation
        return false;
    }

    // Accumulate duration
    m_current_tone_samples += 1.0;

    // Check if expected duration is met (within tolerance)
    double expected_samples = duration_ms * m_samples_per_ms;
    double tolerance_samples = tolerance_ms * m_samples_per_ms;

    if (m_current_tone_samples >= (expected_samples - tolerance_samples)) {
        // Duration reached (or exceeded)
        return true;
    }
    return false;
}

void VISDecoder::transition_to(State new_state) {
    m_state = new_state;
    m_current_tone_samples = 0.0; // Reset for new state
    // std::cout << "VIS State: " << static_cast<int>(new_state) << std::endl; // For debug
}

bool VISDecoder::process_frequencies(const std::vector<double>& frequencies) {
    for (double freq : frequencies) {
        if (freq == 0.0) { // No frequency detected (silence/noise)
            reset();
            return false;
        }

        switch (m_state) {
            case State::IDLE: {
                if (check_duration_and_freq(freq, DEFAULT_PREAMBLE_TONES[0].frequency, DEFAULT_PREAMBLE_TONES[0].duration_ms)) {
                    transition_to(State::PREAMBLE_1900_1);
                }
                break;
            }
            case State::PREAMBLE_1900_1: {
                if (m_current_tone_samples >= DEFAULT_PREAMBLE_TONES[0].duration_ms * m_samples_per_ms) {
                    m_preamble_step = 1;
                    transition_to(State::PREAMBLE_OTHER);
                } else if (std::abs(freq - DEFAULT_PREAMBLE_TONES[0].frequency) > FREQ_TOLERANCE_HZ) {
                    reset(); // Preamble broken
                } else {
                    m_current_tone_samples += 1.0;
                }
                break;
            }
            case State::PREAMBLE_OTHER: {
                if (m_preamble_step >= DEFAULT_PREAMBLE_TONES.size()) {
                    // All preamble tones processed, move to Leader Burst
                    transition_to(State::LEADER_BURST_1);
                } else {
                    const PreambleTone& pt = DEFAULT_PREAMBLE_TONES[m_preamble_step];
                    if (check_duration_and_freq(freq, pt.frequency, pt.duration_ms)) {
                        m_preamble_step++;
                        m_current_tone_samples = 0.0; // Reset for next tone
                        if (m_preamble_step >= DEFAULT_PREAMBLE_TONES.size()) {
                            transition_to(State::LEADER_BURST_1);
                        }
                    } else if (std::abs(freq - pt.frequency) > FREQ_TOLERANCE_HZ) {
                        reset(); // Preamble broken
                    } else {
                        m_current_tone_samples += 1.0;
                    }
                }
                break;
            }
            case State::LEADER_BURST_1: {
                if (check_duration_and_freq(freq, VIS_LEADER_BURST_FREQ, VIS_LEADER_BURST_DURATION_MS)) {
                    transition_to(State::BREAK_1200);
                } else if (std::abs(freq - VIS_LEADER_BURST_FREQ) > FREQ_TOLERANCE_HZ) {
                    reset();
                } else {
                    m_current_tone_samples += 1.0;
                }
                break;
            }
            case State::BREAK_1200: {
                if (check_duration_and_freq(freq, VIS_BREAK_FREQ, VIS_BREAK_DURATION_MS)) {
                    transition_to(State::LEADER_BURST_2);
                } else if (std::abs(freq - VIS_BREAK_FREQ) > FREQ_TOLERANCE_HZ) {
                    reset();
                } else {
                    m_current_tone_samples += 1.0;
                }
                break;
            }
            case State::LEADER_BURST_2: {
                if (check_duration_and_freq(freq, VIS_LEADER_BURST_FREQ, VIS_LEADER_BURST_DURATION_MS)) {
                    transition_to(State::START_BIT);
                } else if (std::abs(freq - VIS_LEADER_BURST_FREQ) > FREQ_TOLERANCE_HZ) {
                    reset();
                } else {
                    m_current_tone_samples += 1.0;
                }
                break;
            }
            case State::START_BIT: {
                if (check_duration_and_freq(freq, VIS_START_STOP_FREQ, VIS_BIT_DURATION_MS)) {
                    m_decoded_vis_bits = 0; // Reset for data bits
                    m_bit_count = 0;
                    m_parity_sum = 0;
                    transition_to(State::DATA_BITS);
                } else if (std::abs(freq - VIS_START_STOP_FREQ) > FREQ_TOLERANCE_HZ) {
                    reset();
                } else {
                    m_current_tone_samples += 1.0;
                }
                break;
            }
            case State::DATA_BITS: {
                if (m_bit_count < 7) {
                    double current_bit_freq = freq;
                    double bit_duration_samples = VIS_BIT_DURATION_MS * m_samples_per_ms;

                    // Accumulate samples for the current bit
                    m_current_tone_samples += 1.0;

                    if (m_current_tone_samples >= bit_duration_samples) {
                        int bit_value = -1;
                        if (std::abs(current_bit_freq - VIS_LOGIC_1_FREQ) < FREQ_TOLERANCE_HZ) {
                            bit_value = 1;
                        } else if (std::abs(current_bit_freq - VIS_LOGIC_0_FREQ) < FREQ_TOLERANCE_HZ) {
                            bit_value = 0;
                        }

                        if (bit_value != -1) {
                            m_decoded_vis_bits |= (bit_value << m_bit_count); // LSB first
                            m_parity_sum += bit_value;
                            m_bit_count++;
                            m_current_tone_samples = 0.0; // Reset for next bit
                        } else {
                            reset(); // Invalid frequency for data bit
                        }
                    }
                } else {
                    transition_to(State::PARITY_BIT);
                }
                break;
            }
            case State::PARITY_BIT: {
                double current_parity_freq = freq;
                double bit_duration_samples = VIS_BIT_DURATION_MS * m_samples_per_ms;

                m_current_tone_samples += 1.0;

                if (m_current_tone_samples >= bit_duration_samples) {
                    int parity_bit_value = -1;
                    if (std::abs(current_parity_freq - VIS_LOGIC_1_FREQ) < FREQ_TOLERANCE_HZ) {
                        parity_bit_value = 1;
                    } else if (std::abs(current_parity_freq - VIS_LOGIC_0_FREQ) < FREQ_TOLERANCE_HZ) {
                        parity_bit_value = 0;
                    }

                    if (parity_bit_value != -1) {
                        bool expected_parity_is_even = (m_parity_sum % 2 == 0);
                        if (parity_bit_value == (expected_parity_is_even ? 0 : 1)) {
                            // Parity check passed for SSTV (even parity)
                            transition_to(State::STOP_BIT);
                        } else {
                            // Parity check failed
                            std::cerr << "VIS: Parity check failed. Expected " << (expected_parity_is_even ? 0 : 1) << ", Got " << parity_bit_value << std::endl;
                            reset();
                        }
                    } else {
                        reset(); // Invalid frequency for parity bit
                    }
                }
                break;
            }
            case State::STOP_BIT: {
                if (check_duration_and_freq(freq, VIS_START_STOP_FREQ, VIS_BIT_DURATION_MS)) {
                    std::cout << "VIS Decoded: " << m_decoded_vis_bits << " (0x" << std::hex << m_decoded_vis_bits << std::dec << ")" << std::endl;
                    
                    auto it = VIS_MODE_MAP.find(m_decoded_vis_bits);
                    if (it != VIS_MODE_MAP.end()) {
                        m_on_mode_detected(it->second);
                        transition_to(State::COMPLETE);
                        return true; // VIS decoding complete
                    } else {
                        std::cerr << "VIS: Unknown mode for code " << m_decoded_vis_bits << std::endl;
                        reset();
                    }
                } else if (std::abs(freq - VIS_START_STOP_FREQ) > FREQ_TOLERANCE_HZ) {
                    reset();
                } else {
                    m_current_tone_samples += 1.0;
                }
                break;
            }
            case State::COMPLETE:
                // Stay in complete state until reset
                break;
        }
    }
    return (m_state == State::COMPLETE);
}

} // namespace sstv
