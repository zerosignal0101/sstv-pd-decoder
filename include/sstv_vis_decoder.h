#pragma once

#include "sstv_types.h"
#include "dsp_freq_estimator.h"
#include <vector>
#include <memory>

namespace sstv {

class VISDecoder {
public:
    VISDecoder(double sample_rate, ModeDetectedCallback on_mode_detected_cb);

    // Process a block of frequency estimates
    // Returns true if VIS decoding is complete and a mode is detected.
    bool process_frequencies(const std::vector<double>& frequencies);

    void reset();

private:
    enum class State {
        IDLE,                   // Waiting for any signal
        PREAMBLE_1900_1,        // Looking for the first 1900 Hz preamble tone
        PREAMBLE_OTHER,         // Processing other preamble tones
        LEADER_BURST_1,         // Detecting the first 300ms 1900Hz burst
        BREAK_1200,             // Detecting the 10ms 1200Hz break
        LEADER_BURST_2,         // Detecting the second 300ms 1900Hz burst
        START_BIT,              // Detecting the 30ms 1200Hz start bit
        DATA_BITS,              // Decoding 7 data bits
        PARITY_BIT,             // Decoding the parity bit
        STOP_BIT,               // Detecting the 30ms 1200Hz stop bit
        COMPLETE                // VIS decoding finished
    };
    State m_state;
    double m_sample_rate;
    double m_samples_per_ms;

    ModeDetectedCallback m_on_mode_detected;

    // State tracking variables
    double m_current_tone_samples;
    double m_expected_duration_samples;
    double m_expected_freq;

    // Preamble tracking
    size_t m_preamble_step;

    // VIS bit decoding
    int m_decoded_vis_bits; // Holds 7 data bits
    int m_bit_count;        // Count of bits decoded
    int m_parity_sum;       // For even parity check

    bool check_duration_and_freq(double freq, double threshold_freq_hz, double duration_ms, double tolerance_ms = 5.0);
    void transition_to(State new_state);
};

} // namespace sstv
