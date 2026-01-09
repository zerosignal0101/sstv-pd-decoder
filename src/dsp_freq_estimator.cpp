#include "dsp_freq_estimator.h"
#include <algorithm> // For std::clamp

namespace sstv::dsp {

FrequencyEstimator::FrequencyEstimator(double sample_rate)
    : m_sample_rate(sample_rate),
      m_last_freq(0.0),
      m_last_sample(0.0f),
      m_zero_cross_count(0),
      m_samples_since_last_zero_cross(0.0),
      m_last_sign_positive(false)
{
    clear();
}

void FrequencyEstimator::clear() {
    m_last_freq = 0.0;
    m_last_sample = 0.0f;
    m_zero_cross_count = 0;
    m_samples_since_last_zero_cross = 0.0;
    m_last_sign_positive = false; // Initial state
}

double FrequencyEstimator::process_sample(float input_sample) {
    m_samples_since_last_zero_cross += 1.0;

    // Check for zero crossing
    bool current_sign_positive = (input_sample >= 0.0f);
    if (current_sign_positive != m_last_sign_positive && std::abs(input_sample) > 0.001f) { // Avoid noise at zero
        // A zero crossing occurred
        if (m_samples_since_last_zero_cross > 2.0) { // Filter out very high frequency noise (e.g., if samples_since_last_zero_cross is 1)
            // Estimate period from two zero crossings (one full cycle)
            // Or from one zero crossing (half cycle), then double it
            if (m_zero_cross_count == 0) {
                // First zero crossing in a potential cycle
                m_zero_cross_count = 1;
            } else {
                // Second zero crossing in a cycle
                double period_samples = m_samples_since_last_zero_cross * 2.0; // Two half-cycles for a full period
                if (period_samples > 0.0) {
                    m_last_freq = m_sample_rate / period_samples;
                }
                m_zero_cross_count = 0; // Reset for next cycle
            }
        }
        m_samples_since_last_zero_cross = 0.0;
    }

    m_last_sample = input_sample;
    m_last_sign_positive = current_sign_positive;
    return m_last_freq;
}

std::vector<double> FrequencyEstimator::process_block(const float* input_samples, size_t count) {
    std::vector<double> frequencies(count);
    for (size_t i = 0; i < count; ++i) {
        frequencies[i] = process_sample(input_samples[i]);
    }
    return frequencies;
}

uint8_t freq_to_pixel_value(double frequency) {
    if (frequency < BLACK_FREQ) return 0;
    if (frequency > WHITE_FREQ) return 255;

    // Linear mapping between BLACK_FREQ and WHITE_FREQ to 0-255
    double normalized = (frequency - BLACK_FREQ) / FREQ_RANGE;
    return static_cast<uint8_t>(std::clamp(normalized * 255.0, 0.0, 255.0));
}

} // namespace sstv::dsp
