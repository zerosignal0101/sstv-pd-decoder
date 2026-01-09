#pragma once

#include "sstv_types.h"
#include <vector>
#include <cmath>
#include <numeric>

namespace sstv::dsp {

// Simplified Frequency Estimator (zero-crossing based, for demo)
// A more robust implementation would use Hilbert transform or FFT.
class FrequencyEstimator {
public:
    explicit FrequencyEstimator(double sample_rate);

    // Process a block of samples and return estimated frequencies
    std::vector<double> process_block(const float* input_samples, size_t count);

    // Process a single sample
    double process_sample(float input_sample);

    // Get the last estimated frequency
    [[nodiscard]] double get_last_frequency() const { return m_last_freq; }

    void clear();

private:
    double m_sample_rate;
    double m_last_freq;
    float m_last_sample;
    int m_zero_cross_count;
    double m_samples_since_last_zero_cross;
    bool m_last_sign_positive; // true if last_sample > 0
};

// Converts frequency to pixel value (0-255)
uint8_t freq_to_pixel_value(double frequency);

} // namespace sstv::dsp
