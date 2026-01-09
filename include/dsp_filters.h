#pragma once

#include "sstv_types.h"
#include <vector>
#include <cmath>
#include <numeric>

namespace sstv::dsp {

// Function to generate FIR filter coefficients (Hamming windowed sinc)
// Adapted from the original `MakeFilter` but simplified for common use cases.
FilterCoefficients make_fir_coeffs(
    int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high,
    double attenuation_db = 60.0, double gain = 1.0);

class FIRFilter {
public:
    FIRFilter(int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high);
    
    // Clear the internal delay line (filter state)
    void clear();

    // Process a single sample
    float process_sample(float input_sample);

    // Process a block of samples
    void process_block(const float* input_samples, float* output_samples, size_t count);

private:
    FilterCoefficients m_coeffs;
    FilterDelayLine m_delay_line;
    size_t m_tap_count;
    size_t m_current_pos; // Write pointer for the circular delay line
};

} // namespace sstv::dsp
