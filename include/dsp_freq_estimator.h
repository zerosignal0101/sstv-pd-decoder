#pragma once

#include "sstv_types.h"
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace sstv::dsp {

    /**
     * @brief Frequency Estimator based on Hilbert Transform (Quadrature Demodulator)
     *
     * This provides much higher accuracy than zero-crossing detection by calculating
     * the instantaneous phase of the signal and differentiating it.
     */
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
        // Generates a windowed sinc-based FIR filter for Hilbert transform
        void generate_hilbert_coeffs();

        double m_sample_rate;
        double m_last_freq;

        // Filter and Delay Line State
        std::vector<float> m_buffer; // Circular buffer for input samples
        std::vector<float> m_coeffs; // Hilbert transformer coefficients
        size_t m_buffer_size;        // Size of the filter (number of taps)
        size_t m_write_pos;          // Current write index in circular buffer
        size_t m_group_delay;        // Group delay of the filter in samples (buffer_size / 2)

        // Phase detection state
        double m_prev_phase;
        size_t m_samples_processed;  // To handle startup transient
    };

    // Converts frequency to pixel value (0-255)
    // Note: This helper is kept as it was in the original design.
    uint8_t freq_to_pixel_value(double frequency);

} // namespace sstv::dsp
