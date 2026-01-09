#include "dsp_filters.h"
#include <numbers>
#include <algorithm> // For std::reverse

namespace sstv::dsp {

// Helper for sinc function
static double sinc(double x) {
    if (x == 0.0) return 1.0;
    return std::sin(std::numbers::pi * x) / (std::numbers::pi * x);
}

FilterCoefficients make_fir_coeffs(
    int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high,
    double attenuation_db, double gain)
{
    if (tap_count <= 0) return {};
    
    FilterCoefficients coeffs(tap_count);
    
    // Calculate normalized cutoff frequencies
    double fc1_norm = cutoff_freq_low / sample_rate;
    double fc2_norm = cutoff_freq_high / sample_rate;

    // Calculate filter coefficients for a bandpass filter (sinc function)
    double M = static_cast<double>(tap_count - 1);
    for (int i = 0; i < tap_count; ++i) {
        double n = static_cast<double>(i) - M / 2.0;

        // Bandpass filter: LPF(fc2) - LPF(fc1) shifted to bandpass
        // This is a simplified approach, a more robust design would involve windowing directly.
        coeffs[i] = 2.0 * fc2_norm * sinc(2.0 * fc2_norm * n) - 
                    2.0 * fc1_norm * sinc(2.0 * fc1_norm * n);

        // Apply Hamming window
        double window = 0.54 - 0.46 * std::cos(2.0 * std::numbers::pi * static_cast<double>(i) / M);
        coeffs[i] *= window;
    }

    // Normalize gain
    double sum_coeffs = 0.0;
    for (double c : coeffs) {
        sum_coeffs += c;
    }
    if (sum_coeffs != 0.0) {
        for (double& c : coeffs) {
            c /= sum_coeffs;
        }
    }
    
    // Apply final gain
    for (double& c : coeffs) {
        c *= gain;
    }

    return coeffs;
}


FIRFilter::FIRFilter(int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high)
    : m_tap_count(tap_count), m_current_pos(0)
{
    m_coeffs = make_fir_coeffs(tap_count, sample_rate, cutoff_freq_low, cutoff_freq_high);
    m_delay_line.resize(tap_count, 0.0);
}

void FIRFilter::clear() {
    std::fill(m_delay_line.begin(), m_delay_line.end(), 0.0);
    m_current_pos = 0;
}

float FIRFilter::process_sample(float input_sample) {
    if (m_tap_count == 0) return input_sample;

    // Place new sample in the circular buffer
    m_delay_line[m_current_pos] = input_sample;

    float output_sample = 0.0f;
    size_t coeff_idx = 0;
    
    // Convolve with coefficients (from oldest to newest sample in delay line)
    // The coefficients are applied in reverse order to the delay line
    for (size_t i = 0; i < m_tap_count; ++i) {
        size_t delay_idx = (m_current_pos + m_tap_count - i) % m_tap_count;
        output_sample += static_cast<float>(m_delay_line[delay_idx] * m_coeffs[i]);
    }

    // Advance write pointer
    m_current_pos = (m_current_pos + 1) % m_tap_count;

    return output_sample;
}

void FIRFilter::process_block(const float* input_samples, float* output_samples, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        output_samples[i] = process_sample(input_samples[i]);
    }
}

} // namespace sstv::dsp
