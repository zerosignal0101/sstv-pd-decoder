#include "dsp_freq_estimator.h"
#include <numbers>
#include <cmath>

namespace sstv::dsp {

// Constants for Hilbert filter design
constexpr int DEFAULT_HILBERT_TAPS = 63; // Must be odd for symmetric FIR

FrequencyEstimator::FrequencyEstimator(double sample_rate)
    : m_sample_rate(sample_rate),
      m_last_freq(0.0),
      m_write_pos(0),
      m_prev_phase(0.0),
      m_samples_processed(0)
{
    // Initialize filter
    m_buffer_size = DEFAULT_HILBERT_TAPS;
    m_group_delay = m_buffer_size / 2; // Integer division

    generate_hilbert_coeffs();

    // Resize and zero the delay line
    m_buffer.resize(m_buffer_size, 0.0f);
}

void FrequencyEstimator::clear() {
    std::fill(m_buffer.begin(), m_buffer.end(), 0.0f);
    m_write_pos = 0;
    m_last_freq = 0.0;
    m_prev_phase = 0.0;
    m_samples_processed = 0;
}

void FrequencyEstimator::generate_hilbert_coeffs() {
    m_coeffs.resize(m_buffer_size);

    // Generate coefficients for a Hilbert Transformer (Odd taps)
    // Ideal Hilbert: h[n] = 2/(pi * n) for n odd, 0 for n even.
    // Centered at n = 0.
    // We apply a window (Blackman) to reduce Gibbs phenomenon.

    int M = m_buffer_size - 1;
    for (int i = 0; i < m_buffer_size; ++i) {
        int n = i - M / 2; // Index from -M/2 to M/2

        if (n == 0) {
            m_coeffs[i] = 0.0f;
        } else {
            // Ideal impulse response
            double val = 2.0 / (std::numbers::pi * n);

            // Apply Blackman window for better sidelobe suppression
            double window = 0.42 - 0.5 * std::cos(2.0 * std::numbers::pi * i / M) +
                            0.08 * std::cos(4.0 * std::numbers::pi * i / M);

            m_coeffs[i] = static_cast<float>(val * window);
        }
    }
}

double FrequencyEstimator::process_sample(float input_sample) {
    // 1. Write new sample to circular buffer
    m_buffer[m_write_pos] = input_sample;

    // 2. Compute Convolution (Hilbert Transform - Quadrature Component Q)
    // The buffer contains history: newest at m_write_pos, oldest at (m_write_pos + 1) % size
    float q = 0.0f;
    for (size_t i = 0; i < m_buffer_size; ++i) {
        // Access buffer backwards from the current write position
        size_t tap_idx = (m_write_pos + m_buffer_size - i) % m_buffer_size;
        q += m_buffer[tap_idx] * m_coeffs[i];
    }

    // 3. Retrieve In-Phase Component (I)
    // The FIR filter output 'q' corresponds to the signal aligned with the center tap.
    // The center tap is 'group_delay' samples behind the current input.
    // So we need to read the sample that was written 'group_delay' samples ago.
    size_t read_idx = (m_write_pos + m_buffer_size - m_group_delay) % m_buffer_size;
    float i_val = m_buffer[read_idx];

    // 4. Advance write pointer
    m_write_pos = (m_write_pos + 1) % m_buffer_size;
    m_samples_processed++;

    // 5. Handle Startup Transient
    // Wait until the buffer is at least half-full (passed the group delay)
    if (m_samples_processed <= m_group_delay) {
        return 0.0; // Silence during startup
    }

    // 6. Check for signal presence to avoid atan2 noise on silence
    float mag_sq = i_val * i_val + q * q;
    constexpr float NOISE_THRESHOLD = 1e-6f; // Threshold for silence
    if (mag_sq < NOISE_THRESHOLD) {
        return 0.0; // Or return m_last_freq to hold last value
    }

    // 7. Calculate Instantaneous Phase
    double phase = std::atan2(static_cast<double>(q), static_cast<double>(i_val));

    // 8. Unwrap Phase
    double delta_phase = phase - m_prev_phase;

    // Correct for +/- PI jumps
    while (delta_phase <= -std::numbers::pi) delta_phase += 2.0 * std::numbers::pi;
    while (delta_phase > std::numbers::pi) delta_phase -= 2.0 * std::numbers::pi;

    m_prev_phase = phase;

    // 9. Calculate Frequency
    // f = d_phi / d_t
    // d_t = 1 / sample_rate
    // f = delta_phase * sample_rate / (2 * pi)
    m_last_freq = delta_phase * m_sample_rate / (2.0 * std::numbers::pi);

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
