#include "sstv_decoder.h"
#include <algorithm> // For std::min

namespace sstv {

constexpr int FIR_TAP_COUNT = 100; // Example tap count, adjust for desired filter quality
constexpr double INTERNAL_SAMPLE_RATE = 11025.0; // Target sample rate for internal processing

Decoder::Decoder(double sample_rate)
    : m_state(State::SEARCHING_VIS),
      m_sample_rate(sample_rate),
      m_mode_is_pd120(false)
{
    // Initialize DSP components
    // Bandpass for SSTV audio spectrum (e.g., 500 Hz to 2500 Hz)
    m_bandpass_filter = std::make_unique<dsp::FIRFilter>(FIR_TAP_COUNT, INTERNAL_SAMPLE_RATE, 500.0, 2500.0);
    m_freq_estimator = std::make_unique<dsp::FrequencyEstimator>(INTERNAL_SAMPLE_RATE);

    // Initialize protocol components with internal callbacks
    m_vis_decoder = std::make_unique<VISDecoder>(INTERNAL_SAMPLE_RATE, 
        [this](const SSTVMode& mode){ handle_mode_detected(mode); });
    
    m_pd120_demodulator = std::make_unique<PD120Demodulator>(INTERNAL_SAMPLE_RATE,
        [this](int line_idx, const std::vector<Pixel>& pixels){ handle_line_decoded(line_idx, pixels); },
        [this](int width, int height){ handle_image_complete(width, height); });

    // Ensure initial state is reset
    reset();
}

Decoder::~Decoder() {
    // Unique pointers handle memory cleanup
}

void Decoder::reset() {
    m_state = State::SEARCHING_VIS;
    m_mode_is_pd120 = false;
    m_current_mode = {}; // Clear current mode data

    m_bandpass_filter->clear();
    m_freq_estimator->clear();
    m_vis_decoder->reset();
    m_pd120_demodulator->reset();
    
    std::cout << "SSTV Decoder reset. Searching for VIS..." << std::endl;
}

void Decoder::process(const float* samples, size_t count) {
    // If the input sample rate doesn't match the internal, a resampler would be needed here.
    // For this example, we assume `samples` are already at `INTERNAL_SAMPLE_RATE`.
    if (m_sample_rate != INTERNAL_SAMPLE_RATE) {
        std::cerr << "Warning: Input sample rate (" << m_sample_rate 
                  << ") does not match internal rate (" << INTERNAL_SAMPLE_RATE << "). "
                  << "Resampling is not implemented in this demo." << std::endl;
        // In a real system, you'd resample the 'samples' block here.
    }

    std::vector<float> filtered_samples(count);
    m_bandpass_filter->process_block(samples, filtered_samples.data(), count);

    std::vector<double> estimated_frequencies = m_freq_estimator->process_block(filtered_samples.data(), count);
    
    switch (m_state) {
        case State::SEARCHING_VIS: {
            bool vis_decoded = m_vis_decoder->process_frequencies(estimated_frequencies);
            if (vis_decoded) {
                // State change handled by `handle_mode_detected` callback
            }
            break;
        }
        case State::DECODING_IMAGE_DATA: {
            if (m_mode_is_pd120) {
                // Convert frequencies to pixel values for PD120Demodulator
                std::vector<uint8_t> pixel_values(estimated_frequencies.size());
                for(size_t i = 0; i < estimated_frequencies.size(); ++i) {
                    pixel_values[i] = dsp::freq_to_pixel_value(estimated_frequencies[i]);
                }
                m_pd120_demodulator->process_pixel_values(pixel_values);
            }
            // Add logic for other modes here if implemented
            break;
        }
        case State::IMAGE_COMPLETE: {
            // Wait for a reset or a new transmission to start
            break;
        }
        case State::DECODING_IMAGE_HEADER: {
            // Not implemented for PD120, but other modes might have post-VIS headers
            m_state = State::DECODING_IMAGE_DATA; // Fall through for PD120
            break;
        }
    }
}

// Internal callback handlers
void Decoder::handle_mode_detected(const SSTVMode& mode) {
    m_current_mode = mode;
    m_mode_is_pd120 = (mode.vis_code == PD120ModeConfig::VIS_CODE);
    
    if (m_on_mode_detected_cb) {
        m_on_mode_detected_cb(mode);
    }
    
    if (m_mode_is_pd120) {
        std::cout << "Mode detected: " << mode.name << ". Starting PD120 demodulation." << std::endl;
        m_state = State::DECODING_IMAGE_DATA;
        m_pd120_demodulator->reset(); // Ensure PD120 demodulator is ready
    } else {
        std::cout << "Mode detected: " << mode.name << ". Not PD120. Resetting." << std::endl;
        // For other modes, we'd transition to their specific demodulators,
        // but for this example, we just reset if it's not PD120.
        reset(); 
    }
}

void Decoder::handle_line_decoded(int line_idx, const std::vector<Pixel>& pixels) {
    if (m_on_line_decoded_cb) {
        m_on_line_decoded_cb(line_idx, pixels);
    }
}

void Decoder::handle_image_complete(int width, int height) {
    std::cout << "Image transmission complete (" << width << "x" << height << ")." << std::endl;
    if (m_on_image_complete_cb) {
        m_on_image_complete_cb(width, height);
    }
    m_state = State::IMAGE_COMPLETE;
    // The PD120Demodulator will reset itself. VISDecoder is already complete.
}

} // namespace sstv
