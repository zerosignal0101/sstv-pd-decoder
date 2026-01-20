#include "sstv_decoder.h"
#include <algorithm> // For std::min

namespace sstv {

constexpr int FIR_TAP_COUNT = 31; // Example tap count, adjust for desired filter quality
constexpr double INTERNAL_SAMPLE_RATE = 11025.0; // Target sample rate for internal processing

Decoder::Decoder(double sample_rate)
    : m_state(State::SEARCHING_VIS),
      m_sample_timer(0.0),
      m_sample_rate(sample_rate)
{
    // Initialize DSP components
    // Resampler
    if (std::abs(sample_rate - INTERNAL_SAMPLE_RATE) > 1.0) {
        m_resampler = std::make_unique<dsp::Resampler>(sample_rate, INTERNAL_SAMPLE_RATE);
        // std::cout << "Resampler initialized: " << sample_rate << "Hz -> " << INTERNAL_SAMPLE_RATE << "Hz" << std::endl;
    }
    // Bandpass for SSTV audio spectrum (e.g., 500 Hz to 2500 Hz)
    m_bandpass_filter = std::make_unique<dsp::FIRFilter>(FIR_TAP_COUNT, INTERNAL_SAMPLE_RATE, 500.0, 2500.0);
    m_freq_estimator = std::make_unique<dsp::FrequencyEstimator>(INTERNAL_SAMPLE_RATE);

    // Initialize protocol components with internal callbacks
    m_vis_decoder = std::make_unique<VISDecoder>(INTERNAL_SAMPLE_RATE, 
        [this](const SSTVMode& mode){ handle_mode_detected(mode); });
    
    m_pd_demodulator = std::make_unique<PDDemodulator>(INTERNAL_SAMPLE_RATE,
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
    m_current_mode = {}; // Clear current mode data

    m_bandpass_filter->clear();
    m_freq_estimator->clear();
    if (m_resampler) m_resampler->reset();

    m_vis_decoder->reset();
    m_pd_demodulator->reset();
    
    // std::cout << "SSTV Decoder reset. Searching for VIS..." << std::endl;
}

void Decoder::process(const float* samples, size_t count) {
    const float* current_input = samples;
    size_t current_count = count;
    std::vector<float> resampled_storage;

    // --- 第一步：重采样 (如果需要) ---
    if (m_resampler) {
        resampled_storage = m_resampler->process_block(samples, count);
        current_input = resampled_storage.data();
        current_count = resampled_storage.size();
    }

    if (current_count == 0) return;

    std::vector<float> filtered_samples(current_count);
    m_bandpass_filter->process_block(current_input, filtered_samples.data(), current_count);

    std::vector<double> estimated_frequencies = m_freq_estimator->process_block(filtered_samples.data(), current_count);

    // // Debug freq output
    // for (double freq : estimated_frequencies) {
    //     std::cout << freq << ", ";
    // }
    // std::cout << std::endl;
    // return;

    for (double freq : estimated_frequencies) {
        m_sample_timer += 1.0;
        switch (m_state) {
            case State::SEARCHING_VIS: {
                bool vis_decoded = m_vis_decoder->process_frequency(freq);
                if (vis_decoded) {
                    // State change handled by `handle_mode_detected` callback
                }
                break;
            }
            case State::DECODING_IMAGE_DATA: {
                switch (m_current_mode.family) {
                    case SSTVFamily::PD: {
                        m_pd_demodulator->process_frequency(freq);
                        break;
                    }
                    case SSTVFamily::UNKNOWN:
                    default: {
                        std::cerr << "Unsupported family!" << std::endl;
                        reset();
                        break;
                    }
                    // Other families
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
}

// Internal callback handlers
void Decoder::handle_mode_detected(const SSTVMode& mode) {
    m_current_mode = mode;

    if (m_on_mode_detected_cb) {
        m_on_mode_detected_cb(mode);
    }
    
    switch (m_current_mode.family) {
        case SSTVFamily::PD: {
            // 只在 PD 分支里去查私有的 PD_TIMINGS_MAP
            auto it = PD_TIMINGS_MAP.find(m_current_mode.vis_code);
            if (it != PD_TIMINGS_MAP.end()) {
                m_pd_demodulator->configure(m_current_mode, it->second);
                m_state = State::DECODING_IMAGE_DATA;
            }
            break;
        }
        case SSTVFamily::UNKNOWN:
        default:
            std::cerr << "Unsupported family!" << std::endl;
            reset();
            break;
    }
}

void Decoder::handle_line_decoded(int line_idx, const std::vector<Pixel>& pixels) {
    if (m_on_line_decoded_cb) {
        // Debug info
        // std::cout << "Current sample idx: " << static_cast<uint32_t>(m_sample_timer) << ", Time: " << m_sample_timer / m_sample_rate << std::endl;
        m_on_line_decoded_cb(line_idx, pixels);
    }
}

void Decoder::handle_image_complete(int width, int height) {
    // // Debug info
    // std::cout << "Image transmission complete (" << width << "x" << height << ")." << std::endl;
    if (m_on_image_complete_cb) {
        m_on_image_complete_cb(width, height);
    }
    m_state = State::IMAGE_COMPLETE;
    // The PD120Demodulator will reset itself. VISDecoder is already complete.
}

} // namespace sstv
