#pragma once

#include "sstv_types.h"
#include "dsp_filters.h"
#include "dsp_freq_estimator.h"
#include "sstv_vis_decoder.h"
#include "sstv_pd120_demodulator.h"

#include <vector>
#include <functional>
#include <memory>
#include <iostream>

namespace sstv {

// The top-level SSTV Decoder class
class Decoder {
public:
    explicit Decoder(double sample_rate = 11025.0);
    ~Decoder();

    // Core entry point: Push audio samples into the decoder
    // `samples` should be normalized float values between -1.0 and 1.0.
    // The samples are assumed to be at the Decoder's sample_rate.
    void process(const float* samples, size_t count);

    // Reset the decoder to its initial state (e.g., to search for a new transmission)
    void reset();

    // Callbacks for UI or storage
    void set_on_mode_detected_callback(ModeDetectedCallback cb) { m_on_mode_detected_cb = cb; }
    void set_on_line_decoded_callback(LineDecodedCallback cb) { m_on_line_decoded_cb = cb; }
    void set_on_image_complete_callback(ImageCompleteCallback cb) { m_on_image_complete_cb = cb; }

private:
    enum class State {
        SEARCHING_VIS,
        DECODING_IMAGE_HEADER, // For modes with special header after VIS (not implemented for PD120)
        DECODING_IMAGE_DATA,
        IMAGE_COMPLETE
    };
    State m_state;
    double m_sample_timer;
    double m_sample_rate;

    // DSP Components
    std::unique_ptr<dsp::FIRFilter> m_bandpass_filter;
    std::unique_ptr<dsp::FrequencyEstimator> m_freq_estimator;

    // Protocol Components
    std::unique_ptr<VISDecoder> m_vis_decoder;
    std::unique_ptr<PD120Demodulator> m_pd120_demodulator;

    // Current Mode detected by VIS
    SSTVMode m_current_mode;
    bool m_mode_is_pd120;

    // Callback handlers
    ModeDetectedCallback m_on_mode_detected_cb;
    LineDecodedCallback m_on_line_decoded_cb;
    ImageCompleteCallback m_on_image_complete_cb;

    // Internal callback wrappers to handle mode state changes
    void handle_mode_detected(const SSTVMode& mode);
    void handle_line_decoded(int line_idx, const std::vector<Pixel>& pixels);
    void handle_image_complete(int width, int height);
};

} // namespace sstv
