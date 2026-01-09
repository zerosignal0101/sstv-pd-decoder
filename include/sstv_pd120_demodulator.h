#pragma once

#include "sstv_types.h"
#include "dsp_freq_estimator.h"
#include <vector>
#include <memory>
#include <iostream>

namespace sstv {

class PD120Demodulator {
public:
    PD120Demodulator(double sample_rate, LineDecodedCallback on_line_decoded_cb, ImageCompleteCallback on_image_complete_cb);

    // Process a block of estimated pixel values
    // Returns true if image transmission is complete
    bool process_pixel_values(const std::vector<uint8_t>& pixel_values);

    void reset();

private:
    enum class SegmentType {
        IDLE, SYNC, PORCH, Y1, RY, BY, Y2
    };
    SegmentType m_current_segment;
    double m_sample_rate;
    double m_samples_per_ms;

    LineDecodedCallback m_on_line_decoded;
    ImageCompleteCallback m_on_image_complete;

    // State tracking variables
    double m_current_segment_samples;
    int m_current_line_idx; // 0 to PD120ModeConfig::HEIGHT - 1

    std::vector<uint8_t> m_y1_buffer;
    std::vector<uint8_t> m_y2_buffer;
    std::vector<uint8_t> m_cr_buffer; // R-Y
    std::vector<uint8_t> m_cb_buffer; // B-Y

    void advance_segment(double current_freq);
    void decode_lines();

    // YCrCb to RGB conversion
    Pixel ycbcr_to_rgb(uint8_t Y, uint8_t Cb, uint8_t Cr);
};

} // namespace sstv
