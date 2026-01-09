#include "sstv_pd120_demodulator.h"
#include <algorithm> // For std::clamp

namespace sstv {

// Frequency comparison tolerance for sync and porch
constexpr double SYNC_FREQ_TOLERANCE = 50.0;

PD120Demodulator::PD120Demodulator(double sample_rate, LineDecodedCallback on_line_decoded_cb, ImageCompleteCallback on_image_complete_cb)
    : m_current_segment(SegmentType::IDLE),
      m_sample_rate(sample_rate),
      m_samples_per_ms(sample_rate / 1000.0),
      m_on_line_decoded(on_line_decoded_cb),
      m_on_image_complete(on_image_complete_cb)
{
    reset();
}

void PD120Demodulator::reset() {
    m_current_segment = SegmentType::IDLE;
    m_current_segment_samples = 0.0;
    m_current_line_idx = 0;

    m_y1_buffer.clear();
    m_y2_buffer.clear();
    m_cr_buffer.clear();
    m_cb_buffer.clear();
}

// Convert YCrCb to RGB
Pixel PD120Demodulator::ycbcr_to_rgb(uint8_t Y, uint8_t Cb, uint8_t Cr) {
    // ITU-R BT.601 conversion (common for analog video)
    // R = Y + 1.402 * (Cr - 128)
    // G = Y - 0.344136 * (Cb - 128) - 0.714136 * (Cr - 128)
    // B = Y + 1.772 * (Cb - 128)
    
    // Normalize to 0-1 range first for float calculations
    double Y_norm = static_cast<double>(Y) / 255.0;
    double Cb_norm = static_cast<double>(Cb) / 255.0;
    double Cr_norm = static_cast<double>(Cr) / 255.0;

    // Adjust Cr, Cb to be centered around 0 (e.g., -0.5 to 0.5)
    double Cr_adj = Cr_norm - 0.5;
    double Cb_adj = Cb_norm - 0.5;

    double R = Y_norm + 1.402 * Cr_adj;
    double G = Y_norm - 0.344136 * Cb_adj - 0.714136 * Cr_adj;
    double B = Y_norm + 1.772 * Cb_adj;

    // Clamp and convert back to 0-255
    return {
        static_cast<uint8_t>(std::clamp(R * 255.0, 0.0, 255.0)),
        static_cast<uint8_t>(std::clamp(G * 255.0, 0.0, 255.0)),
        static_cast<uint8_t>(std::clamp(B * 255.0, 0.0, 255.0))
    };
}


void PD120Demodulator::decode_lines() {
    if (m_y1_buffer.size() < PD120ModeConfig::WIDTH ||
        m_y2_buffer.size() < PD120ModeConfig::WIDTH ||
        m_cr_buffer.size() < PD120ModeConfig::WIDTH ||
        m_cb_buffer.size() < PD120ModeConfig::WIDTH) {
        std::cerr << "PD120Demodulator: Buffer sizes mismatch for decoding. Resetting." << std::endl;
        reset(); // Critical error, reset
        return;
    }

    // Decode Line 1
    if (m_current_line_idx < PD120ModeConfig::HEIGHT) {
        std::vector<Pixel> line1_pixels(PD120ModeConfig::WIDTH);
        for (int i = 0; i < PD120ModeConfig::WIDTH; ++i) {
            line1_pixels[i] = ycbcr_to_rgb(m_y1_buffer[i], m_cb_buffer[i], m_cr_buffer[i]);
        }
        m_on_line_decoded(m_current_line_idx, line1_pixels);
        m_current_line_idx++;
    }

    // Decode Line 2 (if available)
    if (m_current_line_idx < PD120ModeConfig::HEIGHT) {
        std::vector<Pixel> line2_pixels(PD120ModeConfig::WIDTH);
        for (int i = 0; i < PD120ModeConfig::WIDTH; ++i) {
            line2_pixels[i] = ycbcr_to_rgb(m_y2_buffer[i], m_cb_buffer[i], m_cr_buffer[i]);
        }
        m_on_line_decoded(m_current_line_idx, line2_pixels);
        m_current_line_idx++;
    }

    // Clear buffers for next group
    m_y1_buffer.clear();
    m_y2_buffer.clear();
    m_cr_buffer.clear();
    m_cb_buffer.clear();
}


void PD120Demodulator::advance_segment(double current_freq) {
    double segment_duration_samples = PD120ModeConfig::SEGMENT_DURATION_MS * m_samples_per_ms;

    switch (m_current_segment) {
        case SegmentType::IDLE:
            // Expecting SYNC signal
            if (std::abs(current_freq - SYNC_FREQ) < SYNC_FREQ_TOLERANCE) {
                m_current_segment = SegmentType::SYNC;
            } else {
                // If not sync, could be noise or end of image
                return;
            }
            break;

        case SegmentType::SYNC:
            if (std::abs(current_freq - SYNC_FREQ) < SYNC_FREQ_TOLERANCE) {
                m_current_segment_samples += 1.0;
                if (m_current_segment_samples >= PD120ModeConfig::SYNC_DURATION_MS * m_samples_per_ms) {
                    m_current_segment = SegmentType::PORCH;
                    m_current_segment_samples = 0.0;
                }
            } else {
                // Sync lost
                reset();
                return;
            }
            break;

        case SegmentType::PORCH:
            if (std::abs(current_freq - BLACK_FREQ) < SYNC_FREQ_TOLERANCE) {
                m_current_segment_samples += 1.0;
                if (m_current_segment_samples >= PD120ModeConfig::PORCH_DURATION_MS * m_samples_per_ms) {
                    m_current_segment = SegmentType::Y1;
                    m_current_segment_samples = 0.0;
                    m_y1_buffer.reserve(PD120ModeConfig::WIDTH);
                }
            } else {
                // Porch lost
                reset();
                return;
            }
            break;

        case SegmentType::Y1:
        case SegmentType::RY:
        case SegmentType::BY:
        case SegmentType::Y2: {
            uint8_t pixel_val = dsp::freq_to_pixel_value(current_freq);
            
            // Determine number of pixel samples per segment
            // This assumes 640 pixels per segment
            double pixels_per_sample = PD120ModeConfig::WIDTH / segment_duration_samples;
            
            // To properly match pixel data to fixed width, we need to collect enough samples
            // and then average/decimate them into 640 discrete pixels.
            // For simplicity here, we'll store all incoming freq-mapped samples and then average.
            // A more precise approach involves clock recovery for exact pixel timing.
            
            if (m_current_segment == SegmentType::Y1) m_y1_buffer.push_back(pixel_val);
            else if (m_current_segment == SegmentType::RY) m_cr_buffer.push_back(pixel_val);
            else if (m_current_segment == SegmentType::BY) m_cb_buffer.push_back(pixel_val);
            else if (m_current_segment == SegmentType::Y2) m_y2_buffer.push_back(pixel_val);

            m_current_segment_samples += 1.0;
            
            if (m_current_segment_samples >= segment_duration_samples) {
                // After collecting samples for a segment, ensure we have exactly WIDTH pixels
                // This is a crude averaging/resampling. A real implementation would use proper
                // resampling on the collected raw frequencies.
                auto resample_buffer = [&](std::vector<uint8_t>& buffer) {
                    if (buffer.size() > PD120ModeConfig::WIDTH) {
                        std::vector<uint8_t> new_buffer(PD120ModeConfig::WIDTH);
                        for (int i = 0; i < PD120ModeConfig::WIDTH; ++i) {
                            size_t start_idx = static_cast<size_t>(i * buffer.size() / PD120ModeConfig::WIDTH);
                            size_t end_idx = static_cast<size_t>((i + 1) * buffer.size() / PD120ModeConfig::WIDTH);
                            if (start_idx == end_idx) end_idx = start_idx + 1; // Ensure at least one sample
                            
                            int sum = 0;
                            size_t count = 0;
                            for (size_t j = start_idx; j < end_idx && j < buffer.size(); ++j) {
                                sum += buffer[j];
                                count++;
                            }
                            new_buffer[i] = static_cast<uint8_t>(count > 0 ? sum / count : 0);
                        }
                        buffer = std::move(new_buffer);
                    } else if (buffer.size() < PD120ModeConfig::WIDTH) {
                        // Pad with last value or black if not enough samples
                        buffer.resize(PD120ModeConfig::WIDTH, buffer.empty() ? 0 : buffer.back());
                    }
                };

                if (m_current_segment == SegmentType::Y1) {
                    resample_buffer(m_y1_buffer);
                    m_current_segment = SegmentType::RY;
                    m_cr_buffer.reserve(PD120ModeConfig::WIDTH);
                } else if (m_current_segment == SegmentType::RY) {
                    resample_buffer(m_cr_buffer);
                    m_current_segment = SegmentType::BY;
                    m_cb_buffer.reserve(PD120ModeConfig::WIDTH);
                } else if (m_current_segment == SegmentType::BY) {
                    resample_buffer(m_cb_buffer);
                    m_current_segment = SegmentType::Y2;
                    m_y2_buffer.reserve(PD120ModeConfig::WIDTH);
                } else if (m_current_segment == SegmentType::Y2) {
                    resample_buffer(m_y2_buffer);
                    // Two lines worth of data collected!
                    decode_lines();
                    if (m_current_line_idx >= PD120ModeConfig::HEIGHT) {
                        m_on_image_complete(PD120ModeConfig::WIDTH, PD120ModeConfig::HEIGHT);
                        reset(); // Image complete, reset demodulator
                        return;
                    }
                    m_current_segment = SegmentType::IDLE; // Loop back to wait for next SYNC
                }
                m_current_segment_samples = 0.0;
            }
            break;
        }
    }
}


bool PD120Demodulator::process_pixel_values(const std::vector<uint8_t>& pixel_values) {
    if (m_current_line_idx >= PD120ModeConfig::HEIGHT) {
        return true; // Image already complete
    }

    for (uint8_t pv : pixel_values) {
        double freq = dsp::freq_to_pixel_value(pv); // This conversion is a bit circular, but for demo...
                                                   // Ideally, pass raw frequencies from the estimator
        advance_segment(freq);
        if (m_current_line_idx >= PD120ModeConfig::HEIGHT) return true; // Check for completion after each segment
    }
    return false;
}

} // namespace sstv
