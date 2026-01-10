#include "sstv_pd120_demodulator.h"
#include "dsp_freq_estimator.h"
#include <algorithm>
#include <iostream>

namespace sstv {

PD120Demodulator::PD120Demodulator(double sample_rate,
                                 LineDecodedCallback on_line_decoded_cb,
                                 ImageCompleteCallback on_image_complete_cb)
    : m_sample_rate(sample_rate),
      m_samples_per_ms(sample_rate / 1000.0),
      m_on_line_decoded(on_line_decoded_cb),
      m_on_image_complete(on_image_complete_cb)
{
    reset();
}

void PD120Demodulator::reset() {
    m_current_segment = SegmentType::IDLE;
    m_segment_timer = 0;
    m_current_line_idx = 0;
    m_segment_buffer.clear();
    m_y1_pixels.clear();
    m_y2_pixels.clear();
    m_cr_pixels.clear();
    m_cb_pixels.clear();
}

bool PD120Demodulator::process_frequency(double freq) {
    m_segment_timer += 1.0;

    // 预计算阈值，避免重复计算
    const double sync_duration_samples = PD120ModeConfig::SYNC_DURATION_MS * m_samples_per_ms;
    const double porch_duration_samples = PD120ModeConfig::PORCH_DURATION_MS * m_samples_per_ms;
    const double segment_duration_samples = PD120ModeConfig::SEGMENT_DURATION_MS * m_samples_per_ms;

    switch (m_current_segment) {
        case SegmentType::IDLE:
            // IDLE -> SYNC 是硬同步点，这里必须归零，这是为了对齐发送端的时钟
            if (std::abs(freq - SYNC_FREQ) < FREQ_TOLERANCE) {
                m_current_segment = SegmentType::SYNC;
                m_segment_timer = 0;
            }
            break;

        case SegmentType::SYNC:
            // 逻辑优化：如果时间到了，或者频率明显变成了 Porch (1500Hz) 且时间已经过半
            // 这允许我们根据信号特征自动对齐，而不是死板地等待时间
            if (m_segment_timer >= sync_duration_samples) {
                m_current_segment = SegmentType::PORCH;
                m_segment_timer = 0; // Sync 之后是硬对齐，重新计数
            }
            // 提前检测 Porch 转换 (Smart Sync):
            // 如果已经在 Sync 里待了超过 15ms，且频率接近 1500Hz，强制进入 Porch
            else if (m_segment_timer > (15.0 * m_samples_per_ms) && std::abs(freq - BLACK_FREQ) < FREQ_TOLERANCE) {
                m_current_segment = SegmentType::PORCH;
                m_segment_timer = 0;
            }
            else if (std::abs(freq - SYNC_FREQ) > FREQ_TOLERANCE * 2) {
                // 只有在极短时间内丢失信号才认为是噪声并重置
                if (m_segment_timer < (5.0 * m_samples_per_ms)) {
                    m_current_segment = SegmentType::IDLE;
                }
            }
            break;

        case SegmentType::PORCH:
            if (m_segment_timer >= porch_duration_samples) {
                m_current_segment = SegmentType::Y1;
                // 这里也进行硬重置，开始数据段的精确计数
                m_segment_timer = 0;
                m_segment_buffer.clear();
                m_segment_buffer.reserve(static_cast<size_t>(segment_duration_samples + 10));
            }
            break;

        case SegmentType::Y1:
        case SegmentType::RY:
        case SegmentType::BY:
        case SegmentType::Y2:
            m_segment_buffer.push_back(freq);

            if (m_segment_timer >= segment_duration_samples) {
                process_current_segment();

                // 状态流转
                if (m_current_segment == SegmentType::Y1) m_current_segment = SegmentType::RY;
                else if (m_current_segment == SegmentType::RY) m_current_segment = SegmentType::BY;
                else if (m_current_segment == SegmentType::BY) m_current_segment = SegmentType::Y2;
                else if (m_current_segment == SegmentType::Y2) {
                    finalize_line_group();
                    m_current_segment = SegmentType::IDLE;
                    // Y2 结束回到 IDLE，不需要保留误差，因为我们需要等待下一个 Sync 信号
                    // 下一次 Sync 检测会自动消除之前的累积误差
                    m_segment_timer = 0;
                    m_segment_buffer.clear();
                    break; // 跳出 switch
                }

                // 核心修复：保留小数部分的误差！
                // 只有在数据段连续切换时（Y1->RY->BY->Y2），才保留时间残余
                // 这样 0.64 + 0.64 + ... 最终会凑成一个完整的样本，自动修正漂移
                m_segment_timer -= segment_duration_samples;

                m_segment_buffer.clear();
            }
            break;
    }

    if (m_current_line_idx >= PD120ModeConfig::HEIGHT) {
        return true;
    }
    return false;
}

// ... 其余函数 (resample_segment, finalize_line_group, ycbcr_to_rgb) 保持不变 ...
// ... 记得把 resample_segment 等函数的实现也放进来，或者只替换上面的 process_frequency ...
// 此处省略其余未变动代码以节省空间
void PD120Demodulator::process_current_segment() {
    std::vector<uint8_t> pixels = resample_segment(m_segment_buffer, PD120ModeConfig::WIDTH);
    switch (m_current_segment) {
        case SegmentType::Y1: m_y1_pixels = std::move(pixels); break;
        case SegmentType::RY: m_cr_pixels = std::move(pixels); break;
        case SegmentType::BY: m_cb_pixels = std::move(pixels); break;
        case SegmentType::Y2: m_y2_pixels = std::move(pixels); break;
        default: break;
    }
}

std::vector<uint8_t> PD120Demodulator::resample_segment(const std::vector<double>& buffer, int target_count) {
    if (buffer.empty()) return std::vector<uint8_t>(target_count, 0);
    std::vector<uint8_t> result(target_count);
    double src_size = static_cast<double>(buffer.size());
    for (int i = 0; i < target_count; ++i) {
        double pos = (static_cast<double>(i) / target_count) * src_size;
        size_t idx = std::clamp(static_cast<size_t>(pos), (size_t)0, buffer.size() - 1);
        result[i] = dsp::freq_to_pixel_value(buffer[idx]);
    }
    return result;
}

void PD120Demodulator::finalize_line_group() {
    if (m_y1_pixels.empty() || m_y2_pixels.empty() || m_cr_pixels.empty() || m_cb_pixels.empty()) return;

    if (m_current_line_idx < PD120ModeConfig::HEIGHT) {
        std::vector<Pixel> line1(PD120ModeConfig::WIDTH);
        for (int i = 0; i < PD120ModeConfig::WIDTH; ++i) line1[i] = ycbcr_to_rgb(m_y1_pixels[i], m_cb_pixels[i], m_cr_pixels[i]);
        m_on_line_decoded(m_current_line_idx++, line1);
    }
    if (m_current_line_idx < PD120ModeConfig::HEIGHT) {
        std::vector<Pixel> line2(PD120ModeConfig::WIDTH);
        for (int i = 0; i < PD120ModeConfig::WIDTH; ++i) line2[i] = ycbcr_to_rgb(m_y2_pixels[i], m_cb_pixels[i], m_cr_pixels[i]);
        m_on_line_decoded(m_current_line_idx++, line2);
    }
    if (m_current_line_idx >= PD120ModeConfig::HEIGHT) m_on_image_complete(PD120ModeConfig::WIDTH, PD120ModeConfig::HEIGHT);
}

Pixel PD120Demodulator::ycbcr_to_rgb(uint8_t Y, uint8_t Cb, uint8_t Cr) {
    int y = Y - 16;
    int cb = Cb - 128;
    int cr = Cr - 128;
    int r = (298 * y + 409 * cr + 128) >> 8;
    int g = (298 * y - 100 * cb - 208 * cr + 128) >> 8;
    int b = (298 * y + 516 * cb + 128) >> 8;
    return { static_cast<uint8_t>(std::clamp(r, 0, 255)), static_cast<uint8_t>(std::clamp(g, 0, 255)), static_cast<uint8_t>(std::clamp(b, 0, 255)) };
}

} // namespace sstv
