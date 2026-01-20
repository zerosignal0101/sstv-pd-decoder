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
    m_freq_offset = 0.0;
    m_segment_buffer.clear();
    m_y1_pixels.clear();
    m_y2_pixels.clear();
    m_cr_pixels.clear();
    m_cb_pixels.clear();
}

bool PD120Demodulator::process_frequency(double freq) {
    double corrected_freq = freq - m_freq_offset;

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
            // 在同步脉冲的中间段（例如 5ms 到 15ms 之间）进行测量，避开边缘跳变
            if (m_segment_timer > (5.0 * m_samples_per_ms) &&
                m_segment_timer < (15.0 * m_samples_per_ms)) {

                double measured_offset = freq - SYNC_FREQ;

                // 使用 IIR 滤波器平滑 offset
                // 新偏移 = 10% 当前测量值 + 90% 历史记录
                m_freq_offset = (AFC_ALPHA * measured_offset) + ((1.0 - AFC_ALPHA) * m_freq_offset);
            }

            // 探测 1200 -> 1500 的跳变沿作为 PORCH 的起始（硬同步）
            if (m_segment_timer > (10.0 * m_samples_per_ms)) {
                // 如果修正后的频率更接近黑色 (1500)，说明同步结束了
                if (std::abs(corrected_freq - BLACK_FREQ) < std::abs(corrected_freq - SYNC_FREQ)) {
                    m_current_segment = SegmentType::PORCH;
                    m_segment_timer = 0;
                    break;
                }
            }

            // 超时退出
            if (m_segment_timer >= sync_duration_samples) {
                m_current_segment = SegmentType::PORCH;
                m_segment_timer = 0;
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
            m_segment_buffer.push_back(corrected_freq);

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
        size_t idx_a = static_cast<size_t>(pos);
        size_t idx_b = std::min(idx_a + 1, buffer.size() - 1);
        double weight = pos - idx_a;

        // 线性插值频率，然后再转像素值
        double interpolated_freq = buffer[idx_a] * (1.0 - weight) + buffer[idx_b] * weight;
        result[i] = dsp::freq_to_pixel_value(interpolated_freq);
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
