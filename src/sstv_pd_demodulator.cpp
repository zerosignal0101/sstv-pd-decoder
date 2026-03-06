#include "sstv_pd_demodulator.h"
#include "dsp_freq_estimator.h"
#include <algorithm>
#include <iostream>

namespace sstv {

PDDemodulator::PDDemodulator(double sample_rate,
                                 LineDecodedCallback on_line_decoded_cb,
                                 ImageCompleteCallback on_image_complete_cb)
    : m_sample_rate(sample_rate),
      m_samples_per_ms(sample_rate / 1000.0),
      m_on_line_decoded(on_line_decoded_cb),
      m_on_image_complete(on_image_complete_cb)
{
    init_filters();
    reset();
}

void PDDemodulator::init_filters() {
    m_iir1200.setup_bandpass(1200.0 + m_afc_offset, 100.0, m_sample_rate);
    m_iir1500.setup_bandpass(1500.0 + m_afc_offset, 100.0, m_sample_rate);
    m_lpf1200.setup_lowpass(50.0, m_sample_rate);
    m_lpf1500.setup_lowpass(50.0, m_sample_rate);
}

void PDDemodulator::configure(const SSTVMode& mode, const PDTimings& timings) {
    m_timings = timings;
    m_width = mode.width;
    m_height = mode.height;

    m_segment_buffer.reserve(static_cast<size_t>(m_timings.segment_ms * m_samples_per_ms * 1.2));
    m_y1_pixels.resize(m_width); m_y2_pixels.resize(m_width); m_cr_pixels.resize(m_width); m_cb_pixels.resize(m_width);
}


void PDDemodulator::reset() {
    m_current_segment = SegmentType::IDLE;
    m_segment_timer = 0;
    m_current_line_idx = 0;
    // 中值滤波重置
    m_median_buffer.clear();
    // AFC 重置
    m_afc_offset = 0.0;
    m_segment_buffer.clear();
    m_y1_pixels.clear();
    m_y2_pixels.clear();
    m_cr_pixels.clear();
    m_cb_pixels.clear();
}

void PDDemodulator::set_afc_offset(double afc_offset) {
    // 从 VIS 解码器继承 AFC 偏移，用于数据段解调
    m_afc_offset = afc_offset;
    init_filters();
    // std::cout << "PD Demodulator: Initial freq offset set to " << freq_offset << " Hz" << std::endl;
}

double PDDemodulator::get_smoothed_freq(double raw_freq) {
    m_median_buffer.push_back(raw_freq);
    if (m_median_buffer.size() > MEDIAN_WINDOW) m_median_buffer.pop_front();

    std::vector<double> sorted = {m_median_buffer.begin(), m_median_buffer.end()};
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
}

void PDDemodulator::reserve_samples(double reserved_samples)
{
    m_segment_timer = reserved_samples - m_segment_timer;
}

bool PDDemodulator::process(float sample, double freq) {
    // 1. 提取 1200Hz 通道包络
    float s12 = m_iir1200.process(sample);
    float env12 = m_lpf1200.process(std::abs(s12));

    // 2. 提取 1500Hz 参考通道包络
    float s15 = m_iir1500.process(sample);
    float env15 = m_lpf1500.process(std::abs(s15));

    // 3. 动态更新背景噪声水平 (Slow Attack, Fast Release)
    // 用于处理长时间的静默或增益变化
    m_adaptive_threshold = 0.999f * m_adaptive_threshold + 0.001f * (env12 + 0.005f);

    double corrected_freq = freq - m_afc_offset;

    m_segment_timer += 1.0;

    // 预计算阈值，避免重复计算
    const double sync_duration_samples = m_timings.sync_ms * m_samples_per_ms;
    const double porch_duration_samples = m_timings.porch_ms * m_samples_per_ms;
    const double segment_duration_samples = m_timings.segment_ms * m_samples_per_ms;

    switch (m_current_segment) {
        case SegmentType::IDLE: {
            // // 调试代码：输出 corrected_freq 并限制输出次数
            // static int debug_counter = 0;
            // if (debug_counter < 20) {
            //     std::cout << "Debug [" << debug_counter << "]: corrected_freq = " << corrected_freq << std::endl;
            //     debug_counter++;
            // } else if (debug_counter == 20) {
            //     std::cout << "Reached 20 debug outputs. Exiting..." << std::endl;
            //     exit(0);
            // }
            // IDLE -> SYNC 是硬同步点，这里必须归零，这是为了对齐发送端的时钟
            if (env12 > m_adaptive_threshold && env12 > env15 * 2.0) {
                m_current_segment = SegmentType::SYNC;
                m_segment_timer = 0;
                // 重置中值滤波
                m_median_buffer.clear();
            }
            break;
        }

        case SegmentType::SYNC: {
            double smoothed_freq = get_smoothed_freq(freq);  // 使用原始 freq 计算 AFC 偏置
            // std::cout << "Debug SYNC [" << m_segment_timer << "]: corrected_freq = " << corrected_freq << " env12 = " << env12 << " env15 = " << env15 << std::endl;
            if (m_segment_timer > (sync_duration_samples * 0.25) &&
                m_segment_timer < (sync_duration_samples * 0.5)) {

                double measured_offset = smoothed_freq - SYNC_FREQ;

                // 使用 IIR 滤波器平滑 offset
                // 新偏移 = 10% 当前测量值 + 90% 历史记录
                m_afc_offset = (AFC_ALPHA * measured_offset) + ((1.0 - AFC_ALPHA) * m_afc_offset);
                }

            // 探测 1200 -> 1500 的跳变沿作为 PORCH 的起始（硬同步）
            if (m_segment_timer > (sync_duration_samples * 0.5)) {
                // 如果修正后的频率更接近黑色 (1500)，说明同步结束了
                double corrected_smoothed_freq = smoothed_freq - m_afc_offset;
                if (std::abs(corrected_smoothed_freq - BLACK_FREQ) < std::abs(corrected_smoothed_freq - SYNC_FREQ)) {
                    // std::cout << "Transit to PORCH with m_afc_offset: " << m_afc_offset << "Hz" << std::endl;
                    // std::cout << "Corrected smoothed freq:" << corrected_smoothed_freq << " Smoothed freq: " << smoothed_freq << " Origianl freq: " << freq << std::endl;
                    // std::cout << "Sync duration sample num: " << sync_duration_samples <<
                    //     " Porch duration sample num: " << porch_duration_samples << std::endl;
                    m_current_segment = SegmentType::PORCH;
                    m_segment_timer = static_cast<double>(MEDIAN_WINDOW) / 2;
                    break;
                }
            }

            // 超时退出
            if (m_segment_timer >= sync_duration_samples) {
                m_current_segment = SegmentType::PORCH;
                // std::cout << "Timeout! Transit to PORCH with m_afc_offset: " << m_afc_offset << "Hz" << std::endl;
                // std::cout << "Smoothed freq: " << smoothed_freq << " Origianl freq: " << freq << std::endl;
                // std::cout << "Sync duration sample num: " << sync_duration_samples <<
                //     " Porch duration sample num: " << porch_duration_samples << std::endl;
                reserve_samples(sync_duration_samples);
            }
            break;
        }

        case SegmentType::PORCH:{
            // // 调试代码：输出 corrected_freq 并限制输出次数
            // static int debug_counter = 0;
            // if (debug_counter < 30) {
            //     std::cout << "Debug [" << debug_counter << "]: corrected_freq = " << corrected_freq << " env12 = " << env12 << " env15 = " << env15 << std::endl;
            //     debug_counter++;
            // } else if (debug_counter == 30) {
            //     std::cout << "Reached 30 debug outputs. Stopping output..." << std::endl;
            //     // debug_counter++;
            //     exit(0);
            // }
            if (m_segment_timer >= porch_duration_samples) {
                m_current_segment = SegmentType::Y1;
                // 这里也进行硬重置，开始数据段的精确计数
                // // Debug 调试代码重置
                // debug_counter = 0;
                reserve_samples(porch_duration_samples);
                m_segment_buffer.clear();
                m_segment_buffer.reserve(static_cast<size_t>(segment_duration_samples + 10));
            }
            break;
        }

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
                    reserve_samples(segment_duration_samples);
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

    if (m_current_line_idx >= m_height) {
        return true;
    }
    return false;
}

void PDDemodulator::process_current_segment() {
    std::vector<uint8_t> pixels = resample_segment(m_segment_buffer, m_width);
    switch (m_current_segment) {
        case SegmentType::Y1: m_y1_pixels = std::move(pixels); break;
        case SegmentType::RY: m_cr_pixels = std::move(pixels); break;
        case SegmentType::BY: m_cb_pixels = std::move(pixels); break;
        case SegmentType::Y2: m_y2_pixels = std::move(pixels); break;
        default: break;
    }
}

std::vector<uint8_t> PDDemodulator::resample_segment(const std::vector<double>& buffer, int target_count) {
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

void PDDemodulator::finalize_line_group() {
    if (m_y1_pixels.empty() || m_y2_pixels.empty() || m_cr_pixels.empty() || m_cb_pixels.empty()) return;

    if (m_current_line_idx < m_height) {
        std::vector<Pixel> line1(m_width);
        for (int i = 0; i < m_width; ++i) line1[i] = ycbcr_to_rgb(m_y1_pixels[i], m_cb_pixels[i], m_cr_pixels[i]);
        m_on_line_decoded(m_current_line_idx++, line1);
    }
    if (m_current_line_idx < m_height) {
        std::vector<Pixel> line2(m_width);
        for (int i = 0; i < m_width; ++i) line2[i] = ycbcr_to_rgb(m_y2_pixels[i], m_cb_pixels[i], m_cr_pixels[i]);
        m_on_line_decoded(m_current_line_idx++, line2);
    }
    if (m_current_line_idx >= m_height) m_on_image_complete(m_width, m_height);
}

Pixel PDDemodulator::ycbcr_to_rgb(uint8_t Y, uint8_t Cb, uint8_t Cr) {
    int y = Y - 16;
    int cb = Cb - 128;
    int cr = Cr - 128;
    int r = (298 * y + 409 * cr + 128) >> 8;
    int g = (298 * y - 100 * cb - 208 * cr + 128) >> 8;
    int b = (298 * y + 516 * cb + 128) >> 8;
    return { static_cast<uint8_t>(std::clamp(r, 0, 255)), static_cast<uint8_t>(std::clamp(g, 0, 255)), static_cast<uint8_t>(std::clamp(b, 0, 255)) };
}

} // namespace sstv
