// include/sstv_pd120_demodulator.h
#pragma once

#include "sstv_types.h"
#include <vector>
#include <memory>
#include <cmath>

namespace sstv {

class PD120Demodulator {
public:
    /**
     * @brief 构造函数
     * @param sample_rate 音频输入采样率
     * @param on_line_decoded_cb 当一行像素解码完成时的回调
     * @param on_image_complete_cb 当整张图像处理完成时的回调
     */
    PD120Demodulator(double sample_rate,
                     LineDecodedCallback on_line_decoded_cb,
                     ImageCompleteCallback on_image_complete_cb);

    /**
     * @brief 处理从频率估计器得到的频率流
     * @param freq 原始频率 (Hz)
     * @return 如果图像传输完成返回 true
     */
    bool process_frequency(double freq);

    /**
     * @brief 重置解调器状态，准备接收新的一帧
     */
    void reset();

private:
    enum class SegmentType {
        IDLE,       // 等待同步信号
        SYNC,       // 1200Hz 同步脉冲 (20ms)
        PORCH,      // 1500Hz 黑色后沿 (2.08ms)
        Y1,         // 第 N 行亮度 (121.6ms)
        RY,         // 第 N & N+1 行红色差 Cr (121.6ms)
        BY,         // 第 N & N+1 行蓝色差 Cb (121.6ms)
        Y2          // 第 N+1 行亮度 (121.6ms)
    };

    SegmentType m_current_segment;
    double m_sample_rate;
    double m_samples_per_ms;

    LineDecodedCallback m_on_line_decoded;
    ImageCompleteCallback m_on_image_complete;

    // 状态追踪
    double m_segment_timer;         // 当前段已持续的采样数
    int    m_current_line_idx;      // 当前处理到的行数 (0 - 495)
    double m_freq_offset;           // 当前检测到的频偏 (Hz)

    // 原始频率缓冲区：存储当前段内的所有频率样本
    // 待一段结束时，再通过重采样算法提取出 640 个像素点
    std::vector<double> m_segment_buffer;

    // 像素缓冲区：存储重采样后的 640 个像素分量
    std::vector<uint8_t> m_y1_pixels;
    std::vector<uint8_t> m_y2_pixels;
    std::vector<uint8_t> m_cr_pixels;
    std::vector<uint8_t> m_cb_pixels;

    // 内部核心逻辑
    void process_current_segment();
    void finalize_line_group();

    // 工具函数
    std::vector<uint8_t> resample_segment(const std::vector<double>& buffer, int target_count);
    Pixel ycbcr_to_rgb(uint8_t Y, uint8_t Cb, uint8_t Cr);

    // 容错常量
    static constexpr double FREQ_TOLERANCE = 60.0;
    static constexpr double AFC_ALPHA = 0.1;
};

} // namespace sstv
