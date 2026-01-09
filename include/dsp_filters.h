// include/dsp_filters.h
#pragma once

#include "sstv_types.h" // 包含 FilterCoefficients 和 FilterDelayLine
#include <vector>
#include <cmath>

namespace sstv::dsp {

// 函数：生成FIR滤波器系数（Hamming窗加Sinc函数法）
// 适用于生成带通滤波器系数 (LPF(fc_high) - LPF(fc_low))。
// tap_count: 滤波器抽头数量（即系数数量）
// sample_rate: 采样率
// cutoff_freq_low: 低截止频率
// cutoff_freq_high: 高截止频率
// attenuation_db: 衰减量（目前在此简化实现中未直接使用，但可用于更高级的滤波器设计或抽头数选择）
// gain: 最终系数的整体增益因子
FilterCoefficients make_fir_coeffs(
    int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high,
    double attenuation_db = 60.0, double gain = 1.0);

class FIRFilter {
public:
    // 构造函数：根据参数初始化滤波器
    FIRFilter(int tap_count, double sample_rate, double cutoff_freq_low, double cutoff_freq_high);

    // 清空内部延迟线（滤波器状态），将其所有样本置零
    void clear();

    // 处理单个样本，返回滤波后的输出样本
    float process_sample(float input_sample);

    // 处理一个样本块
    // input_samples: 输入样本数组
    // output_samples: 输出样本数组 (与输入数组大小相同)
    // count: 样本数量
    void process_block(const float* input_samples, float* output_samples, size_t count);

private:
    FilterCoefficients m_coeffs;    // 滤波器系数
    FilterDelayLine m_delay_line;   // 延迟线（循环缓冲区），存储历史输入样本
    size_t m_tap_count;             // 滤波器抽头数量 (m_coeffs.size())
    size_t m_current_pos;           // 延迟线中的写入指针，指向最新样本的位置
};

} // namespace sstv::dsp
