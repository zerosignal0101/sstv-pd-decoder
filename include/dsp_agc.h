#pragma once

#include <vector>
#include <cmath>

namespace sstv::dsp {

class AGC {
public:
    explicit AGC(float target_level = 0.5f, float attack = 0.01f, float release = 0.001f)
        : m_target(target_level), m_attack(attack), m_release(release), m_gain(1.0f) {}

    float process(float input) {
        float abs_input = std::abs(input);
        // 峰值检测
        if (abs_input > m_envelope) {
            m_envelope = m_attack * abs_input + (1.0f - m_attack) * m_envelope;
        } else {
            m_envelope = m_release * abs_input + (1.0f - m_release) * m_envelope;
        }

        // 计算目标增益 (避免除以零)
        if (m_envelope > 1e-6f) {
            float desired_gain = m_target / m_envelope;
            // 限制增益变化速度，防止引入过大的调制噪声
            m_gain = 0.1f * desired_gain + 0.9f * m_gain;
        }

        return input * m_gain;
    }

private:
    float m_target;
    float m_attack;  // 攻击时间（反应快）
    float m_release; // 释放时间（恢复慢）
    float m_envelope = 0.0f;
    float m_gain = 1.0f;
};

} // namespace sstv::dsp