"""
PD120 SSTV 解码器综合测试脚本
测试解码器在不同干扰条件下的工作能力，测出极限
"""

import numpy as np
from PIL import Image
import scipy.io.wavfile as wavfile
from scipy.signal import hilbert
import os
import sys
import shutil
from datetime import datetime
from typing import Tuple, Dict, List
import json

# 导入 C++ 解码器
import sstv_decoder

# ============== 常量定义 ==============
SAMPLE_RATE = 44100
TARGET_SR_RAW = 11025
AMPLITUDE = 0.8
WIDTH, HEIGHT = 640, 496

# 频率定义 (Hz)
FREQ_SYNC = 1200
FREQ_BLACK = 1500
FREQ_WHITE = 2300
FREQ_RANGE = FREQ_WHITE - FREQ_BLACK
FREQ_LEADER_BURST = 1900
FREQ_LOGIC_0 = 1300
FREQ_LOGIC_1 = 1100
FREQ_BREAK = 1200

# 输出目录
OUTPUT_DIR = "test_results"


# ============== PD120 编码器 ==============
def rgb_to_ycrcb(img_path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """读取图像并转换为 YCrCb (BT.601 Limited Range)"""
    img = Image.open(img_path).convert('RGB')
    data = np.array(img, dtype=np.float32)

    R, G, B = data[:,:,0], data[:,:,1], data[:,:,2]

    Y  = 16.0  + (0.256789 * R + 0.504129 * G + 0.097906 * B)
    Cb = 128.0 + (-0.148223 * R - 0.290992 * G + 0.439215 * B)
    Cr = 128.0 + (0.439215 * R - 0.367789 * G - 0.071426 * B)

    return Y, Cr, Cb


def generate_tone_sequence(Y, Cr, Cb) -> List[Tuple[float, float]]:
    """构建 PD120 的所有音调规格 (频率，持续时间 ms)"""
    spec = []

    # 1. Preamble (800ms)
    for f in [1900, 1500, 1900, 1500, 2300, 1500, 2300, 1500]:
        spec.append((f, 100))

    # 2. VIS Header (PD120 = 95)
    vis_bits = [1, 1, 1, 1, 1, 0, 1]
    parity_bit = 0 if sum(vis_bits) % 2 == 0 else 1

    spec.append((FREQ_LEADER_BURST, 300))
    spec.append((FREQ_BREAK, 10))
    spec.append((FREQ_LEADER_BURST, 300))
    spec.append((FREQ_BREAK, 30))

    for bit in vis_bits:
        spec.append((FREQ_LOGIC_1 if bit == 1 else FREQ_LOGIC_0, 30))

    spec.append((FREQ_LOGIC_1 if parity_bit == 1 else FREQ_LOGIC_0, 30))
    spec.append((FREQ_BREAK, 30))

    # 3. 图像数据部分
    pixel_time = 0.19

    for i in range(0, HEIGHT, 2):
        spec.append((FREQ_SYNC, 20.0))
        spec.append((FREQ_BLACK, 2.08))

        for j in range(WIDTH):
            spec.append((FREQ_BLACK + (Y[i,j] / 255.0) * FREQ_RANGE, pixel_time))
        for j in range(WIDTH):
            avg_cr = (Cr[i,j] + Cr[i+1,j]) / 2.0
            spec.append((FREQ_BLACK + (avg_cr / 255.0) * FREQ_RANGE, pixel_time))
        for j in range(WIDTH):
            avg_cb = (Cb[i,j] + Cb[i+1,j]) / 2.0
            spec.append((FREQ_BLACK + (avg_cb / 255.0) * FREQ_RANGE, pixel_time))
        for j in range(WIDTH):
            spec.append((FREQ_BLACK + (Y[i+1,j] / 255.0) * FREQ_RANGE, pixel_time))

    return spec


def synthesize_waveform(spec) -> np.ndarray:
    """使用累计相位积分生成平滑的 FM 波形"""
    total_duration_ms = sum(item[1] for item in spec)
    total_samples = int(np.round(total_duration_ms / 1000.0 * SAMPLE_RATE))

    freq_array = np.zeros(total_samples, dtype=np.float64)
    curr_idx = 0
    cum_time = 0.0

    for freq, dur in spec:
        cum_time += dur
        next_idx = int(np.round(cum_time / 1000.0 * SAMPLE_RATE))
        if next_idx > curr_idx:
            freq_array[curr_idx:next_idx] = freq
        curr_idx = next_idx

    phase_deltas = (2.0 * np.pi * freq_array) / SAMPLE_RATE
    phases = np.cumsum(phase_deltas)
    return AMPLITUDE * np.sin(phases)


# ============== 信号挑战器 ==============
class SignalChallenger:
    """信号挑战器：为音频信号添加各种现实世界的干扰"""

    @staticmethod
    def add_awgn(waveform, snr_db) -> np.ndarray:
        """添加高斯白噪声 (AWGN)"""
        sig_avg_watts = np.mean(waveform ** 2)
        sig_avg_db = 10 * np.log10(sig_avg_watts)
        noise_avg_db = sig_avg_db - snr_db
        noise_avg_watts = 10 ** (noise_avg_db / 10)

        noise = np.random.normal(0, np.sqrt(noise_avg_watts), len(waveform))
        return waveform + noise

    @staticmethod
    def add_frequency_offset(waveform, offset_hz, sr) -> np.ndarray:
        """模拟载波频偏 (Carrier Shift)"""
        analytic_signal = hilbert(waveform)
        t = np.arange(len(waveform)) / sr
        offset_signal = analytic_signal * np.exp(1j * 2 * np.pi * offset_hz * t)
        return np.real(offset_signal)

    @staticmethod
    def add_timing_drift(waveform, drift_ppm) -> np.ndarray:
        """模拟定时漂移 (Clock Drift / Slant)"""
        num_samples = len(waveform)
        new_indices = np.linspace(0, num_samples - 1, int(num_samples * (1 + drift_ppm / 1e6)))
        return np.interp(new_indices, np.arange(num_samples), waveform)

    @staticmethod
    def add_fading(waveform, sr, speed=0.5, depth=0.7) -> np.ndarray:
        """模拟衰落 (QSB / Fading)"""
        t = np.arange(len(waveform)) / sr
        envelope = 1.0 - (depth * 0.5 * (1 + np.sin(2 * np.pi * speed * t)))
        return waveform * envelope

    @staticmethod
    def add_impulse_noise(waveform, probability=0.0001) -> np.ndarray:
        """模拟脉冲干扰 (Lightning / Static crashes)"""
        noise = np.zeros(len(waveform))
        pulses = np.random.rand(len(waveform)) < probability
        noise[pulses] = np.random.uniform(-1, 1, np.sum(pulses))
        return waveform + noise

    @staticmethod
    def add_multipath(waveform, sr, delays_ms: List[float] = None, attenuations: List[float] = None) -> np.ndarray:
        """
        模拟多径效应 (Multipath / Echo)
        HF 频段常见信号反射导致的重影现象

        delays_ms: 多径延迟列表 (毫秒)，典型值 [5, 15, 30] ms
        attenuations: 各路径衰减 dB，典型值 [-6, -12, -20] dB
        """
        if delays_ms is None:
            delays_ms = [10.0]  # 默认 10ms 延迟
        if attenuations is None:
            attenuations = [-10.0]  # 默认 -10dB 衰减

        output = waveform.copy()
        sr_samples_per_ms = sr / 1000.0

        for delay_ms, attn_db in zip(delays_ms, attenuations):
            delay_samples = int(delay_ms * sr_samples_per_ms)
            attenuation = 10 ** (attn_db / 20.0)  # dB 转线性

            if delay_samples < len(waveform):
                # 创建延迟副本并叠加
                delayed = np.zeros_like(waveform)
                delayed[delay_samples:] = waveform[:-delay_samples] * attenuation
                output += delayed

        return output


# ============== 解码器运行器 ==============
class SSTVTestRunner:
    def __init__(self, sample_rate):
        self.decoder = sstv_decoder.Decoder(sample_rate)
        self.image_buffer = None
        self.current_mode = None
        self.lines_decoded = 0
        self.decoded_pixels = []

        self.decoder.set_on_mode_detected_callback(self.on_mode_detected)
        self.decoder.set_on_line_decoded_callback(self.on_line_decoded)
        self.decoder.set_on_image_complete_callback(self.on_image_complete)

    def on_mode_detected(self, mode):
        self.current_mode = mode
        self.image_buffer = np.zeros((mode.height, mode.width, 3), dtype=np.uint8)

    def on_line_decoded(self, line_idx, pixels):
        self.lines_decoded += 1
        if self.image_buffer is not None and 0 <= line_idx < self.image_buffer.shape[0]:
            row_data = np.array([[p.r, p.g, p.b] for p in pixels], dtype=np.uint8)
            self.image_buffer[line_idx, :len(pixels)] = row_data

    def on_image_complete(self, width, height):
        pass

    def reset(self):
        """重置解码器状态，用于下一次独立解码"""
        self.image_buffer = None
        self.current_mode = None
        self.lines_decoded = 0
        self.decoded_pixels = []
        # 重新创建解码器以清除内部状态
        self.decoder = sstv_decoder.Decoder(SAMPLE_RATE)
        self.decoder.set_on_mode_detected_callback(self.on_mode_detected)
        self.decoder.set_on_line_decoded_callback(self.on_line_decoded)
        self.decoder.set_on_image_complete_callback(self.on_image_complete)

    def decode(self, audio_data) -> np.ndarray:
        """解码音频数据并返回图像"""
        # 每次解码前重置状态
        self.reset()

        chunk_size = int(SAMPLE_RATE * 0.02)
        for i in range(0, len(audio_data), chunk_size):
            chunk = audio_data[i : i + chunk_size]
            self.decoder.process(chunk)

        return self.image_buffer


# ============== 图像质量评估 ==============
def calculate_psnr(img1: np.ndarray, img2: np.ndarray) -> float:
    """计算峰值信噪比 (PSNR)"""
    mse = np.mean((img1.astype(np.float32) - img2.astype(np.float32)) ** 2)
    if mse == 0:
        return float('inf')
    max_pixel = 255.0
    psnr = 20 * np.log10(max_pixel / np.sqrt(mse))
    return psnr


def calculate_ssim(img1: np.ndarray, img2: np.ndarray) -> float:
    """计算结构相似性 (SSIM) - 简化版本"""
    C1 = (0.01 * 255) ** 2
    C2 = (0.03 * 255) ** 2

    img1 = img1.astype(np.float64)
    img2 = img2.astype(np.float64)

    mu1 = np.mean(img1)
    mu2 = np.mean(img2)
    sigma1_sq = np.var(img1)
    sigma2_sq = np.var(img2)
    sigma12 = np.mean((img1 - mu1) * (img2 - mu2))

    ssim = ((2 * mu1 * mu2 + C1) * (2 * sigma12 + C2)) / \
           ((mu1**2 + mu2**2 + C1) * (sigma1_sq + sigma2_sq + C2))

    return ssim


def calculate_image_metrics(original: np.ndarray, decoded: np.ndarray) -> Dict:
    """计算图像质量指标"""
    if decoded is None:
        return {
            'psnr': None,
            'ssim': None,
            'mae': None,
            'completion_rate': 0,
            'success': False
        }

    # 处理可能的尺寸不匹配
    h1, w1 = original.shape[:2]
    h2, w2 = decoded.shape[:2]
    min_h, min_w = min(h1, h2), min(w1, w2)

    orig_crop = original[:min_h, :min_w]
    dec_crop = decoded[:min_h, :min_w]

    psnr = calculate_psnr(orig_crop, dec_crop)
    ssim = calculate_ssim(orig_crop, dec_crop)
    mae = np.mean(np.abs(orig_crop.astype(np.float32) - dec_crop.astype(np.float32)))
    completion_rate = h2 / HEIGHT if h2 > 0 else 0

    return {
        'psnr': psnr,
        'ssim': ssim,
        'mae': mae,
        'completion_rate': completion_rate,
        'decoded_size': (w2, h2),
        'success': completion_rate > 0.5
    }


# ============== 综合测试执行器 ==============
class ComprehensiveTester:
    def __init__(self, input_image: str):
        self.input_image = input_image
        self.original_img = None
        self.runner = SSTVTestRunner(SAMPLE_RATE)
        self.results = []

    def setup(self):
        """设置测试环境"""
        if os.path.exists(OUTPUT_DIR):
            shutil.rmtree(OUTPUT_DIR)
        os.makedirs(OUTPUT_DIR)
        os.makedirs(os.path.join(OUTPUT_DIR, "decoded_images"))
        os.makedirs(os.path.join(OUTPUT_DIR, "raw_files"))
        os.makedirs(os.path.join(OUTPUT_DIR, "wav_files"))

        self.original_img = np.array(Image.open(self.input_image).convert('RGB'))
        print(f"[+] 原始图像：{self.original_img.shape}")

    def generate_base_signal(self) -> np.ndarray:
        """生成基础 PD120 信号"""
        Y, Cr, Cb = rgb_to_ycrcb(self.input_image)
        spec = generate_tone_sequence(Y, Cr, Cb)
        return synthesize_waveform(spec)

    def run_single_test(self, test_name: str, challenges: Dict) -> Dict:
        """运行单个测试"""
        print(f"\n{'='*60}")
        print(f"测试：{test_name}")
        print(f"干扰参数：{challenges}")

        # 生成基础信号
        waveform = self.generate_base_signal()

        # 应用干扰
        if 'snr_db' in challenges:
            waveform = SignalChallenger.add_awgn(waveform, challenges['snr_db'])
        if 'freq_offset' in challenges:
            waveform = SignalChallenger.add_frequency_offset(
                waveform, challenges['freq_offset'], SAMPLE_RATE)
        if 'drift_ppm' in challenges:
            waveform = SignalChallenger.add_timing_drift(waveform, challenges['drift_ppm'])
        if 'fading' in challenges:
            waveform = SignalChallenger.add_fading(
                waveform, SAMPLE_RATE,
                challenges['fading'].get('speed', 0.5),
                challenges['fading'].get('depth', 0.5))
        if 'impulse_prob' in challenges:
            waveform = SignalChallenger.add_impulse_noise(waveform, challenges['impulse_prob'])
        if 'multipath' in challenges:
            mp_config = challenges['multipath']
            waveform = SignalChallenger.add_multipath(
                waveform, SAMPLE_RATE,
                mp_config.get('delays_ms'),
                mp_config.get('attenuations'))

        # 限幅
        waveform = np.clip(waveform, -1.0, 1.0)
        
        # 保存 RAW 文件
        raw_path = os.path.join(OUTPUT_DIR, "raw_files", f"{test_name}.raw")
        waveform_padded = np.pad(waveform, (0, 2048), mode='constant')
        with open(raw_path, 'wb') as f:
            f.write(waveform_padded.astype(np.float32).tobytes())
            
        # 保存 WAV
        wav_path = os.path.join(OUTPUT_DIR, "wav_files", f"{test_name}.wav")
        wav_data = (waveform_padded * 32767).astype(np.int16)
        wavfile.write(wav_path, SAMPLE_RATE, wav_data)

        # 解码
        decoded_img = self.runner.decode(waveform_padded)

        # 保存解码图像
        decoded_path = None
        if decoded_img is not None:
            decoded_path = os.path.join(OUTPUT_DIR, "decoded_images", f"{test_name}.png")
            Image.fromarray(decoded_img).save(decoded_path)

        # 计算指标
        metrics = calculate_image_metrics(self.original_img, decoded_img)
        metrics['test_name'] = test_name
        metrics['challenges'] = challenges
        metrics['decoded_path'] = decoded_path

        # 打印结果
        if metrics['success']:
            print(f"  [✓] PSNR: {metrics['psnr']:.2f} dB, SSIM: {metrics['ssim']:.4f}, "
                  f"MAE: {metrics['mae']:.2f}, 完成率：{metrics['completion_rate']*100:.1f}%")
        else:
            print(f"  [x] 解码失败或完成率过低：{metrics['completion_rate']*100:.1f}%")

        return metrics

    def run_snr_sweep(self) -> List[Dict]:
        """SNR 扫描测试 - 扩大范围至极限"""
        print("\n" + "="*60)
        print("【SNR 扫描测试】(扩大范围)")
        results = []
        # 扩大范围：从 50dB 到 -20dB，步长更细
        snr_values = [50, 30, 20, 15, 12, 10, 8, 6, 5, 4, 3, 2, 1, 0, -2, -5, -10, -15, -20]

        for snr in snr_values:
            m = self.run_single_test(f"snr_{snr}dB", {'snr_db': snr})
            results.append(m)
        return results

    def run_frequency_offset_sweep(self) -> List[Dict]:
        """频偏扫描测试 - 扩大范围至极限"""
        print("\n" + "="*60)
        print("【频偏扫描测试】(扩大范围)")
        results = []
        # 扩大范围：正负各 1000Hz，PD120 信号范围是 1500-2300Hz，测试解码器跟踪能力
        offset_values = [
            0, 50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000,
            -50, -100, -150, -200, -250, -300, -400, -500, -600, -700, -800
        ]

        for offset in offset_values:
            m = self.run_single_test(f"freq_offset_{offset}Hz", {'freq_offset': offset})
            results.append(m)
        return results

    def run_timing_drift_sweep(self) -> List[Dict]:
        """时钟漂移扫描测试 - 扩大范围至极限"""
        print("\n" + "="*60)
        print("【时钟漂移扫描测试】(扩大范围)")
        results = []
        # 扩大范围：测试更高 ppm 值
        drift_values = [
            0, 10, 25, 50, 75, 100, 150, 200, 300, 400, 500, 750, 1000, 1500, 2000,
            -10, -25, -50, -75, -100, -150, -200, -300, -400, -500
        ]

        for drift in drift_values:
            m = self.run_single_test(f"drift_{drift}ppm", {'drift_ppm': drift})
            results.append(m)
        return results

    def run_fading_sweep(self) -> List[Dict]:
        """衰落扫描测试 - 扩大范围"""
        print("\n" + "="*60)
        print("【衰落扫描测试】(扩大范围)")
        results = []
        fading_configs = [
            {'speed': 0.1, 'depth': 0.2},
            {'speed': 0.1, 'depth': 0.5},
            {'speed': 0.1, 'depth': 0.8},
            {'speed': 0.1, 'depth': 0.95},  # 深度衰落
            {'speed': 0.5, 'depth': 0.2},
            {'speed': 0.5, 'depth': 0.5},
            {'speed': 0.5, 'depth': 0.8},
            {'speed': 0.5, 'depth': 0.95},
            {'speed': 1.0, 'depth': 0.3},
            {'speed': 1.0, 'depth': 0.6},
            {'speed': 1.0, 'depth': 0.9},
            {'speed': 2.0, 'depth': 0.5},  # 快速衰落
            {'speed': 2.0, 'depth': 0.8},
        ]

        for i, config in enumerate(fading_configs):
            m = self.run_single_test(f"fading_s{config['speed']}_d{config['depth']}",
                                     {'fading': config})
            results.append(m)
        return results

    def run_multipath_sweep(self) -> List[Dict]:
        """多径效应扫描测试 - HF 频段典型现象"""
        print("\n" + "="*60)
        print("【多径效应扫描测试】(HF 频段典型)")
        results = []

        # 不同延迟和衰减组合
        multipath_configs = [
            # 单路径延迟
            {'delays_ms': [5.0], 'attenuations': [-6.0]},    # 短延迟强反射
            {'delays_ms': [5.0], 'attenuations': [-10.0]},
            {'delays_ms': [5.0], 'attenuations': [-20.0]},   # 短延迟弱反射
            {'delays_ms': [10.0], 'attenuations': [-6.0]},   # 中等延迟
            {'delays_ms': [10.0], 'attenuations': [-10.0]},
            {'delays_ms': [10.0], 'attenuations': [-20.0]},
            {'delays_ms': [20.0], 'attenuations': [-6.0]},   # 长延迟
            {'delays_ms': [20.0], 'attenuations': [-10.0]},
            {'delays_ms': [20.0], 'attenuations': [-20.0]},
            {'delays_ms': [30.0], 'attenuations': [-10.0]},  # 很长延迟
            {'delays_ms': [50.0], 'attenuations': [-15.0]},  # 极端延迟
            # 双路径反射
            {'delays_ms': [5.0, 15.0], 'attenuations': [-10.0, -15.0]},
            {'delays_ms': [5.0, 15.0], 'attenuations': [-6.0, -12.0]},
            {'delays_ms': [10.0, 30.0], 'attenuations': [-8.0, -16.0]},
            # 三路径反射 (复杂多径)
            {'delays_ms': [5.0, 15.0, 30.0], 'attenuations': [-6.0, -10.0, -18.0]},
            {'delays_ms': [5.0, 15.0, 30.0], 'attenuations': [-10.0, -15.0, -25.0]},
        ]

        for i, config in enumerate(multipath_configs):
            delays_str = '_'.join([f"{d}ms" for d in config['delays_ms']])
            m = self.run_single_test(f"multipath_{delays_str}",
                                     {'multipath': config})
            results.append(m)
        return results

    def run_combined_stress_test(self) -> List[Dict]:
        """组合压力测试 - 增加更多级别和多径效应"""
        print("\n" + "="*60)
        print("【组合压力测试】(扩展版)")
        results = []

        # 轻度干扰
        m = self.run_single_test("combined_light", {
            'snr_db': 30,
            'freq_offset': 50,
            'drift_ppm': 25
        })
        results.append(m)

        # 中度干扰
        m = self.run_single_test("combined_medium", {
            'snr_db': 20,
            'freq_offset': 100,
            'drift_ppm': 50,
            'fading': {'speed': 0.3, 'depth': 0.4}
        })
        results.append(m)

        # 中重度干扰
        m = self.run_single_test("combined_medium_heavy", {
            'snr_db': 15,
            'freq_offset': 200,
            'drift_ppm': 100,
            'fading': {'speed': 0.5, 'depth': 0.6},
            'multipath': {'delays_ms': [10.0], 'attenuations': [-15.0]}
        })
        results.append(m)

        # 重度干扰
        m = self.run_single_test("combined_heavy", {
            'snr_db': 12,
            'freq_offset': 300,
            'drift_ppm': 200,
            'fading': {'speed': 0.5, 'depth': 0.7},
            'impulse_prob': 0.00005,
            'multipath': {'delays_ms': [5.0, 15.0], 'attenuations': [-10.0, -15.0]}
        })
        results.append(m)

        # 极限干扰
        m = self.run_single_test("combined_extreme", {
            'snr_db': 8,
            'freq_offset': 400,
            'drift_ppm': 500,
            'fading': {'speed': 0.8, 'depth': 0.8},
            'impulse_prob': 0.0001,
            'multipath': {'delays_ms': [5.0, 15.0, 30.0], 'attenuations': [-6.0, -10.0, -18.0]}
        })
        results.append(m)

        # 地狱模式干扰
        m = self.run_single_test("combined_hell", {
            'snr_db': 5,
            'freq_offset': 600,
            'drift_ppm': 1000,
            'fading': {'speed': 1.0, 'depth': 0.95},
            'impulse_prob': 0.0002,
            'multipath': {'delays_ms': [10.0, 30.0, 50.0], 'attenuations': [-8.0, -12.0, -20.0]}
        })
        results.append(m)

        return results

    def run_all_tests(self):
        """运行所有测试"""
        self.setup()

        print("\n" + "="*60)
        print("PD120 SSTV 解码器综合极限测试 (扩展版)")
        print(f"原始图像：{self.input_image}")
        print(f"输出目录：{OUTPUT_DIR}")
        print("="*60)

        # 基准测试（无干扰）
        self.results.append(self.run_single_test("baseline", {}))

        # 各项扫描测试
        self.results.extend(self.run_snr_sweep())
        self.results.extend(self.run_frequency_offset_sweep())
        self.results.extend(self.run_timing_drift_sweep())
        self.results.extend(self.run_fading_sweep())
        self.results.extend(self.run_multipath_sweep())  # 新增多径测试
        self.results.extend(self.run_combined_stress_test())

        # 生成报告
        self.generate_report()

        return self.results

    def generate_report(self):
        """生成测试报告"""
        print("\n" + "="*60)
        print("生成测试报告...")

        # 分析极限
        snr_limits = [r for r in self.results if 'snr_db' in r.get('challenges', {})]
        freq_limits = [r for r in self.results if 'freq_offset' in r.get('challenges', {})]
        drift_limits = [r for r in self.results if 'drift_ppm' in r.get('challenges', {})]
        multipath_limits = [r for r in self.results if 'multipath' in r.get('challenges', {})]

        # 找出成功解码的极限值
        snr_success = [r for r in snr_limits if r['success']]
        freq_success = [r for r in freq_limits if r['success']]
        drift_success = [r for r in drift_limits if r['success']]
        multipath_success = [r for r in multipath_limits if r['success']]

        report = []
        report.append("=" * 60)
        report.append("PD120 SSTV 解码器极限测试报告")
        report.append(f"生成时间：{datetime.now().isoformat()}")
        report.append("=" * 60)

        report.append("\n【极限指标】")

        if snr_success:
            min_snr = min(r['challenges']['snr_db'] for r in snr_success)
            report.append(f"  最低可解码 SNR: {min_snr} dB")
        else:
            report.append("  最低可解码 SNR: 无法解码任何 SNR 测试")

        if freq_success:
            max_pos_offset = max((r['challenges']['freq_offset'] for r in freq_success
                                  if r['challenges']['freq_offset'] > 0), default=0)
            max_neg_offset = min((r['challenges']['freq_offset'] for r in freq_success
                                  if r['challenges']['freq_offset'] < 0), default=0)
            report.append(f"  最大正向频偏：+{max_pos_offset} Hz")
            report.append(f"  最大负向频偏：{max_neg_offset} Hz")
        else:
            report.append("  频偏容限：无法解码任何频偏测试")

        if drift_success:
            max_pos_drift = max((r['challenges']['drift_ppm'] for r in drift_success
                                 if r['challenges']['drift_ppm'] > 0), default=0)
            max_neg_drift = min((r['challenges']['drift_ppm'] for r in drift_success
                                 if r['challenges']['drift_ppm'] < 0), default=0)
            report.append(f"  最大正向漂移：+{max_pos_drift} ppm")
            report.append(f"  最大负向漂移：{max_neg_drift} ppm")
        else:
            report.append("  漂移容限：无法解码任何漂移测试")

        # 多径效应分析
        if multipath_success:
            report.append(f"  多径效应容限：成功解码 {len(multipath_success)}/{len(multipath_limits)} 组")
        else:
            report.append("  多径效应容限：无法解码任何多径测试")

        report.append("\n【详细结果】")
        report.append(f"{'测试名称':<35} {'PSNR':>10} {'SSIM':>10} {'MAE':>10} {'完成率':>10} {'状态':>8}")
        report.append("-" * 95)

        for r in self.results:
            status = "OK" if r['success'] else "FAIL"
            psnr_str = f"{r['psnr']:.2f}" if r['psnr'] else "N/A"
            ssim_str = f"{r['ssim']:.4f}" if r['ssim'] else "N/A"
            mae_str = f"{r['mae']:.2f}" if r['mae'] else "N/A"
            comp_str = f"{r['completion_rate']*100:.1f}%"
            report.append(f"{r['test_name']:<35} {psnr_str:>10} {ssim_str:>10} {mae_str:>10} {comp_str:>10} {status:>8}")

        # 保存报告
        report_path = os.path.join(OUTPUT_DIR, "test_report.txt")
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(report))

        # 保存 JSON 结果
        json_path = os.path.join(OUTPUT_DIR, "test_results.json")

        # 清理不可 JSON 序列化的字段
        json_results = []
        for r in self.results:
            jr = r.copy()
            json_results.append(jr)

        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(json_results, f, indent=2, default=str)

        print(f"\n[+] 报告已保存至：{report_path}")
        print(f"[+] JSON 结果已保存至：{json_path}")

        # 打印报告
        print('\n' + '\n'.join(report))


# ============== 主函数 ==============
def main():
    if len(sys.argv) > 1:
        input_image = sys.argv[1]
    else:
        input_image = 'No-Signal-TV_small.png'

    if not os.path.exists(input_image):
        print(f"错误：找不到文件 {input_image}")
        sys.exit(1)

    print(f"[*] 使用图像：{input_image}")

    tester = ComprehensiveTester(input_image)
    tester.run_all_tests()


if __name__ == "__main__":
    main()
