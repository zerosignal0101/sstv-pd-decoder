import numpy as np
from PIL import Image
import scipy.io.wavfile as wavfile
from scipy.signal import hilbert
import librosa
import os
import math

# --- 常量定义 ---
SAMPLE_RATE = 44100
TARGET_SR_RAW = 11025
AMPLITUDE = 0.8
WIDTH, HEIGHT = 640, 496

# 频率定义 (Hz)
FREQ_SYNC = 1200
FREQ_BLACK = 1500
FREQ_WHITE = 2300
FREQ_RANGE = FREQ_WHITE - FREQ_BLACK # 800Hz
FREQ_LEADER_BURST = 1900
FREQ_LOGIC_0 = 1300
FREQ_LOGIC_1 = 1100
FREQ_BREAK = 1200

def rgb_to_ycrcb(img_path):
    """读取图像并转换为 YCrCb (BT.601 Limited Range)"""
    img = Image.open(img_path).convert('RGB')
    data = np.array(img, dtype=np.float32)
    
    R, G, B = data[:,:,0], data[:,:,1], data[:,:,2]
    
    # BT.601 转换公式
    Y  = 16.0  + (0.256789 * R + 0.504129 * G + 0.097906 * B)
    Cb = 128.0 + (-0.148223 * R - 0.290992 * G + 0.439215 * B)
    Cr = 128.0 + (0.439215 * R - 0.367789 * G - 0.071426 * B)
    
    # 限制范围 (虽然 SSTV 通常使用 0-255 映射到 1500-2300)
    # 这里的 Y/Cr/Cb 将被映射到 1500-2300Hz
    return Y, Cr, Cb

def generate_tone_sequence(Y, Cr, Cb):
    """构建 PD120 的所有音调规格 (频率, 持续时间ms)"""
    spec = []
    
    # 1. Preamble (800ms)
    for f in [1900, 1500, 1900, 1500, 2300, 1500, 2300, 1500]:
        spec.append((f, 100))
        
    # 2. VIS Header (PD120 = 95)
    # VIS 95 -> 1011111 binary -> 低位先行: 1111101
    vis_bits = [1, 1, 1, 1, 1, 0, 1]
    parity_bit = 0 if sum(vis_bits) % 2 == 0 else 1
    
    spec.append((FREQ_LEADER_BURST, 300)) # Leader 1
    spec.append((FREQ_BREAK, 10))         # Break
    spec.append((FREQ_LEADER_BURST, 300)) # Leader 2
    spec.append((FREQ_BREAK, 30))         # Start Bit
    
    for bit in vis_bits:
        spec.append((FREQ_LOGIC_1 if bit == 1 else FREQ_LOGIC_0, 30))
        
    spec.append((FREQ_LOGIC_1 if parity_bit == 1 else FREQ_LOGIC_0, 30)) # Parity
    spec.append((FREQ_BREAK, 30))         # Stop Bit

    # 3. 图像数据部分 (每2行一组)
    # 结构: Sync(20ms) -> Porch(2.08ms) -> Y1 -> Cr -> Cb -> Y2
    pixel_time = 0.19 # ms
    
    for i in range(0, HEIGHT, 2):
        spec.append((FREQ_SYNC, 20.0))
        spec.append((FREQ_BLACK, 2.08))
        
        # Y1
        for j in range(WIDTH):
            spec.append((FREQ_BLACK + (Y[i,j] / 255.0) * FREQ_RANGE, pixel_time))
        # Cr (Average of 2 lines)
        for j in range(WIDTH):
            avg_cr = (Cr[i,j] + Cr[i+1,j]) / 2.0
            spec.append((FREQ_BLACK + (avg_cr / 255.0) * FREQ_RANGE, pixel_time))
        # Cb (Average of 2 lines)
        for j in range(WIDTH):
            avg_cb = (Cb[i,j] + Cb[i+1,j]) / 2.0
            spec.append((FREQ_BLACK + (avg_cb / 255.0) * FREQ_RANGE, pixel_time))
        # Y2
        for j in range(WIDTH):
            spec.append((FREQ_BLACK + (Y[i+1,j] / 255.0) * FREQ_RANGE, pixel_time))
            
    return spec

def synthesize_waveform(spec):
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
        
    # 相位积分: phase = sum(2 * pi * f * dt)
    phase_deltas = (2.0 * np.pi * freq_array) / SAMPLE_RATE
    phases = np.cumsum(phase_deltas)
    return AMPLITUDE * np.sin(phases)
  
class SignalChallenger:
    """信号挑战器：为音频信号添加各种现实世界的干扰"""
    
    @staticmethod
    def add_awgn(waveform, snr_db):
        """添加高斯白噪声 (AWGN)"""
        sig_avg_watts = np.mean(waveform ** 2)
        sig_avg_db = 10 * np.log10(sig_avg_watts)
        noise_avg_db = sig_avg_db - snr_db
        noise_avg_watts = 10 ** (noise_avg_db / 10)
        
        noise = np.random.normal(0, np.sqrt(noise_avg_watts), len(waveform))
        return waveform + noise

    @staticmethod
    def add_frequency_offset(waveform, offset_hz, sr):
        """
        模拟载波频偏 (Carrier Shift)
        在 SSB 通信中，调谐不准会导致所有频率平移，造成图像色彩偏色。
        """
        # 使用希尔伯特变换进行移频 (SSB 调制误差模拟)
        analytic_signal = hilbert(waveform)
        t = np.arange(len(waveform)) / sr
        offset_signal = analytic_signal * np.exp(1j * 2 * np.pi * offset_hz * t)
        return np.real(offset_signal)

    @staticmethod
    def add_timing_drift(waveform, drift_ppm):
        """
        模拟定时漂移 (Clock Drift / Slant)
        ppm: 百万分之几。例如 100ppm 会导致图像明显倾斜。
        """
        num_samples = len(waveform)
        # 实际采样率与标准采样率的微小偏差
        new_indices = np.linspace(0, num_samples - 1, int(num_samples * (1 + drift_ppm / 1e6)))
        return np.interp(new_indices, np.arange(num_samples), waveform)

    @staticmethod
    def add_fading(waveform, sr, speed=0.5, depth=0.7):
        """
        模拟衰落 (QSB / Fading)
        speed: 衰落速度 (Hz)
        depth: 衰落深度 (0-1)
        """
        t = np.arange(len(waveform)) / sr
        # 使用低频正弦波模拟包络起伏
        envelope = 1.0 - (depth * 0.5 * (1 + np.sin(2 * np.pi * speed * t)))
        return waveform * envelope

    @staticmethod
    def add_impulse_noise(waveform, probability=0.0001):
        """模拟脉冲干扰 (Lightning / Static crashes)"""
        noise = np.zeros(len(waveform))
        pulses = np.random.rand(len(waveform)) < probability
        noise[pulses] = np.random.uniform(-1, 1, np.sum(pulses))
        return waveform + noise

def main(input_image):
    output_wav = "pd120_signal_script.wav"
    output_raw = f"pd120_{SAMPLE_RATE}Hz_script.raw"
    
    print(f"[*] 正在处理图像: {input_image}")
    Y, Cr, Cb = rgb_to_ycrcb(input_image)
    
    print("[*] 正在构建音调序列...")
    spec = generate_tone_sequence(Y, Cr, Cb)
    
    print("[*] 正在合成音频...")
    waveform = synthesize_waveform(spec)
    
    # --- 添加挑战项目 ---
    print("[!] 正在注入干扰以挑战解码器...")
    
    # 1. 模拟 50ppm 的时钟误差（这会导致图像向一边倾斜）
    waveform = SignalChallenger.add_timing_drift(waveform, drift_ppm=50)
    
    # 2. 模拟载波偏移 10Hz（这会导致颜色整体偏移，例如偏绿或偏紫）
    waveform = SignalChallenger.add_frequency_offset(waveform, offset_hz=15, sr=SAMPLE_RATE)
    
    # 3. 添加衰落 (信号忽强忽弱)
    waveform = SignalChallenger.add_fading(waveform, sr=SAMPLE_RATE, speed=0.2, depth=0.5)
    
    # 4. 添加噪声 (SNR = 15dB，这是一个挑战性但可读的信号)
    waveform = SignalChallenger.add_awgn(waveform, snr_db=15)
    
    # 5. 添加随机电磁脉冲
    waveform = SignalChallenger.add_impulse_noise(waveform, probability=0.00005)

    # 防止溢出
    waveform = np.clip(waveform, -1.0, 1.0)

    # 保存 WAV
    print(f"[+] 正在保存 WAV: {output_wav}")
    wav_data = (waveform * 32767).astype(np.int16)
    wavfile.write(output_wav, SAMPLE_RATE, wav_data)
    
    # 末尾填充
    waveform_padded = np.pad(waveform, (0, 2048), mode='constant')
    
    print(f"[+] 正在保存 RAW (Float32): {output_raw}")
    with open(output_raw, 'wb') as f:
        f.write(waveform_padded.astype(np.float32).tobytes())
    
    print("[!] 完成！")

if __name__ == "__main__":
    import sys
    image_path = sys.argv[1] if len(sys.argv) > 1 else 'No-Signal-TV_small.png'
    if os.path.exists(image_path):
        main(image_path)
    else:
        print(f"错误: 找不到文件 {image_path}")
