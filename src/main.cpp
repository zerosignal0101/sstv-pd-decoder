// src/main.cpp
#include "sstv_decoder.h"
#include "sstv_types.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

using namespace sstv;

// --- 全局变量用于存储图像 ---
const int PD120_WIDTH = 640;
const int PD120_HEIGHT = 496;
std::vector<Pixel> g_image_buffer; // 存储整个图像的像素

int main() {
    float DEMO_SAMPLE_RATE = 44100;

    // 预先分配内存大小
    g_image_buffer.resize(PD120_WIDTH * PD120_HEIGHT);

    Decoder sstv_decoder(DEMO_SAMPLE_RATE);

    // 设置 Mode Detected 回调
    sstv_decoder.set_on_mode_detected_callback([](const SSTVMode& mode) {
        std::cout << "MAIN: Mode Detected! Name: " << mode.name << ", VIS: " << mode.vis_code << std::endl;
    });

    // 设置 Line Decoded 回调：将每一行的像素数据复制到全局缓冲区
    sstv_decoder.set_on_line_decoded_callback([](int line_idx, const std::vector<Pixel>& pixels) {
        if (line_idx < PD120_HEIGHT && pixels.size() == PD120_WIDTH) {
            // 计算在全局缓冲区中的偏移量
            size_t offset = line_idx * PD120_WIDTH;

            // 将当前行像素拷贝到全局缓冲区
            // 使用 std::copy 或者 memcpy 都可以，因为是 POD 类型
            std::copy(pixels.begin(), pixels.end(), g_image_buffer.begin() + offset);

            // 可以在此处打印进度，例如每48行打印一次
            if (line_idx % 48 == 0) {
                std::cout << "Stored line " << line_idx << " to buffer." << std::endl;
            }
        } else {
            std::cerr << "Warning: Line index or pixel count mismatch!" << std::endl;
        }
    });

    // 设置 Image Complete 回调：保存为 .raw 文件
    sstv_decoder.set_on_image_complete_callback([](int width, int height) {
        std::cout << "MAIN: Image Complete! " << width << "x" << height << std::endl;
        std::cout << "Saving image to 'output.raw'..." << std::endl;

        std::ofstream outfile("output.raw", std::ios::binary | std::ios::out);
        if (outfile.is_open()) {
            // 直接写入内存中的二进制数据
            // Pixel 结构体是 {uint8_t r, g, b}，占 3 字节
            outfile.write(reinterpret_cast<const char*>(g_image_buffer.data()),
                         g_image_buffer.size() * sizeof(Pixel));
            outfile.close();
            std::cout << "File saved successfully." << std::endl;
        } else {
            std::cerr << "Error: Could not open output.raw for writing." << std::endl;
        }
    });

    // --- 模拟传输 (读取音频文件) ---
    std::vector<float> full_audio_signal;
    const char* filename = R"(D:\C-Codes\Radio-HAM\sstv-toolkit\pd120_44100Hz.raw)";

    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filename << std::endl;
        return 1;
    }

    std::streamsize file_size = file.tellg();
    std::size_t num_samples = file_size / sizeof(float);

    file.seekg(0, std::ios::beg);
    full_audio_signal.resize(num_samples);
    if (!file.read(reinterpret_cast<char*>(full_audio_signal.data()), file_size)) {
        std::cerr << "Failed to read file data" << std::endl;
        return 1;
    }
    else {
        std::cout << "Read data completed (" << num_samples << " samples)." << std::endl;
    }
    file.close();

    // 实时处理模拟
    const size_t chunk_size = 2048;
    std::cout << "\nStarting SSTV Demodulation...\n" << std::endl;

    for (size_t i = 0; i < full_audio_signal.size(); i += chunk_size) {
        size_t current_chunk_size = std::min(chunk_size, full_audio_signal.size() - i);
        if (current_chunk_size == 0) break;

        sstv_decoder.process(full_audio_signal.data() + i, current_chunk_size);
    }

    std::cout << "\nSimulation Complete." << std::endl;
    return 0;
}
