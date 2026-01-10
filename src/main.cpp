#include "sstv_decoder.h"
#include "sstv_types.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <random> // For simulating noise

int main() {
    using namespace sstv;

    float DEMO_SAMPLE_RATE = 11025;

    Decoder sstv_decoder(DEMO_SAMPLE_RATE);

    // Set up callbacks
    sstv_decoder.set_on_mode_detected_callback([](const SSTVMode& mode) {
        std::cout << "MAIN: Mode Detected! Name: " << mode.name << ", VIS: " << mode.vis_code << std::endl;
        });

    sstv_decoder.set_on_line_decoded_callback([](int line_idx, const std::vector<Pixel>& pixels) {
        std::cout << "MAIN: Line Decoded: " << line_idx << ", Pixels: " << pixels.size() << " (R:" << (int)pixels[0].r << ", G:" << (int)pixels[0].g << ", B:" << (int)pixels[0].b << " ...)" << std::endl;
        });

    sstv_decoder.set_on_image_complete_callback([](int width, int height) {
        std::cout << "MAIN: Image Complete! " << width << "x" << height << std::endl;
        });

    // --- Simulate an SSTV transmission ---
    std::vector<float> full_audio_signal;

    const char* filename = R"(D:\C-Codes\Radio-HAM\sstv-toolkit\pd120_11025Hz.raw)";

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
        std::cout << "Read data completed." << std::endl;
    }
    file.close();
    
    // Simulate real-time processing
    const size_t chunk_size = 2048; // Process in small blocks
    std::cout << "\nStarting SSTV Demodulation Simulation...\n" << std::endl;

    for (size_t i = 0; i < full_audio_signal.size(); i += chunk_size) {
        size_t current_chunk_size = std::min(chunk_size, full_audio_signal.size() - i);
        if (current_chunk_size == 0) break;

        sstv_decoder.process(full_audio_signal.data() + i, current_chunk_size);

        // Simulate a small delay for real-time processing, or just run at full speed
        // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    }

    std::cout << "\nSimulation Complete." << std::endl;

    return 0;
}
