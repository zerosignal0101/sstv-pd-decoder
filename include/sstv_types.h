#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <map>
#include <numeric> // For std::iota

namespace sstv {

// Global Constants for SSTV Modes
constexpr double SYNC_FREQ = 1200.0;
constexpr double BLACK_FREQ = 1500.0;
constexpr double WHITE_FREQ = 2300.0;
constexpr double FREQ_RANGE = WHITE_FREQ - BLACK_FREQ; // 800 Hz

constexpr double VIS_LOGIC_0_FREQ = 1300.0;
constexpr double VIS_LOGIC_1_FREQ = 1100.0;
constexpr double VIS_START_STOP_FREQ = 1200.0;
constexpr double VIS_LEADER_BURST_FREQ = 1900.0;
constexpr double VIS_BREAK_FREQ = 1200.0;

constexpr double VIS_LEADER_BURST_DURATION_MS = 300.0; // Every 300 ms
constexpr double VIS_BREAK_DURATION_MS = 10.0;
constexpr double VIS_BIT_DURATION_MS = 30.0; // Start, Stop, Data, Parity bits

// Preamble tones (from the problem description)
struct PreambleTone {
    double frequency;
    double duration_ms;
};
const std::vector<PreambleTone> DEFAULT_PREAMBLE_TONES = {
    {1900, 100}, {1500, 100}, {1900, 100}, {1500, 100},
    {2300, 100}, {1500, 100}, {2300, 100}, {1500, 100}
};

// --- Image Data Structures ---
struct Pixel {
    uint8_t r, g, b;
};

// PD120 Specifics
struct PD120ModeConfig {
    static constexpr int VIS_CODE = 95; // 0x5F, which is 01011111 binary, LSB first: 11111010
    static constexpr int WIDTH = 640;
    static constexpr int HEIGHT = 496;
    static constexpr double SYNC_DURATION_MS = 20.0;
    static constexpr double PORCH_DURATION_MS = 2.08;
    static constexpr double SEGMENT_DURATION_MS = 121.6; // Y1, R-Y, B-Y, Y2 segments

    static constexpr double TOTAL_GROUP_DURATION_MS = 
        SYNC_DURATION_MS + PORCH_DURATION_MS + (4 * SEGMENT_DURATION_MS); // 508.48 ms

    // Total transmission time per image (approx)
    static constexpr double TOTAL_IMAGE_DURATION_SECONDS = (TOTAL_GROUP_DURATION_MS * HEIGHT / 2) / 1000.0; // (508.48 * 248) / 1000 = 126.0424 seconds
};

// Mode configuration for other SSTV modes (simplified for this example)
struct SSTVMode {
    std::string name;
    int vis_code;
    int width;
    int height;
    // Add other mode-specific timings if needed
};

// Map VIS code to mode configuration
const std::map<int, SSTVMode> VIS_MODE_MAP = {
    {95, {"PD120", 95, 640, 496}},
    // Add other modes here (e.g., Martin 1, Robot 36)
    // {0xac, {"Martin 1", 0xac, 320, 256}},
};


// Callbacks
using ModeDetectedCallback = std::function<void(const SSTVMode& mode)>;
using LineDecodedCallback = std::function<void(int line_index, const std::vector<Pixel>& pixels)>;
using ImageCompleteCallback = std::function<void(int width, int height)>;

// --- Internal DSP types ---
// For FIR filter coefficients and delay line
using FilterCoefficients = std::vector<double>;
using FilterDelayLine = std::vector<double>;

} // namespace sstv
