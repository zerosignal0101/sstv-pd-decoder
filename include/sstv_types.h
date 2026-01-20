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

// 家族标识
enum class SSTVFamily {
    PD,
    // ROBOT,
    // MARTIN,
    // SCOTTIE,
    UNKNOWN
};

// 暴露给上层（Main/UI）的结构体
struct SSTVMode {
    std::string name;    // "PD120", "PD90", "Martin 1"
    int vis_code;        // 95, 99, 172...
    int width;           // 640
    int height;          // 496
    double duration_s;   // 预计总耗时
    SSTVFamily family; // 新增：标识属于哪个家族
};

// --- 全局 VIS 注册表 (VISDecoder 使用) ---
// 包含所有已知的 SSTV 模式，不分家族
static const std::map<int, SSTVMode> GLOBAL_VIS_MAP = {
    // PD 系列
    {95, {"PD120", 95, 640, 496, 126.0, SSTVFamily::PD}},
    {93, {"PD50",  93, 320, 256, 50.0,  SSTVFamily::PD}},
    {99, {"PD90",  99, 320, 256, 90.0,  SSTVFamily::PD}},
    {98, {"PD160", 98, 512, 400, 161.0, SSTVFamily::PD}},
    {96, {"PD180", 96, 640, 496, 187.0, SSTVFamily::PD}},
    {97, {"PD240", 97, 640, 496, 248.0, SSTVFamily::PD}},
};

// PD 解调器内部使用的详细时序参数
struct PDTimings {
    double sync_ms;
    double porch_ms;
    double segment_ms;
};

// --- 全局 PD 模式表 ---
// 数据参考自标准的 PD 模式时序表
static const std::map<int, PDTimings> PD_TIMINGS_MAP = {
    {95, {20.0, 2.08, 121.60}},
    {93, {20.0, 2.08, 91.52}},
    {99, {20.0, 2.08, 170.24}},
    {98, {20.0, 2.08, 195.85}},
    {96, {20.0, 2.08, 183.04}},
    {97, {20.0, 2.08, 244.48}},
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
