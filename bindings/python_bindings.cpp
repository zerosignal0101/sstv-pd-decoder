#include <pybind11/pybind11.h>
#include <pybind11/stl.h>       // 自动转换 std::vector, std::map
#include <pybind11/functional.h> // 自动转换 std::function
#include <pybind11/numpy.h>      // 处理 Python 的 NumPy 数组 (代替 float*)

#include "sstv_decoder.h"
#include "sstv_types.h"

namespace py = pybind11;
using namespace sstv;

PYBIND11_MODULE(_core, m) {
    m.doc() = "SSTV Decoder Python Bindings (C++23)";

    // 1. 绑定结构体 Pixel
    py::class_<Pixel>(m, "Pixel")
        .def(py::init<uint8_t, uint8_t, uint8_t>())
        .def_readwrite("r", &Pixel::r)
        .def_readwrite("g", &Pixel::g)
        .def_readwrite("b", &Pixel::b)
        .def("__repr__", [](const Pixel &p) {
            return "(" + std::to_string(p.r) + "," + std::to_string(p.g) + "," + std::to_string(p.b) + ")";
        });

    // 2. 绑定结构体 SSTVMode
    py::enum_<SSTVFamily>(m, "SSTVFamily")
        .value("PD", SSTVFamily::PD)
        // .value("ROBOT", SSTVFamily::ROBOT) // 如果以后启用了，取消注释即可
        // .value("MARTIN", SSTVFamily::MARTIN)
        // .value("SCOTTIE", SSTVFamily::SCOTTIE)
        .value("UNKNOWN", SSTVFamily::UNKNOWN);
    py::class_<SSTVMode>(m, "SSTVMode")
        .def_readonly("name", &SSTVMode::name)
        .def_readonly("vis_code", &SSTVMode::vis_code)
        .def_readonly("width", &SSTVMode::width)
        .def_readonly("height", &SSTVMode::height)
        .def_readonly("duration_s", &SSTVMode::duration_s)
        .def_readonly("family", &SSTVMode::family);

    // 3. 核心类 Decoder 的封装
    py::class_<Decoder>(m, "Decoder")
        .def(py::init<double>(), py::arg("sample_rate"))

        // 关键：将 process(const float*, size_t) 封装为接受 NumPy 数组的接口
        .def("process", [](Decoder &self, const py::array_t<float>& samples) {
            py::buffer_info buf = samples.request();
            if (buf.ndim != 1) {
                throw std::runtime_error("Buffer must be 1D");
            }
            // 调用原始 C++ 接口
            self.process(static_cast<float*>(buf.ptr), static_cast<size_t>(buf.shape[0]));
        }, py::arg("samples"), "Process audio samples (NumPy array)")

        .def("reset", &Decoder::reset)

        // 绑定回调函数
        .def("set_on_mode_detected_callback", &Decoder::set_on_mode_detected_callback)
        .def("set_on_line_decoded_callback", &Decoder::set_on_line_decoded_callback)
        .def("set_on_image_complete_callback", &Decoder::set_on_image_complete_callback);
}
