# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.15) # 3.15 或更高版本对 C++20/23 支持更好

# 项目名称
project(SSTVDemodulator VERSION 0.1.0 LANGUAGES CXX)

# 设置 C++ 标准为 C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # 禁用 GNU 扩展

# 寻找 pybind11
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB SOURCES "src/*.cpp")

pybind11_add_module(_core ${SOURCES} bindings/python_bindings.cpp)

add_executable(sstv_demod 
    ${SOURCES}
)

# 可选：设置输出目录
set_target_properties(sstv_demod PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# 匹配在 pybind11_add_module 中定义的名字
install(TARGETS _core LIBRARY DESTINATION sstv_decoder)
