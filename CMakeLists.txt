# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.15) # 3.15 或更高版本对 C++20/23 支持更好

# 解决旧项目兼容性问题，必须放在 project() 之前
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
# 项目名称
project(SSTVDemodulator VERSION 0.1.0 LANGUAGES CXX)

# 设置 C++ 标准为 C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # 禁用 GNU 扩展

# 寻找 pybind11
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

# --- 使用 FetchContent 获取 libsamplerate ---
include(FetchContent)
# 强制所有目标生成位置无关代码 (fPIC)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# 强制设置为 OFF，确保作为静态库链接，避免 wheel 安装缺少依赖
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE) 
set(ENABLE_CPACK OFF CACHE BOOL "" FORCE) # 默认不打包
FetchContent_Declare(
        libsamplerate
        GIT_REPOSITORY https://github.com/libsndfile/libsamplerate.git
        GIT_TAG        0.2.2
)
set(LIBSAMPLERATE_EXAMPLES OFF CACHE BOOL "")
set(LIBSAMPLERATE_INSTALL OFF CACHE BOOL "")
FetchContent_MakeAvailable(libsamplerate)

include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB SOURCES "src/*.cpp")

pybind11_add_module(_core ${SOURCES} bindings/python_bindings.cpp)

if(NOT SKBUILD)
    add_executable(sstv_demod ${SOURCES})
    # 链接 libsamplerate
    target_link_libraries(sstv_demod PRIVATE SampleRate::samplerate)
    set_target_properties(sstv_demod PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    )
endif()

# 链接 libsamplerate
target_link_libraries(_core PRIVATE SampleRate::samplerate)

# 匹配在 pybind11_add_module 中定义的名字
install(TARGETS _core LIBRARY DESTINATION sstv_decoder)
