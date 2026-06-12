cmake_minimum_required(VERSION 3.18)
project(camera_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pybind11
find_package(pybind11 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# V4L2 (Linux only)
if(UNIX AND NOT APPLE)
    find_library(V4L2_LIBRARY v4l2)
    find_path(V4L2_INCLUDE_DIR linux/videodev2.h PATHS /usr/include)
endif()

# pthread
find_package(Threads REQUIRED)

# 构建共享库
pybind11_add_module(camera_driver SHARED
    camera_driver.cpp
)

target_include_directories(camera_driver PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${V4L2_INCLUDE_DIR}
)

target_link_libraries(camera_driver PRIVATE
    ${OpenCV_LIBS}
    Threads::Threads
)

if(V4L2_LIBRARY)
    target_link_libraries(camera_driver PRIVATE ${V4L2_LIBRARY})
    target_compile_definitions(camera_driver PRIVATE HAS_V4L2=1)
endif()

# 针对RK3588S优化
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(camera_driver PRIVATE
        -O3 -march=armv8.2-a+fp16+dotprod -mtune=cortex-a76
        -ffast-math -funroll-loops
    )
    # NEON SIMD支持
    target_compile_definitions(camera_driver PRIVATE USE_NEON=1)
else()
    target_compile_options(camera_driver PRIVATE -O3 -march=native -ffast-math)
endif()

# 安装
install(TARGETS camera_driver DESTINATION ${CMAKE_INSTALL_PREFIX}/camera/)
