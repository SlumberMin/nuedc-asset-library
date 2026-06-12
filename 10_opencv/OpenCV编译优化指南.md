# OpenCV 4.9 编译优化指南 — RK3588S 平台

> **目标**: 在 RK3588S (ARM Cortex-A76 + Mali-G610) 上编译 OpenCV 4.9，开启 NEON、OpenCL、GStreamer、RGA 等硬件加速，最大化视觉处理性能。

---

## 1. 环境准备

### 1.1 交叉编译工具链

```bash
# 安装 aarch64 交叉编译工具链
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 或使用 Rockchip 官方 SDK 工具链
# 路径: /opt/rk3588-sdk/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/
export CC=/opt/rk3588-sdk/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc
export CXX=/opt/rk3588-sdk/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++
```

### 1.2 本机编译依赖

```bash
sudo apt update && sudo apt install -y \
    build-essential cmake git pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgtk-3-dev libcanberra-gtk3-dev \
    libatlas-base-dev gfortran \
    python3-dev python3-numpy \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libv4l-dev v4l-utils \
    libxvidcore-dev libx264-dev \
    libhdf5-dev libprotobuf-dev protobuf-compiler \
    libgoogle-glog-dev libgflags-dev
```

### 1.3 Mali-G610 OpenCL 驱动

```bash
# 确保 Mali GPU 驱动已安装
# Rockchip SDK 通常自带，路径:
ls /usr/lib/aarch64-linux-gnu/libmali*.so*

# 设置 OpenCL ICD
sudo mkdir -p /etc/OpenCL/vendors/
echo "/usr/lib/aarch64-linux-gnu/libmali.so" | sudo tee /etc/OpenCL/vendors/mali.icd
```

### 1.4 RGA (Rockchip Graphics Acceleration) 库

```bash
# 从 Rockchip MPP/RGA 仓库编译安装
git clone https://github.com/airockchip/rga.git
cd rga
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install
sudo ldconfig
```

---

## 2. 下载 OpenCV 4.9 源码

```bash
cd ~/opencv_build
git clone --depth 1 --branch 4.9.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.9.0 https://github.com/opencv/opencv_contrib.git
```

---

## 3. CMake 配置（关键！）

### 3.1 完整 CMake 命令（本机编译）

```bash
cd ~/opencv_build/opencv
mkdir build && cd build

cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    \
    # ===== ARM NEON 优化 =====
    -D ENABLE_NEON=ON \
    -D ENABLE_VFPV3=OFF \
    -D CPU_BASELINE="NEON;NEON_FP16;VFPv4;FP_ARMv8" \
    -D CPU_DISPATCH="" \
    -D CV_ENABLE_INTRINSICS=ON \
    -D WITH_TBB=OFF \
    \
    # ===== OpenCL (Mali-G610) =====
    -D WITH_OPENCL=ON \
    -D WITH_OPENCL_SVM=ON \
    -D OPENCL_INCLUDE_DIR=/usr/include \
    -D OPENCL_LIBRARY=/usr/lib/aarch64-linux-gnu/libmali.so \
    \
    # ===== GStreamer =====
    -D WITH_GSTREAMER=ON \
    -D WITH_GSTREAMER_0_10=OFF \
    \
    # ===== V4L2 摄像头 =====
    -D WITH_V4L=ON \
    -D WITH_LIBV4L=ON \
    \
    # ===== FFmpeg =====
    -D WITH_FFMPEG=ON \
    \
    # ===== 禁用不需要的模块 (减少编译时间) =====
    -D WITH_CUDA=OFF \
    -D WITH_MATLAB=OFF \
    -D WITH_OPENEXR=OFF \
    -D WITH_PROTOBUF=OFF \
    -D BUILD_opencv_java=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=ON \
    -D BUILD_opencv_js=OFF \
    -D BUILD_opencv_apps=OFF \
    \
    # ===== 构建优化 =====
    -D BUILD_SHARED_LIBS=ON \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_DOCS=OFF \
    \
    # ===== Contrib 模块 =====
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
    -D BUILD_opencv_dnn=ON \
    -D OPENCV_DNN_OPENCL=ON \
    \
    # ===== 安装 Python 路径 =====
    -D PYTHON3_EXECUTABLE=$(which python3) \
    -D PYTHON3_INCLUDE_DIR=$(python3 -c "import sysconfig; print(sysconfig.get_path('include'))") \
    -D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c "import numpy; print(numpy.get_include())") \
    ..
```

### 3.2 交叉编译 CMake 工具链文件

**创建 `rk3588s_toolchain.cmake`:**

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CROSS_COMPILE /opt/rk3588-sdk/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-)

set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)

set(CMAKE_FIND_ROOT_PATH /opt/rk3588-sdk/rootfs)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# RK3588S CPU 特性
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a+fp16+dotprod -mcpu=cortex-a76 -mtune=cortex-a76")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a+fp16+dotprod -mcpu=cortex-a76 -mtune=cortex-a76")
```

**交叉编译命令:**

```bash
cmake \
    -D CMAKE_TOOLCHAIN_FILE=rk3588s_toolchain.cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D ENABLE_NEON=ON \
    -D WITH_OPENCL=ON \
    # ... (同上其他参数) \
    ..
```

---

## 4. 编译与安装

```bash
# 使用所有核心编译 (RK3588S 通常8核)
make -j$(nproc)

# 或限制并行数避免内存不足
make -j4

# 安装
sudo make install
sudo ldconfig

# 验证安装
python3 -c "
import cv2
print(f'OpenCV Version: {cv2.__version__}')
print(f'NEON: {cv2.useOptimized()}')
print(f'Build Info:')
print(cv2.getBuildInformation())
"
```

---

## 5. 关键编译选项详解

### 5.1 NEON SIMD 加速

| 选项 | 说明 | 推荐值 |
|------|------|--------|
| `ENABLE_NEON` | ARM NEON SIMD 指令 | **ON** |
| `CPU_BASELINE` | 基线 CPU 特性 | `NEON;NEON_FP16;VFPv4;FP_ARMv8` |
| `CPU_DISPATCH` | 动态分发特性 | 留空(基线已覆盖) |
| `CV_ENABLE_INTRINSICS` | 内部 SIMD 优化 | **ON** |

**RK3588S Cortex-A76 支持的特性:**
- NEON (必须)
- NEON FP16 (半精度浮点加速)
- Dot Product (int8 点积加速，DNN 推理)
- SVE/SVE2 (4.9版本暂不支持)

**影响的 OpenCV 模块:**
- `cv::resize` — NEON 加速 ~3x
- `cv::cvtColor` — NEON 加速 ~4x
- `cv::filter2D` — NEON 加速 ~2.5x
- `cv::warpAffine` — NEON 加速 ~2x
- `cv::matchTemplate` — NEON 加速 ~2x

### 5.2 OpenCL (Mali-G610 GPU)

| 选项 | 说明 | 推荐值 |
|------|------|--------|
| `WITH_OPENCL` | OpenCL 支持 | **ON** |
| `WITH_OPENCL_SVM` | 共享虚拟内存 | **ON** |
| `OPENCV_DNN_OPENCL` | DNN OpenCL 加速 | **ON** |

**注意:**
- Mali-G610 支持 OpenCL 3.0 (Full Profile)
- `T-API` (Transparent API) 会自动将计算卸载到 GPU
- 使用 `cv::UMat` 替代 `cv::Mat` 可触发 GPU 加速

```python
# 使用 UMat 触发 GPU 加速
import cv2
gpu_frame = cv2.UMat(frame)  # 数据上传到 GPU
gpu_result = cv2.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)  # GPU 执行
result = gpu_result.get()  # 下载回 CPU
```

### 5.3 GStreamer 管线

| 选项 | 说明 | 推荐值 |
|------|------|--------|
| `WITH_GSTREAMER` | GStreamer 支持 | **ON** |

**RK3588S 硬件编解码管线:**

```python
# 使用 MPP 硬件解码 (RK3588S 8K@60fps)
cap = cv2.VideoCapture(
    'filesrc location=test.mp4 ! mppvideodec ! videoconvert ! appsink',
    cv2.CAP_GSTREAMER
)

# 使用 RGA 硬件缩放 + 色彩转换
cap = cv2.VideoCapture(
    'v4l2src device=/dev/video0 ! video/x-raw,width=1920,height=1080,format=NV12 '
    '! rkrgaconvert ! video/x-raw,width=640,height=480,format=BGR ! appsink',
    cv2.CAP_GSTREAMER
)
```

### 5.4 RGA (Rockchip Graphics Acceleration)

RGA 需要通过 Rockchip MPP 库间接使用，OpenCV 本身不直接支持 RGA。

**替代方案：通过 librga 直接调用**

```cpp
#include <im2d.h>
#include <rga/RgaApi.h>

// RGA 硬件色彩转换 NV12 -> BGR
rga_buffer_t src = wrapbuffer_virtualaddr(src_nv12, w, h, RK_FORMAT_YCbCr_420_SP);
rga_buffer_t dst = wrapbuffer_virtualaddr(dst_bgr, w, h, RK_FORMAT_BGR_888);
imresize(src, dst);

// RGA 硬件缩放
rga_buffer_t src = wrapbuffer_virtualaddr(src_buf, 1920, 1080, RK_FORMAT_BGR_888);
rga_buffer_t dst = wrapbuffer_virtualaddr(dst_buf, 640, 480, RK_FORMAT_BGR_888);
imresize(src, dst);
```

---

## 6. 验证编译结果

### 6.1 检查构建信息

```python
import cv2

# 打印完整构建信息
info = cv2.getBuildInformation()
print(info)

# 关键检查项
checks = {
    'NEON': 'NEON:' in info and 'YES' in info.split('NEON:')[1].split('\n')[0],
    'OpenCL': 'OpenCL:' in info and 'YES' in info.split('OpenCL:')[1].split('\n')[0],
    'GStreamer': 'GStreamer:' in info and 'YES' in info.split('GStreamer:')[1].split('\n')[0],
    'V4L2': 'V4L/V4L2:' in info and 'YES' in info.split('V4L/V4L2:')[1].split('\n')[0],
    'FFmpeg': 'FFMPEG:' in info and 'YES' in info.split('FFMPEG:')[1].split('\n')[0],
}
for key, val in checks.items():
    status = '✅' if val else '❌'
    print(f'{status} {key}')
```

### 6.2 NEON 加速验证

```python
import cv2
import numpy as np
import time

# 检查优化是否开启
print(f"OpenCV 优化开启: {cv2.useOptimized()}")
print(f"优化级别: {cv2.getNumberOfCPUs()} CPU cores")

# 测试 resize 性能 (应有 NEON 加速)
frame = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)

times = []
for _ in range(100):
    t0 = time.perf_counter()
    small = cv2.resize(frame, (640, 480))
    times.append((time.perf_counter() - t0) * 1000)

print(f"resize 1920x1080 -> 640x480: {np.mean(times):.2f} ms (NEON)")
# 预期: ~1.5ms (NEON) vs ~6ms (无NEON)
```

### 6.3 OpenCL 验证

```python
import cv2
import numpy as np
import time

# 检查 OpenCL 设备
print("OpenCL 设备:")
platforms = cv2.ocl.getPlatfomsInfo()
# 或使用
cv2.ocl.setUseOpenCL(True)
print(f"OpenCL 可用: {cv2.ocl.useOpenCL()}")

# CPU vs GPU 性能对比
frame = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)

# CPU 模式
cv2.ocl.setUseOpenCL(False)
t0 = time.perf_counter()
for _ in range(100):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
cpu_time = (time.perf_counter() - t0) / 100 * 1000

# GPU 模式 (UMat)
cv2.ocl.setUseOpenCL(True)
gpu_frame = cv2.UMat(frame)
t0 = time.perf_counter()
for _ in range(100):
    gpu_gray = cv2.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
    gpu_blur = cv2.GaussianBlur(gpu_gray, (5, 5), 0)
gpu_time = (time.perf_counter() - t0) / 100 * 1000

print(f"CPU: {cpu_time:.2f} ms")
print(f"GPU: {gpu_time:.2f} ms")
print(f"加速比: {cpu_time/gpu_time:.2f}x")
```

---

## 7. 常见问题排查

### 7.1 NEON 未启用

```
症状: resize/cvtColor 性能与 x86 相当
排查:
1. cmake 输出检查 "NEON: YES"
2. 检查 CPU_BASELINE 是否正确
3. 检查编译器是否支持 -mfpu=neon (32位ARM)
64位ARM (aarch64) 默认支持 NEON
```

### 7.2 OpenCL 启动失败

```
症状: cv2.ocl.useOpenCL() 返回 False
排查:
1. ls /etc/OpenCL/vendors/ — 是否有 Mali ICD 文件
2. clinfo — 是否能看到 Mali 设备
3. ldd /usr/lib/libOpenCL.so — 依赖是否完整
4. Mali GPU 驱动版本是否匹配内核版本
```

### 7.3 GStreamer 管线错误

```
症状: VideoCapture GStreamer 管线打开失败
排查:
1. pkg-config --cflags gstreamer-1.0
2. gst-inspect-1.0 mppvideodec — MPP 插件是否安装
3. gst-inspect-1.0 rkrgaconvert — RGA 插件是否安装
4. 检查 GStreamer 版本兼容性
```

### 7.4 编译时间优化

```bash
# 并行编译
make -j8

# 只编译需要的模块
cmake -D BUILD_opencv_calib3d=OFF \
      -D BUILD_opencv_features2d=OFF \
      -D BUILD_opencv_flann=OFF \
      -D BUILD_opencv_stitching=OFF \
      -D BUILD_opencv_superres=OFF \
      -D BUILD_opencv_videostab=OFF \
      # ...

# 使用 ccache 加速重复编译
sudo apt install ccache
export CC="ccache gcc" CXX="ccache g++"
```

---

## 8. 运行时优化建议

### 8.1 线程配置

```python
import cv2

# 根据任务调整线程数
# 简单任务(颜色检测): 单线程更好
cv2.setNumThreads(1)

# 复杂任务(多目标匹配): 多线程
cv2.setNumThreads(4)

# 禁用多线程(避免与应用线程冲突)
cv2.setNumThreads(0)
```

### 8.2 内存管理

```python
# 预分配缓冲区复用
class FrameBuffer:
    def __init__(self, width, height):
        self.gray = np.empty((height, width), dtype=np.uint8)
        self.blur = np.empty((height, width), dtype=np.uint8)
    
    def process(self, frame):
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, dst=self.gray)
        cv2.GaussianBlur(self.gray, (5, 5), 0, dst=self.blur)
        return self.blur
```

### 8.3 帧率控制

```python
# V4L2 摄像头参数优化
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))  # 避免MJPEG解码
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲减少延迟
```

---

## 附录: 编译耗时参考

| 配置 | -j4 | -j8 |
|------|-----|-----|
| 全模块 | ~45min | ~25min |
| 最小模块集 | ~15min | ~8min |
| 仅 core + imgproc | ~8min | ~4min |
