# USB 相机驱动优化

## 1. V4L2 框架概述

### 1.1 V4L2 架构
```
┌──────────────────────────────────────┐
│           用户空间应用                │
├──────────────────────────────────────┤
│           V4L2 API                   │
│  open() / ioctl() / mmap() / read()  │
├──────────────────────────────────────┤
│         V4L2 核心框架                │
│  video_device / v4l2_subdev          │
├──────────┬───────────────┬───────────┤
│ USB 摄像头│  CSI 摄像头   │ 虚拟设备  │
│ (UVC)    │ (sensor+ISP) │ (test)    │
└──────────┴───────────────┴───────────┘
```

### 1.2 关键概念
- **Buffer**: 图像帧缓冲区
- **Format**: 图像格式 (YUYV, MJPEG, NV12 等)
- **Controls**: 曝光、白平衡、对焦等参数
- **Streaming**: 流式传输机制

---

## 2. V4L2 基本采集

### 2.1 完整采集流程
```c
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#define BUFFER_COUNT 4

struct buffer {
    void *start;
    size_t length;
};

int camera_init(const char *device, int width, int height, 
                __u32 pixfmt, struct buffer *buffers) {
    // 1. 打开设备
    int fd = open(device, O_RDWR | O_NONBLOCK);
    
    // 2. 查询能力
    struct v4l2_capability cap;
    ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "Not a capture device\n");
        return -1;
    }
    
    // 3. 设置格式
    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width = width,
            .height = height,
            .pixelformat = pixfmt,  // V4L2_PIX_FMT_YUYV / V4L2_PIX_FMT_MJPEG
            .field = V4L2_FIELD_NONE
        }
    };
    ioctl(fd, VIDIOC_S_FMT, &fmt);
    
    // 4. 申请缓冲区
    struct v4l2_requestbuffers req = {
        .count = BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP
    };
    ioctl(fd, VIDIOC_REQBUFS, &req);
    
    // 5. 映射缓冲区
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i
        };
        ioctl(fd, VIDIOC_QUERYBUF, &buf);
        
        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);
    }
    
    // 6. 入队所有缓冲区
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i
        };
        ioctl(fd, VIDIOC_QBUF, &buf);
    }
    
    // 7. 开始流
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);
    
    return fd;
}

int camera_capture(int fd, struct buffer *buffers, void **frame_data, 
                   size_t *frame_size) {
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP
    };
    
    // 出队（获取帧）
    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        return -1;
    }
    
    *frame_data = buffers[buf.index].start;
    *frame_size = buf.bytesused;
    
    // 处理完成后重新入队
    ioctl(fd, VIDIOC_QBUF, &buf);
    
    return buf.index;
}
```

### 2.2 select/poll 等待帧
```c
#include <sys/select.h>

int wait_for_frame(int fd, int timeout_ms) {
    fd_set fds;
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };
    
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    
    return select(fd + 1, &fds, NULL, NULL, &tv);
}
```

---

## 3. 帧率优化

### 3.1 设置帧率
```c
struct v4l2_streamparm parm = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .parm.capture = {
        .capability = V4L2_CAP_TIMEPERFRAME,
        .timeperframe = {
            .numerator = 1,
            .denominator = 60  // 60fps
        }
    }
};
ioctl(fd, VIDIOC_S_PARM, &parm);
```

### 3.2 帧率测试
```c
void fps_test(int fd, struct buffer *buffers, int num_frames) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    for (int i = 0; i < num_frames; i++) {
        wait_for_frame(fd, 1000);
        camera_capture(fd, buffers, NULL, NULL);
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = (end.tv_sec - start.tv_sec) + 
                     (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("FPS: %.2f\n", num_frames / elapsed);
}
```

### 3.3 帧率优化技巧
- **使用 MJPEG**: 带宽需求比 YUYV 少约 5x
- **降低分辨率**: 1080P → 720P 帧率可翻倍
- **增加缓冲区**: 减少丢帧 (4 → 8)
- **使用 DMA**: 避免 CPU 拷贝

---

## 4. 图像格式选择

### 4.1 常见格式对比
| 格式 | 大小 (1080P) | CPU处理 | 硬件加速 |
|------|-------------|---------|----------|
| YUYV | 6MB | 直接可用 | RGA/ISP |
| MJPEG | 0.3-1MB | 需解码 | 硬件解码 |
| NV12 | 3MB | 需转换 | ISP/NPU |
| RGB24 | 6MB | 直接可用 | RGA |
| H264 | 0.1-0.5MB | 需解码 | VPU |

### 4.2 格式协商
```c
// 查询支持的格式
struct v4l2_fmtdesc fmtdesc = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE
};
while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
    printf("Format: %s (%s)\n",
           fmtdesc.description,
           (fmtdesc.flags & V4L2_FMT_FLAG_COMPRESSED) ? "compressed" : "raw");
    fmtdesc.index++;
}

// 尝试设置首选格式
struct v4l2_format fmt = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .fmt.pix = {
        .width = 1920,
        .height = 1080,
        .pixelformat = V4L2_PIX_FMT_MJPEG,  // 首选 MJPEG
    }
};
int ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
if (ret < 0 || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
    // 回退到 YUYV
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    ioctl(fd, VIDIOC_S_FMT, &fmt);
}
```

---

## 5. UVC 控制优化

### 5.1 曝光控制
```c
// 设置手动曝光
struct v4l2_control ctrl = {
    .id = V4L2_CID_EXPOSURE_AUTO,
    .value = V4L2_EXPOSURE_MANUAL  // 1=手动
};
ioctl(fd, VIDIOC_S_CTRL, &ctrl);

// 设置曝光值
ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
ctrl.value = 156;  // 曝光时间（单位取决于设备）
ioctl(fd, VIDIOC_S_CTRL, &ctrl);

// 查询范围
struct v4l2_queryctrl qctrl = {
    .id = V4L2_CID_EXPOSURE_ABSOLUTE
};
ioctl(fd, VIDIOC_QUERYCTRL, &qctrl);
printf("Exposure range: %d - %d\n", qctrl.minimum, qctrl.maximum);
```

### 5.2 自动曝光优化
```c
// 设置自动曝光优先级
// 优先保证帧率
ctrl.id = V4L2_CID_EXPOSURE_AUTO;
ctrl.value = V4L2_EXPOSURE_APERTURE_PRIORITY;  // 光圈优先
ioctl(fd, VIDIOC_S_CTRL, &ctrl);

// 设置 AE 目标亮度
ctrl.id = V4L2_CID_CAMERA_CLASS_BASE + 0x10;  // 厂商自定义
ctrl.value = 128;  // 目标亮度
ioctl(fd, VIDIOC_S_CTRL, &ctrl);
```

### 5.3 白平衡控制
```c
// 手动白平衡
ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
ctrl.value = 0;  // 关闭自动
ioctl(fd, VIDIOC_S_CTRL, &ctrl);

ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
ctrl.value = 5600;  // 色温 K
ioctl(fd, VIDIOC_S_CTRL, &ctrl);
```

### 5.4 增益控制
```c
// 手动增益
ctrl.id = V4L2_CID_AUTOGAIN;
ctrl.value = 0;
ioctl(fd, VIDIOC_S_CTRL, &ctrl);

ctrl.id = V4L2_CID_GAIN;
ctrl.value = 10;  // 增益值
ioctl(fd, VIDIOC_S_CTRL, &ctrl);
```

---

## 6. libuvc 高级功能

### 6.1 libuvc 异步采集
```c
#include <libuvc/libuvc.h>

uvc_context_t *ctx;
uvc_device_t *dev;
uvc_device_handle_t *devh;
uvc_stream_ctrl_t ctrl;

// 初始化
uvc_init(&ctx, NULL);
uvc_find_device(ctx, &dev, 0, 0, NULL);  // 找第一个设备
uvc_open(dev, &devh);

// 配置流
uvc_get_stream_ctrl_format_size(devh, &ctrl,
    UVC_FRAME_FORMAT_YUYV, 640, 480, 30);

// 异步回调
void cb(uvc_frame_t *frame, void *ptr) {
    // frame->data 图像数据
    // frame->width, frame->height 尺寸
    // frame->sequence 帧序号
    // frame->capture_time 时间戳
    
    // 在此处理图像（注意不要阻塞太久）
    process_frame(frame->data, frame->width, frame->height);
}

// 开始流
uvc_start_streaming(devh, &ctrl, cb, NULL, 0);

// ...
uvc_stop_streaming(devh);
uvc_close(devh);
uvc_unref_device(dev);
uvc_exit(ctx);
```

### 6.2 libuvc 扩展控制
```c
// 获取支持的格式
uvc_format_desc_t *fmt_desc = uvc_get_format_descs(devh);
while (fmt_desc) {
    printf("Format: %d\n", fmt_desc->bDescriptorSubtype);
    uvc_frame_desc_t *frame_desc = fmt_desc->frame_descs;
    while (frame_desc) {
        printf("  %dx%d @ %dfps\n",
               frame_desc->wWidth, frame_desc->wHeight,
               frame_desc->intervals[0]);
        frame_desc = frame_desc->next;
    }
    fmt_desc = fmt_desc->next;
}

// 设置曝光（通过 UVC 扩展）
uint8_t data[2];
data[0] = exposure & 0xFF;
data[1] = (exposure >> 8) & 0xFF;
uvc_set_ctrl(devh, UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL, data, 2);
```

---

## 7. 多相机管理

### 7.1 设备识别
```bash
# 列出所有 V4L2 设备
v4l2-ctl --list-devices

# 查看设备详细信息
v4l2-ctl -d /dev/video0 --all

# 查看支持的格式
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### 7.2 多相机同步采集
```c
#include <pthread.h>

typedef struct {
    int fd;
    struct buffer *buffers;
    int id;
    pthread_mutex_t *mutex;
} CameraThread;

void *camera_thread(void *arg) {
    CameraThread *cam = (CameraThread *)arg;
    
    while (running) {
        wait_for_frame(cam->fd, 1000);
        
        void *data;
        size_t size;
        int idx = camera_capture(cam->fd, cam->buffers, &data, &size);
        
        if (idx >= 0) {
            pthread_mutex_lock(cam->mutex);
            // 处理帧
            process_frame(cam->id, data, size);
            pthread_mutex_unlock(cam->mutex);
        }
    }
    return NULL;
}

// 创建多相机线程
pthread_t threads[MAX_CAMERAS];
CameraThread cam_threads[MAX_CAMERAS];

for (int i = 0; i < num_cameras; i++) {
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/video%d", camera_devices[i]);
    
    cam_threads[i].fd = camera_init(dev_path, 640, 480, 
                                     V4L2_PIX_FMT_MJPEG, cam_threads[i].buffers);
    cam_threads[i].id = i;
    cam_threads[i].mutex = &frame_mutex;
    
    pthread_create(&threads[i], NULL, camera_thread, &cam_threads[i]);
}
```

### 7.3 硬件触发同步
```c
// 使用外部触发信号同步多相机
// 需要相机支持硬件触发模式

// 设置为外部触发
ctrl.id = V4L2_CID_CAMERA_CLASS_BASE + 0x100;  // 厂商自定义触发控制
ctrl.value = 1;  // 外部触发
ioctl(fd, VIDIOC_S_CTRL, &ctrl);
```

---

## 8. 性能监控与调试

### 8.1 丢帧检测
```c
typedef struct {
    uint32_t last_sequence;
    uint64_t last_timestamp;
    uint32_t dropped_frames;
    double avg_fps;
} FrameStats;

void update_stats(FrameStats *stats, struct v4l2_buffer *buf) {
    if (stats->last_sequence > 0) {
        uint32_t expected = stats->last_sequence + 1;
        if (buf->sequence > expected) {
            stats->dropped_frames += (buf->sequence - expected);
            printf("Dropped %d frames (seq %u -> %u)\n",
                   buf->sequence - expected, stats->last_sequence, buf->sequence);
        }
    }
    
    // 计算 FPS
    uint64_t now = buf->timestamp.tv_sec * 1000000ULL + buf->timestamp.tv_usec;
    if (stats->last_timestamp > 0) {
        double dt = (now - stats->last_timestamp) / 1e6;
        stats->avg_fps = 0.9 * stats->avg_fps + 0.1 * (1.0 / dt);
    }
    
    stats->last_sequence = buf->sequence;
    stats->last_timestamp = now;
}
```

### 8.2 USB 带宽监控
```bash
# 查看 USB 设备信息
lsusb -v -d <vendor_id>:<product_id>

# 监控 USB 传输
cat /sys/kernel/debug/usb/devices

# 查看 URB 统计
cat /sys/kernel/debug/usb/urbstat

# 带宽计算
# 1080P YUYV @ 30fps = 1920*1080*2*30 = 124 MB/s (超过 USB2.0)
# 1080P MJPEG @ 30fps ≈ 10-30 MB/s (USB2.0 可用)
```

### 8.3 常见问题诊断
```bash
# 检查设备权限
ls -la /dev/video*
sudo chmod 666 /dev/video0

# 检查是否被占用
fuser /dev/video0

# 查看内核日志
dmesg | grep -i uvc
dmesg | grep -i v4l2

# 测试采集
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=100
```

---

## 9. RGA 加速后处理

### 9.1 V4L2 + RGA 零拷贝
```c
#include "im2d.h"
#include "rga.h"

// 从 V4L2 获取 DMA-BUF fd
struct v4l2_exportbuffer expbuf = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .index = buf.index
};
ioctl(fd, VIDIOC_EXPBUF, &expbuf);

// 直接传给 RGA 处理
rga_buffer_t src = wrapbuffer_fd(expbuf.fd, width, height, 
                                  RK_FORMAT_YCbCr_420_SP);
rga_buffer_t dst = wrapbuffer_virtualaddr(dst_buf, dst_w, dst_h,
                                           RK_FORMAT_RGB_888);

imresize(src, dst);  // 缩放 + 格式转换，零拷贝
```

### 9.2 典型处理流水线
```
USB Camera (MJPEG/YUYV)
    ↓ V4L2 mmap
DMA Buffer
    ↓ RGA (零拷贝)
RGB/NV12 Buffer
    ↓ NPU/CPU
Detection Results
```

---

## 10. Python 封装

### 10.1 OpenCV 采集
```python
import cv2
import time

class USBCamera:
    def __init__(self, device=0, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        
        # 验证设置
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Camera: {actual_w}x{actual_h} @ {actual_fps}fps")
    
    def read(self):
        ret, frame = self.cap.read()
        return frame if ret else None
    
    def set_exposure(self, value):
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 手动
        self.cap.set(cv2.CAP_PROP_EXPOSURE, value)
    
    def set_gain(self, value):
        self.cap.set(cv2.CAP_PROP_GAIN, value)
    
    def release(self):
        self.cap.release()

# 使用
cam = USBCamera(0, 640, 480, 60)
while True:
    frame = cam.read()
    if frame is not None:
        cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
```

### 10.2 V4L2 直接操作
```python
import v4l2
import fcntl
import mmap
import select

class V4L2Camera:
    def __init__(self, device='/dev/video0', width=640, height=480):
        self.fd = open(device, 'rb+', buffering=0)
        
        # 设置格式
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmt.fmt.pix.width = width
        fmt.fmt.pix.height = height
        fmt.fmt.pix.pixelformat = v4l2.V4L2_PIX_FMT_MJPEG
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_FMT, fmt)
        
        # 申请缓冲区
        req = v4l2.v4l2_requestbuffers()
        req.count = 4
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, v4l2.VIDIOC_REQBUFS, req)
        
        # 映射缓冲区
        self.buffers = []
        for i in range(req.count):
            buf = v4l2.v4l2_buffer()
            buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = v4l2.V4L2_MEMORY_MMAP
            buf.index = i
            fcntl.ioctl(self.fd, v4l2.VIDIOC_QUERYBUF, buf)
            
            mm = mmap.mmap(self.fd.fileno(), buf.length,
                          mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                          offset=buf.m.offset)
            self.buffers.append(mm)
            
            # 入队
            fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
        
        # 开始流
        buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_STREAMON, buf_type)
    
    def read(self, timeout=1.0):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return None
        
        buf = v4l2.v4l2_buffer()
        buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = v4l2.V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, v4l2.VIDIOC_DQBUF, buf)
        
        data = self.buffers[buf.index][:buf.bytesused]
        
        fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
        return data
    
    def close(self):
        buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_STREAMOFF, buf_type)
        for mm in self.buffers:
            mm.close()
        self.fd.close()
```

---

## 11. 参考资源

| 资源 | 链接 |
|------|------|
| V4L2 文档 | https://linuxtv.org/downloads/v4l-dvb-apis/ |
| libuvc | https://github.com/libuvc/libuvc |
| v4l2-ctl | https://linuxtv.org/ |
| OpenCV V4L2 | https://docs.opencv.org/ |
| Rockchip Camera | https://opensource.rock-chips.com |
