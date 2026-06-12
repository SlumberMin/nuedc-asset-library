"""
图像预处理工具集
缩放 / Letterbox / 归一化 / 量化 / 颜色空间转换
"""
import numpy as np
import cv2


def resize_keep_ratio(img, target_size):
    """等比缩放到目标尺寸(短边对齐)"""
    h, w = img.shape[:2]
    scale = target_size / min(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(img, (new_w, new_h)), scale


def letterbox(img, target_size=640, fill_value=114):
    """
    Letterbox缩放: 等比缩放 + 灰边填充
    Returns: (padded_img, ratio, (pad_w, pad_h))
    """
    h, w = img.shape[:2]
    ratio = min(target_size / h, target_size / w)
    new_h, new_w = int(h * ratio), int(w * ratio)
    img_resized = cv2.resize(img, (new_w, new_h))

    pad_h = (target_size - new_h) // 2
    pad_w = (target_size - new_w) // 2
    img_padded = np.full((target_size, target_size, 3), fill_value, dtype=np.uint8)
    img_padded[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = img_resized

    return img_padded, ratio, (pad_w, pad_h)


def center_crop(img, crop_size):
    """中心裁剪"""
    h, w = img.shape[:2]
    y_start = (h - crop_size) // 2
    x_start = (w - crop_size) // 2
    return img[y_start:y_start + crop_size, x_start:x_start + crop_size]


def normalize(img, mean, std):
    """
    归一化: (img - mean) / std
    mean/std: [R, G, B], img为RGB格式, 值范围0-255或0-1
    """
    img_f = img.astype(np.float32)
    for c in range(3):
        img_f[:, :, c] = (img_f[:, :, c] - mean[c]) / std[c]
    return img_f


def normalize_255(img, mean=[0, 0, 0], std=[255, 255, 255]):
    """归一化到0-1: /255, 然后减均值除标准差"""
    return normalize(img, mean, std)


def normalize_imagenet(img_rgb):
    """ImageNet标准归一化 (输入RGB, 0-255)"""
    mean = [0.485, 0.456, 0.406]
    std = [0.229, 0.224, 0.225]
    img_f = img_rgb.astype(np.float32) / 255.0
    return normalize(img_f, mean, std)


def hwc_to_nchw(img):
    """HWC -> NCHW (添加batch维度)"""
    data = np.transpose(img, (2, 0, 1))
    return np.expand_dims(data, axis=0)


def bgr_to_rgb(img):
    """BGR -> RGB"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def quantize_to_uint8(data, scale, zero_point):
    """
    量化: float32 -> uint8
    q = round(data / scale) + zero_point
    """
    q = np.round(data / scale) + zero_point
    return np.clip(q, 0, 255).astype(np.uint8)


def dequantize_from_uint8(q_data, scale, zero_point):
    """反量化: uint8 -> float32"""
    return (q_data.astype(np.float32) - zero_point) * scale


def compute_quantize_params(data_min, data_max, num_bits=8):
    """计算量化参数 (scale, zero_point)"""
    q_min = 0
    q_max = 2 ** num_bits - 1
    scale = (data_max - data_min) / (q_max - q_min)
    zero_point = round(q_min - data_min / scale)
    zero_point = max(q_min, min(q_max, zero_point))
    return scale, int(zero_point)


class Preprocessor:
    """通用预处理流水线"""

    def __init__(self, input_size, mode='letterbox', mean=None, std=None,
                 bgr_to_rgb_flag=True, dtype=np.float32):
        """
        Args:
            input_size: 目标尺寸 (int or (h, w))
            mode: 'letterbox' | 'resize' | 'center_crop'
            mean: 归一化均值 (None则跳过)
            std: 归一化标准差
            bgr_to_rgb_flag: 是否BGR转RGB
            dtype: 输出数据类型
        """
        if isinstance(input_size, int):
            self.input_size = (input_size, input_size)
        else:
            self.input_size = tuple(input_size)
        self.mode = mode
        self.mean = np.array(mean, dtype=np.float32) if mean else None
        self.std = np.array(std, dtype=np.float32) if std else None
        self.bgr_to_rgb = bgr_to_rgb_flag
        self.dtype = dtype

    def __call__(self, img):
        """
        执行预处理
        Returns: (input_data, meta) - meta包含逆变换信息
        """
        meta = {}
        h, w = img.shape[:2]
        meta['orig_shape'] = (h, w)

        # 颜色空间
        proc = img.copy()
        if self.bgr_to_rgb:
            proc = bgr_to_rgb(proc)

        # 缩放
        if self.mode == 'letterbox':
            proc, ratio, pad_info = letterbox(proc, self.input_size[0])
            meta['ratio'] = ratio
            meta['pad'] = pad_info
        elif self.mode == 'resize':
            proc = cv2.resize(proc, (self.input_size[1], self.input_size[0]))
            meta['ratio'] = (self.input_size[1] / w, self.input_size[0] / h)
        elif self.mode == 'center_crop':
            proc, scale = resize_keep_ratio(proc, max(self.input_size))
            proc = center_crop(proc, self.input_size[0])

        # 归一化
        if self.mean is not None and self.std is not None:
            proc = normalize(proc, self.mean, self.std)
        else:
            proc = proc.astype(self.dtype)

        # HWC -> NCHW
        input_data = hwc_to_nchw(proc.astype(self.dtype))

        return input_data, meta
