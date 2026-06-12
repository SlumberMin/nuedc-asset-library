#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EfficientNet-B3 分类 RKNN 推理封装
====================================
适用于 Rockchip RK3588/RK3576/RK3568 NPU 平台
"""

import numpy as np
import cv2
import time
import logging
from typing import List, Tuple, Optional, Dict

logger = logging.getLogger(__name__)


class EfficientNetB3RKNN:
    """EfficientNet-B3 图像分类 RKNN 推理引擎"""

    def __init__(
        self,
        model_path: str,
        input_size: Tuple[int, int] = (300, 300),
        num_classes: int = 1000,
        class_names: Optional[List[str]] = None,
        top_k: int = 5,
        mean: Tuple[float, float, float] = (0.485, 0.456, 0.406),
        std: Tuple[float, float, float] = (0.229, 0.224, 0.225),
        core_mask: int = 0,
        target_platform: str = 'rk3588',
    ):
        self.model_path = model_path
        self.input_size = input_size
        self.num_classes = num_classes
        self.class_names = class_names
        self.top_k = top_k
        self.mean = np.array(mean, dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)
        self.core_mask = core_mask
        self.target_platform = target_platform
        self.rknn = None
        self.is_initialized = False
        self._load_model()

    def _load_model(self):
        try:
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            ret = self.rknn.load_rknn(self.model_path)
            if ret != 0:
                raise RuntimeError(f'加载失败: {self.model_path}')
            ret = self.rknn.init_runtime(core_mask=self.core_mask)
            if ret != 0:
                raise RuntimeError('初始化运行时失败')
            self.is_initialized = True
            logger.info(f'EfficientNet-B3 加载成功: {self.model_path}')
        except ImportError:
            logger.warning('rknnlite未安装, 使用模拟模式')
            self.is_initialized = True

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """EfficientNet预处理: 等比缩放+CenterCrop+归一化"""
        h, w = self.input_size
        img_h, img_w = image.shape[:2]

        scale = max(h, w) * 1.15 / min(img_h, img_w)
        new_w, new_h = int(img_w * scale), int(img_h * scale)
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        dy, dx = (new_h - h) // 2, (new_w - w) // 2
        cropped = resized[dy:dy + h, dx:dx + w]

        rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        rgb = (rgb - self.mean) / self.std
        blob = np.clip(rgb * 255, 0, 255).astype(np.uint8)
        return np.expand_dims(blob, axis=0)

    def postprocess(self, outputs: List[np.ndarray]) -> List[Dict]:
        logits = outputs[0]
        if logits.ndim > 1:
            logits = logits[0]
        logits = logits.astype(np.float32)

        exp_logits = np.exp(logits - np.max(logits))
        probs = exp_logits / np.sum(exp_logits)

        topk_idx = np.argsort(probs)[::-1][:self.top_k]
        results = []
        for idx in topk_idx:
            name = self.class_names[idx] if self.class_names and idx < len(self.class_names) else f'class_{idx}'
            results.append({
                'class_id': int(idx),
                'class_name': name,
                'confidence': float(probs[idx]),
            })
        return results

    def infer(self, image: np.ndarray) -> List[Dict]:
        if not self.is_initialized:
            raise RuntimeError('RKNN引擎未初始化')
        blob = self.preprocess(image)
        t0 = time.time()
        outputs = self.rknn.inference(inputs=[blob]) if self.rknn else [
            np.random.randn(1, self.num_classes).astype(np.float32)
        ]
        infer_time = time.time() - t0
        results = self.postprocess(outputs)
        for r in results:
            r['infer_time_ms'] = infer_time * 1000
        return results

    def release(self):
        if self.rknn:
            self.rknn.release()
            self.rknn = None
        self.is_initialized = False

    def __del__(self):
        self.release()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.release()
