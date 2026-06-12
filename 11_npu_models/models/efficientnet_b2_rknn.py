#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EfficientNet-B2 图像分类 RKNN 推理封装
========================================
支持 RK3588 / RK3576 / RK3568 等平台
多核 NPU 推理、Top-K 分类、特征提取、注意力可视化

使用方法:
    model = EfficientNetB2RKNN(model_path='efficientnet_b2.rknn')
    result = model.classify(image, top_k=5)
    model.release()
"""

import numpy as np
import cv2
import time
import logging
import json
from typing import List, Tuple, Dict, Optional
from pathlib import Path

try:
    from rknnlite.api import RKNNLite
except ImportError:
    try:
        from rknn.api import RKNN
        RKNNLite = RKNN
    except ImportError:
        raise ImportError("请安装 RKNN Toolkit 或 RKNNLite Runtime")

logger = logging.getLogger(__name__)


class EfficientNetB2RKNN:
    """EfficientNet-B2 图像分类模型 RKNN 推理封装"""

    DEFAULT_CLASSES = [
        'tench', 'goldfish', 'great white shark', 'tiger shark', 'hammerhead',
        'electric ray', 'stingray', 'cock', 'hen', 'ostrich',
        'brambling', 'goldfinch', 'house finch', 'junco', 'indigo bunting',
        'robin', 'bulbul', 'jay', 'magpie', 'chickadee',
    ]

    def __init__(
        self,
        model_path: str,
        target: str = 'rk3588',
        core_mask: int = None,
        input_size: int = 260,
        labels_path: str = None,
        mean: Tuple[float, ...] = (0.485, 0.456, 0.406),
        std: Tuple[float, ...] = (0.229, 0.224, 0.225),
        crop_pct: float = 0.875,
    ):
        """
        初始化 EfficientNet-B2 RKNN 模型

        Args:
            model_path: RKNN 模型文件路径
            target: 目标平台
            core_mask: NPU 核心掩码
            input_size: 输入图像尺寸 (EfficientNet-B2 默认 260)
            labels_path: 标签文件路径
            mean: 归一化均值
            std: 归一化标准差
            crop_pct: 中心裁剪比例
        """
        self.model_path = model_path
        self.target = target
        self.input_size = input_size
        self.mean = np.array(mean, dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)
        self.crop_pct = crop_pct
        self.model = None

        self.classes = self._load_labels(labels_path)

        self._init_model(core_mask)
        logger.info(f"EfficientNet-B2 初始化完成: target={target}, input_size={input_size}, classes={len(self.classes)}")

    def _load_labels(self, labels_path: str = None) -> List[str]:
        """加载分类标签"""
        if labels_path and Path(labels_path).exists():
            path = Path(labels_path)
            if path.suffix == '.json':
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if isinstance(data, list):
                        return data
                    elif isinstance(data, dict):
                        return [data[str(i)] for i in range(len(data))]
            else:
                with open(path, 'r', encoding='utf-8') as f:
                    return [line.strip() for line in f if line.strip()]

        logger.warning("未找到标签文件，使用默认标签")
        return self.DEFAULT_CLASSES

    def _init_model(self, core_mask=None):
        """初始化 RKNN 模型"""
        self.model = RKNNLite(verbose=False)

        ret = self.model.load_rknn(self.model_path)
        if ret != 0:
            raise RuntimeError(f"加载 RKNN 模型失败: {self.model_path}")

        if core_mask is None:
            if self.target == 'rk3588':
                core_mask = RKNNLite.NPU_CORE_0_1_2
            else:
                core_mask = RKNNLite.NPU_CORE_0

        ret = self.model.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError("RKNN 运行时初始化失败")

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        预处理: Resize + CenterCrop + 归一化

        EfficientNet-B2 预处理流程:
        1. 短边缩放到 int(input_size / crop_pct) = 297
        2. 中心裁剪到 input_size x input_size = 260x260
        3. 归一化
        """
        h, w = image.shape[:2]
        size = self.input_size

        # 短边缩放到 resize_size
        resize_size = int(size / self.crop_pct)
        scale = resize_size / min(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # 中心裁剪
        dy = (new_h - size) // 2
        dx = (new_w - size) // 2
        cropped = resized[dy:dy + size, dx:dx + size]

        # BGR -> RGB
        rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        # 归一化
        blob = rgb.astype(np.float32) / 255.0
        blob = (blob - self.mean) / self.std

        # HWC -> CHW, 添加 batch 维度
        blob = blob.transpose(2, 0, 1)
        return np.expand_dims(blob, axis=0)

    def _preprocess_quantized(self, image: np.ndarray) -> np.ndarray:
        """
        量化模型预处理 (输入为 uint8)

        对于 INT8 量化模型，输入直接使用 uint8 格式
        """
        h, w = image.shape[:2]
        size = self.input_size

        resize_size = int(size / self.crop_pct)
        scale = resize_size / min(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        dy = (new_h - size) // 2
        dx = (new_w - size) // 2
        cropped = resized[dy:dy + size, dx:dx + size]

        rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        # 直接返回 uint8，量化信息已在模型中内置
        blob = rgb.astype(np.uint8)
        blob = blob.transpose(2, 0, 1)
        return np.expand_dims(blob, axis=0)

    def _softmax(self, x: np.ndarray) -> np.ndarray:
        """数值稳定的 Softmax"""
        e_x = np.exp(x - np.max(x, axis=-1, keepdims=True))
        return e_x / e_x.sum(axis=-1, keepdims=True)

    def _sigmoid(self, x: np.ndarray) -> np.ndarray:
        """Sigmoid 激活"""
        return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))

    def classify(
        self,
        image: np.ndarray,
        top_k: int = 5,
        quantized: bool = False,
    ) -> Dict:
        """
        图像分类推理

        Args:
            image: BGR 格式输入图像
            top_k: 返回 Top-K 结果
            quantized: 是否使用量化预处理

        Returns:
            分类结果字典
        """
        if quantized:
            blob = self._preprocess_quantized(image)
        else:
            blob = self._preprocess(image)

        t0 = time.time()
        outputs = self.model.inference(inputs=[blob])
        infer_time = time.time() - t0

        logits = outputs[0].flatten()
        probs = self._softmax(logits)

        top_k = min(top_k, len(probs))
        indices = np.argsort(probs)[::-1][:top_k]

        results = []
        for idx in indices:
            results.append({
                'class_id': int(idx),
                'class_name': self.classes[idx] if idx < len(self.classes) else f'class_{idx}',
                'probability': float(probs[idx]),
            })

        return {
            'top_k': results,
            'top1': results[0],
            'infer_time_ms': infer_time * 1000,
            'confidence_map': probs,
        }

    def classify_batch(
        self,
        images: List[np.ndarray],
        top_k: int = 5,
        quantized: bool = False,
    ) -> List[Dict]:
        """批量分类"""
        return [self.classify(img, top_k, quantized) for img in images]

    def extract_features(self, image: np.ndarray) -> np.ndarray:
        """
        提取特征向量

        Returns:
            特征向量 (用于相似度计算、检索等)
        """
        blob = self._preprocess(image)
        outputs = self.model.inference(inputs=[blob])

        if len(outputs) > 1:
            return outputs[-2].flatten()
        return outputs[0].flatten()

    def compute_similarity(
        self,
        image1: np.ndarray,
        image2: np.ndarray,
        metric: str = 'cosine',
    ) -> float:
        """
        计算两张图像的相似度

        Args:
            image1: 图像1
            image2: 图像2
            metric: 距离度量 ('cosine' / 'euclidean')

        Returns:
            相似度分数
        """
        feat1 = self.extract_features(image1)
        feat2 = self.extract_features(image2)

        if metric == 'cosine':
            dot = np.dot(feat1, feat2)
            norm = np.linalg.norm(feat1) * np.linalg.norm(feat2)
            return float(dot / (norm + 1e-8))
        elif metric == 'euclidean':
            return float(-np.linalg.norm(feat1 - feat2))
        else:
            raise ValueError(f"不支持的度量: {metric}")

    def benchmark(self, iterations: int = 100, warmup: int = 10) -> Dict:
        """性能基准测试"""
        dummy = np.random.randint(0, 255,
                                  (self.input_size, self.input_size, 3),
                                  dtype=np.uint8)

        for _ in range(warmup):
            self.classify(dummy)

        times = []
        for _ in range(iterations):
            t0 = time.time()
            self.classify(dummy)
            times.append((time.time() - t0) * 1000)

        times = np.array(times)
        return {
            'model': 'EfficientNet-B2',
            'target': self.target,
            'input_size': self.input_size,
            'num_classes': len(self.classes),
            'iterations': iterations,
            'avg_ms': float(np.mean(times)),
            'min_ms': float(np.min(times)),
            'max_ms': float(np.max(times)),
            'std_ms': float(np.std(times)),
            'fps': float(1000.0 / np.mean(times)),
            'p50_ms': float(np.percentile(times, 50)),
            'p95_ms': float(np.percentile(times, 95)),
            'p99_ms': float(np.percentile(times, 99)),
        }

    def draw_result(self, image: np.ndarray, result: Dict) -> np.ndarray:
        """绘制分类结果"""
        canvas = image.copy()
        top1 = result['top1']

        label = f"{top1['class_name']}: {top1['probability']:.3f}"
        cv2.putText(canvas, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        y_offset = 60
        for i, item in enumerate(result['top_k'][1:], 2):
            label = f"#{i} {item['class_name']}: {item['probability']:.3f}"
            cv2.putText(canvas, label, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 1)
            y_offset += 25

        # 显示推理时间
        time_label = f"Infer: {result['infer_time_ms']:.1f}ms"
        cv2.putText(canvas, time_label, (10, canvas.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return canvas

    def release(self):
        """释放模型资源"""
        if self.model:
            self.model.release()
            self.model = None
            logger.info("EfficientNet-B2 模型资源已释放")

    def __del__(self):
        self.release()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.release()


if __name__ == '__main__':
    import sys
    logging.basicConfig(level=logging.INFO)

    if len(sys.argv) < 2:
        print("用法: python efficientnet_b2_rknn.py <rknn_model> [image] [labels] [target]")
        sys.exit(1)

    model_path = sys.argv[1]
    image_path = sys.argv[2] if len(sys.argv) > 2 else None
    labels_path = sys.argv[3] if len(sys.argv) > 3 else None
    target = sys.argv[4] if len(sys.argv) > 4 else 'rk3588'

    model = EfficientNetB2RKNN(model_path, target=target, labels_path=labels_path)

    stats = model.benchmark(iterations=50)
    print(f"\n=== EfficientNet-B2 基准测试 ===")
    for k, v in stats.items():
        print(f"  {k}: {v}")

    if image_path:
        img = cv2.imread(image_path)
        if img is not None:
            result = model.classify(img, top_k=5)
            print(f"\n分类结果:")
            for item in result['top_k']:
                print(f"  #{item['class_id']} {item['class_name']}: {item['probability']:.4f}")
            print(f"  推理时间: {result['infer_time_ms']:.2f} ms")

    model.release()
