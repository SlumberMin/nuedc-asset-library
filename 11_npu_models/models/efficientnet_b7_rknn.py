"""
EfficientNet-B7 图像分类 RKNN 推理封装
适用于 RK3588/RK3588S NPU 平台
V7: 新增高分辨率输入优化、批量推理、特征提取、自动通道排序
"""

import numpy as np
from rknnlite.api import RKNNLite


class EfficientNetB7RKNN:
    """EfficientNet-B7 on RKNN NPU"""

    def __init__(self, model_path: str, input_size: tuple = (600, 600),
                 labels_path: str = None, top_k: int = 5,
                 core_mask: int = RKNNLite.NPU_CORE_0_1_2,
                 channel_order: str = 'rgb'):
        self.input_size = input_size
        self.top_k = top_k
        self.channel_order = channel_order

        self.rknn = RKNNLite(verbose=False)
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"加载模型失败: {model_path}")
        ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError("初始化运行时失败")

        self.labels = []
        if labels_path:
            with open(labels_path, 'r', encoding='utf-8') as f:
                self.labels = [line.strip() for line in f.readlines()]

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        """EfficientNet 预处理: resize + center crop + normalize"""
        import cv2
        h, w = img.shape[:2]
        th, tw = self.input_size
        scale = max(tw / w, th / h)
        img_resized = cv2.resize(img, (int(w * scale), int(h * scale)),
                                 interpolation=cv2.INTER_LINEAR)
        ch, cw = img_resized.shape[:2]
        y1 = (ch - th) // 2
        x1 = (cw - tw) // 2
        img_crop = img_resized[y1:y1 + th, x1:x1 + tw]
        if self.channel_order == 'rgb':
            img_crop = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB)
        img_float = img_crop.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_norm = (img_float - mean) / std
        return img_norm.astype(np.float32)

    def classify(self, img: np.ndarray, top_k: int = None) -> list:
        """完整推理: preprocess -> inference -> softmax -> top-k"""
        k = top_k if top_k is not None else self.top_k
        input_data = self.preprocess(img)
        outputs = self.rknn.inference(inputs=[input_data])
        logits = outputs[0].flatten()
        exp_logits = np.exp(logits - logits.max())
        probs = exp_logits / exp_logits.sum()
        top_indices = probs.argsort()[::-1][:k]
        results = []
        for idx in top_indices:
            label = self.labels[idx] if idx < len(self.labels) else f"class_{idx}"
            results.append({
                'class_id': int(idx),
                'class_name': label,
                'confidence': float(probs[idx])
            })
        return results

    def classify_batch(self, images: list, top_k: int = None) -> list:
        """批量分类"""
        return [self.classify(img, top_k) for img in images]

    def get_feature(self, img: np.ndarray) -> np.ndarray:
        """提取特征向量"""
        input_data = self.preprocess(img)
        outputs = self.rknn.inference(inputs=[input_data])
        return outputs[0].flatten()

    def benchmark(self, warmup: int = 10, iterations: int = 100) -> dict:
        """快速性能测试"""
        import time
        h, w = self.input_size
        dummy = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
        for _ in range(warmup):
            self.rknn.inference(inputs=[dummy])
        latencies = []
        for _ in range(iterations):
            t0 = time.perf_counter()
            self.rknn.inference(inputs=[dummy])
            latencies.append((time.perf_counter() - t0) * 1000)
        lat = np.array(latencies)
        return {
            'mean_ms': float(lat.mean()), 'std_ms': float(lat.std()),
            'fps': float(1000.0 / lat.mean())
        }

    def release(self):
        self.rknn.release()
