"""
EfficientNet-B6 图像分类 RKNN 推理封装
适用于 RK3588/RK3588S NPU 平台
"""

import numpy as np
from rknnlite.api import RKNNLite


class EfficientNetB6RKNN:
    """EfficientNet-B6 on RKNN NPU"""

    def __init__(self, model_path: str, input_size: tuple = (528, 528),
                 labels_path: str = None, top_k: int = 5,
                 core_mask: int = RKNNLite.NPU_CORE_0_1_2):
        self.input_size = input_size
        self.top_k = top_k

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
        img_resized = cv2.resize(img, (int(w * scale), int(h * scale)))
        ch, cw = img_resized.shape[:2]
        y1 = (ch - th) // 2
        x1 = (cw - tw) // 2
        img_crop = img_resized[y1:y1 + th, x1:x1 + tw]
        img_float = img_crop.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_norm = (img_float - mean) / std
        return img_norm.astype(np.float32)

    def classify(self, img: np.ndarray) -> list:
        input_data = self.preprocess(img)
        outputs = self.rknn.inference(inputs=[input_data])
        logits = outputs[0].flatten()
        exp_logits = np.exp(logits - logits.max())
        probs = exp_logits / exp_logits.sum()
        top_indices = probs.argsort()[::-1][:self.top_k]
        results = []
        for idx in top_indices:
            label = self.labels[idx] if idx < len(self.labels) else f"class_{idx}"
            results.append({
                'class_id': int(idx),
                'class_name': label,
                'confidence': float(probs[idx])
            })
        return results

    def release(self):
        self.rknn.release()
