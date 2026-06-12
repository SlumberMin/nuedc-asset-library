#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EfficientNet-B0 图像分类 RKNN 推理封装
======================================
"""

import time
import numpy as np
from typing import List, Optional, Dict, Tuple

try:
    from rknnlite.api import RKNNLite
except ImportError:
    RKNNLite = None

try:
    import cv2
except ImportError:
    cv2 = None


class EfficientNetB0RKNN:
    """EfficientNet-B0 分类 RKNN 推理器"""

    def __init__(
        self,
        model_path: str,
        input_size: int = 224,
        top_k: int = 5,
        core_mask: int = 0,
        class_names: Optional[List[str]] = None,
        mean: Tuple[float, float, float] = (0.485, 0.456, 0.406),
        std: Tuple[float, float, float] = (0.229, 0.224, 0.225),
    ):
        self.model_path = model_path
        self.input_size = input_size
        self.top_k = top_k
        self.core_mask = core_mask
        self.class_names = class_names
        self.mean = np.array(mean, dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)
        self.rknn = None
        self._perf_stats: Dict[str, float] = {}

    def load(self) -> bool:
        if RKNNLite is None:
            raise ImportError("rknnlite 未安装")
        self.rknn = RKNNLite(verbose=False)
        ret = self.rknn.load_rknn(self.model_path)
        if ret != 0:
            raise RuntimeError(f"加载模型失败: {self.model_path}")
        ret = self.rknn.init_runtime(core_mask=self.core_mask)
        if ret != 0:
            raise RuntimeError(f"初始化运行时失败")
        return True

    def release(self):
        if self.rknn:
            self.rknn.release()
            self.rknn = None

    def preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """CenterCrop + 归一化 + 量化"""
        h, w = image.shape[:2]
        s = self.input_size

        # 等比缩放短边到 input_size
        scale = s / min(h, w)
        nw, nh = int(w * scale), int(h * scale)
        if cv2 is not None:
            resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
        else:
            from PIL import Image
            resized = np.array(Image.fromarray(image).resize((nw, nh), Image.BILINEAR))

        # CenterCrop
        y0 = (nh - s) // 2
        x0 = (nw - s) // 2
        cropped = resized[y0:y0 + s, x0:x0 + s]

        meta = {"orig_shape": (h, w)}
        return cropped, meta

    def infer(self, image: np.ndarray) -> Tuple[List[np.ndarray], Dict]:
        if self.rknn is None:
            raise RuntimeError("模型未加载")
        input_tensor, meta = self.preprocess(image)
        t0 = time.perf_counter()
        outputs = self.rknn.inference(inputs=[input_tensor])
        t1 = time.perf_counter()
        self._perf_stats["infer_ms"] = (t1 - t0) * 1000
        return outputs, meta

    def postprocess(self, outputs: List[np.ndarray]) -> List[Dict]:
        """返回 Top-K 分类结果"""
        t0 = time.perf_counter()
        logits = outputs[0].flatten()

        # softmax
        exp_logits = np.exp(logits - logits.max())
        probs = exp_logits / exp_logits.sum()

        top_k = min(self.top_k, len(probs))
        indices = probs.argsort()[::-1][:top_k]

        results = []
        for idx in indices:
            name = str(idx)
            if self.class_names and idx < len(self.class_names):
                name = self.class_names[idx]
            results.append({
                "class_id": int(idx),
                "class_name": name,
                "confidence": float(probs[idx]),
            })

        t1 = time.perf_counter()
        self._perf_stats["postprocess_ms"] = (t1 - t0) * 1000
        return results

    def classify(self, image: np.ndarray) -> List[Dict]:
        outputs, _ = self.infer(image)
        return self.postprocess(outputs)

    @property
    def perf_stats(self):
        return dict(self._perf_stats)

    def __enter__(self):
        self.load()
        return self

    def __exit__(self, *args):
        self.release()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="EfficientNet-B0 RKNN 分类")
    parser.add_argument("--model", required=True, help="RKNN 模型路径")
    parser.add_argument("--image", required=True, help="输入图片")
    parser.add_argument("--size", type=int, default=224)
    parser.add_argument("--topk", type=int, default=5)
    parser.add_argument("--labels", type=str, default=None, help="标签文件(每行一个)")
    parser.add_argument("--core", type=int, default=0)
    args = parser.parse_args()

    labels = None
    if args.labels:
        with open(args.labels, "r", encoding="utf-8") as f:
            labels = [l.strip() for l in f if l.strip()]

    with EfficientNetB0RKNN(
        model_path=args.model, input_size=args.size,
        top_k=args.topk, core_mask=args.core, class_names=labels,
    ) as clf:
        img = cv2.imread(args.image)
        results = clf.classify(img)
        print("分类结果:")
        for r in results:
            print(f"  [{r['class_id']:4d}] {r['class_name']}: {r['confidence']:.4f}")
        print(f"性能: {clf.perf_stats}")
