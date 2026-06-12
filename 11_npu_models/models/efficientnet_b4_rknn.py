"""
EfficientNet-B4 图像分类 RKNN 推理封装
高精度分类模型，适用于 RK3588/RK3576 NPU
"""

import numpy as np
import cv2
import time
import logging
from typing import List, Tuple, Optional, Dict

logger = logging.getLogger(__name__)


class EfficientNetB4RKNN:
    """EfficientNet-B4 图像分类 RKNN 推理引擎"""

    DEFAULT_CONFIG = {
        'model_path': 'models/efficientnet_b4.rknn',
        'input_size': (380, 380),
        'num_classes': 1000,
        'top_k': 5,
        'mean': [0.485, 0.456, 0.406],
        'std': [0.229, 0.224, 0.225],
        'quantize': True,
        'core_mask': 0,
        'labels_path': None,
        'crop_pct': 0.875,
    }

    def __init__(self, model_path: Optional[str] = None, config: Optional[Dict] = None):
        self.config = {**self.DEFAULT_CONFIG, **(config or {})}
        if model_path:
            self.config['model_path'] = model_path
        self.input_size = self.config['input_size']
        self.rknn = None
        self.labels = self._load_labels()
        self._perf_stats = {}
        self._init_model()

    def _load_labels(self) -> List[str]:
        path = self.config.get('labels_path')
        if path:
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    return [line.strip() for line in f.readlines()]
            except Exception as e:
                logger.warning(f"加载标签失败: {e}")
        return [f'class_{i}' for i in range(self.config['num_classes'])]

    def _init_model(self):
        try:
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            ret = self.rknn.load_rknn(self.config['model_path'])
            if ret != 0:
                raise RuntimeError(f"加载模型失败: {self.config['model_path']}")
            core_mask_map = {0: None, 1: 1, 2: 2, 4: 4, 7: 7}
            mask = core_mask_map.get(self.config['core_mask'])
            if mask is not None:
                self.rknn.init_runtime(core_mask=mask)
            else:
                self.rknn.init_runtime()
            logger.info(f"EfficientNet-B4 模型加载成功: {self.config['model_path']}")
        except ImportError:
            logger.warning("rknnlite 未安装，进入仿真模式")

    # ────────────────────────────────────────────
    #  前处理 (timm 风格)
    # ────────────────────────────────────────────
    def _resize_with_crop(self, img: np.ndarray) -> np.ndarray:
        """Resize 短边 → CenterCrop"""
        crop_pct = self.config['crop_pct']
        scale_size = tuple([int(s / crop_pct) for s in self.input_size])
        h, w = img.shape[:2]
        target_h, target_w = scale_size
        if h < w:
            new_h, new_w = target_h, int(w * target_h / h)
        else:
            new_h, new_w = int(h * target_w / w), target_w
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

        # CenterCrop
        ch, cw = self.input_size
        top = (new_h - ch) // 2
        left = (new_w - cw) // 2
        return img[top:top + ch, left:left + cw]

    def preprocess(self, img: np.ndarray) -> Tuple[np.ndarray, None, None]:
        """前处理: Resize → CenterCrop → 归一化 → 量化"""
        img_crop = self._resize_with_crop(img)
        img_rgb = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB)
        img_norm = img_rgb.astype(np.float32) / 255.0
        for c in range(3):
            img_norm[:, :, c] = (img_norm[:, :, c] - self.config['mean'][c]) / self.config['std'][c]

        if self.config['quantize']:
            img_out = np.clip(img_norm * 255, 0, 255).astype(np.uint8)
        else:
            img_out = img_norm
        return np.expand_dims(img_out, 0), None, None

    def inference(self, input_data: np.ndarray) -> List[np.ndarray]:
        if self.rknn is None:
            raise RuntimeError("RKNN 未初始化")
        return self.rknn.inference(inputs=[input_data])

    # ────────────────────────────────────────────
    #  后处理
    # ────────────────────────────────────────────
    def _softmax(self, x: np.ndarray) -> np.ndarray:
        e = np.exp(x - np.max(x))
        return e / e.sum()

    def postprocess(self, outputs: List[np.ndarray]) -> List[Dict]:
        t0 = time.perf_counter()
        logits = outputs[0].flatten().astype(np.float32)
        probs = self._softmax(logits)
        top_k = self.config['top_k']
        indices = probs.argsort()[::-1][:top_k]
        results = []
        for idx in indices:
            cls_id = int(idx)
            results.append({
                'class_id': cls_id,
                'class_name': self.labels[cls_id] if cls_id < len(self.labels) else str(cls_id),
                'confidence': float(probs[idx]),
            })
        self._perf_stats['postprocess_ms'] = (time.perf_counter() - t0) * 1000
        return results

    # ────────────────────────────────────────────
    #  一站式推理
    # ────────────────────────────────────────────
    def classify(self, img: np.ndarray) -> List[Dict]:
        t_total = time.perf_counter()

        t0 = time.perf_counter()
        input_data, _, _ = self.preprocess(img)
        self._perf_stats['preprocess_ms'] = (time.perf_counter() - t0) * 1000

        t0 = time.perf_counter()
        outputs = self.inference(input_data)
        self._perf_stats['inference_ms'] = (time.perf_counter() - t0) * 1000

        results = self.postprocess(outputs)

        self._perf_stats['total_ms'] = (time.perf_counter() - t_total) * 1000
        self._perf_stats['fps'] = 1000.0 / max(self._perf_stats['total_ms'], 1e-6)
        return results

    def classify_batch(self, images: List[np.ndarray]) -> List[List[Dict]]:
        return [self.classify(img) for img in images]

    def extract_features(self, img: np.ndarray) -> np.ndarray:
        if self.rknn is None:
            raise RuntimeError("RKNN 未初始化")
        input_data, _, _ = self.preprocess(img)
        outputs = self.rknn.inference(inputs=[input_data])
        if len(outputs) >= 2:
            return outputs[-2].flatten()
        return outputs[-1].flatten()

    def draw_results(self, img: np.ndarray, results: List[Dict]) -> np.ndarray:
        vis = img.copy()
        for i, r in enumerate(results):
            text = f"{r['class_name']}: {r['confidence']:.3f}"
            color = (0, 255, 0) if i == 0 else (200, 200, 200)
            cv2.putText(vis, text, (10, 30 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return vis

    def get_perf_stats(self) -> Dict:
        return dict(self._perf_stats)

    def benchmark(self, img: np.ndarray, warmup: int = 5, repeat: int = 50) -> Dict:
        for _ in range(warmup):
            self.classify(img)
        times = []
        for _ in range(repeat):
            t0 = time.perf_counter()
            self.classify(img)
            times.append((time.perf_counter() - t0) * 1000)
        times = np.array(times)
        return {
            'avg_ms': float(times.mean()),
            'min_ms': float(times.min()),
            'max_ms': float(times.max()),
            'std_ms': float(times.std()),
            'fps': 1000.0 / float(times.mean()),
        }

    def release(self):
        if self.rknn:
            self.rknn.release()

    def __del__(self):
        self.release()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.release()


def quick_classify(img_path: str, model_path: str = 'models/efficientnet_b4.rknn') -> List[Dict]:
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"图像不存在: {img_path}")
    with EfficientNetB4RKNN(model_path) as cls:
        results = cls.classify(img)
        stats = cls.get_perf_stats()
        print(f"推理耗时: {stats.get('inference_ms', 0):.1f}ms | FPS: {stats.get('fps', 0):.1f}")
        for r in results:
            print(f"  {r['class_name']}: {r['confidence']:.4f}")
        return results


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("用法: python efficientnet_b4_rknn.py <图片路径> [模型路径]")
        sys.exit(1)
    quick_classify(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else 'models/efficientnet_b4.rknn')
