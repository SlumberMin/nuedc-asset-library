"""
RKNN 模型量化工具
==================
RK3588S NPU支持INT8/INT16/FP16量化推理，INT8推理速度最快(6 TOPS)。
本模块提供模型量化所需的校准数据集管理和量化参数配置。

量化流程：
1. 准备ONNX/TF/PyTorch模型
2. 收集校准数据集 (100-500张代表性图片)
3. 使用RKNN-Toolkit2进行量化
4. 评估量化精度
5. 导出.rknn模型

依赖：
    pip install rknn-toolkit2 numpy opencv-python
"""

import numpy as np
import os
from typing import List, Tuple, Optional, Callable, Dict
from pathlib import Path
from dataclasses import dataclass, field
import logging
import json

logger = logging.getLogger(__name__)


@dataclass
class QuantConfig:
    """量化配置"""
    model_path: str                         # 输入模型路径 (ONNX/TF/PyTorch)
    output_path: str                        # 输出RKNN模型路径
    target_platform: str = 'rk3588'         # 目标平台
    quantized_dtype: str = 'asymmetric_quantized-8'  # 量化类型
    # asymmetric_quantized-8: 非对称INT8 (推荐)
    # asymmetric_quantized-16: 非对称INT16
    # dynamic_fixed_point-8: 对称INT8
    # dynamic_fixed_point-16: 对称INT16
    quantized_algorithm: str = 'normal'     # normal / mmse (最小化均方误差)
    quantized_method: str = 'channel'       # layer / channel
    optimize_level: int = 3                 # 优化级别 (0-3)
    target: str = 'rk3588'
    # 输入预处理
    mean_values: Optional[List[List[float]]] = None
    std_values: Optional[List[List[float]]] = None
    reorder_channel: str = '2 1 0'         # BGR: 2 1 0, RGB: 0 1 2
    # 校准
    calibration_dataset: Optional[str] = None  # 校准数据集路径
    calibration_num: int = 100              # 校准样本数


class CalibrationDataset:
    """
    校准数据集管理器
    
    量化INT8模型需要校准数据来确定激活值的量化范围。
    推荐使用100-500张代表性图片（覆盖实际使用场景）。
    """

    def __init__(self, dataset_dir: str = '',
                 transform: Optional[Callable] = None,
                 max_samples: int = 100):
        """
        Args:
            dataset_dir: 数据集目录 (包含图片文件)
            transform: 数据预处理函数 (img_path -> np.ndarray)
            max_samples: 最大样本数
        """
        self._dir = dataset_dir
        self._transform = transform
        self._max_samples = max_samples
        self._image_paths: List[str] = []
        self._images: List[np.ndarray] = []

        if dataset_dir:
            self._scan_directory(dataset_dir)

    def _scan_directory(self, directory: str):
        """扫描目录中的图片文件"""
        valid_ext = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff'}
        for root, _, files in os.walk(directory):
            for f in sorted(files):
                if Path(f).suffix.lower() in valid_ext:
                    self._image_paths.append(os.path.join(root, f))
                    if len(self._image_paths) >= self._max_samples:
                        break
            if len(self._image_paths) >= self._max_samples:
                break

        logger.info(f"校准数据集: 找到 {len(self._image_paths)} 张图片")

    def add_image(self, img: np.ndarray):
        """手动添加图像"""
        self._images.append(img)

    def add_images(self, images: List[np.ndarray]):
        """批量添加图像"""
        self._images.extend(images)

    def add_from_directory(self, directory: str):
        """从目录添加图片"""
        self._scan_directory(directory)

    def __len__(self) -> int:
        return len(self._image_paths) + len(self._images)

    def __getitem__(self, idx: int) -> np.ndarray:
        if idx < len(self._images):
            return self._images[idx]

        file_idx = idx - len(self._images)
        if file_idx < len(self._image_paths):
            import cv2
            img = cv2.imread(self._image_paths[file_idx])
            if self._transform:
                return self._transform(img)
            return img

        raise IndexError(f"索引越界: {idx}")

    def save_to_file(self, output_dir: str):
        """保存校准数据集列表到文件"""
        os.makedirs(output_dir, exist_ok=True)
        list_file = os.path.join(output_dir, 'calibration_list.txt')
        with open(list_file, 'w') as f:
            for p in self._image_paths:
                f.write(p + '\n')
        logger.info(f"校准列表已保存: {list_file}")
        return list_file


class RKNNQuantizer:
    """
    RKNN模型量化器
    
    使用示例:
        quantizer = RKNNQuantizer()
        
        # 配置
        config = QuantConfig(
            model_path='model.onnx',
            output_path='model.rknn',
            calibration_dataset='./calibration_images/'
        )
        
        # 量化
        quantizer.quantize(config)
        
        # 评估
        metrics = quantizer.evaluate('model.rknn', test_images, test_labels)
    """

    def __init__(self):
        self._rknn = None

    def quantize(self, config: QuantConfig, dataset: Optional[CalibrationDataset] = None) -> bool:
        """
        执行模型量化
        
        Args:
            config: 量化配置
            dataset: 校准数据集 (如果config中已指定目录则可选)
            
        Returns:
            是否成功
        """
        try:
            from rknn.api import RKNN  # type: ignore
        except ImportError:
            logger.error("rknn-toolkit2未安装，请在x86主机上安装: pip install rknn-toolkit2")
            logger.info("注: rknn-toolkit2用于模型转换和量化，最终在RK3588S上使用rknn-lite运行")
            return False

        self._rknn = RKNN(verbose=(logger.level <= logging.DEBUG))

        try:
            # 配置RKNN
            logger.info(f"配置RKNN模型 (目标: {config.target_platform})")
            ret = self._rknn.config(
                mean_values=config.mean_values,
                std_values=config.std_values,
                target_platform=config.target_platform,
                quantized_dtype=config.quantized_dtype,
                quantized_algorithm=config.quantized_algorithm,
                quantized_method=config.quantized_method,
                reorder_channel=config.reorder_channel,
                optimize_level=config.optimize_level,
            )
            if ret != 0:
                logger.error("RKNN配置失败")
                return False

            # 加载模型
            logger.info(f"加载模型: {config.model_path}")
            model_ext = Path(config.model_path).suffix.lower()
            if model_ext == '.onnx':
                ret = self._rknn.load_onnx(model=config.model_path)
            elif model_ext in ('.pb', '.tflite'):
                ret = self._rknn.load_tflite(model=config.model_path)
            elif model_ext == '.pt' or model_ext == '.torchscript':
                ret = self._rknn.load_pytorch(model=config.model_path)
            else:
                ret = self._rknn.load_onnx(model=config.model_path)

            if ret != 0:
                logger.error("模型加载失败")
                return False

            # 准备校准数据
            if config.quantized_dtype != 'float' and dataset is None:
                if config.calibration_dataset:
                    dataset = CalibrationDataset(
                        config.calibration_dataset,
                        max_samples=config.calibration_num)

            # 构建（量化）
            if dataset is not None:
                logger.info(f"开始量化 (校准样本数: {len(dataset)})")
                dataset_list = self._prepare_dataset_list(dataset)
                ret = self._rknn.build(
                    do_quantization=True,
                    dataset=dataset_list
                )
            else:
                logger.info("无校准数据，使用浮点模式")
                ret = self._rknn.build(do_quantization=False)

            if ret != 0:
                logger.error("模型构建失败")
                return False

            # 导出RKNN模型
            os.makedirs(os.path.dirname(config.output_path) or '.', exist_ok=True)
            ret = self._rknn.export_rknn(config.output_path)
            if ret != 0:
                logger.error("RKNN模型导出失败")
                return False

            logger.info(f"量化成功: {config.output_path}")
            return True

        except Exception as e:
            logger.error(f"量化过程出错: {e}")
            return False

        finally:
            if self._rknn:
                self._rknn.release()

    def _prepare_dataset_list(self, dataset: CalibrationDataset) -> str:
        """准备校准数据集列表文件"""
        import tempfile
        list_file = os.path.join(tempfile.gettempdir(), 'rknn_calibration.txt')

        with open(list_file, 'w') as f:
            for i in range(len(dataset)):
                try:
                    img = dataset[i]
                    if isinstance(img, np.ndarray):
                        # 保存临时图片
                        import cv2
                        tmp_path = os.path.join(tempfile.gettempdir(),
                                                f'calib_{i:06d}.jpg')
                        cv2.imwrite(tmp_path, img)
                        f.write(tmp_path + '\n')
                    else:
                        # 文件路径
                        f.write(str(img) + '\n')
                except Exception as e:
                    logger.warning(f"校准样本 {i} 处理失败: {e}")

        return list_file

    def compare_precision(self, fp32_model_path: str, quant_model_path: str,
                          test_images: List[np.ndarray],
                          transform: Optional[Callable] = None) -> Dict:
        """
        对比浮点模型和量化模型的精度差异
        
        Returns:
            精度对比指标
        """
        try:
            from rknnlite.api import RKNNLite

            # 加载浮点模型
            fp32_engine = RKNNLite()
            fp32_engine.load_rknn(fp32_model_path)
            fp32_engine.init_runtime()

            # 加载量化模型
            quant_engine = RKNNLite()
            quant_engine.load_rknn(quant_model_path)
            quant_engine.init_runtime()

            fp32_outputs = []
            quant_outputs = []

            for img in test_images[:20]:  # 限制测试数量
                inp = transform(img) if transform else img
                out_fp32 = fp32_engine.inference(inputs=[inp])
                out_quant = quant_engine.inference(inputs=[inp])
                fp32_outputs.append(out_fp32[0])
                quant_outputs.append(out_quant[0])

            fp32_engine.release()
            quant_engine.release()

            # 计算精度指标
            fp32_all = np.concatenate([o.flatten() for o in fp32_outputs])
            quant_all = np.concatenate([o.flatten() for o in quant_outputs])

            mse = float(np.mean((fp32_all - quant_all) ** 2))
            cos_sim = float(np.dot(fp32_all, quant_all) /
                            (np.linalg.norm(fp32_all) * np.linalg.norm(quant_all)))

            return {
                'mse': round(mse, 6),
                'cosine_similarity': round(cos_sim, 6),
                'max_diff': round(float(np.max(np.abs(fp32_all - quant_all))), 6),
                'mean_diff': round(float(np.mean(np.abs(fp32_all - quant_all))), 6),
            }

        except ImportError:
            logger.warning("rknn-lite未安装，无法执行精度对比")
            return {}

    @staticmethod
    def generate_calibration_dataset(video_path: str, output_dir: str,
                                     num_frames: int = 100,
                                     interval: int = 10) -> str:
        """从视频生成校准数据集"""
        import cv2
        os.makedirs(output_dir, exist_ok=True)

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise FileNotFoundError(f"无法打开视频: {video_path}")

        frame_count = 0
        saved = 0

        while saved < num_frames:
            ret, frame = cap.read()
            if not ret:
                break

            if frame_count % interval == 0:
                path = os.path.join(output_dir, f'frame_{saved:06d}.jpg')
                cv2.imwrite(path, frame)
                saved += 1

            frame_count += 1

        cap.release()
        logger.info(f"生成校准数据集: {saved} 帧 -> {output_dir}")
        return output_dir


if __name__ == '__main__':
    print("RKNN模型量化工具")
    print("  支持: ONNX/TFLite/PyTorch -> RKNN (INT8/INT16)")
    print()

    # 示例配置
    config = QuantConfig(
        model_path='yolov5s.onnx',
        output_path='yolov5s.rknn',
        calibration_dataset='./calibration/',
        quantized_dtype='asymmetric_quantized-8',
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        reorder_channel='0 1 2',
    )

    print("量化配置:")
    print(f"  输入模型: {config.model_path}")
    print(f"  输出模型: {config.output_path}")
    print(f"  量化类型: {config.quantized_dtype}")
    print(f"  目标平台: {config.target_platform}")
    print(f"  优化级别: {config.optimize_level}")
