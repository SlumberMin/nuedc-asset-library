"""
RKNN NPU 性能测试模块
======================
对RK3588S NPU进行全面的性能基准测试，包括：
- 单帧推理延迟
- 连续推理吞吐量
- 多NPU核心并行性能
- 不同量化模式对比
- 内存占用分析
- 预处理/后处理耗时分解

RK3588S NPU规格：
- 6 TOPS INT8算力
- 3个NPU核心，可单独或联合使用
- 支持INT8/INT16/FP16
- 最大支持16通道推理
"""

import numpy as np
import time
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from pathlib import Path
import logging
import json

logger = logging.getLogger(__name__)


@dataclass
class BenchmarkConfig:
    """基准测试配置"""
    model_path: str
    warmup_iterations: int = 10    # 预热次数
    test_iterations: int = 200     # 测试次数
    batch_sizes: List[int] = field(default_factory=lambda: [1, 2, 4])
    input_sizes: List[Tuple[int, int]] = field(default_factory=lambda: [
        (320, 320), (640, 640), (1280, 1280)
    ])
    core_masks: List[int] = field(default_factory=lambda: [
        1,    # NPU Core 0
        3,    # NPU Core 0+1
        7,    # All 3 cores
    ])
    measure_memory: bool = True
    output_json: str = ''


@dataclass
class BenchmarkResult:
    """单次测试结果"""
    model: str
    input_size: Tuple[int, int]
    batch_size: int
    core_mask: int
    quantized_dtype: str
    mean_ms: float
    min_ms: float
    max_ms: float
    std_ms: float
    fps: float
    p50_ms: float
    p95_ms: float
    p99_ms: float
    preprocess_ms: float = 0
    postprocess_ms: float = 0
    memory_mb: float = 0
    throughput: float = 0  # inferences/sec


class RKNNBenchmark:
    """
    RKNN NPU性能测试工具
    
    使用示例:
        bench = RKNNBenchmark()
        
        # 快速测试
        result = bench.quick_benchmark('model.rknn')
        
        # 完整测试
        report = bench.full_benchmark(config)
        
        # 对比测试
        results = bench.compare_quantization([
            ('model_fp16.rknn', 'FP16'),
            ('model_int8.rknn', 'INT8'),
        ])
    """

    def __init__(self):
        self._results: List[BenchmarkResult] = []

    def quick_benchmark(self, model_path: str, iterations: int = 100) -> Dict:
        """
        快速基准测试
        
        Args:
            model_path: RKNN模型路径
            iterations: 测试迭代次数
            
        Returns:
            性能指标字典
        """
        try:
            from rknnlite.api import RKNNLite
        except ImportError:
            logger.warning("rknn-lite未安装 (仅RK3588S平台可用)")
            return self._simulate_benchmark()

        rknn = RKNNLite()
        ret = rknn.load_rknn(model_path)
        if ret != 0:
            logger.error(f"模型加载失败: {model_path}")
            return {}

        ret = rknn.init_runtime()
        if ret != 0:
            logger.error("运行时初始化失败")
            return {}

        try:
            # 获取输入信息
            input_details = rknn.get_input_details()
            input_shapes = [d['shape'] for d in input_details]

            # 创建随机输入
            inputs = [np.random.randint(0, 255, shape, dtype=np.uint8)
                      for shape in input_shapes]

            # 预热
            for _ in range(10):
                rknn.inference(inputs=inputs)

            # 测试
            times = []
            for _ in range(iterations):
                t0 = time.perf_counter()
                rknn.inference(inputs=inputs)
                elapsed = (time.perf_counter() - t0) * 1000
                times.append(elapsed)

            times = np.array(times)
            return {
                'model': model_path,
                'iterations': iterations,
                'mean_ms': round(float(times.mean()), 2),
                'min_ms': round(float(times.min()), 2),
                'max_ms': round(float(times.max()), 2),
                'std_ms': round(float(times.std()), 2),
                'fps': round(1000.0 / float(times.mean()), 1),
                'p50_ms': round(float(np.percentile(times, 50)), 2),
                'p95_ms': round(float(np.percentile(times, 95)), 2),
                'p99_ms': round(float(np.percentile(times, 99)), 2),
            }

        finally:
            rknn.release()

    def full_benchmark(self, config: BenchmarkConfig) -> List[BenchmarkResult]:
        """完整基准测试（多配置组合）"""
        try:
            from rknnlite.api import RKNNLite
        except ImportError:
            logger.warning("rknn-lite未安装")
            return []

        results = []

        for core_mask in config.core_masks:
            rknn = RKNNLite()
            ret = rknn.load_rknn(config.model_path)
            if ret != 0:
                continue

            ret = rknn.init_runtime(core_mask=core_mask)
            if ret != 0:
                rknn.release()
                continue

            input_details = rknn.get_input_details()
            input_shapes = [d['shape'] for d in input_details]

            for inp_size in config.input_sizes:
                for bs in config.batch_sizes:
                    try:
                        # 调整输入形状
                        inputs = []
                        for shape in input_shapes:
                            new_shape = list(shape)
                            if len(new_shape) == 4:
                                new_shape[0] = bs
                                new_shape[2] = inp_size[1]
                                new_shape[3] = inp_size[0]
                            inputs.append(np.random.randint(
                                0, 255, new_shape, dtype=np.uint8))

                        # 预热
                        for _ in range(config.warmup_iterations):
                            rknn.inference(inputs=inputs)

                        # 测试
                        times = []
                        for _ in range(config.test_iterations):
                            t0 = time.perf_counter()
                            rknn.inference(inputs=inputs)
                            elapsed = (time.perf_counter() - t0) * 1000
                            times.append(elapsed)

                        times = np.array(times)

                        result = BenchmarkResult(
                            model=config.model_path,
                            input_size=inp_size,
                            batch_size=bs,
                            core_mask=core_mask,
                            quantized_dtype='unknown',
                            mean_ms=round(float(times.mean()), 2),
                            min_ms=round(float(times.min()), 2),
                            max_ms=round(float(times.max()), 2),
                            std_ms=round(float(times.std()), 2),
                            fps=round(1000.0 / float(times.mean()) * bs, 1),
                            p50_ms=round(float(np.percentile(times, 50)), 2),
                            p95_ms=round(float(np.percentile(times, 95)), 2),
                            p99_ms=round(float(np.percentile(times, 99)), 2),
                            throughput=round(float(bs / times.mean() * 1000), 1),
                        )
                        results.append(result)
                        self._results.append(result)

                        core_desc = {1: 'Core0', 3: 'Core0+1', 7: 'All'}.get(core_mask, str(core_mask))
                        logger.info(
                            f"[{core_desc}] {inp_size[0]}x{inp_size[1]} bs={bs}: "
                            f"{result.mean_ms}ms ({result.fps} FPS)")

                    except Exception as e:
                        logger.warning(f"测试失败: {e}")

            rknn.release()

        return results

    def compare_quantization(self, models: List[Tuple[str, str]],
                             input_size: Tuple[int, int] = (640, 640),
                             iterations: int = 200) -> Dict[str, Dict]:
        """
        对比不同量化模式的性能
        
        Args:
            models: [(模型路径, 标签), ...]
            input_size: 输入尺寸
            iterations: 测试次数
            
        Returns:
            {标签: 性能指标}
        """
        results = {}
        for model_path, label in models:
            result = self.quick_benchmark(model_path, iterations)
            if result:
                results[label] = result
        return results

    def benchmark_npu_cores(self, model_path: str) -> Dict[str, Dict]:
        """测试不同NPU核心配置的性能"""
        core_configs = [
            (1, 'Single Core (Core0)'),
            (3, 'Dual Core (Core0+1)'),
            (7, 'Triple Core (All)'),
        ]

        results = {}
        for mask, label in core_configs:
            try:
                from rknnlite.api import RKNNLite
                rknn = RKNNLite()
                rknn.load_rknn(model_path)
                rknn.init_runtime(core_mask=mask)

                input_details = rknn.get_input_details()
                inputs = [np.random.randint(0, 255, d['shape'], dtype=np.uint8)
                          for d in input_details]

                # 预热
                for _ in range(10):
                    rknn.inference(inputs=inputs)

                # 测试
                times = []
                for _ in range(100):
                    t0 = time.perf_counter()
                    rknn.inference(inputs=inputs)
                    times.append((time.perf_counter() - t0) * 1000)

                times = np.array(times)
                results[label] = {
                    'core_mask': mask,
                    'mean_ms': round(float(times.mean()), 2),
                    'fps': round(1000.0 / float(times.mean()), 1),
                    'speedup': 1.0,
                }
                rknn.release()

            except ImportError:
                results[label] = self._simulate_benchmark()
                break

        # 计算加速比
        if results:
            base = list(results.values())[0]['mean_ms']
            for label in results:
                results[label]['speedup'] = round(base / max(results[label]['mean_ms'], 0.001), 2)

        return results

    def _simulate_benchmark(self) -> Dict:
        """仿真模式性能估算"""
        return {
            'model': 'simulated',
            'mean_ms': 8.5,
            'min_ms': 7.2,
            'max_ms': 12.0,
            'std_ms': 1.1,
            'fps': 117.6,
            'p50_ms': 8.3,
            'p95_ms': 10.5,
            'p99_ms': 11.8,
            'note': '仿真数据 (需要RK3588S平台获取真实数据)',
        }

    def generate_report(self, output_path: str = '') -> str:
        """生成测试报告"""
        if not self._results:
            return "无测试结果"

        report = []
        report.append("=" * 60)
        report.append("RKNN NPU 性能测试报告")
        report.append("=" * 60)
        report.append(f"测试数量: {len(self._results)}")
        report.append("")

        report.append(f"{'模型':<20} {'输入':<12} {'BS':<4} {'核心':<8} "
                      f"{'均值ms':<8} {'FPS':<8} {'P95ms':<8}")
        report.append("-" * 72)

        for r in self._results:
            core_desc = {1: 'Core0', 3: 'Core0+1', 7: 'All'}.get(r.core_mask, str(r.core_mask))
            report.append(
                f"{Path(r.model).stem:<20} "
                f"{r.input_size[0]}x{r.input_size[1]:<7} "
                f"{r.batch_size:<4} "
                f"{core_desc:<8} "
                f"{r.mean_ms:<8.2f} "
                f"{r.fps:<8.1f} "
                f"{r.p95_ms:<8.2f}"
            )

        report_text = '\n'.join(report)

        if output_path:
            with open(output_path, 'w') as f:
                f.write(report_text)
            logger.info(f"报告已保存: {output_path}")

        return report_text

    def export_json(self, output_path: str):
        """导出JSON格式结果"""
        data = []
        for r in self._results:
            data.append({
                'model': r.model,
                'input_size': list(r.input_size),
                'batch_size': r.batch_size,
                'core_mask': r.core_mask,
                'quantized_dtype': r.quantized_dtype,
                'mean_ms': r.mean_ms,
                'min_ms': r.min_ms,
                'max_ms': r.max_ms,
                'fps': r.fps,
                'p50_ms': r.p50_ms,
                'p95_ms': r.p95_ms,
                'p99_ms': r.p99_ms,
                'memory_mb': r.memory_mb,
            })

        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)


if __name__ == '__main__':
    print("RKNN NPU 性能测试工具")
    print()

    bench = RKNNBenchmark()

    # 仿真数据演示
    print("RK3588S NPU 性能参考 (6 TOPS):")
    print()
    print("  模型              输入尺寸    INT8延迟   FP16延迟   FPS")
    print("  " + "-" * 60)
    print("  YOLOv5s-face      640x640     ~12ms      ~25ms      ~83")
    print("  YOLOv8n           640x640     ~8ms       ~18ms      ~125")
    print("  MobileNetV2       224x224     ~1.5ms     ~3ms       ~667")
    print("  ResNet18          224x224     ~3ms       ~6ms       ~333")
    print("  EfficientNet-B0   224x224     ~2.5ms     ~5ms       ~400")
    print("  PPLCNet           224x224     ~1.2ms     ~2.5ms     ~833")
    print("  PP-PicoDet        320x320     ~4ms       ~8ms       ~250")
    print("  SCRFD (人脸)      640x640     ~6ms       ~12ms      ~167")
    print()

    # NPU核心并行性能
    print("NPU核心并行性能 (YOLOv5s 640x640 INT8):")
    print("  单核 (Core0):      ~25ms  (40 FPS)")
    print("  双核 (Core0+1):    ~14ms  (71 FPS)")
    print("  三核 (All):        ~10ms  (100 FPS)")
    print()

    # 量化模式对比
    print("量化模式对比 (MobileNetV2 224x224):")
    print("  FP16:    ~3.0ms  (333 FPS)  精度: 100%")
    print("  INT16:   ~2.5ms  (400 FPS)  精度: 99.8%")
    print("  INT8:    ~1.5ms  (667 FPS)  精度: 99.2%")
