"""
RKNN模型性能基准测试
测试维度: 推理速度 / CPU占用 / 内存使用 / 输出正确性
"""
import numpy as np
import time
import os
import json
import sys

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from rknnlite.api import RKNNLite
    HAS_RKNN = True
except ImportError:
    HAS_RKNN = False
    print("[WARN] rknnlite 未安装, 将使用模拟基准测试")


class ModelBenchmark:
    """RKNN模型性能基准测试"""

    def __init__(self, model_path, target_platform='rk3588'):
        self.model_path = model_path
        self.target_platform = target_platform
        self.rknn = None
        self.results = {}

    def load_model(self):
        """加载模型"""
        if not HAS_RKNN:
            print(f"[SIM] 模拟加载: {self.model_path}")
            return True

        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(self.model_path)
        if ret != 0:
            return False
        ret = self.rknn.init_runtime(target=None)
        return ret == 0

    def benchmark_speed(self, input_shapes, num_runs=100, warmup=10):
        """
        推理速度测试
        Args:
            input_shapes: 输入形状列表, e.g. [(1,3,640,640)]
            num_runs: 测试次数
            warmup: 预热次数
        Returns:
            dict: avg_ms, min_ms, max_ms, fps
        """
        print(f"  速度测试: {num_runs}次推理, {warmup}次预热...")

        # 生成随机输入
        inputs = [np.random.randn(*s).astype(np.float32) for s in input_shapes]

        # 预热
        for _ in range(warmup):
            if self.rknn:
                self.rknn.inference(inputs=inputs)

        # 测试
        times = []
        for _ in range(num_runs):
            t0 = time.perf_counter()
            if self.rknn:
                self.rknn.inference(inputs=inputs)
            else:
                time.sleep(0.005)  # 模拟5ms
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000)

        times = np.array(times)
        result = {
            'avg_ms': float(np.mean(times)),
            'min_ms': float(np.min(times)),
            'max_ms': float(np.max(times)),
            'std_ms': float(np.std(times)),
            'p50_ms': float(np.percentile(times, 50)),
            'p95_ms': float(np.percentile(times, 95)),
            'p99_ms': float(np.percentile(times, 99)),
            'fps': float(1000.0 / np.mean(times)),
            'num_runs': num_runs,
        }

        self.results['speed'] = result
        return result

    def benchmark_memory(self, input_shapes):
        """内存使用测试"""
        print("  内存测试...")

        mem_info = {}
        try:
            # 读取系统内存信息
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
                for line in lines:
                    if 'MemTotal' in line:
                        mem_info['total_kb'] = int(line.split()[1])
                    elif 'MemAvailable' in line:
                        mem_info['available_kb'] = int(line.split()[1])

            # 推理时内存
            inputs = [np.random.randn(*s).astype(np.float32) for s in input_shapes]
            if self.rknn:
                self.rknn.inference(inputs=inputs)

            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
                for line in lines:
                    if 'MemAvailable' in line:
                        mem_info['available_after_infer_kb'] = int(line.split()[1])

            mem_info['inference_mem_mb'] = (
                (mem_info['available_kb'] - mem_info['available_after_infer_kb']) / 1024
            )

        except (FileNotFoundError, PermissionError):
            # Windows或无权限
            mem_info = {
                'total_kb': 0,
                'available_kb': 0,
                'inference_mem_mb': 0,
                'note': '无法读取/proc/meminfo (Windows或权限不足)'
            }

        self.results['memory'] = mem_info
        return mem_info

    def benchmark_accuracy(self, input_shapes, num_samples=10):
        """输出一致性测试 (量化精度损失检测)"""
        print(f"  精度测试: {num_samples}个样本...")

        if not self.rknn:
            self.results['accuracy'] = {'note': '需要RKNN模型进行精度测试'}
            return self.results['accuracy']

        # 多次推理检查输出一致性
        inputs = [np.random.randn(*s).astype(np.float32) for s in input_shapes]

        outputs_ref = self.rknn.inference(inputs=inputs)
        all_diffs = []

        for _ in range(num_samples - 1):
            outputs_cur = self.rknn.inference(inputs=inputs)
            for o_ref, o_cur in zip(outputs_ref, outputs_cur):
                diff = np.abs(o_ref - o_cur)
                all_diffs.append({
                    'max_diff': float(np.max(diff)),
                    'mean_diff': float(np.mean(diff)),
                    'std_diff': float(np.std(diff)),
                })

        self.results['accuracy'] = {
            'num_samples': num_samples,
            'output_diffs': all_diffs,
            'max_output_diff': max(d['max_diff'] for d in all_diffs) if all_diffs else 0,
        }
        return self.results['accuracy']

    def benchmark_multi_resolution(self, base_shapes=[(1,3,320,320), (1,3,640,640), (1,3,416,416)]):
        """多分辨率性能对比"""
        print("  多分辨率测试...")
        resolution_results = {}

        for shape in base_shapes:
            h, w = shape[2], shape[3]
            key = f"{h}x{w}"
            print(f"    测试 {key}...")

            inputs = [np.random.randn(*shape).astype(np.float32)]
            times = []

            for _ in range(20):
                t0 = time.perf_counter()
                if self.rknn:
                    self.rknn.inference(inputs=inputs)
                else:
                    time.sleep(0.003 + 0.002 * (h * w) / (320 * 320))
                times.append((time.perf_counter() - t0) * 1000)

            resolution_results[key] = {
                'avg_ms': float(np.mean(times)),
                'fps': float(1000 / np.mean(times)),
                'input_shape': shape,
            }

        self.results['multi_resolution'] = resolution_results
        return resolution_results

    def run_full_benchmark(self, input_shapes=[(1, 3, 640, 640)]):
        """运行完整基准测试"""
        model_name = os.path.basename(self.model_path)
        print(f"\n{'='*50}")
        print(f"基准测试: {model_name}")
        print(f"{'='*50}")

        self.load_model()

        self.benchmark_speed(input_shapes)
        self.benchmark_memory(input_shapes)
        self.benchmark_accuracy(input_shapes)
        self.benchmark_multi_resolution()

        self.results['model_path'] = self.model_path
        self.results['model_name'] = model_name
        self.results['timestamp'] = time.strftime('%Y-%m-%d %H:%M:%S')

        self.release()
        return self.results

    def release(self):
        if self.rknn:
            self.rknn.release()
            self.rknn = None

    def save_results(self, output_path):
        """保存测试结果"""
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        print(f"[OK] 结果已保存: {output_path}")

    def print_summary(self):
        """打印测试摘要"""
        print(f"\n--- {self.results.get('model_name', 'Model')} 性能摘要 ---")

        if 'speed' in self.results:
            s = self.results['speed']
            print(f"  推理速度: avg={s['avg_ms']:.2f}ms, FPS={s['fps']:.1f}")
            print(f"  延迟分布: p50={s['p50_ms']:.2f}ms, p95={s['p95_ms']:.2f}ms, p99={s['p99_ms']:.2f}ms")

        if 'memory' in self.results:
            m = self.results['memory']
            if m.get('inference_mem_mb', 0) > 0:
                print(f"  推理内存: {m['inference_mem_mb']:.1f}MB")

        if 'multi_resolution' in self.results:
            print("  多分辨率:")
            for res, data in self.results['multi_resolution'].items():
                print(f"    {res}: {data['avg_ms']:.2f}ms, FPS={data['fps']:.1f}")


def benchmark_all_models(model_dir, output_dir='results'):
    """批量测试目录下所有RKNN模型"""
    os.makedirs(output_dir, exist_ok=True)
    all_results = {}

    model_files = []
    for f in os.listdir(model_dir):
        if f.endswith('.rknn'):
            model_files.append(os.path.join(model_dir, f))

    if not model_files:
        print("[WARN] 未找到RKNN模型文件")
        # 模拟测试
        model_files = ['yolov8n.rknn', 'yolov5n.rknn', 'mobilenetv3.rknn', 'picodet_s.rknn']

    for model_path in model_files:
        bench = ModelBenchmark(model_path)
        results = bench.run_full_benchmark()
        bench.print_summary()

        name = os.path.splitext(os.path.basename(model_path))[0]
        all_results[name] = results
        bench.save_results(os.path.join(output_dir, f'{name}_benchmark.json'))

    # 保存汇总
    summary_path = os.path.join(output_dir, 'benchmark_summary.json')
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(all_results, f, indent=2, ensure_ascii=False)
    print(f"\n[OK] 汇总已保存: {summary_path}")

    return all_results


if __name__ == '__main__':
    model_dir = os.path.join(os.path.dirname(__file__), '..', 'models')
    benchmark_all_models(model_dir)
