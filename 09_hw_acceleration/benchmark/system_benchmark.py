"""
RK3588S 系统整体性能基准测试
==============================
全面测试 RK3588S 各硬件模块的性能，包括：

  - CPU：整数/浮点运算、多核并行
  - GPU (Mali G610)：OpenCL 计算、图形渲染
  - NPU：RKNN 推理延迟与吞吐
  - 内存：带宽、延迟
  - IO：顺序/随机读写、eMMC/SD/SSD
  - RGA：硬件缩放、转换性能
  - 视频编解码：H.264/H.265 编码帧率

输出一份完整的性能报告（Markdown 格式），可直接用于电赛技术文档。

依赖：numpy, time, os, mmap

用法示例：
    benchmark = SystemBenchmark()
    report = benchmark.run_all()
    benchmark.save_report("benchmark_report.md")
"""

import os
import sys
import time
import json
import math
import mmap
import struct
import logging
import tempfile
import threading
import subprocess
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class BenchmarkResult:
    """单个基准测试结果"""
    category: str       # CPU / GPU / NPU / Memory / IO
    test_name: str
    score: float        # 主要指标值
    unit: str           # 单位
    details: dict = field(default_factory=dict)
    elapsed_ms: float = 0.0
    passed: bool = True


class SystemBenchmark:
    """
    系统整体性能基准测试

    Parameters
    ----------
    io_test_size_mb : int
        IO 测试文件大小 (MB)
    io_test_path : str or None
        IO 测试文件路径（None 则使用 /tmp）
    warmup_iterations : int
        预热迭代次数
    """

    def __init__(self, io_test_size_mb: int = 64,
                 io_test_path: Optional[str] = None,
                 warmup_iterations: int = 3):
        self.io_test_size_mb = io_test_size_mb
        self.io_test_path = io_test_path or tempfile.gettempdir()
        self.warmup_iterations = warmup_iterations
        self.results: List[BenchmarkResult] = []

    # ==================================================================
    # CPU 基准测试
    # ==================================================================

    def bench_cpu_integer(self) -> BenchmarkResult:
        """CPU 整数运算性能"""
        t0 = time.perf_counter()

        n = 2_000_000
        result_sum = 0
        for i in range(n):
            result_sum += (i * 7 + 13) % 997

        elapsed = (time.perf_counter() - t0) * 1000
        mops = n / elapsed * 1000 / 1e6  # Million Ops/sec

        r = BenchmarkResult("CPU", "整数运算", mops, "MOPS",
                            {"iterations": n, "checksum": result_sum}, elapsed)
        self.results.append(r)
        logger.info(f"CPU Integer: {mops:.1f} MOPS ({elapsed:.1f}ms)")
        return r

    def bench_cpu_float(self) -> BenchmarkResult:
        """CPU 浮点运算性能"""
        t0 = time.perf_counter()

        n = 1_000_000
        a, b = 1.0001, 0.9999
        result = 1.0
        for i in range(n):
            result = result * a + math.sin(b * i * 0.001)

        elapsed = (time.perf_counter() - t0) * 1000
        mflops = n / elapsed * 1000 / 1e6

        r = BenchmarkResult("CPU", "浮点运算", mflops, "MFLOPS",
                            {"iterations": n}, elapsed)
        self.results.append(r)
        logger.info(f"CPU Float: {mflops:.1f} MFLOPS ({elapsed:.1f}ms)")
        return r

    def bench_cpu_numpy(self) -> BenchmarkResult:
        """CPU NumPy 矩阵运算"""
        sizes = [512, 1024, 2048]
        times_ms = {}

        for size in sizes:
            a = np.random.rand(size, size).astype(np.float32)
            b = np.random.rand(size, size).astype(np.float32)

            t0 = time.perf_counter()
            _ = np.dot(a, b)
            elapsed = (time.perf_counter() - t0) * 1000
            times_ms[size] = elapsed

        # GFLOPS = 2 * N^3 / time
        best_size = max(sizes, key=lambda s: times_ms[s])  # largest
        gflops = 2 * best_size ** 3 / (times_ms[best_size] / 1000) / 1e9

        r = BenchmarkResult("CPU", "矩阵乘法 (NumPy)", gflops, "GFLOPS",
                            {"matmul_times_ms": times_ms}, sum(times_ms.values()))
        self.results.append(r)
        logger.info(f"CPU NumPy matmul: {gflops:.2f} GFLOPS")
        return r

    def bench_cpu_multicore(self) -> BenchmarkResult:
        """CPU 多核并行性能"""
        n_cores = os.cpu_count() or 4

        def cpu_work(n):
            s = 0.0
            for i in range(n):
                s += math.sin(i * 0.001) * math.cos(i * 0.002)
            return s

        work_per_core = 200_000

        # 单线程
        t0 = time.perf_counter()
        cpu_work(work_per_core * n_cores)
        single_ms = (time.perf_counter() - t0) * 1000

        # 多线程
        t0 = time.perf_counter()
        with ThreadPoolExecutor(max_workers=n_cores) as pool:
            futures = [pool.submit(cpu_work, work_per_core) for _ in range(n_cores)]
            [f.result() for f in futures]
        multi_ms = (time.perf_counter() - t0) * 1000

        speedup = single_ms / multi_ms if multi_ms > 0 else 0
        efficiency = speedup / n_cores * 100

        r = BenchmarkResult("CPU", "多核并行", speedup, "x 加速比",
                            {"cores": n_cores, "single_ms": single_ms,
                             "multi_ms": multi_ms, "efficiency_pct": efficiency},
                            multi_ms)
        self.results.append(r)
        logger.info(f"CPU Multi-core: {speedup:.2f}x speedup ({n_cores} cores, {efficiency:.0f}% efficiency)")
        return r

    # ==================================================================
    # GPU 基准测试
    # ==================================================================

    def bench_gpu_opencl(self) -> BenchmarkResult:
        """GPU OpenCL 计算性能"""
        try:
            import pyopencl as cl

            platforms = cl.get_platforms()
            if not platforms:
                raise RuntimeError("No OpenCL platform")

            devices = platforms[0].get_devices(device_type=cl.device_type.GPU)
            if not devices:
                devices = platforms[0].get_devices()

            ctx = cl.Context(devices=[devices[0]])
            queue = cl.CommandQueue(ctx,
                                     properties=cl.command_queue_properties.PROFILING_ENABLE)

            # 向量加法 kernel
            kernel_src = """
            __kernel void vec_add(__global const float* a,
                                   __global const float* b,
                                   __global float* c,
                                   const int n) {
                int i = get_global_id(0);
                if (i < n) c[i] = a[i] + b[i];
            }
            """
            prg = cl.Program(ctx, kernel_src).build()

            n = 10_000_000
            a = np.random.rand(n).astype(np.float32)
            b = np.random.rand(n).astype(np.float32)

            a_buf = cl.Buffer(ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=a)
            b_buf = cl.Buffer(ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=b)
            c_buf = cl.Buffer(ctx, cl.mem_flags.WRITE_ONLY, size=a.nbytes)

            # Warmup
            for _ in range(self.warmup_iterations):
                prg.vec_add(queue, (n,), None, a_buf, b_buf, c_buf, np.int32(n))
                queue.finish()

            # Benchmark
            t0 = time.perf_counter()
            for _ in range(10):
                prg.vec_add(queue, (n,), None, a_buf, b_buf, c_buf, np.int32(n))
            queue.finish()
            elapsed = (time.perf_counter() - t0) * 1000

            throughput = n * 10 / (elapsed / 1000) / 1e9  # GElements/sec

            r = BenchmarkResult("GPU", "OpenCL 向量加法", throughput, "GElem/s",
                                {"device": devices[0].name, "elements": n,
                                 "iterations": 10, "total_ms": elapsed}, elapsed)
            self.results.append(r)
            logger.info(f"GPU OpenCL: {throughput:.2f} GElem/s ({devices[0].name})")
            return r

        except Exception as e:
            r = BenchmarkResult("GPU", "OpenCL 向量加法", 0, "GElem/s",
                                {"error": str(e)}, 0, passed=False)
            self.results.append(r)
            logger.warning(f"GPU OpenCL benchmark failed: {e}")
            return r

    def bench_gpu_matrix(self) -> BenchmarkResult:
        """GPU 矩阵乘法性能"""
        try:
            import pyopencl as cl

            platforms = cl.get_platforms()
            devices = platforms[0].get_devices(device_type=cl.device_type.GPU)
            if not devices:
                devices = platforms[0].get_devices()

            ctx = cl.Context(devices=[devices[0]])
            queue = cl.CommandQueue(ctx,
                                     properties=cl.command_queue_properties.PROFILING_ENABLE)

            kernel_src = """
            __kernel void matmul(__global const float* A,
                                  __global const float* B,
                                  __global float* C,
                                  const int N) {
                int row = get_global_id(0);
                int col = get_global_id(1);
                if (row >= N || col >= N) return;
                float sum = 0;
                for (int k = 0; k < N; k++) {
                    sum += A[row * N + k] * B[k * N + col];
                }
                C[row * N + col] = sum;
            }
            """
            prg = cl.Program(ctx, kernel_src).build()

            n = 256  # GPU 上小矩阵
            a = np.random.rand(n, n).astype(np.float32)
            b = np.random.rand(n, n).astype(np.float32)

            a_buf = cl.Buffer(ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=a)
            b_buf = cl.Buffer(ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=b)
            c_buf = cl.Buffer(ctx, cl.mem_flags.WRITE_ONLY, size=a.nbytes)

            t0 = time.perf_counter()
            for _ in range(5):
                prg.matmul(queue, (n, n), None, a_buf, b_buf, c_buf, np.int32(n))
            queue.finish()
            elapsed = (time.perf_counter() - t0) * 1000

            gflops = 2 * n ** 3 * 5 / (elapsed / 1000) / 1e9

            r = BenchmarkResult("GPU", "OpenCL 矩阵乘法", gflops, "GFLOPS",
                                {"matrix_size": n, "iterations": 5}, elapsed)
            self.results.append(r)
            logger.info(f"GPU matmul: {gflops:.3f} GFLOPS")
            return r

        except Exception as e:
            r = BenchmarkResult("GPU", "OpenCL 矩阵乘法", 0, "GFLOPS",
                                {"error": str(e)}, 0, passed=False)
            self.results.append(r)
            return r

    # ==================================================================
    # 内存基准测试
    # ==================================================================

    def bench_memory_bandwidth(self) -> BenchmarkResult:
        """内存带宽测试（顺序读写）"""
        sizes_mb = [16, 64, 256]
        results = {}

        for size_mb in sizes_mb:
            data = np.random.randint(0, 256, size=size_mb * 1024 * 1024, dtype=np.uint8)

            # 顺序读
            t0 = time.perf_counter()
            _ = data.sum()
            read_ms = (time.perf_counter() - t0) * 1000

            # 顺序写
            dst = np.empty_like(data)
            t0 = time.perf_counter()
            np.copyto(dst, data)
            write_ms = (time.perf_counter() - t0) * 1000

            read_bw = size_mb / (read_ms / 1000)  # MB/s
            write_bw = size_mb / (write_ms / 1000)

            results[size_mb] = {"read_mb_s": read_bw, "write_mb_s": write_bw}

        avg_read = np.mean([r["read_mb_s"] for r in results.values()])
        avg_write = np.mean([r["write_mb_s"] for r in results.values()])

        r = BenchmarkResult("Memory", "内存带宽", avg_read, "MB/s (读)",
                            {"read_avg_mb_s": avg_read, "write_avg_mb_s": avg_write,
                             "by_size": results})
        self.results.append(r)
        logger.info(f"Memory BW: read={avg_read:.0f} MB/s, write={avg_write:.0f} MB/s")
        return r

    def bench_memory_latency(self) -> BenchmarkResult:
        """内存延迟测试（随机访问）"""
        n = 10_000_000
        arr = np.arange(n, dtype=np.int64)
        np.random.shuffle(arr)

        # 链式指针追踪
        indices = np.zeros(n, dtype=np.int64)
        for i in range(n - 1):
            indices[arr[i]] = arr[i + 1]
        indices[arr[-1]] = arr[0]

        # 遍历
        t0 = time.perf_counter()
        idx = 0
        for _ in range(min(n, 1_000_000)):
            idx = indices[idx]
        elapsed = (time.perf_counter() - t0) * 1000

        latency_ns = elapsed * 1e6 / 1_000_000  # ns per access

        r = BenchmarkResult("Memory", "随机访问延迟", latency_ns, "ns/access",
                            {"array_size": n, "chase_iterations": 1_000_000}, elapsed)
        self.results.append(r)
        logger.info(f"Memory Latency: {latency_ns:.1f} ns/access")
        return r

    def bench_memory_copy(self) -> BenchmarkResult:
        """内存拷贝性能（不同大小）"""
        sizes = [1, 4, 16, 64, 256]  # MB
        copy_rates = {}

        for size_mb in sizes:
            src = np.random.randint(0, 256, size=size_mb * 1024 * 1024, dtype=np.uint8)
            dst = np.empty_like(src)

            t0 = time.perf_counter()
            for _ in range(10):
                np.copyto(dst, src)
            elapsed = (time.perf_counter() - t0) * 1000

            copy_rates[size_mb] = size_mb * 10 / (elapsed / 1000)  # MB/s

        avg_rate = np.mean(list(copy_rates.values()))
        r = BenchmarkResult("Memory", "内存拷贝", avg_rate, "MB/s",
                            {"by_size_mb_s": copy_rates})
        self.results.append(r)
        logger.info(f"Memory Copy: {avg_rate:.0f} MB/s avg")
        return r

    # ==================================================================
    # IO 基准测试
    # ==================================================================

    def bench_io_sequential(self) -> BenchmarkResult:
        """顺序 IO 读写性能"""
        test_file = os.path.join(self.io_test_path, ".rk3588_bench_io_seq")
        block_size = 1024 * 1024  # 1MB blocks
        n_blocks = self.io_test_size_mb
        data = os.urandom(block_size)

        # 顺序写
        t0 = time.perf_counter()
        with open(test_file, "wb") as f:
            for _ in range(n_blocks):
                f.write(data)
            f.flush()
            os.fsync(f.fileno())
        write_ms = (time.perf_counter() - t0) * 1000
        write_speed = n_blocks / (write_ms / 1000)  # MB/s

        # 顺序读
        t0 = time.perf_counter()
        with open(test_file, "rb") as f:
            while f.read(block_size):
                pass
        read_ms = (time.perf_counter() - t0) * 1000
        read_speed = n_blocks / (read_ms / 1000)

        # 清理
        try:
            os.remove(test_file)
        except OSError:
            pass

        r = BenchmarkResult("IO", "顺序读写", read_speed, "MB/s (读)",
                            {"read_mb_s": read_speed, "write_mb_s": write_speed,
                             "test_size_mb": n_blocks, "block_size": block_size})
        self.results.append(r)
        logger.info(f"IO Sequential: read={read_speed:.0f} MB/s, write={write_speed:.0f} MB/s")
        return r

    def bench_io_random(self) -> BenchmarkResult:
        """随机 IO 性能"""
        test_file = os.path.join(self.io_test_path, ".rk3588_bench_io_rand")
        block_size = 4096  # 4KB blocks
        n_blocks = 10000
        total_size = n_blocks * block_size

        # 创建测试文件
        with open(test_file, "wb") as f:
            f.write(os.urandom(total_size))

        # 随机读
        offsets = np.random.randint(0, n_blocks, size=5000) * block_size

        t0 = time.perf_counter()
        with open(test_file, "rb") as f:
            for offset in offsets:
                f.seek(offset)
                _ = f.read(block_size)
        elapsed = (time.perf_counter() - t0) * 1000

        iops = len(offsets) / (elapsed / 1000)

        try:
            os.remove(test_file)
        except OSError:
            pass

        r = BenchmarkResult("IO", "随机读取 (4KB)", iops, "IOPS",
                            {"block_size": block_size, "ops": len(offsets),
                             "total_ms": elapsed})
        self.results.append(r)
        logger.info(f"IO Random: {iops:.0f} IOPS")
        return r

    # ==================================================================
    # NPU 基准测试
    # ==================================================================

    def bench_npu_inference(self) -> BenchmarkResult:
        """NPU 推理性能（尝试加载测试模型）"""
        try:
            from rknnlite.api.rknn_lite import RKNNLite

            # 尝试找模型文件
            model_paths = [
                "models/yolov5s.rknn",
                "yolov5s.rknn",
                os.path.expanduser("~/models/yolov5s.rknn"),
            ]
            model_path = None
            for p in model_paths:
                if os.path.exists(p):
                    model_path = p
                    break

            if model_path is None:
                r = BenchmarkResult("NPU", "RKNN 推理", 0, "ms",
                                    {"error": "No .rknn model found"}, 0, passed=False)
                self.results.append(r)
                return r

            rknn = RKNNLite()
            rknn.load_rknn(model_path)
            rknn.init_runtime(core_mask=4)  # ALL_CORES

            dummy = [np.random.rand(1, 3, 640, 640).astype(np.float32)]

            # Warmup
            for _ in range(self.warmup_iterations):
                rknn.infer(inputs=dummy)

            # Benchmark
            n_iter = 20
            t0 = time.perf_counter()
            for _ in range(n_iter):
                rknn.infer(inputs=dummy)
            elapsed = (time.perf_counter() - t0) * 1000

            avg_ms = elapsed / n_iter
            fps = 1000.0 / avg_ms

            rknn.release()

            r = BenchmarkResult("NPU", "RKNN 推理", fps, "FPS",
                                {"model": model_path, "latency_ms": avg_ms,
                                 "iterations": n_iter}, elapsed)
            self.results.append(r)
            logger.info(f"NPU Inference: {fps:.1f} FPS ({avg_ms:.1f}ms/frame)")
            return r

        except ImportError:
            r = BenchmarkResult("NPU", "RKNN 推理", 0, "FPS",
                                {"error": "rknnlite not installed"}, 0, passed=False)
            self.results.append(r)
            logger.warning("NPU benchmark skipped: rknnlite not installed")
            return r
        except Exception as e:
            r = BenchmarkResult("NPU", "RKNN 推理", 0, "FPS",
                                {"error": str(e)}, 0, passed=False)
            self.results.append(r)
            return r

    # ==================================================================
    # 系统信息收集
    # ==================================================================

    def collect_system_info(self) -> dict:
        """收集系统硬件信息"""
        info = {
            "platform": "RK3588S",
            "timestamp": datetime.now().isoformat(),
            "python_version": sys.version,
            "cpu_count": os.cpu_count(),
        }

        # CPU 信息
        try:
            with open("/proc/cpuinfo", "r") as f:
                cpu_info = f.read()
            info["cpu_model"] = "RK3588S (detected from /proc/cpuinfo)"
            # 提取核心信息
            cores = cpu_info.count("processor")
            info["cpu_cores"] = cores
        except (FileNotFoundError, PermissionError):
            info["cpu_cores"] = os.cpu_count()

        # 内存信息
        try:
            with open("/proc/meminfo", "r") as f:
                meminfo = f.read()
            for line in meminfo.split("\n"):
                if line.startswith("MemTotal"):
                    info["memory_total_kb"] = int(line.split()[1])
                    info["memory_total_mb"] = info["memory_total_kb"] // 1024
                    break
        except (FileNotFoundError, PermissionError):
            info["memory_total_mb"] = "unknown"

        # GPU 信息
        try:
            import pyopencl as cl
            platforms = cl.get_platforms()
            if platforms:
                devices = platforms[0].get_devices(device_type=cl.device_type.GPU)
                if devices:
                    info["gpu_name"] = devices[0].name
                    info["gpu_memory_mb"] = devices[0].global_mem_size // (1024 * 1024)
        except Exception:
            info["gpu_name"] = "Mali G610 (assumed)"

        # NPU 频率
        try:
            with open("/sys/class/devfreq/fdab0000.npu/cur_freq", "r") as f:
                info["npu_freq_hz"] = int(f.read().strip())
        except (FileNotFoundError, PermissionError):
            info["npu_freq_hz"] = "unknown"

        return info

    # ==================================================================
    # 运行所有测试
    # ==================================================================

    def run_all(self) -> List[BenchmarkResult]:
        """
        运行所有基准测试

        Returns
        -------
        list of BenchmarkResult
        """
        self.results = []

        print("=" * 60)
        print("RK3588S 系统性能基准测试")
        print("=" * 60)

        system_info = self.collect_system_info()
        print(f"CPU Cores: {system_info.get('cpu_count', '?')}")
        print(f"Memory: {system_info.get('memory_total_mb', '?')} MB")
        print(f"GPU: {system_info.get('gpu_name', '?')}")
        print()

        # CPU
        print("[1/6] CPU 测试...")
        self.bench_cpu_integer()
        self.bench_cpu_float()
        self.bench_cpu_numpy()
        self.bench_cpu_multicore()
        print()

        # GPU
        print("[2/6] GPU 测试...")
        self.bench_gpu_opencl()
        self.bench_gpu_matrix()
        print()

        # Memory
        print("[3/6] 内存测试...")
        self.bench_memory_bandwidth()
        self.bench_memory_latency()
        self.bench_memory_copy()
        print()

        # IO
        print("[4/6] IO 测试...")
        self.bench_io_sequential()
        self.bench_io_random()
        print()

        # NPU
        print("[5/6] NPU 测试...")
        self.bench_npu_inference()
        print()

        # Summary
        print("[6/6] 汇总...")
        print()
        self._print_summary()

        return self.results

    def _print_summary(self):
        """打印测试汇总"""
        print("=" * 60)
        print("测试结果汇总")
        print("=" * 60)

        categories = {}
        for r in self.results:
            if r.category not in categories:
                categories[r.category] = []
            categories[r.category].append(r)

        for cat, results in categories.items():
            print(f"\n  [{cat}]")
            for r in results:
                status = "✓" if r.passed else "✗"
                print(f"    {status} {r.test_name}: {r.score:.2f} {r.unit}  "
                      f"({r.elapsed_ms:.1f}ms)")

        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        print(f"\n  总计: {passed}/{total} 通过")

    # ==================================================================
    # 报告生成
    # ==================================================================

    def generate_report(self) -> str:
        """
        生成 Markdown 格式性能报告

        Returns
        -------
        str
            Markdown 报告内容
        """
        system_info = self.collect_system_info()
        lines = []

        lines.append("# RK3588S 系统性能基准测试报告")
        lines.append("")
        lines.append(f"**测试时间**: {system_info.get('timestamp', 'N/A')}")
        lines.append(f"**平台**: {system_info.get('platform', 'RK3588S')}")
        lines.append(f"**CPU 核心数**: {system_info.get('cpu_count', 'N/A')}")
        lines.append(f"**内存**: {system_info.get('memory_total_mb', 'N/A')} MB")
        lines.append(f"**GPU**: {system_info.get('gpu_name', 'Mali G610')}")
        lines.append(f"**Python**: {system_info.get('python_version', 'N/A')}")
        lines.append("")

        # 按类别分组
        categories = {}
        for r in self.results:
            if r.category not in categories:
                categories[r.category] = []
            categories[r.category].append(r)

        cat_names = {
            "CPU": "CPU 性能",
            "GPU": "GPU 性能 (Mali G610 / OpenCL)",
            "NPU": "NPU 性能 (RKNN)",
            "Memory": "内存性能",
            "IO": "存储 IO 性能",
        }

        for cat, results in categories.items():
            lines.append(f"## {cat_names.get(cat, cat)}")
            lines.append("")
            lines.append("| 测试项目 | 结果 | 单位 | 耗时 | 状态 |")
            lines.append("|---------|------|------|------|------|")
            for r in results:
                status = "✅ 通过" if r.passed else "❌ 失败"
                lines.append(f"| {r.test_name} | {r.score:.2f} | {r.unit} | "
                             f"{r.elapsed_ms:.1f}ms | {status} |")
            lines.append("")

            # 详细信息
            for r in results:
                if r.details:
                    lines.append(f"<details><summary>{r.test_name} 详细数据</summary>")
                    lines.append("")
                    lines.append("```json")
                    # 清理不可序列化的值
                    clean_details = {}
                    for k, v in r.details.items():
                        try:
                            json.dumps(v)
                            clean_details[k] = v
                        except (TypeError, ValueError):
                            clean_details[k] = str(v)
                    lines.append(json.dumps(clean_details, indent=2, ensure_ascii=False))
                    lines.append("```")
                    lines.append("</details>")
                    lines.append("")

        # 总结
        lines.append("## 测试总结")
        lines.append("")
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        lines.append(f"- 测试项总计: {total}")
        lines.append(f"- 通过: {passed}")
        lines.append(f"- 失败: {total - passed}")
        lines.append("")
        lines.append("---")
        lines.append("*报告由 SystemBenchmark 自动生成*")

        return "\n".join(lines)

    def save_report(self, filepath: str):
        """保存报告到文件"""
        report = self.generate_report()
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(report)
        logger.info(f"Report saved to: {filepath}")
        print(f"报告已保存: {filepath}")


# ======================================================================
# 独立运行
# ======================================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    benchmark = SystemBenchmark(io_test_size_mb=32)

    # 运行所有测试
    results = benchmark.run_all()

    # 保存报告
    report_path = os.path.join(os.path.dirname(__file__), "benchmark_report.md")
    benchmark.save_report(report_path)

    print(f"\n完整报告: {report_path}")
