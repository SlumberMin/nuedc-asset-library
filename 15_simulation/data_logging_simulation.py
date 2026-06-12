#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据记录仿真 - 采样/存储/压缩/检索/分析
模拟嵌入式系统的数据采集、存储、压缩与检索全流程
"""

import time
import random
import math
import hashlib
import struct
import zlib
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict, Optional, Tuple, Any
from collections import deque
from datetime import datetime, timedelta


class SamplingMode(Enum):
    CONTINUOUS = auto()       # 连续采样
    PERIODIC = auto()         # 周期采样
    EVENT_TRIGGERED = auto()  # 事件触发
    THRESHOLD = auto()        # 阈值触发
    BURST = auto()            # 突发采样


class StorageFormat(Enum):
    RAW = auto()
    CSV = auto()
    BINARY = auto()
    COMPRESSED = auto()
    INDEXED = auto()


class CompressionMethod(Enum):
    NONE = auto()
    RLE = auto()       # 游程编码
    DELTA = auto()     # 差分编码
    LZW = auto()       # LZW压缩
    ZLIB = auto()      # zlib压缩


@dataclass
class SamplePoint:
    """采样数据点"""
    timestamp: float
    channel: str
    value: float
    quality: int = 100  # 0-100
    seq_num: int = 0


@dataclass
class DataRecord:
    """数据记录"""
    record_id: int
    samples: List[SamplePoint]
    format: StorageFormat
    size_bytes: int
    checksum: str
    compressed: bool = False


@dataclass
class QueryResult:
    """查询结果"""
    records: List[SamplePoint]
    total_count: int
    query_time_ms: float
    from_cache: bool = False


@dataclass
class StatisticalSummary:
    """统计摘要"""
    channel: str
    count: int
    mean: float
    std_dev: float
    min_val: float
    max_val: float
    median: float
    percentiles: Dict[int, float] = field(default_factory=dict)


class Sampler:
    """数据采样器"""

    def __init__(self, mode: SamplingMode = SamplingMode.CONTINUOUS, rate_hz: float = 1000):
        self.mode = mode
        self.rate_hz = rate_hz
        self.sample_interval = 1.0 / rate_hz
        self.channels: Dict[str, dict] = {}
        self.seq_counter = 0
        self.total_samples = 0
        self.dropped_samples = 0
        self.buffer: deque = deque(maxlen=100000)

    def add_channel(self, name: str, unit: str = "", range_min: float = 0, range_max: float = 3.3):
        self.channels[name] = {
            "unit": unit, "range": (range_min, range_max),
            "enabled": True, "offset": 0, "gain": 1.0
        }

    def generate_sample(self, channel: str, timestamp: float) -> SamplePoint:
        ch = self.channels.get(channel, {"range": (0, 3.3), "offset": 0, "gain": 1.0})
        raw = random.uniform(*ch["range"])
        value = raw * ch.get("gain", 1.0) + ch.get("offset", 0)
        quality = 100 if random.random() > 0.02 else random.randint(50, 90)
        self.seq_counter += 1
        sp = SamplePoint(timestamp, channel, round(value, 6), quality, self.seq_counter)
        self.buffer.append(sp)
        self.total_samples += 1
        return sp

    def sample_batch(self, count: int, channels: List[str] = None) -> List[SamplePoint]:
        channels = channels or list(self.channels.keys())
        samples = []
        t = time.time()
        for i in range(count):
            for ch in channels:
                if self.channels.get(ch, {}).get("enabled", True):
                    samples.append(self.generate_sample(ch, t + i * self.sample_interval))
        return samples

    def set_calibration(self, channel: str, offset: float, gain: float):
        if channel in self.channels:
            self.channels[channel]["offset"] = offset
            self.channels[channel]["gain"] = gain

    def get_statistics(self) -> dict:
        return {
            "mode": self.mode.name, "rate_hz": self.rate_hz,
            "channels": len(self.channels), "total_samples": self.total_samples,
            "dropped": self.dropped_samples, "buffer_size": len(self.buffer),
        }


class DataCompressor:
    """数据压缩器"""

    def __init__(self, method: CompressionMethod = CompressionMethod.DELTA):
        self.method = method
        self.compressed_bytes = 0
        self.original_bytes = 0

    def compress(self, values: List[float]) -> bytes:
        raw = struct.pack(f'{len(values)}d', *values)
        self.original_bytes += len(raw)

        if self.method == CompressionMethod.NONE:
            result = raw
        elif self.method == CompressionMethod.RLE:
            result = self._rle_compress(values)
        elif self.method == CompressionMethod.DELTA:
            result = self._delta_compress(values)
        elif self.method == CompressionMethod.ZLIB:
            result = zlib.compress(raw, 6)
        else:
            result = raw

        self.compressed_bytes += len(result)
        return result

    def _rle_compress(self, values: List[float]) -> bytes:
        if not values:
            return b''
        result = bytearray()
        prev = values[0]
        count = 1
        for v in values[1:]:
            if abs(v - prev) < 1e-10:
                count += 1
            else:
                result.extend(struct.pack('dI', prev, count))
                prev = v
                count = 1
        result.extend(struct.pack('dI', prev, count))
        return bytes(result)

    def _delta_compress(self, values: List[float]) -> bytes:
        if not values:
            return b''
        result = bytearray(struct.pack('d', values[0]))
        for i in range(1, len(values)):
            delta = values[i] - values[i - 1]
            result.extend(struct.pack('f', delta))
        return bytes(result)

    def get_ratio(self) -> float:
        if self.original_bytes == 0:
            return 0
        return 1.0 - self.compressed_bytes / self.original_bytes


class DataStorage:
    """数据存储引擎"""

    def __init__(self, max_records: int = 1000000):
        self.max_records = max_records
        self.records: Dict[int, DataRecord] = {}
        self.index: Dict[str, List[int]] = {}  # channel -> record_ids
        self.next_id = 1
        self.total_size_bytes = 0
        self.compressor = DataCompressor(CompressionMethod.DELTA)

    def store(self, samples: List[SamplePoint], fmt: StorageFormat = StorageFormat.COMPRESSED) -> DataRecord:
        values = [s.value for s in samples]
        raw_size = len(values) * 8

        if fmt == StorageFormat.COMPRESSED:
            data = self.compressor.compress(values)
            size = len(data)
            compressed = True
        else:
            data = struct.pack(f'{len(values)}d', *values)
            size = raw_size
            compressed = False

        checksum = hashlib.md5(data).hexdigest()[:16]
        record = DataRecord(self.next_id, samples, fmt, size, checksum, compressed)
        self.records[self.next_id] = record

        for s in samples:
            self.index.setdefault(s.channel, []).append(self.next_id)

        self.total_size_bytes += size
        self.next_id += 1
        return record

    def query(self, channel: str, start_time: float = 0, end_time: float = float('inf'),
              limit: int = 10000) -> QueryResult:
        t0 = time.time()
        record_ids = self.index.get(channel, [])
        results = []
        for rid in record_ids:
            rec = self.records.get(rid)
            if rec:
                for s in rec.samples:
                    if start_time <= s.timestamp <= end_time:
                        results.append(s)
                        if len(results) >= limit:
                            break
        elapsed = (time.time() - t0) * 1000
        return QueryResult(results, len(results), elapsed)

    def get_storage_stats(self) -> dict:
        return {
            "records": len(self.records), "total_size_bytes": self.total_size_bytes,
            "compression_ratio": self.compressor.get_ratio(),
            "channels_indexed": len(self.index),
        }


class DataAnalyzer:
    """数据分析器"""

    @staticmethod
    def compute_statistics(samples: List[SamplePoint]) -> StatisticalSummary:
        if not samples:
            return StatisticalSummary("", 0, 0, 0, 0, 0, 0)

        values = sorted([s.value for s in samples])
        n = len(values)
        mean = sum(values) / n
        variance = sum((v - mean) ** 2 for v in values) / n
        std = math.sqrt(variance)

        percentiles = {}
        for p in [5, 25, 50, 75, 95]:
            idx = int(n * p / 100)
            percentiles[p] = values[min(idx, n - 1)]

        return StatisticalSummary(
            channel=samples[0].channel, count=n, mean=round(mean, 6),
            std_dev=round(std, 6), min_val=values[0], max_val=values[-1],
            median=values[n // 2], percentiles=percentiles
        )

    @staticmethod
    def moving_average(samples: List[SamplePoint], window: int = 10) -> List[float]:
        values = [s.value for s in samples]
        result = []
        for i in range(len(values)):
            start = max(0, i - window + 1)
            result.append(sum(values[start:i + 1]) / (i - start + 1))
        return result

    @staticmethod
    def detect_anomalies(samples: List[SamplePoint], threshold_sigma: float = 3.0) -> List[SamplePoint]:
        stats = DataAnalyzer.compute_statistics(samples)
        anomalies = []
        for s in samples:
            if abs(s.value - stats.mean) > threshold_sigma * stats.std_dev:
                anomalies.append(s)
        return anomalies

    @staticmethod
    def compute_fft_magnitude(samples: List[SamplePoint], sample_rate: float) -> List[float]:
        """简化的FFT幅度计算（仅返回前N/2个频率分量幅度）"""
        n = len(samples)
        if n < 2:
            return []
        values = [s.value for s in samples]
        mean = sum(values) / n
        values = [v - mean for v in values]
        magnitudes = []
        half = n // 2
        for k in range(half):
            re = sum(values[m] * math.cos(2 * math.pi * k * m / n) for m in range(n))
            im = sum(values[m] * math.sin(2 * math.pi * k * m / n) for m in range(n))
            magnitudes.append(math.sqrt(re ** 2 + im ** 2) / n)
        return magnitudes


class DataLoggingSimulator:
    """数据记录仿真器"""

    def __init__(self):
        self.sampler = Sampler(SamplingMode.CONTINUOUS, 1000)
        self.storage = DataStorage()
        self.analyzer = DataAnalyzer()
        self.log_entries: List[dict] = []

    def _log(self, msg: str, level: str = "INFO"):
        entry = {"time": time.time(), "level": level, "msg": msg}
        self.log_entries.append(entry)

    def setup_channels(self, channels: List[Tuple[str, str, float, float]]):
        for name, unit, rmin, rmax in channels:
            self.sampler.add_channel(name, unit, rmin, rmax)
            self._log(f"添加通道: {name} ({unit}) 范围=[{rmin},{rmax}]")

    def run_sampling_cycle(self, duration_sec: float = 1.0) -> List[SamplePoint]:
        count = int(duration_sec * self.sampler.rate_hz)
        samples = self.sampler.sample_batch(count)
        self._log(f"采样周期: {duration_sec}s, 生成 {len(samples)} 个样本")
        return samples

    def store_samples(self, samples: List[SamplePoint]) -> DataRecord:
        record = self.storage.store(samples)
        self._log(f"存储记录 #{record.record_id}: {len(samples)} 样本, {record.size_bytes} 字节")
        return record

    def analyze_channel(self, channel: str) -> StatisticalSummary:
        qr = self.storage.query(channel)
        stats = self.analyzer.compute_statistics(qr.records)
        self._log(f"分析通道 '{channel}': {stats.count} 样本, 均值={stats.mean:.4f}")
        return stats

    def run_full_pipeline(self) -> dict:
        """运行完整的数据记录流水线"""
        self.setup_channels([
            ("voltage", "V", 0, 5),
            ("current", "A", 0, 2),
            ("temperature", "°C", 20, 80),
        ])

        results = {}

        # 采样
        samples = self.run_sampling_cycle(2.0)

        # 按通道分组存储
        channel_samples: Dict[str, List[SamplePoint]] = {}
        for s in samples:
            channel_samples.setdefault(s.channel, []).append(s)

        records = []
        for ch, ch_samples in channel_samples.items():
            rec = self.store_samples(ch_samples)
            records.append(rec)
        results["records"] = len(records)

        # 分析
        stats = {}
        for ch in channel_samples:
            stats[ch] = self.analyze_channel(ch)
        results["stats"] = stats

        # 异常检测
        for ch, ch_samples in channel_samples.items():
            anomalies = self.analyzer.detect_anomalies(ch_samples)
            results[f"anomalies_{ch}"] = len(anomalies)

        # 压缩统计
        results["storage"] = self.storage.get_storage_stats()
        results["sampler"] = self.sampler.get_statistics()

        return results


def run_data_logging_simulation():
    """运行数据记录仿真"""
    print("=" * 60)
    print("数据记录仿真 - 采样/存储/压缩/检索/分析")
    print("=" * 60)

    sim = DataLoggingSimulator()

    # 1. 通道配置
    print("\n--- 1. 通道配置 ---")
    sim.setup_channels([
        ("voltage", "V", 0, 5),
        ("current", "A", 0, 2),
        ("temperature", "°C", 20, 80),
    ])
    for name, ch in sim.sampler.channels.items():
        print(f"  {name}: 单位={ch['unit']}, 范围={ch['range']}")

    # 2. 采样仿真
    print("\n--- 2. 采样仿真 ---")
    samples = sim.run_sampling_cycle(1.0)
    channel_counts = {}
    for s in samples:
        channel_counts[s.channel] = channel_counts.get(s.channel, 0) + 1
    for ch, cnt in channel_counts.items():
        print(f"  {ch}: {cnt} 样本")
    print(f"  总样本数: {len(samples)}")

    # 3. 压缩对比
    print("\n--- 3. 压缩方法对比 ---")
    values = [s.value for s in samples if s.channel == "voltage"]
    for method in CompressionMethod:
        comp = DataCompressor(method)
        compressed = comp.compress(values)
        ratio = comp.get_ratio()
        print(f"  {method.name:12s}: 原始={comp.original_bytes:>8d}B -> 压缩={comp.compressed_bytes:>8d}B  比率={ratio:.2%}")

    # 4. 存储与查询
    print("\n--- 4. 存储与查询 ---")
    for ch in ["voltage", "current", "temperature"]:
        ch_samples = [s for s in samples if s.channel == ch]
        rec = sim.store_samples(ch_samples)
        print(f"  存储 {ch}: 记录#{rec.record_id}, {rec.size_bytes}B, checksum={rec.checksum}")
    storage_stats = sim.storage.get_storage_stats()
    print(f"  总记录: {storage_stats['records']}, 总大小: {storage_stats['total_size_bytes']}B")
    print(f"  压缩率: {storage_stats['compression_ratio']:.2%}")

    qr = sim.storage.query("voltage")
    print(f"  查询 'voltage': {qr.total_count} 结果, 耗时 {qr.query_time_ms:.3f}ms")

    # 5. 统计分析
    print("\n--- 5. 统计分析 ---")
    for ch in ["voltage", "current", "temperature"]:
        stats = sim.analyze_channel(ch)
        print(f"  {ch}: 均值={stats.mean:.4f}, 标准差={stats.std_dev:.4f}, "
              f"范围=[{stats.min_val:.4f}, {stats.max_val:.4f}], 中位数={stats.median:.4f}")
        if stats.percentiles:
            pstr = ", ".join(f"P{p}={v:.4f}" for p, v in sorted(stats.percentiles.items()))
            print(f"    百分位: {pstr}")

    # 6. 异常检测
    print("\n--- 6. 异常检测 ---")
    for ch in ["voltage", "current", "temperature"]:
        ch_samples = [s for s in samples if s.channel == ch]
        anomalies = sim.analyzer.detect_anomalies(ch_samples, 2.5)
        print(f"  {ch}: {len(anomalies)}/{len(ch_samples)} 个异常 (2.5σ)")

    # 7. 移动平均
    print("\n--- 7. 移动平均滤波 ---")
    v_samples = [s for s in samples if s.channel == "voltage"][:50]
    ma = sim.analyzer.moving_average(v_samples, 10)
    print(f"  电压前50点移动平均(w=10): 最小={min(ma):.4f}, 最大={max(ma):.4f}")

    # 8. FFT频谱分析
    print("\n--- 8. 频谱分析 ---")
    # 创建含已知频率的信号
    sim2 = DataLoggingSimulator()
    sim2.sampler.add_channel("test", "V", -1, 1)
    test_samples = []
    for i in range(256):
        t = i / 1000.0
        val = 0.5 * math.sin(2 * math.pi * 50 * t) + 0.3 * math.sin(2 * math.pi * 120 * t)
        test_samples.append(SamplePoint(t, "test", val, 100, i))
    magnitudes = sim2.analyzer.compute_fft_magnitude(test_samples, 1000)
    if magnitudes:
        peak_indices = sorted(range(len(magnitudes)), key=lambda i: magnitudes[i], reverse=True)[:3]
        print(f"  FFT峰值频率分量:")
        for idx in peak_indices:
            freq = idx * 1000 / 256
            print(f"    bin={idx}, 频率≈{freq:.1f}Hz, 幅度={magnitudes[idx]:.4f}")

    # 9. 多周期采样与存储
    print("\n--- 9. 多周期仿真 ---")
    for cycle in range(3):
        s = sim.run_sampling_cycle(0.5)
        for ch in sim.sampler.channels:
            ch_s = [x for x in s if x.channel == ch]
            if ch_s:
                sim.store_samples(ch_s)
    print(f"  3个周期后总记录: {len(sim.storage.records)}")
    print(f"  总样本数: {sim.sampler.total_samples}")

    # 10. 日志统计
    print("\n--- 10. 操作日志 ---")
    for entry in sim.log_entries[-8:]:
        print(f"  [{entry['level']}] {entry['msg']}")

    print("\n" + "=" * 60)
    print("数据记录仿真完成")
    print("=" * 60)


if __name__ == "__main__":
    run_data_logging_simulation()
