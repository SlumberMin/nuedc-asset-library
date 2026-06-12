"""
Flash磨损均衡仿真 - 静态/动态均衡/寿命预测
nuedc-asset-library V3
"""
import random
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from enum import Enum
import time

class BlockState(Enum):
    FREE = "free"
    ACTIVE = "active"
    WORN = "worn"        # 超过磨损阈值
    BAD = "bad"          # 坏块

@dataclass
class FlashBlock:
    block_id: int
    erase_count: int = 0
    program_count: int = 0
    state: BlockState = BlockState.FREE
    data_valid: bool = False
    static_data: bool = False  # 是否为静态数据
    last_access_cycle: int = 0

    # 典型NAND参数
    max_erase_cycles: int = 100000
    wear_threshold: int = 80000  # 进入worn状态

    @property
    def remaining_life_pct(self) -> float:
        return max(0, 1 - self.erase_count / self.max_erase_cycles) * 100

    @property
    def is_available(self) -> bool:
        return self.state in (BlockState.FREE, BlockState.ACTIVE)

    def erase(self) -> bool:
        if self.state == BlockState.BAD:
            return False
        self.erase_count += 1
        if self.erase_count >= self.max_erase_cycles:
            self.state = BlockState.BAD
            return False
        elif self.erase_count >= self.wear_threshold:
            self.state = BlockState.WORN
        self.data_valid = False
        self.state = BlockState.FREE if self.state == BlockState.WORN else self.state
        return True

    def program(self, cycle: int) -> bool:
        if self.state == BlockState.BAD:
            return False
        self.program_count += 1
        self.data_valid = True
        self.state = BlockState.ACTIVE
        self.last_access_cycle = cycle
        return True


class WearLevelingStrategy(Enum):
    NONE = "none"
    DYNAMIC = "dynamic"      # 仅均衡活跃块
    STATIC = "static"        # 静态+动态均衡
    HYBRID = "hybrid"        # 智能混合策略


@dataclass
class FlashConfig:
    total_blocks: int = 1024
    pages_per_block: int = 64
    page_size_bytes: int = 2048
    max_erase_cycles: int = 100000
    strategy: WearLevelingStrategy = WearLevelingStrategy.HYBRID
    gc_threshold_pct: float = 0.85  # GC触发阈值
    static_move_threshold: int = 5000  # 静态数据迁移擦写差

    @property
    def total_capacity_mb(self) -> float:
        return self.total_blocks * self.pages_per_block * self.page_size_bytes / 1024 / 1024


class FlashWearLevelingSimulator:
    """Flash磨损均衡仿真器"""

    def __init__(self, config: FlashConfig = None):
        self.config = config or FlashConfig()
        self.blocks = [
            FlashBlock(i, max_erase_cycles=self.config.max_erase_cycles)
            for i in range(self.config.total_blocks)
        ]
        self.write_pointer = 0
        self.cycle = 0
        self.gc_count = 0
        self.static_migrations = 0
        self.stats_history: List[dict] = []

    def _select_block_dynamic(self) -> Optional[FlashBlock]:
        """动态均衡：选择擦写次数最少的空闲块"""
        free = [b for b in self.blocks if b.state == BlockState.FREE]
        if not free:
            return None
        return min(free, key=lambda b: b.erase_count)

    def _select_block_sequential(self) -> Optional[FlashBlock]:
        """顺序写入（无均衡）"""
        for _ in range(self.config.total_blocks):
            b = self.blocks[self.write_pointer]
            self.write_pointer = (self.write_pointer + 1) % self.config.total_blocks
            if b.state == BlockState.FREE:
                return b
        return None

    def _select_block_static(self) -> Optional[FlashBlock]:
        """静态均衡：优先选擦写最少的，但也会迁移冷数据"""
        free = [b for b in self.blocks if b.state == BlockState.FREE]
        if not free:
            return None

        avg_erase = np.mean([b.erase_count for b in self.blocks if b.state != BlockState.BAD])
        # 查找静态数据块（长期未访问且擦写次数高于均值）
        for b in self.blocks:
            if (b.state == BlockState.ACTIVE and b.static_data and
                b.erase_count > avg_erase + self.config.static_move_threshold):
                self.static_migrations += 1
                # 模拟数据迁移：释放旧块，写入新块
                b.static_data = False

        return min(free, key=lambda b: b.erase_count)

    def _select_block_hybrid(self) -> Optional[FlashBlock]:
        """混合策略"""
        free_blocks = [b for b in self.blocks if b.state == BlockState.FREE]
        if not free_blocks:
            return None

        erase_counts = [b.erase_count for b in self.blocks if b.state != BlockState.BAD]
        avg_erase = np.mean(erase_counts) if erase_counts else 0
        max_erase = max(erase_counts) if erase_counts else 0

        # 如果磨损差距大，用静态均衡
        if max_erase > avg_erase * 1.5 and free_blocks:
            return self._select_block_static()
        # 否则用动态均衡
        return self._select_block_dynamic()

    def _garbage_collect(self):
        """垃圾回收"""
        free_count = sum(1 for b in self.blocks if b.state == BlockState.FREE)
        if free_count / self.config.total_blocks < (1 - self.config.gc_threshold_pct):
            # 选择最"冷"的活跃块进行GC
            active = [b for b in self.blocks if b.state == BlockState.ACTIVE]
            active.sort(key=lambda b: b.last_access_cycle)
            gc_count = min(len(active) // 4, 10)
            for b in active[:gc_count]:
                b.erase()
            self.gc_count += gc_count

    def select_block(self) -> Optional[FlashBlock]:
        strategy = self.config.strategy
        if strategy == WearLevelingStrategy.NONE:
            return self._select_block_sequential()
        elif strategy == WearLevelingStrategy.DYNAMIC:
            return self._select_block_dynamic()
        elif strategy == WearLevelingStrategy.STATIC:
            return self._select_block_static()
        else:
            return self._select_block_hybrid()

    def write_data(self, is_static: bool = False) -> bool:
        """写入数据"""
        self._garbage_collect()
        block = self.select_block()
        if block is None:
            return False
        if not block.erase():
            return False
        block.program(self.cycle)
        block.static_data = is_static
        return True

    def simulate(self, num_writes: int = 50000, static_ratio: float = 0.2) -> dict:
        """运行仿真"""
        success = 0
        fail = 0

        for i in range(num_writes):
            self.cycle = i
            is_static = random.random() < static_ratio
            if self.write_data(is_static):
                success += 1
            else:
                fail += 1

            # 每1000次记录一次
            if (i + 1) % 1000 == 0:
                self.stats_history.append(self._get_stats())

        return self._analyze()

    def _get_stats(self) -> dict:
        erase_counts = [b.erase_count for b in self.blocks if b.state != BlockState.BAD]
        return {
            "cycle": self.cycle,
            "avg_erase": np.mean(erase_counts) if erase_counts else 0,
            "max_erase": max(erase_counts) if erase_counts else 0,
            "min_erase": min(erase_counts) if erase_counts else 0,
            "std_erase": np.std(erase_counts) if erase_counts else 0,
            "bad_blocks": sum(1 for b in self.blocks if b.state == BlockState.BAD),
            "free_blocks": sum(1 for b in self.blocks if b.state == BlockState.FREE),
        }

    def _analyze(self) -> dict:
        erase_counts = [b.erase_count for b in self.blocks]
        non_zero = [e for e in erase_counts if e > 0]

        # 预测寿命
        avg_erase = np.mean(non_zero) if non_zero else 0
        max_cycles = self.config.max_erase_cycles
        remaining_pct = (1 - avg_erase / max_cycles) * 100 if max_cycles > 0 else 100

        # 磨损均匀度（越低越好）
        uniformity = np.std(erase_counts) / avg_erase if avg_erase > 0 else 0

        return {
            "strategy": self.config.strategy.value,
            "total_writes": self.cycle + 1,
            "gc_count": self.gc_count,
            "static_migrations": self.static_migrations,
            "avg_erase_count": avg_erase,
            "max_erase_count": max(erase_counts),
            "min_erase_count": min(non_zero) if non_zero else 0,
            "std_erase_count": np.std(erase_counts),
            "wear_uniformity_cv": uniformity,  # 变异系数
            "bad_blocks": sum(1 for b in self.blocks if b.state == BlockState.BAD),
            "remaining_life_pct": remaining_pct,
            "estimated_writes_until_failure": int(avg_erase * max_cycles) if avg_erase > 0 else 0,
            "erase_distribution": {
                "p10": np.percentile(erase_counts, 10),
                "p50": np.percentile(erase_counts, 50),
                "p90": np.percentile(erase_counts, 90),
                "p99": np.percentile(erase_counts, 99),
            },
        }


# ── 演示 ──
def demo():
    print("=" * 60)
    print("Flash磨损均衡仿真 - Demo")
    print("=" * 60)

    for strategy in WearLevelingStrategy:
        cfg = FlashConfig(
            total_blocks=256,
            max_erase_cycles=10000,
            strategy=strategy,
        )
        sim = FlashWearLevelingSimulator(cfg)
        result = sim.simulate(num_writes=20000, static_ratio=0.2)

        print(f"\n[{strategy.value.upper()} 策略]")
        print(f"  总写入: {result['total_writes']}")
        print(f"  平均擦写: {result['avg_erase_count']:.1f}")
        print(f"  最大擦写: {result['max_erase_count']}")
        print(f"  磨损均匀度CV: {result['wear_uniformity_cv']:.3f}")
        print(f"  坏块: {result['bad_blocks']}")
        print(f"  剩余寿命: {result['remaining_life_pct']:.1f}%")
        print(f"  GC次数: {result['gc_count']}")
        print(f"  静态迁移: {result['static_migrations']}")

    print("\n✅ Flash磨损均衡仿真完成")


if __name__ == "__main__":
    demo()
