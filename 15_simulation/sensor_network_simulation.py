"""
传感器网络仿真模块 - Sensor Network Simulation
================================================
功能: 网络拓扑、路由协议、数据汇聚、能耗分析
适用: 电赛物联网/传感器网络题目设计与优化
"""

import numpy as np
from typing import Tuple, List, Dict, Optional, Set
from collections import defaultdict
import heapq
import warnings
warnings.filterwarnings('ignore')


# ─────────────────────────────────────────────
# 1. 网络拓扑
# ─────────────────────────────────────────────

class SensorNode:
    """传感器节点"""
    
    def __init__(self, node_id: int, x: float, y: float,
                 initial_energy_J: float = 0.5, sensing_range_m: float = 50,
                 comm_range_m: float = 100, is_cluster_head: bool = False):
        self.id = node_id
        self.x = x
        self.y = y
        self.energy = initial_energy_J
        self.initial_energy = initial_energy_J
        self.sensing_range = sensing_range_m
        self.comm_range = comm_range_m
        self.is_cluster_head = is_cluster_head
        self.is_alive = True
        self.data_buffer = []
        self.packets_sent = 0
        self.packets_received = 0
        self.cluster_id = None
    
    def distance_to(self, other: 'SensorNode') -> float:
        """到另一个节点的距离"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def can_communicate(self, other: 'SensorNode') -> bool:
        """是否在通信范围内"""
        return self.distance_to(other) <= self.comm_range and self.is_alive and other.is_alive
    
    def consume_energy(self, amount_J: float):
        """消耗能量"""
        self.energy -= amount_J
        if self.energy <= 0:
            self.energy = 0
            self.is_alive = False
    
    def energy_ratio(self) -> float:
        """剩余能量比"""
        return self.energy / self.initial_energy if self.initial_energy > 0 else 0


class SensorNetwork:
    """传感器网络"""
    
    def __init__(self, area_width: float, area_height: float):
        self.width = area_width
        self.height = area_height
        self.nodes: List[SensorNode] = []
        self.base_station: Optional[Tuple[float, float]] = None
        self.adjacency = defaultdict(list)
        self.edges = []
    
    def add_node(self, node: SensorNode):
        """添加节点"""
        self.nodes.append(node)
        self._update_adjacency(node)
    
    def _update_adjacency(self, new_node: SensorNode):
        """更新邻接关系"""
        for existing in self.nodes[:-1]:
            if new_node.can_communicate(existing):
                self.adjacency[new_node.id].append(existing.id)
                self.adjacency[existing.id].append(new_node.id)
                self.edges.append((new_node.id, existing.id))
    
    def set_base_station(self, x: float, y: float):
        """设置基站位置"""
        self.base_station = (x, y)
    
    @staticmethod
    def random_network(n_nodes: int, area_width: float, area_height: float,
                       comm_range: float = 100, energy_J: float = 0.5,
                       bs_position: str = 'center') -> 'SensorNetwork':
        """
        随机生成传感器网络
        
        Args:
            n_nodes: 节点数
            area_width, area_height: 区域尺寸
            comm_range: 通信范围
            energy_J: 初始能量
            bs_position: 基站位置 'center'/'corner'/'edge'
        """
        net = SensorNetwork(area_width, area_height)
        
        for i in range(n_nodes):
            x = np.random.uniform(0, area_width)
            y = np.random.uniform(0, area_height)
            node = SensorNode(i, x, y, initial_energy_J=energy_J, comm_range_m=comm_range)
            net.add_node(node)
        
        # 基站位置
        if bs_position == 'center':
            net.set_base_station(area_width / 2, area_height / 2)
        elif bs_position == 'corner':
            net.set_base_station(0, 0)
        elif bs_position == 'edge':
            net.set_base_station(area_width / 2, area_height)
        
        return net
    
    def get_neighbors(self, node_id: int) -> List[int]:
        """获取邻居节点"""
        return self.adjacency[node_id]
    
    def alive_nodes(self) -> List[SensorNode]:
        """存活节点"""
        return [n for n in self.nodes if n.is_alive]
    
    def dead_nodes(self) -> List[SensorNode]:
        """死亡节点"""
        return [n for n in self.nodes if not n.is_alive]
    
    def network_lifetime(self) -> Dict:
        """网络生命期统计"""
        alive = self.alive_nodes()
        dead = self.dead_nodes()
        energies = [n.energy for n in self.nodes]
        
        return {
            'total_nodes': len(self.nodes),
            'alive': len(alive),
            'dead': len(dead),
            'alive_ratio': len(alive) / len(self.nodes) if self.nodes else 0,
            'avg_energy': np.mean(energies),
            'min_energy': np.min(energies),
            'max_energy': np.max(energies),
            'energy_std': np.std(energies)
        }
    
    def connectivity(self) -> float:
        """网络连通性 (最大连通分量占比)"""
        if not self.nodes:
            return 0
        
        visited = set()
        max_component = 0
        
        for node in self.nodes:
            if node.id not in visited and node.is_alive:
                # BFS
                component = set()
                queue = [node.id]
                while queue:
                    current = queue.pop(0)
                    if current in component:
                        continue
                    component.add(current)
                    visited.add(current)
                    for neighbor in self.adjacency[current]:
                        if neighbor not in component and self.nodes[neighbor].is_alive:
                            queue.append(neighbor)
                max_component = max(max_component, len(component))
        
        alive_count = len(self.alive_nodes())
        return max_component / alive_count if alive_count > 0 else 0
    
    def coverage_ratio(self, sample_points: int = 1000) -> float:
        """覆盖率"""
        if not self.alive_nodes():
            return 0
        
        xs = np.random.uniform(0, self.width, sample_points)
        ys = np.random.uniform(0, self.height, sample_points)
        
        covered = 0
        for px, py in zip(xs, ys):
            for node in self.alive_nodes():
                dist = np.sqrt((px - node.x)**2 + (py - node.y)**2)
                if dist <= node.sensing_range:
                    covered += 1
                    break
        
        return covered / sample_points


# ─────────────────────────────────────────────
# 2. 路由协议
# ─────────────────────────────────────────────

class RoutingProtocol:
    """路由协议基类"""
    
    def __init__(self, network: SensorNetwork):
        self.network = network
        self.routes = {}
    
    def find_route(self, src_id: int, dst_id: int) -> List[int]:
        raise NotImplementedError


class FloodingProtocol(RoutingProtocol):
    """洪泛协议"""
    
    def find_route(self, src_id: int, dst_id: int) -> List[int]:
        """洪泛: 广播到所有邻居 (返回最短路径作为代表)"""
        return self._bfs(src_id, dst_id)
    
    def _bfs(self, src: int, dst: int) -> List[int]:
        queue = [(src, [src])]
        visited = {src}
        
        while queue:
            current, path = queue.pop(0)
            if current == dst:
                return path
            
            for neighbor in self.network.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return []  # 无路径


class SPProtocol(RoutingProtocol):
    """最短路径路由 (Dijkstra)"""
    
    def find_route(self, src_id: int, dst_id: int) -> List[int]:
        """Dijkstra最短路径"""
        nodes = self.network.nodes
        dist = {n.id: float('inf') for n in nodes}
        prev = {n.id: None for n in nodes}
        dist[src_id] = 0
        
        pq = [(0, src_id)]
        visited = set()
        
        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            
            if u == dst_id:
                break
            
            for v in self.network.get_neighbors(u):
                if v not in visited:
                    # 边权 = 距离
                    edge_dist = nodes[u].distance_to(nodes[v])
                    new_dist = d + edge_dist
                    if new_dist < dist[v]:
                        dist[v] = new_dist
                        prev[v] = u
                        heapq.heappush(pq, (new_dist, v))
        
        # 回溯路径
        path = []
        current = dst_id
        while current is not None:
            path.append(current)
            current = prev[current]
        
        return path[::-1] if path and path[-1] == src_id else []


class EnergyAwareRouting(RoutingProtocol):
    """能量感知路由"""
    
    def find_route(self, src_id: int, dst_id: int,
                   energy_weight: float = 0.5) -> List[int]:
        """
        能量感知路由: 综合距离和剩余能量
        
        cost = α * distance + (1-α) * (1/remaining_energy)
        """
        nodes = self.network.nodes
        dist = {n.id: float('inf') for n in nodes}
        prev = {n.id: None for n in nodes}
        dist[src_id] = 0
        
        pq = [(0, src_id)]
        visited = set()
        
        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            
            if u == dst_id:
                break
            
            for v in self.network.get_neighbors(u):
                if v not in visited:
                    edge_dist = nodes[u].distance_to(nodes[v])
                    energy_factor = 1.0 / (nodes[v].energy + 1e-10)
                    
                    # 综合代价
                    cost = (1 - energy_weight) * edge_dist + \
                           energy_weight * energy_factor * 100
                    
                    new_dist = d + cost
                    if new_dist < dist[v]:
                        dist[v] = new_dist
                        prev[v] = u
                        heapq.heappush(pq, (new_dist, v))
        
        path = []
        current = dst_id
        while current is not None:
            path.append(current)
            current = prev[current]
        
        return path[::-1] if path and path[-1] == src_id else []


class LEACHProtocol(RoutingProtocol):
    """LEACH分簇路由协议"""
    
    def __init__(self, network: SensorNetwork, p: float = 0.05):
        """
        Args:
            p: 簇头比例 (期望值)
        """
        super().__init__(network)
        self.p = p
        self.round_num = 0
        self.cluster_heads = []
        self.clusters = defaultdict(list)
    
    def setup_phase(self):
        """建立阶段: 选举簇头"""
        self.round_num += 1
        self.cluster_heads = []
        
        # 簇头选举 (基于概率)
        for node in self.network.alive_nodes():
            threshold = self.p / (1 - self.p * (self.round_num % int(1/self.p)))
            if np.random.rand() < threshold:
                node.is_cluster_head = True
                self.cluster_heads.append(node.id)
            else:
                node.is_cluster_head = False
        
        # 如果没有簇头, 强制选一个
        if not self.cluster_heads:
            alive = self.network.alive_nodes()
            if alive:
                ch = np.random.choice(alive)
                ch.is_cluster_head = True
                self.cluster_heads.append(ch.id)
        
        # 节点加入最近的簇头
        self.clusters = defaultdict(list)
        for node in self.network.alive_nodes():
            if not node.is_cluster_head:
                min_dist = float('inf')
                best_ch = None
                for ch_id in self.cluster_heads:
                    ch = self.network.nodes[ch_id]
                    d = node.distance_to(ch)
                    if d < min_dist:
                        min_dist = d
                        best_ch = ch_id
                if best_ch is not None:
                    node.cluster_id = best_ch
                    self.clusters[best_ch].append(node.id)
    
    def find_route(self, src_id: int, dst_id: int) -> List[int]:
        """LEACH路由: 节点→簇头→基站"""
        src = self.network.nodes[src_id]
        
        if src.is_cluster_head:
            # 簇头直接到基站/目标
            return [src_id, dst_id]
        else:
            # 通过簇头转发
            ch_id = src.cluster_id
            if ch_id is not None:
                return [src_id, ch_id, dst_id]
        
        # 回退到最短路径
        sp = SPProtocol(self.network)
        return sp.find_route(src_id, dst_id)
    
    def get_cluster_info(self) -> Dict:
        """获取簇信息"""
        return {
            'num_clusters': len(self.cluster_heads),
            'cluster_heads': self.cluster_heads,
            'clusters': dict(self.clusters),
            'round': self.round_num
        }


# ─────────────────────────────────────────────
# 3. 数据汇聚
# ─────────────────────────────────────────────

class DataAggregation:
    """数据汇聚算法"""
    
    @staticmethod
    def simple_forwarding(data_packets: List[float]) -> List[float]:
        """简单转发 (不汇聚)"""
        return data_packets
    
    @staticmethod
    def average_aggregation(data_packets: List[float]) -> float:
        """均值汇聚"""
        return np.mean(data_packets) if data_packets else 0
    
    @staticmethod
    def max_aggregation(data_packets: List[float]) -> float:
        """最大值汇聚"""
        return np.max(data_packets) if data_packets else 0
    
    @staticmethod
    def min_aggregation(data_packets: List[float]) -> float:
        """最小值汇聚"""
        return np.min(data_packets) if data_packets else 0
    
    @staticmethod
    def median_aggregation(data_packets: List[float]) -> float:
        """中值汇聚"""
        return np.median(data_packets) if data_packets else 0
    
    @staticmethod
    def weighted_average(data_packets: List[float],
                         weights: List[float]) -> float:
        """加权平均汇聚"""
        if not data_packets or not weights:
            return 0
        weights = np.array(weights)
        weights /= weights.sum()
        return np.dot(data_packets, weights)
    
    @staticmethod
    def threshold_aggregation(data_packets: List[float],
                              threshold: float,
                              aggregator='avg') -> Tuple[float, bool]:
        """
        阈值汇聚: 超过阈值才发送
        
        Args:
            data_packets: 数据
            threshold: 阈值
            aggregator: 汇聚方式
        Returns:
            (汇聚值, 是否触发)
        """
        agg_func = {
            'avg': DataAggregation.average_aggregation,
            'max': DataAggregation.max_aggregation,
            'min': DataAggregation.min_aggregation,
        }.get(aggregator, DataAggregation.average_aggregation)
        
        result = agg_func(data_packets)
        triggered = abs(result) > threshold
        return result, triggered
    
    @staticmethod
    def compressive_sensing_aggregation(data: np.ndarray,
                                        compression_ratio: float = 0.3) -> np.ndarray:
        """
        压缩感知汇聚 (简化版)
        
        Args:
            data: 原始数据
            compression_ratio: 压缩比
        Returns:
            压缩后的数据
        """
        n = len(data)
        m = max(1, int(n * compression_ratio))
        
        # 随机测量矩阵
        Phi = np.random.randn(m, n) / np.sqrt(m)
        
        # 压缩
        compressed = Phi @ data
        return compressed
    
    @staticmethod
    def data_fusion(readings: List[Dict], method: str = 'bayesian') -> Dict:
        """
        多传感器数据融合
        
        Args:
            readings: [{'value': float, 'accuracy': float, 'sensor_id': int}]
            method: 融合方法
        Returns:
            融合结果
        """
        values = np.array([r['value'] for r in readings])
        accuracies = np.array([r['accuracy'] for r in readings])
        
        if method == 'bayesian':
            # 贝叶斯融合 (精度加权)
            weights = 1 / (accuracies**2 + 1e-10)
            weights /= weights.sum()
            fused_value = np.dot(values, weights)
            fused_accuracy = 1 / np.sqrt(np.sum(weights / (accuracies**2 + 1e-10)))
        elif method == 'average':
            fused_value = np.mean(values)
            fused_accuracy = np.std(values) / np.sqrt(len(values))
        elif method == 'median':
            fused_value = np.median(values)
            fused_accuracy = 1.4826 * np.median(np.abs(values - fused_value))
        else:
            fused_value = np.mean(values)
            fused_accuracy = np.std(values)
        
        return {
            'fused_value': fused_value,
            'fused_accuracy': fused_accuracy,
            'n_sensors': len(readings),
            'method': method,
            'agreement': 1 - np.std(values) / (np.mean(np.abs(values)) + 1e-10)
        }


# ─────────────────────────────────────────────
# 4. 能耗模型
# ─────────────────────────────────────────────

class EnergyModel:
    """无线传感器网络能耗模型"""
    
    # 典型参数 (基于LEACH论文)
    E_ELEC = 50e-9      # 发射/接收电路能耗 (J/bit)
    E_FS = 10e-12       # 自由空间模型系数 (J/bit/m²)
    E_MP = 0.0013e-12   # 多径模型系数 (J/bit/m⁴)
    E_DA = 5e-9         # 数据汇聚能耗 (J/bit/signal)
    D_THRESHOLD = 87.7  # 自由空间/多径模型切换距离 (m)
    
    @classmethod
    def tx_energy(cls, n_bits: int, distance_m: float) -> float:
        """
        发送能耗
        
        d < d_threshold: E_tx = n*E_ELEC + n*E_FS*d²
        d >= d_threshold: E_tx = n*E_ELEC + n*E_MP*d⁴
        """
        if distance_m < cls.D_THRESHOLD:
            return n_bits * (cls.E_ELEC + cls.E_FS * distance_m**2)
        else:
            return n_bits * (cls.E_ELEC + cls.E_MP * distance_m**4)
    
    @classmethod
    def rx_energy(cls, n_bits: int) -> float:
        """接收能耗 E_rx = n * E_ELEC"""
        return n_bits * cls.E_ELEC
    
    @classmethod
    def aggregation_energy(cls, n_bits: int, n_inputs: int = 1) -> float:
        """数据汇聚能耗"""
        return n_bits * n_inputs * cls.E_DA
    
    @classmethod
    def sensing_energy(cls, n_bits: int) -> float:
        """感知能耗 (传感器采样)"""
        return n_bits * 10e-9  # 10nJ/bit 典型值


class EnergyManager:
    """网络能耗管理"""
    
    def __init__(self, network: SensorNetwork):
        self.network = network
        self.energy_history = []
    
    def simulate_round(self, data_bits: int = 4000,
                       aggregation: bool = True) -> Dict:
        """
        仿真一轮数据采集
        
        Args:
            data_bits: 每节点数据量 (bits)
            aggregation: 是否汇聚
        Returns:
            本轮统计
        """
        round_energy = 0
        packets_total = 0
        
        for node in self.network.alive_nodes():
            if node.is_cluster_head:
                continue
            
            # 1. 本地感知能耗
            E_sense = EnergyModel.sensing_energy(data_bits)
            node.consume_energy(E_sense)
            
            # 2. 发送到簇头
            if node.cluster_id is not None:
                ch = self.network.nodes[node.cluster_id]
                dist = node.distance_to(ch)
                E_tx = EnergyModel.tx_energy(data_bits, dist)
                node.consume_energy(E_tx)
                ch.consume_energy(EnergyModel.rx_energy(data_bits))
                round_energy += E_tx + E_sense
                packets_total += 1
        
        # 簇头汇聚和发送到基站
        for ch_id in self.network.cluster_heads if hasattr(self.network, 'cluster_heads') else []:
            if ch_id >= len(self.network.nodes):
                continue
            ch = self.network.nodes[ch_id]
            if not ch.is_alive:
                continue
            
            n_members = sum(1 for n in self.network.alive_nodes() 
                          if hasattr(n, 'cluster_id') and n.cluster_id == ch_id)
            
            # 汇聚能耗
            if aggregation and n_members > 0:
                E_agg = EnergyModel.aggregation_energy(data_bits, n_members)
                ch.consume_energy(E_agg)
                round_energy += E_agg
            
            # 发送到基站
            if self.network.base_station:
                bs_x, bs_y = self.network.base_station
                dist_bs = np.sqrt((ch.x - bs_x)**2 + (ch.y - bs_y)**2)
                # 汇聚后数据量减少
                agg_bits = data_bits if not aggregation else data_bits
                E_tx_bs = EnergyModel.tx_energy(agg_bits, dist_bs)
                ch.consume_energy(E_tx_bs)
                round_energy += E_tx_bs
                packets_total += 1
        
        # 记录
        status = self.network.network_lifetime()
        self.energy_history.append({
            'round_energy': round_energy,
            'packets': packets_total,
            **status
        })
        
        return self.energy_history[-1]
    
    def simulate_network_lifetime(self, n_rounds: int = 1000,
                                   data_bits: int = 4000) -> List[Dict]:
        """
        仿真网络生命期
        
        Args:
            n_rounds: 最大轮数
            data_bits: 每轮每节点数据量
        Returns:
            每轮统计
        """
        results = []
        
        for r in range(n_rounds):
            if len(self.network.alive_nodes()) == 0:
                break
            
            result = self.simulate_round(data_bits)
            results.append(result)
            
            # 检查网络死亡
            if result['alive'] == 0:
                break
        
        return results
    
    def energy_consumption_map(self) -> Dict[int, float]:
        """各节点能耗地图"""
        consumption = {}
        for node in self.network.nodes:
            consumption[node.id] = node.initial_energy - node.energy
        return consumption


# ─────────────────────────────────────────────
# 5. 网络仿真器
# ─────────────────────────────────────────────

class NetworkSimulator:
    """完整网络仿真器"""
    
    def __init__(self, n_nodes: int = 50, area: float = 200,
                 comm_range: float = 100, initial_energy: float = 0.5):
        """
        Args:
            n_nodes: 节点数
            area: 区域边长 (m)
            comm_range: 通信范围 (m)
            initial_energy: 初始能量 (J)
        """
        self.network = SensorNetwork.random_network(
            n_nodes, area, area, comm_range, initial_energy,
            bs_position='center'
        )
        self.leach = LEACHProtocol(self.network, p=0.05)
        self.energy_mgr = EnergyManager(self.network)
        self.routing = EnergyAwareRouting(self.network)
    
    def run_leach_simulation(self, n_rounds: int = 500,
                             data_bits: int = 4000) -> Dict:
        """
        LEACH协议仿真
        
        Returns:
            仿真结果汇总
        """
        round_stats = []
        first_dead_round = None
        half_dead_round = None
        all_dead_round = None
        
        for r in range(n_rounds):
            alive = len(self.network.alive_nodes())
            if alive == 0:
                all_dead_round = r
                break
            
            # LEACH建立阶段
            self.leach.setup_phase()
            
            # 数据传输
            result = self.energy_mgr.simulate_round(data_bits)
            round_stats.append(result)
            
            # 检查死亡
            if first_dead_round is None and result['dead'] > 0:
                first_dead_round = r
            if half_dead_round is None and result['dead'] >= len(self.network.nodes) / 2:
                half_dead_round = r
        
        return {
            'n_rounds': len(round_stats),
            'first_dead_round': first_dead_round,
            'half_dead_round': half_dead_round,
            'all_dead_round': all_dead_round,
            'round_stats': round_stats,
            'final_alive': len(self.network.alive_nodes()),
            'total_nodes': len(self.network.nodes)
        }
    
    def compare_protocols(self, n_rounds: int = 300) -> Dict:
        """比较不同路由协议"""
        results = {}
        
        # 网络参数快照
        n_nodes = len(self.network.nodes)
        area = self.network.width
        comm_range = self.network.nodes[0].comm_range if self.network.nodes else 100
        energy = self.network.nodes[0].initial_energy if self.network.nodes else 0.5
        
        # LEACH
        self.network = SensorNetwork.random_network(n_nodes, area, area, comm_range, energy, 'center')
        self.leach = LEACHProtocol(self.network)
        self.energy_mgr = EnergyManager(self.network)
        results['LEACH'] = self.run_leach_simulation(n_rounds)
        
        return results


# ─────────────────────────────────────────────
# 综合演示
# ─────────────────────────────────────────────

def demo_topology():
    """网络拓扑演示"""
    print("=" * 60)
    print("  传感器网络拓扑演示")
    print("=" * 60)
    
    # 随机网络
    np.random.seed(42)
    net = SensorNetwork.random_network(30, 200, 200, comm_range=80, bs_position='center')
    
    print(f"  网络规模: {len(net.nodes)}个节点")
    print(f"  区域大小: {net.width}m × {net.height}m")
    print(f"  通信范围: 80m")
    print(f"  总边数: {len(net.edges)}")
    print(f"  连通性: {net.connectivity():.2%}")
    print(f"  覆盖率: {net.coverage_ratio():.2%}")
    
    # 节点统计
    degrees = [len(net.get_neighbors(n.id)) for n in net.nodes]
    print(f"  平均邻居数: {np.mean(degrees):.1f}")
    print(f"  最大邻居数: {np.max(degrees)}")
    print(f"  最小邻居数: {np.min(degrees)}")


def demo_routing():
    """路由协议演示"""
    print("\n" + "=" * 60)
    print("  路由协议演示")
    print("=" * 60)
    
    np.random.seed(42)
    net = SensorNetwork.random_network(20, 200, 200, comm_range=100, bs_position='center')
    
    # 最短路径
    sp = SPProtocol(net)
    route_sp = sp.find_route(0, 15)
    print(f"  最短路径 (0→15): {route_sp}")
    if route_sp:
        dist = sum(net.nodes[route_sp[i]].distance_to(net.nodes[route_sp[i+1]])
                   for i in range(len(route_sp)-1))
        print(f"    路径长度: {dist:.1f}m")
    
    # 能量感知路由
    ear = EnergyAwareRouting(net)
    route_ear = ear.find_route(0, 15, energy_weight=0.3)
    print(f"\n  能量感知路由 (0→15): {route_ear}")
    
    # LEACH
    leach = LEACHProtocol(net, p=0.1)
    leach.setup_phase()
    info = leach.get_cluster_info()
    print(f"\n  LEACH分簇:")
    print(f"    簇头数: {info['num_clusters']}")
    print(f"    簇头节点: {info['cluster_heads'][:5]}...")


def demo_aggregation():
    """数据汇聚演示"""
    print("\n" + "=" * 60)
    print("  数据汇聚演示")
    print("=" * 60)
    
    # 模拟传感器数据
    np.random.seed(42)
    readings = [
        {'value': 25.3 + np.random.randn() * 0.5, 'accuracy': 0.5, 'sensor_id': i}
        for i in range(10)
    ]
    
    # 各种汇聚方法
    values = [r['value'] for r in readings]
    print(f"  原始读数: {[f'{v:.2f}' for v in values[:5]]}...")
    print(f"  均值汇聚: {DataAggregation.average_aggregation(values):.3f}")
    print(f"  中值汇聚: {DataAggregation.median_aggregation(values):.3f}")
    print(f"  最大值:   {DataAggregation.max_aggregation(values):.3f}")
    print(f"  最小值:   {DataAggregation.min_aggregation(values):.3f}")
    
    # 贝叶斯融合
    fusion = DataAggregation.data_fusion(readings, method='bayesian')
    print(f"\n  贝叶斯融合:")
    print(f"    融合值: {fusion['fused_value']:.3f}")
    print(f"    融合精度: {fusion['fused_accuracy']:.3f}")
    print(f"    一致性: {fusion['agreement']:.3f}")
    
    # 压缩感知
    data = np.random.randn(100)
    compressed = DataAggregation.compressive_sensing_aggregation(data, 0.3)
    print(f"\n  压缩感知汇聚:")
    print(f"    原始数据: {len(data)}点")
    print(f"    压缩后: {len(compressed)}点 (压缩比30%)")


def demo_energy():
    """能耗模型演示"""
    print("\n" + "=" * 60)
    print("  能耗模型演示")
    print("=" * 60)
    
    # 单次传输能耗
    n_bits = 4000
    for dist in [10, 50, 100, 200]:
        E_tx = EnergyModel.tx_energy(n_bits, dist)
        E_rx = EnergyModel.rx_energy(n_bits)
        print(f"  距离{dist}m: Tx={E_tx*1e6:.2f}μJ, Rx={E_rx*1e6:.2f}μJ")
    
    # 网络生命期仿真
    print(f"\n  网络生命期仿真 (LEACH, 50节点, 200×200m):")
    np.random.seed(42)
    sim = NetworkSimulator(n_nodes=50, area=200, comm_range=80, initial_energy=0.5)
    result = sim.run_leach_simulation(n_rounds=200, data_bits=4000)
    
    print(f"    仿真轮数: {result['n_rounds']}")
    print(f"    首个死亡轮次 (FND): {result['first_dead_round']}")
    print(f"    半数死亡轮次 (HND): {result['half_dead_round']}")
    print(f"    最终存活: {result['final_alive']}/{result['total_nodes']}")
    
    # 能耗分布
    consumption = sim.energy_mgr.energy_consumption_map()
    energies = list(consumption.values())
    print(f"\n  能耗分布:")
    print(f"    平均: {np.mean(energies)*1e3:.2f}mJ")
    print(f"    最大: {np.max(energies)*1e3:.2f}mJ")
    print(f"    最小: {np.min(energies)*1e3:.2f}mJ")


if __name__ == "__main__":
    demo_topology()
    demo_routing()
    demo_aggregation()
    demo_energy()
    print("\n✓ 传感器网络仿真演示完成")
