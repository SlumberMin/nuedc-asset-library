#!/usr/bin/env python3
"""
颜色分拣仿真 — 颜色识别 + 分拣逻辑
========================================
模拟传送带上的物体通过RGB颜色传感器识别，
通过舵机/气缸分拣到不同料箱。
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ============ 颜色分类定义 ============
COLOR_CLASSES = {
    'red':    {'target': [200, 30, 30],   'bin': 0, 'label': '红色'},
    'green':  {'target': [30, 180, 30],   'bin': 1, 'label': '绿色'},
    'blue':   {'target': [30, 50, 200],   'bin': 2, 'label': '蓝色'},
    'yellow': {'target': [220, 200, 30],  'bin': 3, 'label': '黄色'},
}

# ============ 传送带模拟 ============
class ConveyorBelt:
    """传送带系统"""
    def __init__(self, length=2.0, speed=0.3, sensor_pos=0.8, pusher_pos=1.2):
        self.length = length        # 传送带长度 (m)
        self.speed = speed          # 传送带速度 (m/s)
        self.sensor_pos = sensor_pos   # 传感器位置 (m)
        self.pusher_pos = pusher_pos   # 推杆位置 (m)
        self.objects = []
        self.sorted_bins = {0: [], 1: [], 2: [], 3: []}
        self.next_spawn = 0

    def spawn_object(self, t):
        """生成待分拣物体"""
        colors = list(COLOR_CLASSES.keys())
        color_name = np.random.choice(colors, p=[0.25, 0.25, 0.25, 0.25])
        target_rgb = COLOR_CLASSES[color_name]['target']
        # 加入噪声模拟真实传感器
        noise = np.random.normal(0, 15, 3)
        measured_rgb = np.clip(np.array(target_rgb) + noise, 0, 255)

        obj = {
            'color_name': color_name,
            'true_rgb': target_rgb,
            'measured_rgb': measured_rgb,
            'position': 0.0,
            'identified': False,
            'sorted': False,
            'identified_as': None,
        }
        self.objects.append(obj)

    def update(self, dt, t):
        """更新传送带状态"""
        # 随机生成物体
        if t >= self.next_spawn:
            self.spawn_object(t)
            self.next_spawn = t + np.random.uniform(0.8, 1.5)

        for obj in self.objects:
            if not obj['sorted']:
                obj['position'] += self.speed * dt

    def get_sensor_reading(self):
        """获取传感器位置的物体读数"""
        for obj in self.objects:
            if not obj['sorted'] and not obj['identified']:
                if abs(obj['position'] - self.sensor_pos) < 0.05:
                    return obj
        return None

    def get_pusher_object(self):
        """获取推杆位置的物体"""
        for obj in self.objects:
            if obj['identified'] and not obj['sorted']:
                if abs(obj['position'] - self.pusher_pos) < 0.05:
                    return obj
        return None

# ============ 颜色识别算法 ============
def classify_color(rgb):
    """
    颜色识别：基于RGB距离最近邻分类
    返回分类结果和置信度
    """
    best_class = None
    best_dist = float('inf')

    for name, info in COLOR_CLASSES.items():
        target = np.array(info['target'])
        dist = np.linalg.norm(np.array(rgb) - target)
        if dist < best_dist:
            best_dist = dist
            best_class = name

    # 置信度（距离越小越确信）
    confidence = max(0, 1 - best_dist / 200)
    return best_class, confidence

# ============ 推杆控制 ============
class SortingPusher:
    """分拣推杆"""
    def __init__(self, response_time=0.15):
        self.response_time = response_time
        self.active = False
        self.target_bin = -1
        self.timer = 0
        self.sort_log = []  # 分拣记录

    def trigger(self, bin_id):
        if not self.active:
            self.active = True
            self.target_bin = bin_id
            self.timer = self.response_time

    def update(self, dt):
        if self.active:
            self.timer -= dt
            if self.timer <= 0:
                self.active = False
                return True, self.target_bin
        return False, -1

# ============ 主仿真 ============
np.random.seed(42)
dt = 0.01
T = 40.0
N = int(T / dt)

belt = ConveyorBelt()
pusher = SortingPusher()

# 记录数据
time_log = []
objects_on_belt = []
identified_log = []
pusher_log = []
confidence_log = []
color_class_log = []

for i in range(N):
    t = i * dt

    # 更新传送带
    belt.update(dt, t)

    # 颜色识别
    sensor_obj = belt.get_sensor_reading()
    if sensor_obj:
        color_class, confidence = classify_color(sensor_obj['measured_rgb'])
        sensor_obj['identified_as'] = color_class
        sensor_obj['identified'] = True
        bin_id = COLOR_CLASSES[color_class]['bin']
        pusher.trigger(bin_id)

    # 推杆动作
    sorted_flag, sorted_bin = pusher.update(dt)
    if sorted_flag:
        pusher_obj = belt.get_pusher_object()
        if pusher_obj:
            pusher_obj['sorted'] = True
            pusher.sort_log.append({
                'time': t,
                'true_color': pusher_obj['color_name'],
                'identified_as': pusher_obj['identified_as'],
                'bin': sorted_bin,
            })

    # 记录
    time_log.append(t)
    n_on_belt = sum(1 for o in belt.objects if not o['sorted'])
    objects_on_belt.append(n_on_belt)
    pusher_log.append(pusher.active)
    identified_log.append(sum(1 for o in belt.objects if o['identified'] and not o['sorted']))

    if sensor_obj:
        confidence_log.append(confidence)
        color_class_log.append(color_class)

# ============ 绘图 ============
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('颜色分拣仿真 — RGB识别 + 分拣逻辑', fontsize=16, fontweight='bold')

# 图1：传送带物体分布（最终状态）
ax = axes[0, 0]
# 画传送带
ax.add_patch(plt.Rectangle((0, -0.1), 2.0, 0.2, color='gray', alpha=0.3))
ax.add_patch(plt.Rectangle((0.75, -0.2), 0.1, 0.4, color='blue', alpha=0.5, label='传感器'))
ax.add_patch(plt.Rectangle((1.15, -0.2), 0.1, 0.4, color='red', alpha=0.5, label='推杆'))
# 画未分拣的物体
for obj in belt.objects:
    if not obj['sorted']:
        color = np.array(obj['true_rgb']) / 255.0
        ax.plot(obj['position'], 0, 'o', color=color, markersize=12)
ax.set_xlim(-0.2, 2.5)
ax.set_ylim(-0.5, 0.5)
ax.set_xlabel('位置 (m)')
ax.set_title('传送带最终状态')
ax.legend()

# 图2：分拣统计
sort_results = pusher.sort_log
if sort_results:
    true_colors = [r['true_color'] for r in sort_results]
    id_colors = [r['identified_as'] for r in sort_results]
    correct = sum(1 for t_c, i_c in zip(true_colors, id_colors) if t_c == i_c)
    accuracy = correct / len(sort_results) * 100

    # 按颜色统计
    color_names = list(COLOR_CLASSES.keys())
    color_labels = [COLOR_CLASSES[c]['label'] for c in color_names]
    true_counts = [true_colors.count(c) for c in color_names]
    correct_counts = [sum(1 for r in sort_results if r['true_color'] == c and r['identified_as'] == c) for c in color_names]

    x_pos = np.arange(len(color_names))
    bar_width = 0.35
    axes[0, 1].bar(x_pos - bar_width/2, true_counts, bar_width, label='实际数量', alpha=0.7)
    axes[0, 1].bar(x_pos + bar_width/2, correct_counts, bar_width, label='正确识别', alpha=0.7)
    axes[0, 1].set_xticks(x_pos)
    axes[0, 1].set_xticklabels(color_labels)
    axes[0, 1].set_ylabel('数量')
    axes[0, 1].set_title(f'分拣统计 (准确率: {accuracy:.1f}%)')
    axes[0, 1].legend()

# 图3：识别置信度
if confidence_log:
    axes[1, 0].plot(range(len(confidence_log)), confidence_log, 'b-', linewidth=1)
    axes[1, 0].axhline(y=0.7, color='r', linestyle='--', alpha=0.5, label='置信阈值')
    axes[1, 0].set_xlabel('识别次数')
    axes[1, 0].set_ylabel('置信度')
    axes[1, 0].set_title('颜色识别置信度')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

# 图4：传送带上物体数量变化
t_arr = np.array(time_log)
axes[1, 1].plot(t_arr, objects_on_belt, 'g-', linewidth=1.5, label='传送带上')
axes[1, 1].plot(t_arr, identified_log, 'b--', linewidth=1, label='已识别待推')
axes[1, 1].set_xlabel('时间 (s)')
axes[1, 1].set_ylabel('物体数量')
axes[1, 1].set_title('传送带物体数量')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('./nuedc-asset-library/15_simulation/color_sorting_result.png', dpi=150, bbox_inches='tight')
print("✅ 颜色分拣仿真完成，图表已保存")

# 统计
total_sorted = len(sort_results)
print(f"  总分拣数量: {total_sorted}")
if sort_results:
    true_colors = [r['true_color'] for r in sort_results]
    id_colors = [r['identified_as'] for r in sort_results]
    correct = sum(1 for t_c, i_c in zip(true_colors, id_colors) if t_c == i_c)
    print(f"  分拣准确率: {correct/total_sorted*100:.1f}%")
    for c_name in COLOR_CLASSES:
        count = true_colors.count(c_name)
        if count > 0:
            c_correct = sum(1 for r in sort_results if r['true_color'] == c_name and r['identified_as'] == c_name)
            print(f"    {COLOR_CLASSES[c_name]['label']}: {c_correct}/{count}")
