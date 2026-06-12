#!/usr/bin/env python3
"""
竞赛评分工具 - 根据评分标准自动评分
=============================================
功能：
  - 电赛各赛题评分标准管理
  - 基本要求/发挥部分自动评分
  - 测量数据自动判定
  - 加分项/扣分项管理
  - 评分报告生成
  - 历史得分对比分析

用法：
  python competition_scoring.py list                          # 列出所有赛题
  python competition_scoring.py score --topic 2024-信号源      # 评分
  python competition_scoring.py template --topic 2024-信号源    # 生成评分模板
  python competition_scoring.py compare score1.json score2.json # 对比
"""

import argparse
import json
import sys
import os
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple
from datetime import datetime


# ============================================================
# 评分项目数据结构
# ============================================================

@dataclass
class ScoreItem:
    """单个评分项"""
    id: str                    # 评分项ID
    name: str                  # 评分项名称
    description: str           # 描述
    max_score: float           # 满分
    actual_score: float = 0.0  # 实际得分
    achieved: bool = False     # 是否完成
    notes: str = ""            # 备注
    test_method: str = ""      # 测试方法
    judge_rule: str = ""       # 判定规则


@dataclass
class ScoreCategory:
    """评分类别"""
    name: str                  # 类别名称
    description: str           # 描述
    items: List[ScoreItem] = field(default_factory=list)

    @property
    def max_score(self) -> float:
        return sum(item.max_score for item in self.items)

    @property
    def actual_score(self) -> float:
        return sum(item.actual_score for item in self.items)


@dataclass
class ScoringStandard:
    """评分标准"""
    topic_id: str              # 赛题ID
    topic_name: str            # 赛题名称
    year: int                  # 年份
    category: str              # 赛题类别
    description: str           # 赛题描述
    categories: List[ScoreCategory] = field(default_factory=list)
    bonus_items: List[ScoreItem] = field(default_factory=list)
    penalty_items: List[ScoreItem] = field(default_factory=list)

    @property
    def total_max(self) -> float:
        base = sum(cat.max_score for cat in self.categories)
        bonus = sum(item.max_score for item in self.bonus_items)
        return base + bonus

    @property
    def total_actual(self) -> float:
        base = sum(cat.actual_score for cat in self.categories)
        bonus = sum(item.actual_score for item in self.bonus_items)
        penalty = sum(item.actual_score for item in self.penalty_items)
        return base + bonus - penalty


# ============================================================
# 电赛赛题评分标准库
# ============================================================

class CompetitionStandards:
    """电赛评分标准库"""

    @staticmethod
    def get_all_topics() -> Dict[str, str]:
        """获取所有赛题列表"""
        return {
            "2024-信号源": "2024年全国电赛 - 信号发生器",
            "2024-功率放大": "2024年全国电赛 - 高效率音频功率放大器",
            "2024-无线充电": "2024年全国电赛 - 无线充电装置",
            "2024-信号分析": "2024年全国电赛 - 信号频谱分析仪",
            "2024-运动控制": "2024年全国电赛 - 运动目标跟踪系统",
            "2024-纸张计数": "2024年全国电赛 - 纸张计数显示装置",
            "2024-同轴线缆": "2024年全国电赛 - 同轴线缆特性阻抗测量",
            "2023-信号分离": "2023年全国电赛 - 信号分离电路",
            "2023-同轴电缆": "2023年全国电赛 - 同轴电缆长度与终端状态检测",
            "2023-频率特性": "2023年全国电赛 - 简易频率特性测试仪",
            "demo-电源": "演示 - 数控直流稳压电源",
        }

    @staticmethod
    def get_standard(topic_id: str) -> Optional[ScoringStandard]:
        """获取指定赛题的评分标准"""

        if topic_id == "2024-信号源":
            return ScoringStandard(
                topic_id="2024-信号源",
                topic_name="信号发生器",
                year=2024,
                category="信号类",
                description="设计制作一个信号发生器，能产生正弦波、方波、三角波等信号",
                categories=[
                    ScoreCategory("基本要求", "必须完成的基本功能", [
                        ScoreItem("B1", "正弦波输出",
                                  "能输出频率1kHz~10MHz的正弦波",
                                  15, test_method="频率计测量",
                                  judge_rule="频率误差≤1%"),
                        ScoreItem("B2", "方波输出",
                                  "能输出频率1kHz~1MHz的方波",
                                  10, test_method="示波器观察波形",
                                  judge_rule="占空比误差≤5%"),
                        ScoreItem("B3", "三角波输出",
                                  "能输出频率1kHz~100kHz的三角波",
                                  10, test_method="示波器观察波形",
                                  judge_rule="线性度误差≤3%"),
                        ScoreItem("B4", "频率可调",
                                  "频率可按10倍步进调节",
                                  10, test_method="按键调节并测量",
                                  judge_rule="步进准确"),
                        ScoreItem("B5", "幅度可调",
                                  "输出幅度可调 (0.1V~5Vpp)",
                                  10, test_method="示波器测量幅度",
                                  judge_rule="幅度误差≤5%"),
                        ScoreItem("B6", "LCD显示",
                                  "显示当前波形类型、频率、幅度",
                                  5, test_method="目视检查",
                                  judge_rule="显示正确清晰"),
                    ]),
                    ScoreCategory("发挥部分", "可选的高级功能", [
                        ScoreItem("A1", "频率扩展",
                                  "正弦波频率上限达到20MHz",
                                  10, test_method="频率计测量",
                                  judge_rule="达到20MHz满分，按比例给分"),
                        ScoreItem("A2", "低失真",
                                  "正弦波失真度≤1%",
                                  10, test_method="频谱仪/失真仪",
                                  judge_rule="THD≤1%满分，≤3%半分"),
                        ScoreItem("A3", "频率稳定度",
                                  "频率稳定度优于10⁻⁴",
                                  10, test_method="频率计长时间测量",
                                  judge_rule="10⁻⁴满分，10⁻³半分"),
                        ScoreItem("A4", "任意波形",
                                  "能输出任意波形（存储/回放）",
                                  10, test_method="回放测试波形",
                                  judge_rule="波形还原度≥90%"),
                        ScoreItem("A5", "扫频功能",
                                  "支持频率自动扫描",
                                  5, test_method="观察扫频过程",
                                  judge_rule="扫频范围和速度可调"),
                    ]),
                ],
                bonus_items=[
                    ScoreItem("EX1", "创新设计",
                              "其他创新性功能或设计",
                              5, judge_rule="评委酌情给分"),
                ],
            )

        elif topic_id == "2024-功率放大":
            return ScoringStandard(
                topic_id="2024-功率放大",
                topic_name="高效率音频功率放大器",
                year=2024,
                category="电源/功率类",
                description="设计制作一个高效率音频功率放大器",
                categories=[
                    ScoreCategory("基本要求", "必须完成的基本功能", [
                        ScoreItem("B1", "输出功率",
                                  "额定负载(8Ω)上输出功率≥1W",
                                  15, test_method="功率计/示波器测量",
                                  judge_rule="≥1W满分，按比例给分"),
                        ScoreItem("B2", "效率",
                                  "放大器效率≥60%",
                                  15, test_method="输入输出功率比",
                                  judge_rule="≥60%满分，按比例给分"),
                        ScoreItem("B3", "频率响应",
                                  "20Hz~20kHz频率范围内增益平坦",
                                  10, test_method="扫频仪测量",
                                  judge_rule="增益波动≤3dB"),
                        ScoreItem("B4", "失真度",
                                  "输出失真度≤5%",
                                  10, test_method="失真仪测量",
                                  judge_rule="THD≤5%"),
                        ScoreItem("B5", "输入灵敏度",
                                  "输入信号≤20mVrms",
                                  5, test_method="信号源输入测量",
                                  judge_rule="灵敏度满足要求"),
                        ScoreItem("B6", "噪声",
                                  "无输入时输出噪声≤50mVrms",
                                  5, test_method="示波器测量",
                                  judge_rule="噪声≤50mV"),
                    ]),
                    ScoreCategory("发挥部分", "可选的高级功能", [
                        ScoreItem("A1", "大功率",
                                  "输出功率≥5W",
                                  10, test_method="功率计测量",
                                  judge_rule="≥5W满分"),
                        ScoreItem("A2", "高效率",
                                  "效率≥80%",
                                  10, test_method="功率计测量",
                                  judge_rule="≥80%满分"),
                        ScoreItem("A3", "低失真",
                                  "失真度≤1%",
                                  10, test_method="失真仪测量",
                                  judge_rule="THD≤1%满分"),
                        ScoreItem("A4", "宽频带",
                                  "频率范围扩展至10Hz~50kHz",
                                  5, test_method="扫频仪测量",
                                  judge_rule="范围内增益平坦"),
                    ]),
                ],
            )

        elif topic_id == "demo-电源":
            return ScoringStandard(
                topic_id="demo-电源",
                topic_name="数控直流稳压电源",
                year=2024,
                category="电源类",
                description="设计制作一个数控直流稳压电源",
                categories=[
                    ScoreCategory("基本要求", "基本功能", [
                        ScoreItem("B1", "输出电压范围",
                                  "输出电压0~15V可调",
                                  15, test_method="万用表测量",
                                  judge_rule="范围满足，步进0.1V"),
                        ScoreItem("B2", "输出电流",
                                  "最大输出电流≥1A",
                                  10, test_method="电子负载测量",
                                  judge_rule="≥1A满分"),
                        ScoreItem("B3", "电压精度",
                                  "输出电压精度≤1%",
                                  10, test_method="高精度万用表",
                                  judge_rule="精度≤1%"),
                        ScoreItem("B4", "纹波",
                                  "输出纹波≤50mVpp",
                                  10, test_method="示波器AC耦合",
                                  judge_rule="纹波≤50mV"),
                        ScoreItem("B5", "过流保护",
                                  "具有过流保护功能",
                                  10, test_method="增大负载至保护",
                                  judge_rule="保护动作正确"),
                        ScoreItem("B6", "显示功能",
                                  "显示输出电压和电流",
                                  5, test_method="LCD显示检查",
                                  judge_rule="显示准确"),
                    ]),
                    ScoreCategory("发挥部分", "高级功能", [
                        ScoreItem("A1", "电压预设",
                                  "支持常用电压一键设置(3.3V/5V/12V)",
                                  10, test_method="按键预设测量",
                                  judge_rule="预设值精度≤1%"),
                        ScoreItem("A2", "效率",
                                  "电源效率≥70%",
                                  10, test_method="输入输出功率比",
                                  judge_rule="≥70%满分"),
                        ScoreItem("A3", "负载调整率",
                                  "负载调整率≤2%",
                                  10, test_method="空载满载测量",
                                  judge_rule="调整率≤2%"),
                        ScoreItem("A4", "线性调整率",
                                  "线性调整率≤1%",
                                  10, test_method="输入电压变化测量",
                                  judge_rule="调整率≤1%"),
                    ]),
                ],
            )

        return None


# ============================================================
# 评分引擎
# ============================================================

class ScoringEngine:
    """评分引擎"""

    @staticmethod
    def auto_score_percentage(actual: float, target: float,
                              max_score: float, reverse: bool = False) -> float:
        """按百分比自动评分
        actual: 实测值
        target: 目标值
        max_score: 满分
        reverse: True表示越小越好(如失真度)
        """
        if target == 0:
            return max_score if actual == 0 else 0

        if reverse:
            if actual <= target:
                return max_score
            ratio = target / actual
        else:
            if actual >= target:
                return max_score
            ratio = actual / target

        return round(max_score * min(1.0, ratio), 2)

    @staticmethod
    def auto_score_threshold(actual: float, thresholds: List[Tuple[float, float]],
                             max_score: float) -> float:
        """阈值评分
        thresholds: [(阈值, 得分比例), ...] 按从高到低排列
        """
        for threshold, ratio in thresholds:
            if actual >= threshold:
                return round(max_score * ratio, 2)
        return 0.0

    @staticmethod
    def generate_report(standard: ScoringStandard, output_format: str = "text") -> str:
        """生成评分报告"""
        if output_format == "text":
            return ScoringEngine._text_report(standard)
        elif output_format == "json":
            return ScoringEngine._json_report(standard)
        elif output_format == "markdown":
            return ScoringEngine._markdown_report(standard)
        return ""

    @staticmethod
    def _text_report(standard: ScoringStandard) -> str:
        """文本格式报告"""
        lines = []
        lines.append("")
        lines.append(f"  {'='*60}")
        lines.append(f"  电赛评分报告")
        lines.append(f"  赛题: {standard.topic_name} ({standard.year})")
        lines.append(f"  类别: {standard.category}")
        lines.append(f"  日期: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
        lines.append(f"  {'='*60}")

        for cat in standard.categories:
            lines.append(f"\n  【{cat.name}】 {cat.description}")
            lines.append(f"  {'-'*55}")
            lines.append(f"  {'ID':>4s} {'项目':12s} {'满分':>6s} {'得分':>6s} {'状态':>6s} {'备注'}")
            lines.append(f"  {'-'*55}")

            for item in cat.items:
                status = "✓" if item.achieved else "✗"
                lines.append(
                    f"  {item.id:>4s} {item.name:12s} {item.max_score:6.1f} "
                    f"{item.actual_score:6.1f} {status:>6s}  {item.notes}"
                )

            lines.append(f"  {'-'*55}")
            lines.append(f"  小计: {cat.actual_score:.1f} / {cat.max_score:.1f}")

        if standard.bonus_items:
            lines.append(f"\n  【加分项】")
            lines.append(f"  {'-'*55}")
            for item in standard.bonus_items:
                lines.append(f"  {item.id:>4s} {item.name:12s} +{item.max_score:.1f} "
                            f"得分:{item.actual_score:.1f}  {item.notes}")

        if standard.penalty_items:
            lines.append(f"\n  【扣分项】")
            lines.append(f"  {'-'*55}")
            for item in standard.penalty_items:
                lines.append(f"  {item.id:>4s} {item.name:12s} -{item.max_score:.1f} "
                            f"扣分:{item.actual_score:.1f}  {item.notes}")

        lines.append(f"\n  {'='*60}")
        lines.append(f"  总分: {standard.total_actual:.1f} / {standard.total_max:.1f}")
        lines.append(f"  得分率: {standard.total_actual/standard.total_max*100:.1f}%")
        lines.append(f"  {'='*60}")

        return "\n".join(lines)

    @staticmethod
    def _json_report(standard: ScoringStandard) -> str:
        """JSON格式报告"""
        report = {
            "topic": standard.topic_name,
            "year": standard.year,
            "category": standard.category,
            "timestamp": datetime.now().isoformat(),
            "total_score": standard.total_actual,
            "max_score": standard.total_max,
            "percentage": round(standard.total_actual / standard.total_max * 100, 1),
            "categories": [],
        }

        for cat in standard.categories:
            cat_data = {
                "name": cat.name,
                "max_score": cat.max_score,
                "actual_score": cat.actual_score,
                "items": [
                    {
                        "id": item.id,
                        "name": item.name,
                        "max_score": item.max_score,
                        "actual_score": item.actual_score,
                        "achieved": item.achieved,
                        "notes": item.notes,
                    }
                    for item in cat.items
                ],
            }
            report["categories"].append(cat_data)

        return json.dumps(report, indent=2, ensure_ascii=False)

    @staticmethod
    def _markdown_report(standard: ScoringStandard) -> str:
        """Markdown格式报告"""
        lines = []
        lines.append(f"# 电赛评分报告")
        lines.append(f"")
        lines.append(f"- **赛题**: {standard.topic_name} ({standard.year})")
        lines.append(f"- **类别**: {standard.category}")
        lines.append(f"- **日期**: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
        lines.append("")

        for cat in standard.categories:
            lines.append(f"## {cat.name}")
            lines.append(f"")
            lines.append(f"| ID | 项目 | 满分 | 得分 | 状态 | 备注 |")
            lines.append(f"|---:|------|-----:|-----:|:----:|------|")

            for item in cat.items:
                status = "✅" if item.achieved else "❌"
                lines.append(
                    f"| {item.id} | {item.name} | {item.max_score} | "
                    f"{item.actual_score} | {status} | {item.notes} |"
                )

            lines.append(f"")
            lines.append(f"**小计: {cat.actual_score:.1f} / {cat.max_score:.1f}**")
            lines.append("")

        lines.append(f"## 总评")
        lines.append(f"")
        lines.append(f"| 项目 | 分数 |")
        lines.append(f"|------|-----:|")
        lines.append(f"| 总分 | **{standard.total_actual:.1f}** / {standard.total_max:.1f} |")
        lines.append(f"| 得分率 | **{standard.total_actual/standard.total_max*100:.1f}%** |")

        return "\n".join(lines)


# ============================================================
# 交互式评分
# ============================================================

class InteractiveScorer:
    """交互式评分器"""

    def __init__(self, standard: ScoringStandard):
        self.standard = standard

    def run(self):
        """运行交互式评分"""
        print(f"\n  赛题: {self.standard.topic_name} ({self.standard.year})")
        print(f"  类别: {self.standard.category}")
        print(f"  总分: {self.standard.total_max}")
        print()

        for cat in self.standard.categories:
            print(f"\n  ═══ {cat.name} (满分 {cat.max_score}) ═══")
            for item in cat.items:
                self._score_item(item)

        if self.standard.bonus_items:
            print(f"\n  ═══ 加分项 ═══")
            for item in self.standard.bonus_items:
                self._score_item(item)

        if self.standard.penalty_items:
            print(f"\n  ═══ 扣分项 ═══")
            for item in self.standard.penalty_items:
                self._score_item(item, is_penalty=True)

    def _score_item(self, item: ScoreItem, is_penalty: bool = False):
        """评分子项"""
        print(f"\n  [{item.id}] {item.name}")
        print(f"  说明: {item.description}")
        print(f"  满分: {item.max_score}")
        if item.test_method:
            print(f"  测试: {item.test_method}")
        if item.judge_rule:
            print(f"  规则: {item.judge_rule}")

        while True:
            try:
                score_str = input(f"  得分 (0-{item.max_score}, 或 a=满分/0=零分/s=跳过): ").strip()

                if score_str.lower() == 's':
                    print(f"  → 跳过")
                    break
                elif score_str.lower() == 'a':
                    item.actual_score = item.max_score
                    item.achieved = True
                    print(f"  → 满分 {item.max_score}")
                    break
                elif score_str == '0':
                    item.actual_score = 0
                    item.achieved = False
                    print(f"  → 0 分")
                    break
                else:
                    score = float(score_str)
                    if 0 <= score <= item.max_score:
                        item.actual_score = score
                        item.achieved = score >= item.max_score
                        print(f"  → {score} 分")
                        break
                    else:
                        print(f"  ⚠ 分数必须在 0-{item.max_score} 之间")
            except ValueError:
                print(f"  ⚠ 请输入有效数字")
            except (EOFError, KeyboardInterrupt):
                print(f"\n  评分中断")
                return

        # 备注
        try:
            note = input(f"  备注 (回车跳过): ").strip()
            if note:
                item.notes = note
        except (EOFError, KeyboardInterrupt):
            pass


# ============================================================
# CLI 接口
# ============================================================

def cmd_list(args):
    """列出所有赛题"""
    topics = CompetitionStandards.get_all_topics()
    print(f"\n  可用赛题:")
    print(f"  {'='*50}")
    for tid, desc in topics.items():
        print(f"  {tid:20s} | {desc}")
    print()


def cmd_score(args):
    """评分"""
    standard = CompetitionStandards.get_standard(args.topic)
    if standard is None:
        print(f"  错误: 未找到赛题 '{args.topic}'")
        print(f"  使用 'list' 命令查看可用赛题")
        return

    if args.interactive:
        scorer = InteractiveScorer(standard)
        scorer.run()
        report = ScoringEngine.generate_report(standard, args.format)
        print(report)
    else:
        # 显示评分标准
        print(f"\n  赛题: {standard.topic_name} ({standard.year})")
        print(f"  类别: {standard.category}")
        print(f"  {standard.description}")
        print()

        for cat in standard.categories:
            print(f"  【{cat.name}】满分: {cat.max_score}")
            for item in cat.items:
                print(f"    {item.id}: {item.name} ({item.max_score}分)")
                print(f"        {item.description}")
                if item.judge_rule:
                    print(f"        判定: {item.judge_rule}")
            print()

        print(f"  总分: {standard.total_max}")


def cmd_template(args):
    """生成评分模板"""
    standard = CompetitionStandards.get_standard(args.topic)
    if standard is None:
        print(f"  错误: 未找到赛题 '{args.topic}'")
        return

    # 生成模板JSON
    template = {
        "topic_id": standard.topic_id,
        "topic_name": standard.topic_name,
        "year": standard.year,
        "scored_at": "",
        "team": "",
        "categories": [],
    }

    for cat in standard.categories:
        cat_data = {
            "name": cat.name,
            "items": [
                {
                    "id": item.id,
                    "name": item.name,
                    "max_score": item.max_score,
                    "actual_score": 0,
                    "achieved": False,
                    "notes": "",
                    "measurement": "",
                }
                for item in cat.items
            ],
        }
        template["categories"].append(cat_data)

    output = json.dumps(template, indent=2, ensure_ascii=False)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(output)
        print(f"  模板已保存到: {args.output}")
    else:
        print(output)


def cmd_compare(args):
    """对比评分结果"""
    files = args.files
    if len(files) < 2:
        print("  错误: 至少需要2个评分文件进行对比")
        return

    results = []
    for f in files:
        with open(f, 'r', encoding='utf-8') as fp:
            data = json.load(fp)
            results.append(data)

    print(f"\n  评分对比")
    print(f"  {'='*60}")

    # 表头
    header = f"  {'项目':15s}"
    for r in results:
        team = r.get("team", r.get("topic_name", "未知"))
        header += f" {team:>12s}"
    print(header)
    print(f"  {'-'*60}")

    # 对比各项
    if all("categories" in r for r in results):
        # 提取所有评分项
        items_by_id = {}
        for r in results:
            for cat in r.get("categories", []):
                for item in cat.get("items", []):
                    items_by_id[item["id"]] = item["name"]

        for item_id, item_name in items_by_id.items():
            line = f"  {item_name:15s}"
            for r in results:
                score = 0
                for cat in r.get("categories", []):
                    for item in cat.get("items", []):
                        if item["id"] == item_id:
                            score = item.get("actual_score", 0)
                line += f" {score:>12.1f}"
            print(line)

    # 总分
    print(f"  {'-'*60}")
    total_line = f"  {'总分':15s}"
    for r in results:
        total = r.get("total_score", 0)
        total_line += f" {total:>12.1f}"
    print(total_line)


def main():
    parser = argparse.ArgumentParser(
        description="竞赛评分工具 - 根据评分标准自动评分",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s list
  %(prog)s score --topic 2024-信号源 --interactive
  %(prog)s score --topic demo-电源
  %(prog)s template --topic 2024-信号源 --output template.json
  %(prog)s compare team1.json team2.json
        """
    )

    subparsers = parser.add_subparsers(dest="command", help="功能")

    # list
    subparsers.add_parser("list", help="列出所有赛题")

    # score
    score_p = subparsers.add_parser("score", help="查看/执行评分")
    score_p.add_argument("--topic", "-t", required=True, help="赛题ID")
    score_p.add_argument("--interactive", "-i", action="store_true", help="交互式评分")
    score_p.add_argument("--format", "-f", default="text",
                         choices=["text", "json", "markdown"], help="输出格式")

    # template
    tmpl_p = subparsers.add_parser("template", help="生成评分模板")
    tmpl_p.add_argument("--topic", "-t", required=True, help="赛题ID")
    tmpl_p.add_argument("--output", "-o", help="输出文件路径")

    # compare
    cmp_p = subparsers.add_parser("compare", help="对比评分结果")
    cmp_p.add_argument("files", nargs="+", help="评分文件列表")

    args = parser.parse_args()

    if args.command == "list":
        cmd_list(args)
    elif args.command == "score":
        cmd_score(args)
    elif args.command == "template":
        cmd_template(args)
    elif args.command == "compare":
        cmd_compare(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
