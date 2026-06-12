#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电赛设计报告自动生成脚本
========================
功能：通过交互式输入或配置文件，快速生成完整的Markdown格式设计报告框架。

使用方法：
  python 报告自动生成脚本.py                    # 交互式模式
  python 报告自动生成脚本.py -c config.json     # 配置文件模式
  python 报告自动生成脚本.py --demo             # 生成演示报告

作者：nuedc-asset-library
"""

import argparse
import json
import os
import sys
from datetime import datetime


# ============================================================
# 默认配置模板
# ============================================================
DEFAULT_CONFIG = {
    "title": "XXXX年全国大学生电子设计竞赛X题设计报告",
    "team": {
        "members": ["队员1", "队员2", "队员3"],
        "school": "XX大学",
        "advisor": "指导教师",
        "date": datetime.now().strftime("%Y年%m月%d日")
    },
    "system": {
        "name": "XX测量/控制系统",
        "controller": "STM32F407ZGT6",
        "solution_summary": "以STM32F407为核心，采用XX方案实现XX功能",
        "key_modules": [
            {"name": "信号采集模块", "desc": "负责模拟信号的数字化采集"},
            {"name": "信号处理模块", "desc": "负责数据处理与算法运算"},
            {"name": "输出控制模块", "desc": "负责信号输出或执行器驱动"},
            {"name": "人机交互模块", "desc": "负责显示与按键输入"}
        ],
        "innovations": [
            "采用XX技术，提升了XX性能",
            "引入XX算法，改善了XX指标",
            "设计XX电路，解决了XX问题"
        ],
        "key_specs": [
            {"name": "精度", "value": "≤1%", "requirement": "≤5%"},
            {"name": "带宽", "value": "10MHz", "requirement": "≥1MHz"},
            {"name": "响应时间", "value": "50ms", "requirement": "≤100ms"}
        ]
    },
    "modules": [
        {
            "name": "主控模块",
            "solutions": [
                {"name": "方案一：STM32F1系列", "pros": "成本低、资源丰富", "cons": "性能有限、无FPU", "selected": False},
                {"name": "方案二：STM32F4系列", "pros": "性能强、有FPU和DSP", "cons": "成本略高", "selected": True},
                {"name": "方案三：FPGA", "pros": "并行处理、灵活性高", "cons": "开发难度大、成本高", "selected": False}
            ],
            "reason": "综合考虑性能、开发难度和成本，选择方案二"
        },
        {
            "name": "ADC采集模块",
            "solutions": [
                {"name": "方案一：内置ADC(12bit)", "pros": "无需外接、成本低", "cons": "精度有限", "selected": False},
                {"name": "方案二：外接SAR-ADC(16bit)", "pros": "精度高、速度快", "cons": "需额外电路", "selected": True},
                {"name": "方案三：外接Σ-Δ ADC(24bit)", "pros": "精度极高", "cons": "速度慢", "selected": False}
            ],
            "reason": "题目对精度要求高(≤1%)，选择方案二"
        }
    ],
    "theory": [
        {
            "title": "信号采集原理",
            "content": "根据奈奎斯特采样定理，采样率应满足 fs ≥ 2fmax。",
            "formulas": ["$f_s \\geq 2f_{max}$"]
        },
        {
            "title": "放大电路设计",
            "content": "采用同相放大器结构，增益 G = 1 + Rf/R1。",
            "formulas": ["$G = 1 + \\frac{R_f}{R_1}$"]
        }
    ],
    "hardware": {
        "circuits": [
            {"name": "前端调理电路", "desc": "采用仪表放大器+二阶有源滤波"},
            {"name": "DAC输出电路", "desc": "采用运放跟随+功率放大"},
            {"name": "电源电路", "desc": "采用DC-DC+LDO两级稳压"}
        ],
        "pcb_notes": "四层板设计，信号层-地层-电源层-信号层"
    },
    "software": {
        "architecture": "前后台架构，主循环负责显示和交互，中断负责采集和控制",
        "algorithms": [
            {"name": "滑动平均滤波", "desc": "窗口长度N=16，抑制随机噪声"},
            {"name": "PID控制", "desc": "增量式PID，参数Kp=XX, Ki=XX, Kd=XX"}
        ],
        "tasks": [
            {"name": "数据采集", "trigger": "定时器中断", "period": "1ms"},
            {"name": "控制运算", "trigger": "采集完成", "period": "1ms"},
            {"name": "显示更新", "trigger": "主循环", "period": "100ms"}
        ]
    },
    "tests": [
        {
            "name": "基本功能测试",
            "items": [
                {"condition": "输入1kHz正弦波", "requirement": "输出频率误差≤0.1%", "result": "0.05%", "pass": True},
                {"condition": "输入1Vpp信号", "requirement": "增益误差≤1%", "result": "0.3%", "pass": True}
            ]
        },
        {
            "name": "性能指标测试",
            "items": [
                {"condition": "全频率范围扫描", "requirement": "带宽≥1MHz", "result": "10MHz", "pass": True},
                {"condition": "长时间工作2小时", "requirement": "漂移≤0.5%", "result": "0.2%", "pass": True}
            ]
        }
    ],
    "errors": [
        {"source": "ADC量化误差", "type": "系统误差", "value": "0.024%", "improvement": "过采样+数字滤波"},
        {"source": "运放失调", "type": "系统误差", "value": "0.05%", "improvement": "选用低失调运放"},
        {"source": "温漂", "type": "随机误差", "value": "0.1%", "improvement": "数字温度补偿"},
        {"source": "噪声", "type": "随机误差", "value": "0.08%", "improvement": "屏蔽+滤波"}
    ],
    "references": [
        "[1] 全国大学生电子设计竞赛组委会. 电子系统设计[M]. 北京: 高等教育出版社.",
        "[2] 张三. STM32嵌入式系统开发实战[M]. 北京: 电子工业出版社, 2023.",
        "[3] STMicroelectronics. STM32F407 Datasheet[Z]. 2023."
    ]
}


# ============================================================
# 报告生成器
# ============================================================
class ReportGenerator:
    """电赛设计报告生成器"""

    def __init__(self, config: dict):
        self.config = config

    def generate(self) -> str:
        """生成完整报告"""
        sections = [
            self._header(),
            self._abstract(),
            self._solution(),
            self._theory(),
            self._hardware_software(),
            self._tests(),
            self._error_analysis(),
            self._summary(),
            self._references(),
            self._appendix()
        ]
        return "\n\n".join(sections)

    def _header(self) -> str:
        c = self.config
        t = c["team"]
        members = "、".join(t["members"])
        return f"""# {c['title']}

**学校**：{t['school']}  |  **队员**：{members}  |  **指导教师**：{t['advisor']}
**日期**：{t['date']}

---"""

    def _abstract(self) -> str:
        c = self.config["system"]
        modules = "、".join([m["name"] for m in c["key_modules"]])
        specs = "\n".join([
            f"- **{s['name']}**：实测 {s['value']}（题目要求 {s['requirement']}）"
            for s in c["key_specs"]
        ])
        innovations = "\n".join([f"{i+1}. {v}" for i, v in enumerate(c["innovations"])])

        return f"""## 一、摘要

本系统以 **{c['controller']}** 为核心控制器，{c['solution_summary']}。
系统主要由 {modules} 等模块组成。

**核心技术亮点**：
{innovations}

**关键指标达成**：
{specs}

经测试，系统各项指标均达到或超过题目设计要求。"""

    def _solution(self) -> str:
        modules = self.config["modules"]
        tables = []
        for m in modules:
            rows = []
            for s in m["solutions"]:
                mark = "✅" if s["selected"] else ""
                rows.append(f"| {s['name']} | {s['pros']} | {s['cons']} | {mark} |")
            table = f"""### {m['name']}方案选择

| 方案 | 优点 | 缺点 | 选择 |
|------|------|------|------|
{chr(10).join(rows)}

> **选择理由**：{m['reason']}"""
            tables.append(table)

        modules_desc = "\n".join([
            f"{i+1}. **{m['name']}**：{m['desc']}"
            for i, m in enumerate(self.config["system"]["key_modules"])
        ])

        return f"""## 二、系统方案论证与选择

### 2.1 系统总体架构

系统总体架构如下，主要由以下模块组成：
{modules_desc}

### 2.2 方案对比

{chr(10).join(tables)}"""

    def _theory(self) -> str:
        sections = []
        for i, t in enumerate(self.config["theory"]):
            formulas = "\n".join(t.get("formulas", []))
            sections.append(f"""### 3.{i+1} {t['title']}

{t['content']}

{formulas}""")

        return f"""## 三、理论分析与计算

{chr(10).join(sections)}"""

    def _hardware_software(self) -> str:
        hw = self.config["hardware"]
        sw = self.config["software"]

        circuits = "\n".join([
            f"- **{c['name']}**：{c['desc']}"
            for c in hw["circuits"]
        ])

        algorithms = "\n".join([
            f"- **{a['name']}**：{a['desc']}"
            for a in sw["algorithms"]
        ])

        tasks = "\n".join([
            f"| {t['name']} | {t['trigger']} | {t['period']} |"
            for t in sw["tasks"]
        ])

        return f"""## 四、电路与程序设计

### 4.1 硬件设计

{circuits}

**PCB设计要点**：{hw['pcb_notes']}

### 4.2 软件设计

**软件架构**：{sw['architecture']}

**关键算法**：
{algorithms}

**任务调度**：

| 任务名称 | 触发方式 | 周期 |
|----------|----------|------|
{tasks}"""

    def _tests(self) -> str:
        test_sections = []
        for test in self.config["tests"]:
            rows = "\n".join([
                f"| {i+1} | {t['condition']} | {t['requirement']} | {t['result']} | {'✅' if t['pass'] else '❌'} |"
                for i, t in enumerate(test["items"])
            ])
            test_sections.append(f"""### {test['name']}

| 序号 | 测试条件 | 题目要求 | 实测值 | 达标 |
|------|----------|----------|--------|------|
{rows}""")

        total_items = sum(len(t["items"]) for t in self.config["tests"])
        passed_items = sum(
            sum(1 for item in t["items"] if item["pass"])
            for t in self.config["tests"]
        )

        return f"""## 五、测试方案与测试结果

### 5.1 测试条件
- **测试仪器**：数字示波器、数字万用表、信号发生器、直流稳压电源
- **测试环境**：室温25℃

{chr(10).join(test_sections)}

### 测试结论

共测试 {total_items} 项指标，达标 {passed_items} 项，达标率 {passed_items/total_items*100:.0f}%。"""

    def _error_analysis(self) -> str:
        rows = "\n".join([
            f"| {e['source']} | {e['type']} | {e['value']} | {e['improvement']} |"
            for e in self.config["errors"]
        ])

        # 简单的RSS计算
        values = []
        for e in self.config["errors"]:
            try:
                v = float(e["value"].replace("%", ""))
                values.append(v)
            except ValueError:
                pass
        rss = sum(v**2 for v in values) ** 0.5 if values else 0
        sum_terms = " + ".join([str(v) + "^2" for v in values])
        formula = "$$\\epsilon_{\\text{总}} = \\sqrt{" + sum_terms + "} = " + f"{rss:.3f}" + "\\%$$"

        return f"""## 六、误差分析

### 6.1 误差来源

| 误差来源 | 类型 | 估计量级 | 改善措施 |
|----------|------|----------|----------|
{rows}

### 6.2 总误差计算

采用方和根法合成总误差：

{formula}

系统综合误差为 {rss:.3f}%，满足题目精度要求。"""

    def _summary(self) -> str:
        c = self.config["system"]
        innovations = "\n".join([f"- {v}" for v in c["innovations"]])

        return f"""## 七、总结与展望

### 7.1 创新点

{innovations}

### 7.2 不足与改进

- 可进一步优化PCB布局，减小信号串扰
- 可增加WiFi通信功能，实现远程监控
- 可采用更高性能主控，提升处理速度

### 7.3 收获

通过本次竞赛，团队在硬件设计、软件开发和系统调试方面获得了宝贵的实践经验。"""

    def _references(self) -> str:
        refs = "\n".join(self.config["references"])
        return f"""## 参考文献

{refs}"""

    def _appendix(self) -> str:
        return """## 附录

### 附录A 系统原理图
[请插入原理图]

### 附录B 关键代码
```c
// 核心算法代码
// ...
```

### 附录C PCB版图
[请插入PCB截图]"""


# ============================================================
# 交互式输入
# ============================================================
def interactive_input() -> dict:
    """交互式获取报告信息"""
    config = DEFAULT_CONFIG.copy()

    print("=" * 60)
    print("  电赛设计报告自动生成工具")
    print("=" * 60)
    print()

    # 基本信息
    config["title"] = input(f"报告标题 [{config['title']}]: ").strip() or config["title"]
    config["team"]["school"] = input(f"学校名称 [{config['team']['school']}]: ").strip() or config["team"]["school"]

    members_str = input(f"队员姓名(逗号分隔) [{'、'.join(config['team']['members'])}]: ").strip()
    if members_str:
        config["team"]["members"] = [m.strip() for m in members_str.split(",")]

    config["team"]["advisor"] = input(f"指导教师 [{config['team']['advisor']}]: ").strip() or config["team"]["advisor"]

    # 系统信息
    print("\n--- 系统信息 ---")
    config["system"]["name"] = input(f"系统名称 [{config['system']['name']}]: ").strip() or config["system"]["name"]
    config["system"]["controller"] = input(f"主控芯片 [{config['system']['controller']}]: ").strip() or config["system"]["controller"]
    config["system"]["solution_summary"] = input(f"方案概述 [{config['system']['solution_summary'][:30]}...]: ").strip() or config["system"]["solution_summary"]

    # 生成文件名
    default_filename = f"设计报告_{config['system']['name']}_{datetime.now().strftime('%Y%m%d')}.md"
    filename = input(f"\n输出文件名 [{default_filename}]: ").strip() or default_filename

    config["_output_file"] = filename

    return config


# ============================================================
# 主函数
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="电赛设计报告自动生成工具")
    parser.add_argument("-c", "--config", type=str, help="配置文件路径(JSON)")
    parser.add_argument("-o", "--output", type=str, help="输出文件路径")
    parser.add_argument("--demo", action="store_true", help="生成演示报告")
    parser.add_argument("--interactive", action="store_true", help="交互式输入模式")
    args = parser.parse_args()

    # 确定配置来源
    if args.demo:
        config = DEFAULT_CONFIG.copy()
        print("[INFO] 使用默认配置生成演示报告...")
    elif args.config:
        print(f"[INFO] 从配置文件加载: {args.config}")
        with open(args.config, "r", encoding="utf-8") as f:
            config = json.load(f)
    elif args.interactive or (not args.config and not args.demo):
        config = interactive_input()
    else:
        config = DEFAULT_CONFIG.copy()

    # 生成报告
    generator = ReportGenerator(config)
    report = generator.generate()

    # 确定输出路径
    if args.output:
        output_path = args.output
    elif "_output_file" in config:
        output_path = config["_output_file"]
    else:
        output_path = f"设计报告_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md"

    # 写入文件
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(report)

    print(f"\n[SUCCESS] 报告已生成: {output_path}")
    print(f"[INFO] 文件大小: {os.path.getsize(output_path)} 字节")
    print(f"[INFO] 共 {report.count(chr(10)) + 1} 行")

    # 提示
    print("\n" + "=" * 60)
    print("  使用提示:")
    print("  1. 用 Markdown 编辑器打开生成的报告")
    print("  2. 替换 [XX] 标记处的实际内容")
    print("  3. 插入原理图、流程图、测试波形等图片")
    print("  4. 补充完整的测试数据和误差分析")
    print("  5. 检查公式、图表编号是否正确")
    print("=" * 60)


# ============================================================
# 配置文件生成工具
# ============================================================
def generate_sample_config():
    """生成示例配置文件"""
    output = "report_config_sample.json"
    with open(output, "w", encoding="utf-8") as f:
        json.dump(DEFAULT_CONFIG, f, ensure_ascii=False, indent=2)
    print(f"[INFO] 示例配置文件已生成: {output}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--gen-config":
        generate_sample_config()
    else:
        main()
