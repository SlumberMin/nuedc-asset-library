#!/usr/bin/env python3
"""
项目脚手架生成器 - 一键创建新项目目录结构+模板文件
用法: python project_scaffold.py <项目名称> [--template 基础模板/信号处理/电源设计/控制系统]
"""
import argparse
import os
import shutil
from pathlib import Path
from datetime import datetime

# 项目模板定义
TEMPLATES = {
    "基础模板": {
        "dirs": ["src", "tests", "docs", "data", "output"],
        "files": {
            "src/__init__.py": "",
            "src/main.py": '#!/usr/bin/env python3\n"""主程序入口"""\n\n\ndef main():\n    print("Hello, 电赛项目!")\n\n\nif __name__ == "__main__":\n    main()\n',
            "tests/__init__.py": "",
            "tests/test_main.py": '#!/usr/bin/env python3\n"""主模块测试"""\nimport sys\nimport os\nsys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))\n\n\ndef test_placeholder():\n    assert True\n',
            "docs/README.md": "# {project_name}\n\n## 简介\n\n\n## 使用方法\n\n\n## 目录结构\n\n",
            "requirements.txt": "# 项目依赖\nnumpy\nmatplotlib\n",
            ".gitignore": "__pycache__/\n*.pyc\n.env\noutput/\nbuild/\n",
            "README.md": "# {project_name}\n\n> 电赛项目 - {date}\n\n快速开始: `python src/main.py`\n",
        },
    },
    "信号处理": {
        "dirs": ["src", "src/dsp", "tests", "docs", "data", "output", "sim"],
        "files": {
            "src/__init__.py": "",
            "src/dsp/__init__.py": "",
            "src/dsp/filters.py": '#!/usr/bin/env python3\n"""数字滤波器模块"""\nimport numpy as np\nfrom scipy import signal\n\n\ndef design_lowpass(cutoff, fs, order=5):\n    """设计低通滤波器"""\n    nyq = 0.5 * fs\n    normal_cutoff = cutoff / nyq\n    b, a = signal.butter(order, normal_cutoff, btype="low")\n    return b, a\n\n\ndef apply_filter(data, b, a):\n    """应用滤波器"""\n    return signal.filtfilt(b, a, data)\n',
            "src/dsp/fft_analysis.py": '#!/usr/bin/env python3\n"""FFT频谱分析模块"""\nimport numpy as np\n\n\ndef compute_fft(data, fs):\n    """计算FFT并返回频率和幅度"""\n    n = len(data)\n    freq = np.fft.rfftfreq(n, 1.0 / fs)\n    fft_vals = np.abs(np.fft.rfft(data)) * 2.0 / n\n    return freq, fft_vals\n',
            "src/main.py": '#!/usr/bin/env python3\n"""信号处理主程序"""\nimport numpy as np\nfrom dsp.filters import design_lowpass, apply_filter\nfrom dsp.fft_analysis import compute_fft\n\n\ndef main():\n    fs = 1000  # 采样率\n    t = np.arange(0, 1, 1/fs)\n    signal_data = np.sin(2*np.pi*50*t) + 0.5*np.sin(2*np.pi*200*t)\n\n    b, a = design_lowpass(cutoff=100, fs=fs)\n    filtered = apply_filter(signal_data, b, a)\n    freq, amp = compute_fft(filtered, fs)\n    print(f"信号处理完成, 频率点数: {len(freq)}")\n\n\nif __name__ == "__main__":\n    main()\n',
            "tests/__init__.py": "",
            "tests/test_filters.py": '#!/usr/bin/env python3\n"""滤波器测试"""\nimport sys, os\nsys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))\nimport numpy as np\nfrom dsp.filters import design_lowpass, apply_filter\n\n\ndef test_lowpass_filter():\n    b, a = design_lowpass(100, 1000)\n    assert len(b) > 0\n    assert len(a) > 0\n',
            "docs/README.md": "# {project_name} - 信号处理\n\n## 模块说明\n- dsp/filters.py: 数字滤波器\n- dsp/fft_analysis.py: FFT频谱分析\n",
            "requirements.txt": "numpy\nscipy\nmatplotlib\n",
            ".gitignore": "__pycache__/\n*.pyc\n.env\noutput/\nbuild/\n",
        },
    },
    "电源设计": {
        "dirs": ["src", "tests", "docs", "data", "output", "sim"],
        "files": {
            "src/__init__.py": "",
            "src/power_design.py": '#!/usr/bin/env python3\n"""电源设计计算模块"""\n\n\ndef calculate_voltage_divider(vin, r1, r2):\n    """电阻分压计算"""\n    vout = vin * r2 / (r1 + r2)\n    return vout\n\n\ndef calculate_buck_converter(vin, duty, efficiency=0.9):\n    """Buck变换器输出电压计算"""\n    vout = vin * duty * efficiency\n    return vout\n\n\ndef calculate_output_ripple(iout, fsw, c):\n    """输出纹波电压估算 (ΔV = I_out / (2 * f_sw * C))"""\n    return iout / (2 * fsw * c)\n',
            "src/main.py": '#!/usr/bin/env python3\n"""电源设计主程序"""\nfrom power_design import calculate_voltage_divider, calculate_buck_converter\n\n\ndef main():\n    vout = calculate_voltage_divider(12, 10000, 5000)\n    print(f"分压输出: {vout:.2f}V")\n    vout_buck = calculate_buck_converter(24, 0.33)\n    print(f"Buck输出: {vout_buck:.2f}V")\n\n\nif __name__ == "__main__":\n    main()\n',
            "tests/__init__.py": "",
            "tests/test_power.py": '#!/usr/bin/env python3\n"""电源设计测试"""\nimport sys, os\nsys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))\nfrom power_design import calculate_voltage_divider\n\n\ndef test_voltage_divider():\n    assert abs(calculate_voltage_divider(12, 10000, 10000) - 6.0) < 0.01\n',
            "requirements.txt": "numpy\nmatplotlib\n",
            ".gitignore": "__pycache__/\n*.pyc\noutput/\n",
        },
    },
    "控制系统": {
        "dirs": ["src", "src/controllers", "tests", "docs", "data", "output", "sim"],
        "files": {
            "src/__init__.py": "",
            "src/controllers/__init__.py": "",
            "src/controllers/pid.py": '#!/usr/bin/env python3\n"""PID控制器"""\n\n\nclass PIDController:\n    def __init__(self, kp, ki, kd, setpoint=0):\n        self.kp = kp\n        self.ki = ki\n        self.kd = kd\n        self.setpoint = setpoint\n        self._prev_error = 0\n        self._integral = 0\n\n    def update(self, measured, dt):\n        error = self.setpoint - measured\n        self._integral += error * dt\n        derivative = (error - self._prev_error) / dt if dt > 0 else 0\n        self._prev_error = error\n        return self.kp * error + self.ki * self._integral + self.kd * derivative\n\n    def reset(self):\n        self._prev_error = 0\n        self._integral = 0\n',
            "src/controllers/encoder.py": '#!/usr/bin/env python3\n"""编码器计数模块"""\n\n\nclass Encoder:\n    def __init__(self, ppr=360):\n        self.ppr = ppr  # 每转脉冲数\n        self._count = 0\n\n    def pulse(self, direction=1):\n        self._count += direction\n\n    def get_angle(self):\n        return (self._count / self.ppr) * 360\n\n    def get_speed(self, dt):\n        angle = self.get_angle()\n        rpm = (angle / 360) / (dt / 60) if dt > 0 else 0\n        return rpm\n\n    def reset(self):\n        self._count = 0\n',
            "src/main.py": '#!/usr/bin/env python3\n"""控制系统主程序"""\nfrom controllers.pid import PIDController\n\n\ndef main():\n    pid = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=100)\n    measured = 0\n    dt = 0.01\n    for _ in range(1000):\n        output = pid.update(measured, dt)\n        measured += output * dt  # 简化模型\n    print(f"最终值: {measured:.2f}, 目标: {pid.setpoint}")\n\n\nif __name__ == "__main__":\n    main()\n',
            "tests/__init__.py": "",
            "tests/test_pid.py": '#!/usr/bin/env python3\n"""PID控制器测试"""\nimport sys, os\nsys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))\nfrom controllers.pid import PIDController\n\n\ndef test_pid_converges():\n    pid = PIDController(kp=2.0, ki=0.5, kd=0.1, setpoint=50)\n    val = 0\n    for _ in range(2000):\n        val += pid.update(val, 0.01) * 0.01\n    assert abs(val - 50) < 1.0\n',
            "requirements.txt": "numpy\nmatplotlib\n",
            ".gitignore": "__pycache__/\n*.pyc\noutput/\n",
        },
    },
}


def create_project(name: str, template: str, base_dir: str = "."):
    """创建项目"""
    tpl = TEMPLATES.get(template, TEMPLATES["基础模板"])
    project_path = Path(base_dir) / name

    if project_path.exists():
        print(f"❌ 项目目录已存在: {project_path}")
        return False

    # 创建目录
    for d in tpl["dirs"]:
        (project_path / d).mkdir(parents=True, exist_ok=True)
        print(f"  📁 {d}/")

    # 创建文件
    for fpath, content in tpl["files"].items():
        full_path = project_path / fpath
        full_path.parent.mkdir(parents=True, exist_ok=True)
        filled = content.replace("{project_name}", name).replace("{date}", datetime.now().strftime("%Y-%m-%d"))
        full_path.write_text(filled, encoding="utf-8")
        print(f"  📄 {fpath}")

    print(f"\n✅ 项目 '{name}' 创建完成 ({template})")
    print(f"   路径: {project_path.resolve()}")
    return True


def main():
    parser = argparse.ArgumentParser(description="电赛项目脚手架生成器")
    parser.add_argument("name", nargs="?", help="项目名称")
    parser.add_argument("--template", "-t", choices=list(TEMPLATES.keys()), default="基础模板", help="项目模板")
    parser.add_argument("--output", "-o", default=".", help="输出目录")
    parser.add_argument("--list", "-l", action="store_true", help="列出可用模板")
    args = parser.parse_args()

    if args.list:
        print("可用模板:")
        for name, tpl in TEMPLATES.items():
            print(f"  • {name} ({len(tpl['dirs'])}个目录, {len(tpl['files'])}个文件)")
        return

    if not args.name:
        parser.error("请指定项目名称")

    create_project(args.name, args.template, args.output)


if __name__ == "__main__":
    main()
