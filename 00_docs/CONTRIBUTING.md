# 贡献指南

感谢你对nuedc-asset-library的关注！本指南帮助你了解如何参与贡献。

---

## 目录

1. [行为准则](#行为准则)
2. [如何贡献](#如何贡献)
3. [代码规范](#代码规范)
4. [提交规范](#提交规范)
5. [目录结构规范](#目录结构规范)
6. [文档规范](#文档规范)
7. [测试规范](#测试规范)
8. [问题反馈](#问题反馈)

---

## 行为准则

- 保持友善和尊重
- 接受建设性批评
- 专注于对社区最有利的事情
- 对他人表示同理心

---

## 如何贡献

### 报告问题

1. 使用 Issues 模板提交 Bug 报告
2. 提供复现步骤、期望行为、实际行为
3. 附上相关日志、截图、硬件配置

### 提交代码

1. Fork 本仓库
2. 创建特性分支: `git checkout -b feature/your-feature`
3. 提交更改 (遵循提交规范)
4. 推送到 Fork: `git push origin feature/your-feature`
5. 创建 Pull Request

### 贡献类型

| 类型 | 说明 | 示例 |
|---|---|---|
| 新算法 | 添加新的控制/视觉算法 | 添加 EKF 扩展卡尔曼 |
| 新硬件 | 添加新平台/模块支持 | 添加 ESP32 代码库 |
| Bug 修复 | 修复已知问题 | 修复 PID 积分溢出 |
| 性能优化 | 提升运行效率 | OpenCV 算法 SIMD 优化 |
| 文档 | 完善文档和示例 | 补充 API 文档 |
| 测试 | 补充测试用例 | 添加边界条件测试 |
| 模板 | 添加报告/电路模板 | 添加新题型解题模板 |

---

## 代码规范

### C 语言 (STM32 / 算法库)

```c
/* 文件头注释 */
/**
 * @file    filename.c
 * @brief   简要描述
 * @details 详细描述
 * @author  作者
 * @version 1.0
 * @date    2026-06-10
 */

/* 命名规范 */
// 文件名: 小写下划线 snake_case (如: pid_controller.c)
// 函数名: 模块名_动词名词 (如: PID_Init, Motor_SetSpeed)
// 类型名: 大驼峰 + _t 后缀 (如: PID_t, MotorDriver_t)
// 宏定义: 全大写下划线 (如: GPIO_SET, CLAMP)
// 枚举值: 大写下划线 (如: PID_MODE_POSITION)

/* 缩进: 4 空格，不用 Tab */
/* 大括号: K&R 风格 */
if (condition) {
    do_something();
} else {
    do_other();
}

/* 参数校验: 函数入口必须校验指针 */
ErrorCode_t SomeFunc(SomeStruct_t *s) {
    if (s == NULL) return HAL_ERR_PARAM;
    if (!s->initialized) return HAL_ERR_NOT_INIT;
    // ...
}
```

### Python (Orange Pi 5 / 视觉)

```python
"""模块文档字符串
简要描述
"""

# 命名规范
# 文件名: 小写下划线 snake_case (如: color_tracker.py)
# 类名: 大驼峰 PascalCase (如: ColorTracker)
# 函数/方法: 小写下划线 (如: detect_color)
# 常量: 全大写 (如: MAX_TARGETS)
# 私有: 单下划线前缀 (如: _internal_method)

# 缩进: 4 空格
# 行宽: 120 字符
# 类型注解: 建议使用
def compute(self, error: float, dt: float = None) -> float:
    """文档字符串说明参数和返回值"""
    pass

# 导入顺序: 标准库 → 第三方库 → 本地模块
import os
import cv2
import numpy as np

from local_module import MyClass
```

### Markdown 文档

- 标题层级不超过 4 级
- 代码块必须标注语言
- 表格对齐使用 `|---|`
- 中英文之间加空格
- 文件名使用中文或英文均可，建议中文便于理解

---

## 提交规范

使用 [Conventional Commits](https://www.conventionalcommits.org/) 格式:

```
<类型>(<范围>): <描述>

<正文>

<脚注>
```

### 类型

| 类型 | 说明 |
|---|---|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `docs` | 文档更新 |
| `style` | 代码格式 (不影响逻辑) |
| `refactor` | 重构 |
| `perf` | 性能优化 |
| `test` | 测试 |
| `chore` | 构建/工具变更 |

### 示例

```
feat(pid): 添加前馈控制功能

- 新增 PID_SetFeedforward() 接口
- 前馈值直接加到输出
- 更新 API 文档

Closes #42
```

---

## 目录结构规范

### 新增模块

```
XX_模块名/
├── README.md           # 模块说明 (必需)
├── 子目录/
│   ├── file.h          # C 头文件
│   ├── file.c          # C 源文件
│   └── file.py         # Python 文件
├── docs/
│   └── 使用指南.md      # 文档
└── tests/
    └── test_xxx.py     # 测试 (Python)
```

### 命名规范

- 顶层目录: `数字_中文名` (如: `01_stm32`)
- 子目录: 英文小写下划线
- 文件: 英文小写下划线 + 扩展名

---

## 文档规范

### API 文档

每个公开函数/类必须包含:

```c
/**
 * @brief 功能简述
 * @param param1 参数说明
 * @param param2 参数说明
 * @return 返回值说明
 * @note 特殊说明 (可选)
 * @warning 警告 (可选)
 */
```

```python
def function(param1: int, param2: str = 'default') -> bool:
    """
    功能简述

    Parameters
    ----------
    param1 : int
        参数说明
    param2 : str
        参数说明 (默认: 'default')

    Returns
    -------
    bool : 返回值说明

    Notes
    -----
    特殊说明
    """
```

### 使用示例

每个模块 README 必须包含快速上手示例。

---

## 测试规范

### Python 测试

- 使用 `unittest` 框架
- 文件命名: `tests/test_模块名.py`
- 测试类: `Test功能名`
- 测试方法: `test_具体场景`

```python
import unittest

class TestPIDController(unittest.TestCase):
    def test_proportional_only(self):
        """纯比例控制测试"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        output = pid.compute(error=5.0, dt=0.01)
        self.assertAlmostEqual(output, 5.0, places=2)

    def test_output_clamping(self):
        """输出限幅测试"""
        pid = PIDController(kp=100.0, output_min=-50, output_max=50)
        output = pid.compute(error=100.0, dt=0.01)
        self.assertLessEqual(output, 50.0)

if __name__ == '__main__':
    unittest.main()
```

### 运行测试

```bash
# 运行全部测试
cd nuedc-asset-library
python -m pytest tests/ -v

# 运行单个测试文件
python -m pytest tests/test_pid.py -v

# 运行带覆盖率
python -m pytest tests/ -v --cov=. --cov-report=html
```

### 测试覆盖要求

| 模块类型 | 最低覆盖率 |
|---|---|
| 控制算法 | 80% |
| 视觉算法 | 60% |
| 硬件驱动 | 40% (需 Mock) |

---

## 问题反馈

### Bug 报告模板

```markdown
**描述**: 简要描述问题

**复现步骤**:
1. 使用 '...'
2. 调用 '...'
3. 观察到 '...'

**期望行为**: ...

**实际行为**: ...

**环境**:
- 平台: Orange Pi 5 / STM32F407
- OS: Ubuntu 22.04 / Keil MDK 5
- OpenCV: 4.8.0
- Python: 3.10

**附加信息**: 日志、截图、波形
```

### 功能请求模板

```markdown
**描述**: 简要描述功能需求

**使用场景**: 为什么需要这个功能？

**建议实现**: 如何实现？

**替代方案**: 其他解决方式
```

---

## 许可证

贡献的代码将使用与本项目相同的许可证。提交贡献即表示你同意将代码置于该项目的许可证下。
