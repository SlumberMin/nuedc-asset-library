# 贡献指南

感谢你对本项目的关注！以下是参与贡献的规范。

## 如何贡献

### 报告问题
- 使用 GitHub Issues 报告 Bug
- 请包含：问题描述、复现步骤、硬件平台、编译器版本

### 提交代码

1. Fork 本仓库
2. 创建特性分支：`git checkout -b feature/your-feature`
3. 提交更改：`git commit -m "feat: 添加xxx驱动"`
4. 推送分支：`git push origin feature/your-feature`
5. 创建 Pull Request

### 代码规范

#### C 代码
- 使用 4 空格缩进
- 函数/变量命名：`Module_FunctionName`（驼峰+下划线）
- 每个函数必须有 Doxygen 中文注释
- 头文件必须有 `#ifndef` 守卫
- 所有公开 API 必须有 NULL 指针检查
- 示例：

```c
/**
 * @brief 初始化电机驱动
 * @param htim PWM定时器句柄
 * @return 0=成功, -1=失败
 * @note 默认PWM频率20kHz
 */
int Motor_Init(TIM_HandleTypeDef *htim);
```

#### Python 代码
- 遵循 PEP 8
- 使用类型注解
- 中文注释和文档字符串
- 示例：

```python
def calculate_pid(error: float, dt: float) -> float:
    """计算PID输出
    
    Args:
        error: 误差值
        dt: 时间步长(s)
    
    Returns:
        控制输出
    """
```

#### Markdown 文档
- 使用中文撰写
- 标题层级不超过4级
- 代码块必须标注语言

### 提交信息规范

使用 Conventional Commits 格式：

```
<type>(<scope>): <description>

[optional body]
```

类型：
- `feat`: 新功能
- `fix`: 修复 Bug
- `docs`: 文档更新
- `style`: 代码格式调整
- `refactor`: 重构
- `test`: 测试用例
- `chore`: 构建/工具

范围示例：`stm32`, `mspm0`, `tm4c`, `opi5`, `algo`, `vision`, `docs`

### 目录结构规范

新增驱动文件放置规则：
- STM32 驱动 → `01_stm32/drivers/`
- MSPM0 驱动 → `02_mspm0g3507/drivers/`
- TM4C 驱动 → `04_tm4c123/drivers/`
- 控制算法 → `13_control_algorithms/common/`
- 赛题方案 → `17_competition_solutions/年份_题号_题目名/`
- 仿真脚本 → `15_simulation/`

每个 `.c` 文件必须有对应的 `.h` 文件。

### 三平台并行

新增传感器/执行器驱动时，建议同时提供三个平台的实现：
1. 先完成一个平台的驱动
2. 参照已有驱动的平台差异，适配另外两个平台
3. 参考 `00_docs/三单片机平台差异对照表.md`

## 许可证

提交代码即表示你同意将代码以 MIT 许可证开源。
