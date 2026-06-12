# 工具集

本目录包含nuedc-asset-library的全部 Python 工具脚本，涵盖硬件选型、测试验证、代码分析、项目管理等多个方面。

> **运行方式**：所有工具均为独立 Python 脚本，可通过 `python <工具名>.py` 直接运行。部分工具支持命令行参数，使用 `--help` 查看详情。

---

## 选型工具

硬件元器件和方案选型辅助工具，帮助在设计初期做出合理选择。

| 工具 | 功能 | 用法 |
|------|------|------|
| `mcu_selector.py` | **MCU 选型** — 根据外设需求、性能指标等推荐最佳微控制器 | `python mcu_selector.py --help` |
| `sensor_selector.py` | **传感器选型** — 根据应用场景（温度/距离/惯性等）推荐传感器型号 | `python sensor_selector.py --help` |
| `driver_selector.py` | **驱动 IC 选型** — 根据负载类型（电机/LED/继电器等）和参数推荐驱动芯片 | `python driver_selector.py --help` |
| `antenna_matching_calculator.py` | **天线匹配计算** — L 型/Π 型/T 型匹配网络设计与阻抗变换计算 | `python antenna_matching_calculator.py` |
| `pcb_impedance_calculator.py` | **PCB 阻抗计算** — 微带线/带状线/差分对阻抗计算 | `python pcb_impedance_calculator.py` |
| `frequency_planner.py` | **频率规划器** — PLL/DDS/时钟树配置参数计算 | `python frequency_planner.py` |
| `power_calculator.py` | **功率计算** — 电池容量/续航时间/功耗预算估算 | `python power_calculator.py` |
| `power_sequencer.py` | **电源时序设计** — 上电顺序规划、时序图生成、保护电路设计 | `python power_sequencer.py` |

---

## 测试工具

硬件和软件测试、验证工具，用于确保系统可靠运行。

| 工具 | 功能 | 用法 |
|------|------|------|
| `battery_test.py` | **电池测试** — 电池充放电曲线、容量衰减、内阻测量 | `python battery_test.py` |
| `motor_test.py` | **电机测试** — 电机性能参数测试 | `python motor_test.py` |
| `motor_characterization.py` | **电机特性分析** — 电机建模、Kv/Kt 参数辨识、效率曲线绘制 | `python motor_characterization.py` |
| `sensor_calibration.py` | **传感器标定** — 零偏/增益/非线性校正 | `python sensor_calibration.py` |
| `sensor_validation.py` | **传感器验证** — 传感器数据完整性与准确性验证 | `python sensor_validation.py` |
| `communication_tester.py` | **通信测试** — 串口/I2C/SPI 通信链路测试 | `python communication_tester.py` |
| `protocol_analyzer.py` | **协议分析** — 解析 UART/SPI/I2C 数据包，排查通信问题 | `python protocol_analyzer.py` |
| `hardware_debugger.py` | **硬件调试** — GPIO/ADC/PWM/I2C/SPI 在线调试辅助 | `python hardware_debugger.py` |
| `system_validator.py` | **系统验证** — 全系统级功能与性能验证 | `python system_validator.py` |
| `test_automation.py` | **测试自动化框架** — 批量运行测试 + 生成报告 | `python test_automation.py` |
| `test_generator.py` | **测试代码生成** — 从驱动 `.h` 文件自动解析并生成测试模板 | `python test_generator.py` |
| `benchmark_runner.py` | **基准测试运行器** — 运行所有仿真和测试，生成性能报告 | `python benchmark_runner.py` |

---

## 调试工具

信号观测、数据可视化和运行时调试工具。

| 工具 | 功能 | 用法 |
|------|------|------|
| `serial_plotter.py` | **串口实时绘图** — 连接串口实时绘制传感器/控制数据波形 | `python serial_plotter.py --port COMx` |
| `data_visualizer.py` | **数据可视化** — 串口数据实时绘图 + CSV 导入 + 多通道显示 | `python data_visualizer.py` |
| `signal_generator_tool.py` | **信号生成** — 正弦/方波/三角波/噪声/任意波形输出 | `python signal_generator_tool.py` |
| `sensor_simulator.py` | **传感器模拟** — 模拟各种传感器数据输出，用于无硬件调试 | `python sensor_simulator.py` |
| `communication_simulator.py` | **通信仿真** — ASK/FSK/PSK/QAM 调制、信道编码、AWGN 信道仿真 | `python communication_simulator.py` |
| `thermal_simulator.py` | **热仿真** — PCB 热阻计算、散热分析、温度场预测 | `python thermal_simulator.py` |
| `firmware_flasher.py` | **固件烧录** — 固件下载与烧录管理 | `python firmware_flasher.py` |
| `pid_tuner.py` | **PID 调参** — PID 参数手动调整与响应曲线预览 | `python pid_tuner.py` |
| `pid_auto_tuner.py` | **PID 自动整定** — 继电反馈法 + Ziegler-Nichols 规则自动计算 PID 参数 | `python pid_auto_tuner.py` |
| `control_tuner.py` | **控制参数整定** — PID 自整定、ADRC 参数设计、MPC 控制仿真 | `python control_tuner.py` |

---

## 分析工具

代码质量、性能和工程管理分析工具。

| 工具 | 功能 | 用法 |
|------|------|------|
| **信号与系统分析** | | |
| `signal_analyzer.py` | **信号分析** — FFT 频谱分析、THD/SNR/ENOB 计算、功率谱密度 | `python signal_analyzer.py` |
| `filter_designer.py` | **滤波器设计** — FIR/IIR 滤波器设计、Butterworth/Chebyshev/椭圆滤波器 | `python filter_designer.py` |
| `timing_analyzer.py` | **时序分析** — 中断延迟/任务响应/最坏执行时间 (WCET) 分析 | `python timing_analyzer.py` |
| `embedded_profiler.py` | **嵌入式性能分析** — 代码执行时间估算、内存使用分析、中断延迟计算 | `python embedded_profiler.py` |
| **代码质量分析** | | |
| `code_complexity.py` | **代码复杂度** — 圈复杂度、认知复杂度、函数长度分析 | `python code_complexity.py` |
| `code_deduplicator.py` | **代码去重** — 检测项目中的重复代码段 | `python code_deduplicator.py` |
| `code_metrics_dashboard.py` | **代码指标仪表盘** — 可视化展示代码质量指标 | `python code_metrics_dashboard.py` |
| `code_review_bot.py` | **代码审查机器人** — 自动代码审查 | `python code_review_bot.py` |
| `code_review_checklist.py` | **代码审查清单** — 基于错误经验库的自动检查 | `python code_review_checklist.py` |
| `quality_scorecard.py` | **质量评分卡** — 电赛资产库 V2 审计评分 | `python quality_scorecard.py` |
| `coverage_analyzer.py` | **测试覆盖率分析** — 分析测试文件对驱动/模块的覆盖情况 | `python coverage_analyzer.py` |
| `memory_usage_analyzer.py` | **内存使用分析** — Flash/RAM 占用分析 | `python memory_usage_analyzer.py` |
| `firmware_size_analyzer.py` | **固件体积分析** — 各段（.text/.data/.bss）大小分析 | `python firmware_size_analyzer.py` |
| `import_graph.py` | **依赖图生成** — 生成模块间的依赖关系图 | `python import_graph.py` |
| **工程管理工具** | | |
| `api_compat_checker.py` | **API 兼容性检查** — 检查 STM32/ESP32/MSP432 三平台 API 一致性 | `python api_compat_checker.py` |
| `api_doc_generator.py` | **API 文档生成** — 从代码注释自动生成 API 参考文档 | `python api_doc_generator.py` |
| `audit_checker.py` | **深度审计** — 电赛资产库错误模式自动检测 | `python audit_checker.py` |
| `asset_stats.py` | **资产统计** — 文件数/代码行数/测试覆盖率/模块分布统计 | `python asset_stats.py` |
| `batch_fix.py` | **批量修复** — 修复综合质量报告 Top 3 问题 | `python batch_fix.py` |
| `batch_fix_critical.py` | **批量修复关键问题** — 修复审计发现的 critical 级别问题 | `python batch_fix_critical.py` |
| `board_config_generator.py` | **板级配置生成** — 从引脚表生成 SysConfig/CubeMX 配置文件 | `python board_config_generator.py` |
| `sysconfig_converter.py` | **SysConfig 转换** — 从引脚表自动生成 TI MSPM0/MSP430 的 .syscfg 配置文件 | `python sysconfig_converter.py` |
| `pin_conflict_checker.py` | **引脚冲突检查** — 检测引脚复用冲突 | `python pin_conflict_checker.py` |
| `dependency_checker.py` | **依赖检查** — 扫描 Python 文件的 import 依赖，检查是否已安装 | `python dependency_checker.py` |
| `code_generator.py` | **代码生成** — 电赛备赛实用代码模板生成 | `python code_generator.py` |
| `driver_generator.py` | **驱动代码生成** — 从传感器数据手册/描述自动生成驱动模板 | `python driver_generator.py` |
| `project_scaffold.py` | **项目脚手架** — 一键创建新项目目录结构 + 模板文件 | `python project_scaffold.py` |
| `test_coverage_report.py` | **测试覆盖率报告** — 生成覆盖率报告文档 | `python test_coverage_report.py` |
| `test_report_generator.py` | **测试报告生成** — 汇总测试结果生成报告 | `python test_report_generator.py` |
| `test_result_analyzer.py` | **测试结果分析** — 分析测试通过率、失败模式 | `python test_result_analyzer.py` |
| `report_generator.py` | **综合报告生成** — 生成项目综合分析报告 | `python report_generator.py` |
| `doc_updater.py` | **文档自动更新** — 从代码注释/Doxygen 标记自动更新 README 和 API 文档 | `python doc_updater.py` |
| `changelog_generator.py` | **变更日志生成** — 自动生成 CHANGELOG | `python changelog_generator.py` |
| `release_notes.py` | **发布说明生成** — 从 git log 自动生成结构化发布说明 | `python release_notes.py` |
| `git_pre_commit_hook.py` | **Git Pre-commit 钩子** — 提交前自动运行代码审计检查 | `python git_pre_commit_hook.py` |
| `system_monitor.py` | **系统监控** — 运行时系统状态监控 | `python system_monitor.py` |
| `competition_scoring.py` | **竞赛评分** — 根据电赛评分标准自动评分 | `python competition_scoring.py` |
| `competition_timer.py` | **竞赛计时器** — 电赛比赛时间管理 | `python competition_timer.py` |

---

## 使用说明

### 基本用法
```bash
# 查看工具帮助
python <工具名>.py --help

# 运行选型工具示例
python mcu_selector.py
python sensor_selector.py

# 运行分析工具示例
python code_complexity.py
python asset_stats.py
```

### 环境要求
- Python 3.8+
- 部分工具依赖 `numpy`、`matplotlib` 等科学计算库
- 串口相关工具需要 `pyserial`
- 可通过 `dependency_checker.py` 检查所有依赖是否已安装

### 工具统计
- **总计**：70 个 Python 工具脚本
- 选型工具：8 个
- 测试工具：12 个
- 调试工具：10 个
- 分析工具：40 个（含信号分析、代码质量、工程管理）
