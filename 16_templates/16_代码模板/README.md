# 代码生成模板库

> **版本**: 2.0  
> **日期**: 2026-06-12  
> **基于**: 错误经验库 47个已知错误模式

---

## 模板清单

| # | 文件名 | 类型 | 覆盖错误模式 | 用途 |
|---|--------|------|-------------|------|
| 1 | `c_driver_template.c` | C源文件 | #1,5,6,7,8,13,16,18,22,31-32,34,41,45-46 | C驱动代码生成 |
| 2 | `c_driver_template.h` | C头文件 | #14,40,42,43 | C头文件生成 |
| 3 | `python_simulation_template.py` | Python | #2,3,4,10,27-30,36-39 | Python仿真代码生成 |
| 4 | `python_test_template.py` | Python | #4,9,10,12,15,17-19,28-29 | Python测试代码生成 |
| 5 | `solution_readme_template.md` | Markdown | #14,40-44 | 赛题方案README生成 |

---

## 使用方式

### C驱动模板 (1+2)
```
复制 c_driver_template.c 和 c_driver_template.h
搜索替换 YOUR_MODULE 为实际模块名 (如 Motor, Servo, Encoder)
```

### Python仿真模板 (3)
```
复制 python_simulation_template.py
搜索替换 YourSimulation 为实际仿真类名
修改 dt/setpoint/模型参数
```

### Python测试模板 (4)
```
复制 python_test_template.py
搜索替换 YOUR_MODULE 为实际模块名
添加具体测试用例
```

### 方案README模板 (5)
```
复制 solution_readme_template.md
填写7大章节内容
确保代码片段使用实际驱动API (第7节API映射表)
```

---

## 防护清单 (模板内置)

### C代码防护
- ✅ 除零保护 (`if(fabsf(x)<1e-6f)`)
- ✅ volatile修饰 (`ISR共享变量`)
- ✅ 超时计数器 (`I2C/UART while循环`)
- ✅ 数组边界检查 (`idx < size`)
- ✅ 角度/脉宽除零 (`range > 0`)
- ✅ 浮点函数f后缀 (`fabsf/sinf/cosf`)
- ✅ 运算符优先级 (`!(x & flag)`)

### Python代码防护
- ✅ Agg后端 (`matplotlib.use('Agg')` 在 `pyplot` 之前)
- ✅ __main__守卫 (`if __name__ == '__main__':`)
- ✅ 相对路径 (`os.path.dirname(__file__)`)
- ✅ numpy兼容 (`_trapz` shim)
- ✅ plt.close('all') 替代 plt.show()
- ✅ 指定异常类型 (不用 bare `except:`)
- ✅ 循环外.close()
- ✅ npz数据正确读取 (用 `data['key']` 不用 `getattr`)
- ✅ 随机种子固定 (`random.seed(42)`)
- ✅ 强断言 (`assertEqual` 不用 `assertTrue`)

### 方案README防护
- ✅ 7大章节结构
- ✅ 可编译代码 (添加#include和宏定义)
- ✅ API映射表 (伪代码→实际API)
- ✅ 引脚冲突速查表
- ✅ 两套驱动架构说明
