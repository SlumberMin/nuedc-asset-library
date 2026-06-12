#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试代码生成器 - 从驱动.h文件自动解析并生成测试模板
用法: python test_generator.py --header drivers/bmp280.h --output tests/
"""

import argparse
import os
import re
from datetime import datetime


def parse_header(header_path: str) -> dict:
    """
    解析.h头文件，提取宏定义、枚举、结构体、函数声明
    """
    with open(header_path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    module_name = os.path.splitext(os.path.basename(header_path))[0]
    info = {
        "module": module_name,
        "macros": [],       # 寄存器宏
        "enums": [],        # 枚举类型
        "structs": [],      # 结构体
        "functions": [],    # 函数声明
    }

    # 提取 #define 宏
    for m in re.finditer(r'#define\s+(\w+)\s+\((.+?)\)\s*(?:/\*.*?\*/)?', content):
        name, value = m.group(1), m.group(2)
        info["macros"].append((name, value))

    # 提取函数声明 (匹配返回值 函数名(参数);)
    func_pattern = r'(?:int|void|bool|uint\d+_t|float|double|static\s+\w+|extern\s+\w+|\w+_t)\s+(\w+)\s*\(([^)]*)\)\s*;'
    for m in re.finditer(func_pattern, content, re.MULTILINE):
        func_name = m.group(1)
        params_str = m.group(2).strip()
        # 解析参数
        params = []
        if params_str:
            for p in params_str.split(","):
                p = p.strip()
                if p:
                    # 取最后一个token作为参数名
                    tokens = p.split()
                    param_name = tokens[-1].lstrip("*") if tokens else "arg"
                    params.append((p, param_name))
        info["functions"].append((func_name, params))

    # 提取枚举名
    for m in re.finditer(r'typedef\s+enum\s*\{[^}]*\}\s*(\w+)', content, re.DOTALL):
        info["enums"].append(m.group(1))

    # 提取结构体名
    for m in re.finditer(r'typedef\s+struct\s*\{[^}]*\}\s*(\w+)', content, content):
        info["structs"].append(m.group(1))

    return info


def generate_test_file(info: dict) -> str:
    """根据解析信息生成测试源文件"""
    mod = info["module"]
    lines = [
        f"/**",
        f" * @file test_{mod}.c",
        f" * @brief {mod.upper()} 驱动测试文件",
        f" * @note  由 test_generator.py 自动生成",
        f" * @date  {datetime.now().strftime('%Y-%m-%d')}",
        f" */",
        f"",
        f'#include <stdio.h>',
        f'#include <assert.h>',
        f'#include <string.h>',
        f'#include "{mod}.h"',
        f"",
        f"/* ========== 测试计数 ========== */",
        f"static int _test_pass = 0;",
        f"static int _test_fail = 0;",
        f"",
        f"#define TEST_ASSERT(cond, msg) do {{ \\",
        f"    if (cond) {{ _test_pass++; printf(\"  [PASS] %s\\n\", msg); }} \\",
        f"    else      {{ _test_fail++; printf(\"  [FAIL] %s\\n\", msg); }} \\",
        f"}} while(0)",
        f"",
        f"#define TEST_BEGIN(name) printf(\"\\n=== %s ===\\n\", name)",
        f"",
    ]

    # 结构体初始化测试
    if info["structs"]:
        for s in info["structs"]:
            lines += [
                f"/* 测试 {s} 结构体初始化 */",
                f"static void test_{s}_init(void) {{",
                f'    TEST_BEGIN("{s} 初始化");',
                f"    {s} dev;",
                f"    memset(&dev, 0, sizeof(dev));",
                f'    TEST_ASSERT(sizeof(dev) > 0, "{s} 大小应大于0");',
                f"}}",
                f"",
            ]

    # 函数测试桩
    for func_name, params in info["functions"]:
        # 跳过init和test函数本身
        if func_name.startswith("test_"):
            continue

        lines += [
            f"/* 测试 {func_name} */",
            f"static void test_{func_name}(void) {{",
            f'    TEST_BEGIN("{func_name}");',
        ]

        # 根据参数类型生成测试代码
        init_lines = []
        call_args = []
        for ptype, pname in params:
            if "*" in ptype and "const" not in ptype:
                # 输出参数
                base_type = ptype.replace("*", "").strip()
                lines.append(f"    {base_type} {pname} = 0;")
                call_args.append(f"&{pname}")
            elif "dev" in pname.lower():
                # 设备参数
                dev_type = ptype.lstrip("*").strip()
                lines.append(f"    {dev_type} {pname};")
                lines.append(f"    memset(&{pname}, 0, sizeof({pname}));")
                call_args.append(f"&{pname}" if "*" in ptype else pname)
            elif "uint" in ptype or "int" in ptype:
                call_args.append("0")
            elif "char" in ptype and "*" in ptype:
                call_args.append('NULL')
            else:
                call_args.append("0")

        args_str = ", ".join(call_args)
        lines += [
            f"",
            f"    /* TODO: 设置测试条件 */",
            f"    int ret = {func_name}({args_str});",
            f'    TEST_ASSERT(ret == {mod.upper()}_OK, "{func_name} 应返回 OK");',
            f"",
            f"    /* TODO: 验证输出参数 */",
            f"}}",
            f"",
        ]

    # 宏测试
    if info["macros"]:
        lines += [
            f"/* 测试宏定义 */",
            f"static void test_macros(void) {{",
            f'    TEST_BEGIN("宏定义值");',
        ]
        for macro_name, macro_val in info["macros"][:10]:  # 最多测10个
            lines.append(f'    TEST_ASSERT({macro_name} == {macro_val}, "{macro_name} == {macro_val}");')
        lines += [
            f"}}",
            f"",
        ]

    # main函数
    lines += [
        f"int main(void) {{",
        f'    printf("===== {mod.upper()} 驱动测试 =====\\n");',
        f"",
    ]

    # 调用所有test_函数
    if info["structs"]:
        for s in info["structs"]:
            lines.append(f"    test_{s}_init();")
    for func_name, _ in info["functions"]:
        if not func_name.startswith("test_"):
            lines.append(f"    test_{func_name}();")
    if info["macros"]:
        lines.append(f"    test_macros();")

    lines += [
        f"",
        f'    printf("\\n===== 测试结果: %d 通过, %d 失败 =====\\n", _test_pass, _test_fail);',
        f"    return _test_fail > 0 ? 1 : 0;",
        f"}}",
    ]

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="测试代码生成器 - 从.h文件自动生成测试模板")
    parser.add_argument("--header", required=True, help="驱动头文件路径 (.h)")
    parser.add_argument("--output", default=".", help="输出目录")
    parser.add_argument("--list-only", action="store_true", help="仅列出解析到的函数，不生成文件")
    args = parser.parse_args()

    if not os.path.isfile(args.header):
        print(f"[✗] 文件不存在: {args.header}")
        return

    # 解析头文件
    info = parse_header(args.header)
    mod = info["module"]
    print(f"[i] 解析模块: {mod}")
    print(f"    宏定义: {len(info['macros'])} 个")
    print(f"    枚举:   {len(info['enums'])} 个")
    print(f"    结构体: {len(info['structs'])} 个")
    print(f"    函数:   {len(info['functions'])} 个")

    if args.list_only:
        print("\n函数列表:")
        for fname, params in info["functions"]:
            pstr = ", ".join(p[0] for p in params)
            print(f"  {fname}({pstr})")
        return

    # 生成测试文件
    os.makedirs(args.output, exist_ok=True)
    test_path = os.path.join(args.output, f"test_{mod}.c")
    with open(test_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(generate_test_file(info))

    print(f"[✓] 测试文件已生成: {test_path}")


if __name__ == "__main__":
    main()
