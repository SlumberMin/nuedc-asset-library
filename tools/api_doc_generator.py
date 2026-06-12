#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
API文档生成器 - 电赛资产库工具
===================================
功能：从C头文件(.h)中提取函数签名、结构体、枚举等定义，自动生成Markdown API文档
用法：
  python api_doc_generator.py                          # 扫描资产库所有.h文件
  python api_doc_generator.py --path /path/to/dir      # 指定扫描目录
  python api_doc_generator.py --file single.h          # 扫描单个文件
  python api_doc_generator.py --output docs/API.md     # 指定输出文件
  python api_doc_generator.py --lang en                # 英文文档

依赖：无额外依赖（纯Python实现）
"""

import argparse
import os
import re
import sys
from datetime import datetime
from pathlib import Path


# ============================================================
# C语言解析引擎
# ============================================================

class CHeaderParser:
    """
    C头文件解析器
    
    使用正则表达式提取：
    - 函数声明（返回类型、函数名、参数列表）
    - 结构体定义（名称、成员字段）
    - 枚举定义（名称、枚举值）
    - 宏定义（#define NAME value）
    - typedef定义
    - 文件注释（doxygen风格）
    """

    def __init__(self):
        """初始化正则表达式模式"""
        
        # 函数声明模式：匹配返回值 函数名(参数列表);
        # 支持多行声明，支持指针返回值，支持各种修饰符
        self.re_func = re.compile(
            r'^\s*'                                          # 行首空白
            r'((?:static\s+|inline\s+|extern\s+|const\s+)*' # 修饰符
            r'[\w][\w\s\*]*?)\s+'                           # 返回类型
            r'(\w+)\s*'                                      # 函数名
            r'\(([^)]*)\)\s*;',                              # 参数列表
            re.MULTILINE
        )
        
        # 结构体定义模式
        self.re_struct = re.compile(
            r'(?:typedef\s+)?struct\s+(?:\w+\s+)?\{([^}]*)\}\s*(\w*)\s*;',
            re.DOTALL
        )
        
        # 枚举定义模式
        self.re_enum = re.compile(
            r'enum\s+(\w*)\s*\{([^}]*)\}',
            re.DOTALL
        )
        
        # 宏定义模式（仅匹配带值的宏）
        self.re_define = re.compile(
            r'#define\s+(\w+)\s+(.+?)(?://.*)?$',
            re.MULTILINE
        )
        
        # typedef模式
        self.re_typedef = re.compile(
            r'typedef\s+(.+?)\s+(\w+)\s*;',
            re.MULTILINE
        )
        
        # Doxygen注释块
        self.re_doxygen = re.compile(
            r'/\*\*(.*?)\*/',
            re.DOTALL
        )
        
        # 单行注释
        self.re_line_comment = re.compile(
            r'//\s*(.*?)$',
            re.MULTILINE
        )

    def parse_file(self, filepath):
        """
        解析单个.h文件，返回结构化数据
        
        参数:
            filepath: .h文件路径
            
        返回:
            dict: 包含文件路径、函数列表、结构体列表、枚举列表、宏列表
        """
        try:
            # 尝试多种编码读取
            content = None
            for encoding in ['utf-8', 'gbk', 'latin-1']:
                try:
                    with open(filepath, 'r', encoding=encoding) as f:
                        content = f.read()
                    break
                except UnicodeDecodeError:
                    continue
            
            if content is None:
                return None
            
            result = {
                'filepath': str(filepath),
                'filename': os.path.basename(filepath),
                'functions': [],
                'structs': [],
                'enums': [],
                'defines': [],
                'typedefs': [],
                'file_comment': ''
            }
            
            # 提取文件头注释
            if content.startswith('/**') or content.startswith('/*'):
                comment_match = re.match(r'/\*\*?(.*?)\*/', content, re.DOTALL)
                if comment_match:
                    result['file_comment'] = self._clean_comment(comment_match.group(1))
            
            # 移除注释（避免在注释中误匹配）
            # 先保存doxygen注释的位置映射
            comment_map = self._build_comment_map(content)
            
            # 移除注释后的内容用于正则匹配
            clean_content = self._strip_comments(content)
            
            # 提取函数声明
            for match in self.re_func.finditer(clean_content):
                ret_type = match.group(1).strip()
                func_name = match.group(2).strip()
                params_str = match.group(3).strip()
                
                # 解析参数列表
                params = self._parse_params(params_str)
                
                # 查找最近的doxygen注释
                comment = self._find_nearest_comment(comment_map, match.start())
                
                result['functions'].append({
                    'return_type': ret_type,
                    'name': func_name,
                    'params': params,
                    'params_raw': params_str,
                    'comment': comment
                })
            
            # 提取结构体
            for match in self.re_struct.finditer(clean_content):
                body = match.group(1)
                name = match.group(2).strip()
                
                # 解析结构体成员
                members = self._parse_struct_members(body)
                
                result['structs'].append({
                    'name': name if name else '(匿名)',
                    'members': members
                })
            
            # 提取枚举
            for match in self.re_enum.finditer(clean_content):
                name = match.group(1).strip()
                body = match.group(2)
                
                values = self._parse_enum_values(body)
                
                result['enums'].append({
                    'name': name if name else '(匿名)',
                    'values': values
                })
            
            # 提取宏定义
            for match in self.re_define.finditer(clean_content):
                macro_name = match.group(1)
                macro_value = match.group(2).strip()
                
                # 跳过包含保护宏和过于复杂的宏
                if macro_name.endswith('_H') or macro_name.startswith('_'):
                    continue
                    
                result['defines'].append({
                    'name': macro_name,
                    'value': macro_value
                })
            
            # 提取typedef
            for match in self.re_typedef.finditer(clean_content):
                original = match.group(1).strip()
                alias = match.group(2).strip()
                
                result['typedefs'].append({
                    'original': original,
                    'alias': alias
                })
            
            return result
            
        except Exception as e:
            print(f"  [警告] 解析 {filepath} 失败: {e}", file=sys.stderr)
            return None

    def _strip_comments(self, content):
        """移除C代码中的注释"""
        # 移除多行注释 /* ... */
        result = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
        # 移除单行注释 // ...
        result = re.sub(r'//.*?$', '', result, flags=re.MULTILINE)
        return result

    def _build_comment_map(self, content):
        """构建注释位置映射，用于关联函数与注释"""
        comments = []
        for match in self.re_doxygen.finditer(content):
            comments.append({
                'start': match.start(),
                'end': match.end(),
                'text': self._clean_comment(match.group(1))
            })
        return comments

    def _find_nearest_comment(self, comment_map, position):
        """查找指定位置之前的最近注释（距离<100字符）"""
        nearest = ''
        min_dist = 100
        
        for c in comment_map:
            dist = position - c['end']
            if 0 < dist < min_dist:
                min_dist = dist
                nearest = c['text']
        
        return nearest

    def _clean_comment(self, text):
        """清理注释文本，去除 * 和空白"""
        lines = text.split('\n')
        cleaned = []
        for line in lines:
            # 移除行首的 * 和空白
            line = re.sub(r'^\s*\*?\s?', '', line)
            line = line.strip()
            if line:
                # 去除doxygen标记
                line = re.sub(r'@(param|return|note|brief|file)\s*', '[\\1] ', line)
                cleaned.append(line)
        return ' | '.join(cleaned)

    def _parse_params(self, params_str):
        """解析函数参数列表"""
        if not params_str or params_str.strip() == 'void':
            return []
        
        params = []
        for part in params_str.split(','):
            part = part.strip()
            if not part:
                continue
            
            # 处理 void 回调参数等复杂情况
            if '(' in part:
                params.append(part)
                continue
            
            # 分离类型和名称
            tokens = part.rsplit(None, 1)
            if len(tokens) == 2:
                params.append({
                    'type': tokens[0].strip(),
                    'name': tokens[1].strip().lstrip('*')
                })
            else:
                params.append({'type': part, 'name': ''})
        
        return params

    def _parse_struct_members(self, body):
        """解析结构体成员"""
        members = []
        for line in body.split(';'):
            line = line.strip()
            if not line or line.startswith('//') or line.startswith('/*'):
                continue
            
            # 移除行内注释
            line = re.sub(r'//.*$', '', line).strip()
            
            tokens = line.rsplit(None, 1)
            if len(tokens) == 2:
                members.append({
                    'type': tokens[0].strip(),
                    'name': tokens[1].strip()
                })
        
        return members

    def _parse_enum_values(self, body):
        """解析枚举值"""
        values = []
        for part in body.split(','):
            part = part.strip()
            # 移除注释
            part = re.sub(r'//.*$', '', part).strip()
            part = re.sub(r'/\*.*?\*/', '', part).strip()
            
            if '=' in part:
                name, val = part.split('=', 1)
                values.append({
                    'name': name.strip(),
                    'value': val.strip()
                })
            elif part:
                values.append({'name': part, 'value': ''})
        
        return values


# ============================================================
# Markdown文档生成器
# ============================================================

class MarkdownGenerator:
    """
    Markdown API文档生成器
    
    将解析结果转换为结构清晰的Markdown文档
    """

    def __init__(self, lang='zh'):
        """
        初始化文档生成器
        
        参数:
            lang: 文档语言 ('zh'=中文, 'en'=英文)
        """
        self.lang = lang
        
        # 语言模板
        self.texts = {
            'zh': {
                'title': 'API参考文档',
                'generated': '自动生成于',
                'source': '源文件',
                'functions': '函数',
                'structs': '结构体',
                'enums': '枚举',
                'defines': '宏定义',
                'typedefs': '类型别名',
                'params': '参数',
                'returns': '返回值',
                'name': '名称',
                'type': '类型',
                'value': '值',
                'desc': '说明',
                'no_params': '无参数',
                'returns_void': 'void',
                'members': '成员',
                'file': '文件',
                'index': '文档索引',
                'nav': '导航',
                'func_count': '函数数量',
                'struct_count': '结构体数量',
                'enum_count': '枚举数量',
                'total_files': '总文件数',
            },
            'en': {
                'title': 'API Reference Documentation',
                'generated': 'Auto-generated on',
                'source': 'Source File',
                'functions': 'Functions',
                'structs': 'Structures',
                'enums': 'Enumerations',
                'defines': 'Macros',
                'typedefs': 'Type Aliases',
                'params': 'Parameters',
                'returns': 'Returns',
                'name': 'Name',
                'type': 'Type',
                'value': 'Value',
                'desc': 'Description',
                'no_params': 'No parameters',
                'returns_void': 'void',
                'members': 'Members',
                'file': 'File',
                'index': 'Index',
                'nav': 'Navigation',
                'func_count': 'Function Count',
                'struct_count': 'Struct Count',
                'enum_count': 'Enum Count',
                'total_files': 'Total Files',
            }
        }

    def t(self, key):
        """获取本地化文本"""
        return self.texts.get(self.lang, self.texts['zh']).get(key, key)

    def generate(self, parsed_files):
        """
        生成完整的Markdown文档
        
        参数:
            parsed_files: parse_file()返回的解析结果列表
            
        返回:
            str: Markdown文档内容
        """
        # 过滤掉None结果
        parsed_files = [f for f in parsed_files if f is not None]
        
        if not parsed_files:
            return "# 未找到任何头文件\n"
        
        sections = []
        
        # ---- 文档标题 ----
        sections.append(f"# {self.t('title')}")
        sections.append('')
        sections.append(f"> {self.t('generated')}: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        sections.append(f"> {self.t('total_files')}: {len(parsed_files)}")
        sections.append('')
        
        # ---- 统计摘要 ----
        total_funcs = sum(len(f['functions']) for f in parsed_files)
        total_structs = sum(len(f['structs']) for f in parsed_files)
        total_enums = sum(len(f['enums']) for f in parsed_files)
        
        sections.append('## 概览统计')
        sections.append('')
        sections.append(f'| 指标 | 数量 |')
        sections.append(f'|------|------|')
        sections.append(f'| {self.t("file")} | {len(parsed_files)} |')
        sections.append(f'| {self.t("func_count")} | {total_funcs} |')
        sections.append(f'| {self.t("struct_count")} | {total_structs} |')
        sections.append(f'| {self.t("enum_count")} | {total_enums} |')
        sections.append('')
        
        # ---- 目录 ----
        sections.append('## 目录')
        sections.append('')
        for pf in parsed_files:
            anchor = pf['filename'].replace('.', '').replace(' ', '-').lower()
            sections.append(f"- [{pf['filename']}](#{anchor})")
        sections.append('')
        sections.append('---')
        sections.append('')
        
        # ---- 每个文件的API ----
        for pf in parsed_files:
            sections.append(f"## {pf['filename']}")
            sections.append('')
            sections.append(f"**{self.t('source')}:** `{pf['filepath']}`")
            sections.append('')
            
            if pf['file_comment']:
                sections.append(f"> {pf['file_comment']}")
                sections.append('')
            
            # 函数
            if pf['functions']:
                sections.append(f"### {self.t('functions')}")
                sections.append('')
                
                for func in pf['functions']:
                    sections.append(f"#### `{func['name']}`")
                    sections.append('')
                    
                    # 函数签名
                    if func['params']:
                        params_str = ', '.join(
                            f"{p['type']} {p['name']}" if isinstance(p, dict) else str(p)
                            for p in func['params']
                        )
                    else:
                        params_str = 'void'
                    
                    sections.append(f"```c")
                    sections.append(f"{func['return_type']} {func['name']}({params_str});")
                    sections.append(f"```")
                    sections.append('')
                    
                    # 注释说明
                    if func['comment']:
                        sections.append(f"> {func['comment']}")
                        sections.append('')
                    
                    # 参数表
                    real_params = [p for p in func['params'] if isinstance(p, dict)]
                    if real_params:
                        sections.append(f"| {self.t('type')} | {self.t('name')} |")
                        sections.append(f"|------|------|")
                        for p in real_params:
                            sections.append(f"| `{p['type']}` | `{p['name']}` |")
                        sections.append('')
                    
                    # 返回值
                    if func['return_type'] != 'void':
                        sections.append(f"**{self.t('returns')}:** `{func['return_type']}`")
                        sections.append('')
                    
                    sections.append('---')
                    sections.append('')
            
            # 结构体
            if pf['structs']:
                sections.append(f"### {self.t('structs')}")
                sections.append('')
                
                for struct in pf['structs']:
                    sections.append(f"#### `{struct['name']}`")
                    sections.append('')
                    
                    if struct['members']:
                        sections.append(f"| {self.t('type')} | {self.t('name')} |")
                        sections.append(f"|------|------|")
                        for m in struct['members']:
                            sections.append(f"| `{m['type']}` | `{m['name']}` |")
                        sections.append('')
            
            # 枚举
            if pf['enums']:
                sections.append(f"### {self.t('enums')}")
                sections.append('')
                
                for enum in pf['enums']:
                    sections.append(f"#### `{enum['name']}`")
                    sections.append('')
                    
                    if enum['values']:
                        sections.append(f"| {self.t('name')} | {self.t('value')} |")
                        sections.append(f"|------|------|")
                        for v in enum['values']:
                            sections.append(f"| `{v['name']}` | `{v['value']}` |")
                        sections.append('')
            
            # 宏定义
            if pf['defines']:
                sections.append(f"### {self.t('defines')}")
                sections.append('')
                sections.append(f"| {self.t('name')} | {self.t('value')} |")
                sections.append(f"|------|------|")
                for d in pf['defines']:
                    sections.append(f"| `{d['name']}` | `{d['value']}` |")
                sections.append('')
            
            # typedef
            if pf['typedefs']:
                sections.append(f"### {self.t('typedefs')}")
                sections.append('')
                sections.append(f"| 原始类型 | 别名 |")
                sections.append(f"|------|------|")
                for td in pf['typedefs']:
                    sections.append(f"| `{td['original']}` | `{td['alias']}` |")
                sections.append('')
            
            sections.append('---')
            sections.append('')
        
        return '\n'.join(sections)


# ============================================================
# 主程序入口
# ============================================================

def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(
        description='API文档生成器 - 从C头文件提取函数签名，生成Markdown文档',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                    # 扫描资产库所有.h文件
  %(prog)s --path ./drivers --output api.md   # 扫描指定目录
  %(prog)s --file my_driver.h                 # 扫描单个文件
  %(prog)s --lang en                          # 生成英文文档
  %(prog)s --show-structs --show-enums        # 仅显示结构体和枚举
        """
    )
    
    parser.add_argument('--path', '-p', type=str, default=None,
                        help='扫描目录路径（默认：资产库根目录）')
    parser.add_argument('--file', '-f', type=str, default=None,
                        help='扫描单个.h文件')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出文件路径（默认：输出到终端）')
    parser.add_argument('--lang', '-l', type=str, choices=['zh', 'en'], default='zh',
                        help='文档语言（默认：zh中文）')
    parser.add_argument('--show-functions', action='store_true', default=True,
                        help='显示函数（默认开启）')
    parser.add_argument('--hide-functions', action='store_true',
                        help='隐藏函数')
    parser.add_argument('--show-structs', action='store_true',
                        help='仅显示结构体')
    parser.add_argument('--show-enums', action='store_true',
                        help='仅显示枚举')
    parser.add_argument('--show-defines', action='store_true',
                        help='仅显示宏定义')
    parser.add_argument('--json', action='store_true',
                        help='输出JSON格式（便于程序解析）')
    
    args = parser.parse_args()
    
    # 确定扫描路径
    if args.file:
        # 单文件模式
        h_files = [args.file]
    else:
        # 目录扫描模式
        scan_dir = args.path
        if scan_dir is None:
            # 默认扫描资产库根目录
            scan_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        print(f"正在扫描目录: {scan_dir}")
        
        # 递归查找所有.h文件
        h_files = []
        for root, dirs, files in os.walk(scan_dir):
            # 跳过隐藏目录和工具目录
            dirs[:] = [d for d in dirs if not d.startswith('.') and d != 'tools']
            for f in files:
                if f.endswith('.h'):
                    h_files.append(os.path.join(root, f))
        
        h_files.sort()
    
    if not h_files:
        print("未找到任何.h文件")
        return
    
    print(f"找到 {len(h_files)} 个头文件")
    
    # 解析所有文件
    parser_obj = CHeaderParser()
    parsed_files = []
    
    for h_file in h_files:
        print(f"  解析: {os.path.basename(h_file)}")
        result = parser_obj.parse_file(h_file)
        if result:
            parsed_files.append(result)
    
    print(f"\n成功解析 {len(parsed_files)} 个文件")
    
    # JSON输出模式
    if args.json:
        import json
        output = json.dumps(parsed_files, ensure_ascii=False, indent=2)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(output)
            print(f"JSON已保存至: {args.output}")
        else:
            print(output)
        return
    
    # 过滤显示内容
    if args.hide_functions:
        for pf in parsed_files:
            pf['functions'] = []
    if args.show_structs:
        for pf in parsed_files:
            pf['functions'] = []
            pf['defines'] = []
            pf['typedefs'] = []
    if args.show_enums:
        for pf in parsed_files:
            pf['functions'] = []
            pf['structs'] = []
            pf['defines'] = []
            pf['typedefs'] = []
    if args.show_defines:
        for pf in parsed_files:
            pf['functions'] = []
            pf['structs'] = []
            pf['enums'] = []
            pf['typedefs'] = []
    
    # 生成Markdown文档
    generator = MarkdownGenerator(lang=args.lang)
    markdown = generator.generate(parsed_files)
    
    # 输出
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(markdown)
        print(f"文档已保存至: {args.output}")
    else:
        print('\n' + markdown)


if __name__ == '__main__':
    main()
