#!/usr/bin/env python3
"""
依赖检查器 - 扫描所有Python文件的import依赖，检查是否已安装
用法: python dependency_checker.py [目录路径] [--fix] [--requirements requirements.txt]
"""
import argparse
import ast
import sys
import os
import subprocess
from pathlib import Path
from collections import defaultdict

# 标准库模块列表（Python 3.11常用）
STDLIB_MODULES = {
    "abc", "aifc", "argparse", "array", "ast", "asynchat", "asyncio", "asyncore",
    "atexit", "base64", "bdb", "binascii", "binhex", "bisect", "builtins",
    "calendar", "cgi", "cgitb", "chunk", "cmath", "cmd", "code", "codecs",
    "codeop", "collections", "colorsys", "compileall", "concurrent", "configparser",
    "contextlib", "contextvars", "copy", "copyreg", "cProfile", "crypt", "csv",
    "ctypes", "curses", "dataclasses", "datetime", "dbm", "decimal", "difflib",
    "dis", "distutils", "doctest", "email", "encodings", "enum", "errno",
    "faulthandler", "fcntl", "filecmp", "fileinput", "fnmatch", "fractions",
    "ftplib", "functools", "gc", "getopt", "getpass", "gettext", "glob", "graphlib",
    "grp", "gzip", "hashlib", "heapq", "hmac", "html", "http", "idlelib",
    "imaplib", "imghdr", "imp", "importlib", "inspect", "io", "ipaddress",
    "itertools", "json", "keyword", "lib2to3", "linecache", "locale", "logging",
    "lzma", "mailbox", "mailcap", "marshal", "math", "mimetypes", "mmap",
    "modulefinder", "multiprocessing", "netrc", "nis", "nntplib", "numbers",
    "operator", "optparse", "os", "ossaudiodev", "pathlib", "pdb", "pickle",
    "pickletools", "pipes", "pkgutil", "platform", "plistlib", "poplib", "posix",
    "posixpath", "pprint", "profile", "pstats", "pty", "pwd", "py_compile",
    "pyclbr", "pydoc", "queue", "quopri", "random", "re", "readline", "reprlib",
    "resource", "rlcompleter", "runpy", "sched", "secrets", "select", "selectors",
    "shelve", "shlex", "shutil", "signal", "site", "smtpd", "smtplib", "sndhdr",
    "socket", "socketserver", "sqlite3", "ssl", "stat", "statistics", "string",
    "stringprep", "struct", "subprocess", "sunau", "symtable", "sys", "sysconfig",
    "syslog", "tabnanny", "tarfile", "telnetlib", "tempfile", "termios", "test",
    "textwrap", "threading", "time", "timeit", "tkinter", "token", "tokenize",
    "tomllib", "trace", "traceback", "tracemalloc", "tty", "turtle", "turtledemo",
    "types", "typing", "unicodedata", "unittest", "urllib", "uu", "uuid",
    "venv", "warnings", "wave", "weakref", "webbrowser", "winreg", "winsound",
    "wsgiref", "xdrlib", "xml", "xmlrpc", "zipapp", "zipfile", "zipimport",
    "zlib", "_thread",
}


def extract_imports(filepath: str) -> list:
    """从Python文件中提取所有import语句"""
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            tree = ast.parse(f.read())
    except (SyntaxError, UnicodeDecodeError):
        return []

    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.append(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module and node.level == 0:  # 只检查绝对导入
                imports.append(node.module.split(".")[0])
    return imports


def check_package_installed(pkg_name: str) -> bool:
    """检查包是否已安装"""
    try:
        result = subprocess.run(
            [sys.executable, "-c", f"import {pkg_name}"],
            capture_output=True, timeout=10
        )
        return result.returncode == 0
    except Exception:
        return False


def scan_directory(directory: str) -> dict:
    """扫描目录中所有Python文件的import"""
    all_imports = defaultdict(set)  # pkg -> set of files
    py_files = list(Path(directory).rglob("*.py"))

    for py_file in py_files:
        imports = extract_imports(str(py_file))
        for imp in imports:
            all_imports[imp].add(str(py_file))

    return all_imports, len(py_files)


def main():
    parser = argparse.ArgumentParser(description="Python依赖检查器")
    parser.add_argument("directory", nargs="?", default=".", help="扫描目录 (默认当前目录)")
    parser.add_argument("--fix", "-f", action="store_true", help="自动安装缺失依赖")
    parser.add_argument("--requirements", "-r", help="对比requirements.txt")
    parser.add_argument("--verbose", "-v", action="store_true", help="显示详细信息")
    args = parser.parse_args()

    directory = os.path.abspath(args.directory)
    print(f"🔍 扫描目录: {directory}")

    imports, file_count = scan_directory(directory)
    print(f"📂 扫描文件: {file_count}个Python文件\n")

    # 分类: 标准库 / 已安装第三方 / 缺失
    stdlib = []
    installed = []
    missing = []

    for pkg in sorted(imports.keys()):
        if pkg in STDLIB_MODULES:
            stdlib.append(pkg)
        elif check_package_installed(pkg):
            installed.append(pkg)
        else:
            missing.append(pkg)

    # 输出结果
    if args.verbose and stdlib:
        print("📦 标准库模块:")
        for m in stdlib:
            print(f"   ✓ {m} (使用于 {len(imports[m])} 个文件)")
        print()

    if installed:
        print("✅ 已安装的第三方包:")
        for m in installed:
            print(f"   ✓ {m} (使用于 {len(imports[m])} 个文件)")
        print()

    if missing:
        print("❌ 缺失的依赖:")
        for m in missing:
            files = ", ".join(sorted(imports[m]))
            print(f"   ✗ {m} (需要于: {files})")

        if args.fix:
            print("\n🔧 尝试安装缺失依赖...")
            for m in missing:
                print(f"   pip install {m} ...")
                result = subprocess.run([sys.executable, "-m", "pip", "install", m], capture_output=True)
                if result.returncode == 0:
                    print(f"   ✓ {m} 安装成功")
                else:
                    print(f"   ✗ {m} 安装失败")
    else:
        print("✅ 所有依赖已满足!")

    # 对比requirements.txt
    if args.requirements and os.path.exists(args.requirements):
        with open(args.requirements, "r") as f:
            req_pkgs = {line.strip().split(">=")[0].split("==")[0].split("<")[0]
                       for line in f if line.strip() and not line.startswith("#")}

        actual_pkgs = {p for p in imports if p not in STDLIB_MODULES}
        extra = actual_pkgs - req_pkgs
        unused = req_pkgs - actual_pkgs

        if extra:
            print(f"\n⚠️  未在requirements.txt中声明: {', '.join(sorted(extra))}")
        if unused:
            print(f"\n⚠️  requirements.txt中未使用的: {', '.join(sorted(unused))}")

    # 汇总
    print(f"\n📊 汇总: 标准库 {len(stdlib)} | 已安装 {len(installed)} | 缺失 {len(missing)}")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
