#!/usr/bin/env python3
"""
Git Pre-commit 钩子 - 提交前自动运行代码审计检查
用法:
  1. 直接运行: python git_pre_commit_hook.py [--repo <仓库路径>]
  2. 安装为Git钩子: python git_pre_commit_hook.py --install [--repo <仓库路径>]
  3. 卸载钩子: python git_pre_commit_hook.py --uninstall [--repo <仓库路径>]
"""
import argparse
import subprocess
import sys
import os
import shutil
from pathlib import Path
from datetime import datetime

# 钩子脚本内容（会被写入.git/hooks/pre-commit）
HOOK_SCRIPT = '''#!/usr/bin/env python3
"""Git Pre-commit 钩子 - 自动生成"""
import subprocess
import sys
import os

def main():
    # 获取仓库根目录
    repo_root = subprocess.check_output(
        ["git", "rev-parse", "--show-toplevel"], text=True
    ).strip()

    tools_dir = os.path.join(repo_root, "tools")
    checks_failed = False

    # 获取暂存的Python文件
    result = subprocess.run(
        ["git", "diff", "--cached", "--name-only", "--diff-filter=ACM"],
        capture_output=True, text=True
    )
    py_files = [f for f in result.stdout.strip().split("\\n") if f.endswith(".py")]

    if not py_files:
        print("✅ 没有Python文件变更，跳过检查")
        sys.exit(0)

    print(f"\\n🔍 Pre-commit 检查 {len(py_files)} 个Python文件...\\n")

    # 1. 语法检查
    print("=" * 40)
    print("📋 [1/3] Python语法检查")
    print("=" * 40)
    for f in py_files:
        full_path = os.path.join(repo_root, f)
        if not os.path.exists(full_path):
            continue
        result = subprocess.run(
            [sys.executable, "-c", f"import py_compile; py_compile.compile(r\\"{full_path}\\", doraise=True)"],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"  ❌ 语法错误: {f}")
            print(f"     {result.stderr.strip()}")
            checks_failed = True
        else:
            print(f"  ✓ {f}")

    # 2. 代码复杂度检查
    complexity_script = os.path.join(tools_dir, "code_complexity.py")
    if os.path.exists(complexity_script):
        print(f"\\n{'=' * 40}")
        print("📋 [2/3] 代码复杂度检查")
        print("=" * 40)
        result = subprocess.run(
            [sys.executable, complexity_script] + py_files,
            capture_output=True, text=True, cwd=repo_root
        )
        if result.stdout:
            print(result.stdout)
        if result.returncode != 0:
            checks_failed = True

    # 3. 依赖检查
    dep_script = os.path.join(tools_dir, "dependency_checker.py")
    if os.path.exists(dep_script):
        print(f"\\n{'=' * 40}")
        print("📋 [3/3] 依赖检查")
        print("=" * 40)
        result = subprocess.run(
            [sys.executable, dep_script, repo_root],
            capture_output=True, text=True
        )
        if result.stdout:
            # 只显示缺失依赖部分
            for line in result.stdout.split("\\n"):
                if "缺失" in line or "✗" in line or "✅ 所有依赖" in line:
                    print(line)

    # 结果
    print(f"\\n{'=' * 40}")
    if checks_failed:
        print("❌ Pre-commit 检查失败！请修复后重新提交。")
        print("   使用 git commit --no-verify 可跳过检查（不推荐）")
        sys.exit(1)
    else:
        print("✅ 所有Pre-commit检查通过！")
        sys.exit(0)

if __name__ == "__main__":
    main()
'''


def install_hook(repo_path: str):
    """安装pre-commit钩子"""
    hooks_dir = Path(repo_path) / ".git" / "hooks"
    if not hooks_dir.exists():
        print(f"❌ 未找到Git hooks目录: {hooks_dir}")
        print("   请确认这是一个Git仓库")
        return False

    hook_path = hooks_dir / "pre-commit"
    if hook_path.exists():
        backup = hook_path.with_suffix(".bak")
        shutil.copy2(hook_path, backup)
        print(f"⚠️  已备份现有钩子到: {backup}")

    hook_path.write_text(HOOK_SCRIPT, encoding="utf-8")
    # 尝试设置可执行权限（Windows上可能不生效但不影响）
    try:
        os.chmod(hook_path, 0o755)
    except Exception:
        pass

    print(f"✅ Pre-commit钩子已安装: {hook_path}")
    return True


def uninstall_hook(repo_path: str):
    """卸载pre-commit钩子"""
    hook_path = Path(repo_path) / ".git" / "hooks" / "pre-commit"
    if hook_path.exists():
        hook_path.unlink()
        print(f"✅ Pre-commit钩子已移除")
        # 恢复备份
        backup = hook_path.with_suffix(".bak")
        if backup.exists():
            shutil.move(backup, hook_path)
            print(f"✅ 已恢复备份钩子")
    else:
        print("⚠️  未找到pre-commit钩子")


def run_checks(repo_path: str):
    """手动运行pre-commit检查"""
    tools_dir = Path(repo_path) / "tools"

    # 获取已修改/新增的Python文件
    try:
        result = subprocess.run(
            ["git", "diff", "--cached", "--name-only", "--diff-filter=ACM"],
            capture_output=True, text=True, cwd=repo_path
        )
        py_files = [f for f in result.stdout.strip().split("\n") if f.endswith(".py")]
    except Exception:
        # 如果不在git环境，扫描所有.py文件
        py_files = [str(f) for f in Path(repo_path).rglob("*.py") if ".git" not in str(f)]

    if not py_files:
        print("✅ 没有需要检查的Python文件")
        return True

    print(f"🔍 检查 {len(py_files)} 个Python文件...\n")
    all_passed = True

    # 语法检查
    print("📋 [1/2] Python语法检查")
    print("-" * 40)
    for f in py_files:
        full_path = os.path.join(repo_path, f)
        if not os.path.exists(full_path):
            continue
        result = subprocess.run(
            [sys.executable, "-m", "py_compile", full_path],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"  ❌ {f}: {result.stderr.strip()}")
            all_passed = False
        else:
            print(f"  ✓ {f}")

    # 复杂度检查
    complexity_script = tools_dir / "code_complexity.py"
    if complexity_script.exists():
        print(f"\n📋 [2/2] 代码复杂度检查")
        print("-" * 40)
        result = subprocess.run(
            [sys.executable, str(complexity_script), repo_path, "--threshold", "15"],
            capture_output=True, text=True
        )
        if result.stdout:
            for line in result.stdout.split("\n"):
                if "D(差)" in line or "⚠️" in line or "✅" in line or "统计" in line:
                    print(f"  {line}")

    print(f"\n{'=' * 40}")
    if all_passed:
        print("✅ 所有检查通过!")
    else:
        print("❌ 存在问题，请修复后再提交")
    return all_passed


def main():
    parser = argparse.ArgumentParser(description="Git Pre-commit钩子管理器")
    parser.add_argument("--repo", "-r", default=".", help="Git仓库路径")
    parser.add_argument("--install", "-i", action="store_true", help="安装pre-commit钩子")
    parser.add_argument("--uninstall", "-u", action="store_true", help="卸载pre-commit钩子")
    parser.add_argument("--check", "-c", action="store_true", help="手动运行检查（不提交）")
    args = parser.parse_args()

    repo_path = os.path.abspath(args.repo)

    if args.install:
        install_hook(repo_path)
    elif args.uninstall:
        uninstall_hook(repo_path)
    else:
        # 默认：手动运行检查
        success = run_checks(repo_path)
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
