#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
比赛计时器 - 4天3夜时间管理 + 任务提醒 + 进度跟踪
============================================================
功能：
  - 4天3夜比赛全程时间管理
  - 分阶段倒计时与提醒（方案设计、硬件搭建、软件调试、测试报告）
  - 任务清单管理（添加、完成、优先级）
  - 进度百分比跟踪
  - 里程碑提醒（关键节点通知）
  - 实时状态面板（终端 UI）
  - 历史记录保存与恢复

用法：
  python competition_timer.py start --problem "2025年F题"
  python competition_timer.py status
  python competition_timer.py task add "完成运放电路搭建" --priority high
  python competition_timer.py task done 1
  python competition_timer.py task list
  python competition_timer.py history
============================================================
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

# ── 状态文件路径 ──────────────────────────────────────────
STATE_DIR = os.path.join(os.path.expanduser('~'), '.competition_timer')
STATE_FILE = os.path.join(STATE_DIR, 'state.json')

# ── 4天3夜时间线配置 ──────────────────────────────────────
# 比赛通常从周三 8:00 开始，到周日 20:00 结束
# 共 108 小时 (4.5天)
PHASES = [
    {
        "name": "📋 题目分析与方案设计",
        "start_pct": 0,
        "end_pct": 10,
        "emoji": "📋",
        "color": "\033[96m",  # 青色
        "tips": [
            "仔细审题，圈出关键指标",
            "列出所有指标要求，标注分值",
            "头脑风暴，至少准备2个方案",
            "确定主控芯片和关键模块",
            "绘制系统框图",
        ],
    },
    {
        "name": "🔧 硬件搭建与焊接",
        "start_pct": 10,
        "end_pct": 35,
        "emoji": "🔧",
        "color": "\033[93m",  # 黄色
        "tips": [
            "先搭建最小系统，确保主控运行",
            "逐模块焊接，每焊一个测试一个",
            "注意电源去耦和信号完整性",
            "准备好调试接口（SWD/串口）",
            "保留面包板备用方案",
        ],
    },
    {
        "name": "💻 软件开发与调试",
        "start_pct": 35,
        "end_pct": 70,
        "emoji": "💻",
        "color": "\033[92m",  # 绿色
        "tips": [
            "先写驱动层，确保各外设工作",
            "模块化开发，逐个功能调试",
            "使用串口打印调试信息",
            "注意中断优先级和实时性",
            "定时保存代码（Git）",
        ],
    },
    {
        "name": "⚙️ 系统联调与优化",
        "start_pct": 70,
        "end_pct": 85,
        "emoji": "⚙️",
        "color": "\033[95m",  # 紫色
        "tips": [
            "各模块联合调试",
            "优化关键性能指标",
            "测试边界条件和异常情况",
            "调整参数达到最优性能",
            "记录所有测试数据",
        ],
    },
    {
        "name": "📝 测试与报告撰写",
        "start_pct": 85,
        "end_pct": 100,
        "emoji": "📝",
        "color": "\033[91m",  # 红色
        "tips": [
            "逐项测试，记录所有指标数据",
            "拍照/录像保留测试过程",
            "撰写设计报告（方案论证、电路设计、程序设计、测试结果）",
            "检查报告格式和排版",
            "提前打包提交材料",
        ],
    },
]

# ── 关键里程碑 ────────────────────────────────────────────
MILESTONES = [
    {"hour": 0,   "msg": "🏁 比赛开始！仔细审题！"},
    {"hour": 2,   "msg": "📋 应该确定题目和方案了"},
    {"hour": 6,   "msg": "🔧 硬件搭建应开始"},
    {"hour": 12,  "msg": "🌙 第一个夜晚，注意休息"},
    {"hour": 24,  "msg": "📅 第1天结束，最小系统应跑通"},
    {"hour": 36,  "msg": "💻 核心功能应基本实现"},
    {"hour": 48,  "msg": "📅 第2天结束，开始系统联调"},
    {"hour": 60,  "msg": "⚙️ 基础功能应全部完成"},
    {"hour": 72,  "msg": "📅 第3天结束，准备测试"},
    {"hour": 84,  "msg": "📝 开始撰写报告"},
    {"hour": 96,  "msg": "📅 第4天，最后冲刺！"},
    {"hour": 100, "msg": "⚠️ 距离结束不到8小时！"},
    {"hour": 104, "msg": "🚨 最后4小时！打包提交材料"},
    {"hour": 107, "msg": "⏰ 最后1小时！确认提交！"},
    {"hour": 108, "msg": "🏁 比赛结束！辛苦了！"},
]


def ensure_state_dir():
    """确保状态目录存在。"""
    os.makedirs(STATE_DIR, exist_ok=True)


def load_state() -> dict:
    """加载状态文件。"""
    if os.path.isfile(STATE_FILE):
        with open(STATE_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    return {}


def save_state(state: dict):
    """保存状态文件。"""
    ensure_state_dir()
    with open(STATE_FILE, 'w', encoding='utf-8') as f:
        json.dump(state, f, ensure_ascii=False, indent=2)


def start_competition(problem: str, hours: int = 108):
    """开始比赛计时。"""
    now = datetime.now()
    state = {
        "problem": problem,
        "start_time": now.isoformat(),
        "total_hours": hours,
        "tasks": [],
        "milestones_hit": [],
        "created_at": now.isoformat(),
    }
    save_state(state)
    print(f"\n{'='*60}")
    print(f"  🏁 比赛计时开始！")
    print(f"  赛题: {problem}")
    print(f"  开始时间: {now.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  总时长: {hours} 小时 ({hours/24:.1f} 天)")
    end_time = now + timedelta(hours=hours)
    print(f"  结束时间: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}\n")


def get_elapsed_hours(state: dict) -> float:
    """计算已经过去的时间（小时）。"""
    start = datetime.fromisoformat(state['start_time'])
    elapsed = datetime.now() - start
    return elapsed.total_seconds() / 3600


def get_current_phase(elapsed_pct: float) -> dict:
    """根据时间进度百分比获取当前阶段。"""
    for phase in PHASES:
        if phase['start_pct'] <= elapsed_pct < phase['end_pct']:
            return phase
    return PHASES[-1]


def format_duration(seconds: float) -> str:
    """格式化时间长度。"""
    if seconds < 0:
        return "已结束"
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    if hours > 24:
        days = hours // 24
        hours = hours % 24
        return f"{days}天{hours:02d}时{minutes:02d}分{secs:02d}秒"
    return f"{hours:02d}时{minutes:02d}分{secs:02d}秒"


def show_status(detailed: bool = False):
    """显示比赛状态面板。"""
    state = load_state()
    if not state or 'start_time' not in state:
        print("[WARN] 比赛尚未开始，请运行: python competition_timer.py start --problem \"赛题\"")
        return

    start = datetime.fromisoformat(state['start_time'])
    total_hours = state.get('total_hours', 108)
    total_seconds = total_hours * 3600

    elapsed = (datetime.now() - start).total_seconds()
    remaining = total_seconds - elapsed
    elapsed_pct = min(100, (elapsed / total_seconds) * 100)

    phase = get_current_phase(elapsed_pct)

    # 任务统计
    tasks = state.get('tasks', [])
    done_count = sum(1 for t in tasks if t.get('done'))
    total_tasks = len(tasks)
    task_pct = (done_count / total_tasks * 100) if total_tasks else 0

    # 检查里程碑
    elapsed_hours = elapsed / 3600
    check_milestones(state, elapsed_hours)

    # 进度条
    bar_width = 40
    filled = int(bar_width * elapsed_pct / 100)
    bar = '█' * filled + '░' * (bar_width - filled)

    # 清屏效果
    print("\n" + "=" * 60)
    print(f"  🏆 电赛计时器 - {state.get('problem', '未知赛题')}")
    print(f"  开始: {start.strftime('%m-%d %H:%M')}")
    print(f"  当前: {datetime.now().strftime('%m-%d %H:%M:%S')}")
    print("=" * 60)

    print(f"\n  时间进度: [{bar}] {elapsed_pct:.1f}%")
    print(f"  已用时: {format_duration(elapsed)}")
    if remaining > 0:
        print(f"  剩余:   {format_duration(remaining)}")
        end_time = start + timedelta(hours=total_hours)
        print(f"  截止:   {end_time.strftime('%m-%d %H:%M')}")
    else:
        print(f"  ⏰ 比赛已结束!")

    # 当前阶段
    color = phase.get('color', '')
    reset = '\033[0m'
    print(f"\n  {color}当前阶段: {phase['name']}{reset}")
    print(f"  阶段进度: {phase['start_pct']}% ~ {phase['end_pct']}%")

    if detailed and phase.get('tips'):
        print(f"\n  💡 当前阶段建议:")
        for tip in phase['tips']:
            print(f"     • {tip}")

    # 阶段时间线
    print(f"\n  阶段时间线:")
    for p in PHASES:
        marker = " ◀── 当前" if p == phase else ""
        p_start_h = total_hours * p['start_pct'] / 100
        p_end_h = total_hours * p['end_pct'] / 100
        pct_bar = '▓' * int(5 * (p['end_pct'] - p['start_pct']) / 100)
        print(f"    {p['emoji']} {p['name']:<25} [{pct_bar:<6}] {p_start_h:.0f}h-{p_end_h:.0f}h{marker}")

    # 任务统计
    print(f"\n  任务进度: {done_count}/{total_tasks} 完成 ({task_pct:.0f}%)")
    if total_tasks > 0:
        task_bar_filled = int(bar_width * task_pct / 100)
        task_bar = '█' * task_bar_filled + '░' * (bar_width - task_bar_filled)
        print(f"  [{task_bar}] {task_pct:.1f}%")

    # 未完成的高优先级任务
    pending_high = [t for t in tasks if not t.get('done') and t.get('priority') == 'high']
    if pending_high:
        print(f"\n  🔴 高优先级待办:")
        for t in pending_high[:5]:
            print(f"     □ {t['text']}")

    print("\n" + "=" * 60)


def check_milestones(state: dict, elapsed_hours: float):
    """检查并提醒里程碑。"""
    hit = set(state.get('milestones_hit', []))
    new_hits = []

    for ms in MILESTONES:
        ms_key = str(ms['hour'])
        if elapsed_hours >= ms['hour'] and ms_key not in hit:
            print(f"\n  🔔 {ms['msg']}")
            new_hits.append(ms_key)

    if new_hits:
        hit.update(new_hits)
        state['milestones_hit'] = list(hit)
        save_state(state)


def add_task(text: str, priority: str = 'normal'):
    """添加新任务。"""
    state = load_state()
    if 'tasks' not in state:
        state['tasks'] = []

    task = {
        "id": len(state['tasks']) + 1,
        "text": text,
        "priority": priority,
        "done": False,
        "created_at": datetime.now().isoformat(),
        "done_at": None,
    }
    state['tasks'].append(task)
    save_state(state)

    priority_emoji = {'high': '🔴', 'normal': '🟡', 'low': '🟢'}.get(priority, '⚪')
    print(f"  ✅ 任务已添加: {priority_emoji} #{task['id']} {text}")


def done_task(task_id: int):
    """标记任务完成。"""
    state = load_state()
    tasks = state.get('tasks', [])

    for t in tasks:
        if t['id'] == task_id:
            t['done'] = True
            t['done_at'] = datetime.now().isoformat()
            save_state(state)
            print(f"  🎉 任务完成: #{t['id']} {t['text']}")
            return

    print(f"  [ERROR] 未找到任务 #{task_id}")


def list_tasks(show_all: bool = False):
    """列出任务。"""
    state = load_state()
    tasks = state.get('tasks', [])

    if not tasks:
        print("  📋 暂无任务。使用 'task add' 添加任务")
        return

    print(f"\n  {'ID':>3} {'状态':<4} {'优先级':<6} {'任务内容':<40} {'创建时间':<16}")
    print("  " + "-" * 75)

    for t in tasks:
        if not show_all and t.get('done'):
            continue

        status = "✅" if t.get('done') else "⬜"
        priority = {'high': '🔴 高', 'normal': '🟡 中', 'low': '🟢 低'}.get(t.get('priority', 'normal'), '⚪')
        created = t.get('created_at', '')[:16]

        print(f"  {t['id']:>3} {status:<4} {priority:<6} {t['text']:<40} {created:<16}")

    done_count = sum(1 for t in tasks if t.get('done'))
    print(f"\n  共 {len(tasks)} 项任务, 完成 {done_count} 项, 待办 {len(tasks) - done_count} 项")


def delete_task(task_id: int):
    """删除任务。"""
    state = load_state()
    tasks = state.get('tasks', [])
    original_len = len(tasks)
    state['tasks'] = [t for t in tasks if t['id'] != task_id]
    if len(state['tasks']) < original_len:
        save_state(state)
        print(f"  🗑️ 任务 #{task_id} 已删除")
    else:
        print(f"  [ERROR] 未找到任务 #{task_id}")


def show_history():
    """显示历史比赛记录。"""
    if not os.path.isdir(STATE_DIR):
        print("  暂无历史记录")
        return

    # 查找所有历史文件
    history_files = []
    for f in os.listdir(STATE_DIR):
        if f.startswith('state_') and f.endswith('.json'):
            history_files.append(os.path.join(STATE_DIR, f))

    if not history_files:
        # 显示当前状态
        state = load_state()
        if state and 'start_time' in state:
            print(f"\n  当前比赛: {state.get('problem', '未知')}")
            print(f"  开始时间: {state['start_time']}")
            tasks = state.get('tasks', [])
            done = sum(1 for t in tasks if t.get('done'))
            print(f"  任务: {done}/{len(tasks)} 完成")
        else:
            print("  暂无历史记录")
        return


def reset_timer():
    """重置计时器（归档当前状态）。"""
    state = load_state()
    if state and 'start_time' in state:
        # 归档
        ensure_state_dir()
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        archive = os.path.join(STATE_DIR, f'state_{ts}.json')
        with open(archive, 'w', encoding='utf-8') as f:
            json.dump(state, f, ensure_ascii=False, indent=2)
        print(f"  📁 当前状态已归档: {archive}")

    # 清除
    if os.path.isfile(STATE_FILE):
        os.remove(STATE_FILE)
    print("  🔄 计时器已重置")


def live_mode(interval: int = 60):
    """实时监控模式，定期刷新状态。"""
    print("  实时监控模式 (按 Ctrl+C 退出)")
    try:
        while True:
            # 简单清屏
            print("\n" * 2)
            show_status(detailed=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n  退出实时监控")


def main():
    parser = argparse.ArgumentParser(
        description='比赛计时器 - 4天3夜时间管理',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python competition_timer.py start --problem "2025年F题"
  python competition_timer.py status
  python competition_timer.py status --detailed
  python competition_timer.py task add "完成运放电路搭建" --priority high
  python competition_timer.py task done 1
  python competition_timer.py task list
  python competition_timer.py task list --all
  python competition_timer.py task delete 1
  python competition_timer.py live --interval 30
  python competition_timer.py reset
        """
    )
    subparsers = parser.add_subparsers(dest='command', help='可用命令')

    # start
    start_parser = subparsers.add_parser('start', help='开始比赛计时')
    start_parser.add_argument('--problem', '-p', required=True, help='赛题名称/编号')
    start_parser.add_argument('--hours', type=int, default=108, help='比赛总时长(小时, 默认108)')

    # status
    status_parser = subparsers.add_parser('status', help='查看比赛状态')
    status_parser.add_argument('--detailed', '-d', action='store_true', help='显示详细信息和建议')

    # task
    task_parser = subparsers.add_parser('task', help='任务管理')
    task_subparsers = task_parser.add_subparsers(dest='task_cmd', help='任务操作')

    # task add
    task_add = task_subparsers.add_parser('add', help='添加任务')
    task_add.add_argument('text', help='任务内容')
    task_add.add_argument('--priority', choices=['high', 'normal', 'low'], default='normal', help='优先级')

    # task done
    task_done = task_subparsers.add_parser('done', help='完成任务')
    task_done.add_argument('id', type=int, help='任务ID')

    # task list
    task_list = task_subparsers.add_parser('list', help='列出任务')
    task_list.add_argument('--all', '-a', action='store_true', help='显示所有任务（含已完成）')

    # task delete
    task_del = task_subparsers.add_parser('delete', help='删除任务')
    task_del.add_argument('id', type=int, help='任务ID')

    # live
    live_parser = subparsers.add_parser('live', help='实时监控模式')
    live_parser.add_argument('--interval', type=int, default=60, help='刷新间隔(秒, 默认60)')

    # history
    subparsers.add_parser('history', help='查看历史记录')

    # reset
    subparsers.add_parser('reset', help='重置计时器')

    args = parser.parse_args()

    if not args.command:
        # 默认显示状态
        show_status()
        return

    if args.command == 'start':
        start_competition(args.problem, args.hours)
    elif args.command == 'status':
        show_status(detailed=args.detailed)
    elif args.command == 'task':
        if args.task_cmd == 'add':
            add_task(args.text, args.priority)
        elif args.task_cmd == 'done':
            done_task(args.id)
        elif args.task_cmd == 'list':
            list_tasks(show_all=args.all)
        elif args.task_cmd == 'delete':
            delete_task(args.id)
        else:
            task_parser.print_help()
    elif args.command == 'live':
        live_mode(args.interval)
    elif args.command == 'history':
        show_history()
    elif args.command == 'reset':
        reset_timer()


if __name__ == '__main__':
    main()
