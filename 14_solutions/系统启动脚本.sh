#!/bin/bash
# ============================================================
# Orange Pi 5 电赛系统启动脚本
# ============================================================
# 功能：系统初始化 + 进程启动 + 看门狗监控
# 用法：sudo ./系统启动脚本.sh [start|stop|restart|status]
# ============================================================

set -e

# === 配置区 ===
PROJECT_DIR="/home/orangepi/competition"
LOG_DIR="$PROJECT_DIR/logs"
PID_DIR="$PROJECT_DIR/pids"
VENV_DIR="$PROJECT_DIR/venv"

# 进程配置
VISION_SCRIPT="$PROJECT_DIR/vision_main.py"
CONTROL_SCRIPT="$PROJECT_DIR/control_main.py"
HMI_SCRIPT="$PROJECT_DIR/hmi_main.py"

# CPU绑定
VISION_CPU=6       # A76大核
CONTROL_CPU=7      # A76大核 (隔离核)
HMI_CPU="0-3"      # A55小核

# 优先级
VISION_RT_PRIO=50
CONTROL_RT_PRIO=90

# 日志
LOG_FILE="$LOG_DIR/system_$(date +%Y%m%d_%H%M%S).log"

# === 函数定义 ===

log() {
    echo "[$(date '+%H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

check_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "错误: 需要root权限运行此脚本"
        echo "用法: sudo $0 $*"
        exit 1
    fi
}

init_dirs() {
    mkdir -p "$LOG_DIR" "$PID_DIR"
    log "目录初始化完成"
}

# --- 系统优化 ---
system_optimize() {
    log "=== 系统实时性优化 ==="

    # 1. 关闭透明大页
    echo never > /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null || true
    echo never > /sys/kernel/mm/transparent_hugepage/defrag 2>/dev/null || true
    log "透明大页已关闭"

    # 2. GPU/NPU频率锁定
    if [ -f /sys/class/devfreq/fb000000.gpu/governor ]; then
        echo performance > /sys/class/devfreq/fb000000.gpu/governor
        log "GPU频率已锁定performance"
    fi
    if [ -f /sys/class/devfreq/2c0000000.npu/governor ]; then
        echo performance > /sys/class/devfreq/2c0000000.npu/governor
        log "NPU频率已锁定performance"
    fi

    # 3. 关闭不必要的服务
    local services=(
        bluetooth.service
        cups.service
        avahi-daemon.service
        ModemManager.service
    )
    for svc in "${services[@]}"; do
        systemctl stop "$svc" 2>/dev/null || true
        systemctl disable "$svc" 2>/dev/null || true
    done
    log "无用服务已关闭"

    # 4. 设置swappiness
    sysctl -w vm.swappiness=10 >/dev/null 2>&1 || true
    log "系统优化完成"
}

# --- 网络配置 (如果需要) ---
network_setup() {
    # 配置静态IP用于串口/网络通信
    log "网络配置..."
    # 按需修改
    # ip addr add 192.168.1.100/24 dev eth0 2>/dev/null || true
    log "网络配置完成"
}

# --- 摄像头初始化 ---
camera_init() {
    log "摄像头初始化..."

    # 检查USB摄像头
    if ls /dev/video* 1>/dev/null 2>&1; then
        log "摄像头设备: $(ls /dev/video*)"
    else
        log "警告: 未检测到摄像头设备!"
    fi

    # 设置摄像头权限
    chmod 666 /dev/video* 2>/dev/null || true

    # 预热摄像头驱动
    python3 -c "
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    print(f'摄像头OK: {frame.shape}')
    cap.release()
else:
    print('警告: 摄像头打开失败!')
" 2>/dev/null || log "摄像头预热脚本执行失败"

    log "摄像头初始化完成"
}

# --- 启动视觉进程 ---
start_vision() {
    log "启动视觉进程 (CPU=$VISION_CPU, RT=$VISION_RT_PRIO)..."

    taskset -c "$VISION_CPU" \
        chrt -f "$VISION_RT_PRIO" \
        python3 "$VISION_SCRIPT" \
        >> "$LOG_DIR/vision.log" 2>&1 &
    echo $! > "$PID_DIR/vision.pid"
    log "视觉进程 PID: $(cat "$PID_DIR/vision.pid")"
}

# --- 启动控制进程 ---
start_control() {
    log "启动控制进程 (CPU=$CONTROL_CPU, RT=$CONTROL_RT_PRIO)..."

    taskset -c "$CONTROL_CPU" \
        chrt -f "$CONTROL_RT_PRIO" \
        python3 "$CONTROL_SCRIPT" \
        >> "$LOG_DIR/control.log" 2>&1 &
    echo $! > "$PID_DIR/control.pid"
    log "控制进程 PID: $(cat "$PID_DIR/control.pid")"
}

# --- 启动人机交互进程 ---
start_hmi() {
    log "启动人机交互进程 (CPU=$HMI_CPU)..."

    taskset -c "$HMI_CPU" \
        nice -n 10 \
        python3 "$HMI_SCRIPT" \
        >> "$LOG_DIR/hmi.log" 2>&1 &
    echo $! > "$PID_DIR/hmi.pid"
    log "人机交互进程 PID: $(cat "$PID_DIR/hmi.pid")"
}

# --- 启动性能监控 ---
start_monitor() {
    log "启动性能监控..."

    python3 "$PROJECT_DIR/性能监控工具.py" \
        --log "$LOG_DIR/monitor.csv" \
        >> "$LOG_DIR/monitor.log" 2>&1 &
    echo $! > "$PID_DIR/monitor.pid"
    log "监控进程 PID: $(cat "$PID_DIR/monitor.pid")"
}

# --- 看门狗 ---
watchdog() {
    log "看门狗启动"
    local fail_count=0
    local max_fail=3

    while true; do
        sleep 2

        # 检查各进程是否存活
        for proc in vision control hmi; do
            pid_file="$PID_DIR/${proc}.pid"
            if [ -f "$pid_file" ]; then
                pid=$(cat "$pid_file")
                if ! kill -0 "$pid" 2>/dev/null; then
                    fail_count=$((fail_count + 1))
                    log "警告: ${proc}进程(PID=$pid)已停止! (失败${fail_count}次)"

                    if [ "$fail_count" -ge "$max_fail" ]; then
                        log "错误: 进程多次重启失败，进入安全模式"
                        stop_all
                        exit 1
                    fi

                    # 重启该进程
                    case "$proc" in
                        vision)  start_vision ;;
                        control) start_control ;;
                        hmi)     start_hmi ;;
                    esac
                fi
            fi
        done

        # 检查CPU温度
        if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
            temp=$(cat /sys/class/thermal/thermal_zone0/temp)
            temp_c=$((temp / 1000))
            if [ "$temp_c" -gt 85 ]; then
                log "警告: CPU温度过高! ${temp_c}°C"
            fi
        fi
    done
}

# --- 停止所有进程 ---
stop_all() {
    log "=== 停止所有进程 ==="
    for proc in vision control hmi monitor; do
        pid_file="$PID_DIR/${proc}.pid"
        if [ -f "$pid_file" ]; then
            pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                kill "$pid" 2>/dev/null || true
                log "已停止 ${proc} (PID=$pid)"
            fi
            rm -f "$pid_file"
        fi
    done

    # 等待进程退出
    sleep 1

    # 强制清理残留
    pkill -f "vision_main.py" 2>/dev/null || true
    pkill -f "control_main.py" 2>/dev/null || true
    pkill -f "hmi_main.py" 2>/dev/null || true

    log "所有进程已停止"
}

# --- 状态查询 ---
show_status() {
    echo "=== 电赛系统状态 ==="
    echo "时间: $(date)"
    echo ""

    # 进程状态
    for proc in vision control hmi monitor; do
        pid_file="$PID_DIR/${proc}.pid"
        if [ -f "$pid_file" ]; then
            pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                cpu=$(ps -p "$pid" -o %cpu= 2>/dev/null || echo "N/A")
                mem=$(ps -p "$pid" -o rss= 2>/dev/null || echo "N/A")
                echo "✅ $proc: PID=$pid, CPU=${cpu}%, MEM=${mem}KB"
            else
                echo "❌ $proc: 已停止 (PID=$pid)"
            fi
        else
            echo "⚪ $proc: 未启动"
        fi
    done

    echo ""

    # 系统资源
    echo "=== 系统资源 ==="
    echo "CPU温度: $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $1/1000"°C"}' || echo 'N/A')"
    echo "内存使用:"
    free -h | head -2
    echo ""
    echo "CPU频率:"
    cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq 2>/dev/null | \
        paste - - - - - - - - | awk '{print "  大核:", $5/1000, $6/1000, $7/1000, $8/1000, "MHz"}' || true
}

# === 主入口 ===

case "${1:-start}" in
    start)
        check_root
        init_dirs
        system_optimize
        camera_init
        network_setup

        log "========================================="
        log "  电赛系统启动"
        log "========================================="

        start_vision
        sleep 1
        start_control
        sleep 1
        start_hmi
        sleep 1
        start_monitor

        log "所有进程已启动"
        show_status

        # 前台运行看门狗
        watchdog
        ;;

    stop)
        check_root
        stop_all
        ;;

    restart)
        check_root
        stop_all
        sleep 2
        exec "$0" start
        ;;

    status)
        show_status
        ;;

    *)
        echo "用法: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
