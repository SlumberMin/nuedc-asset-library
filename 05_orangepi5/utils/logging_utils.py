#!/usr/bin/env python3
"""
日志工具
用于调试和系统监控
适用于Orange Pi 5控制系统
"""
import logging
import logging.handlers
import os
import sys
import time
import json
from datetime import datetime
from typing import Optional, Dict, Any, Callable
from enum import Enum
import threading
import functools


class LogLevel(Enum):
    """日志级别"""
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL


class LogFormat(Enum):
    """日志格式"""
    SIMPLE = "simple"
    DETAILED = "detailed"
    JSON = "json"
    CUSTOM = "custom"


class ColoredFormatter(logging.Formatter):
    """
    彩色日志格式化器
    在终端中显示彩色日志
    """
    
    # 颜色代码
    COLORS = {
        'DEBUG': '\033[36m',      # 青色
        'INFO': '\033[32m',       # 绿色
        'WARNING': '\033[33m',    # 黄色
        'ERROR': '\033[31m',      # 红色
        'CRITICAL': '\033[35m',   # 紫色
        'RESET': '\033[0m'        # 重置
    }
    
    def __init__(self, fmt=None, datefmt=None, style='%', colors=True):
        """
        初始化彩色格式化器
        
        Args:
            fmt: 格式字符串
            datefmt: 日期格式
            style: 格式风格
            colors: 是否启用颜色
        """
        super().__init__(fmt, datefmt, style)
        self.colors = colors
    
    def format(self, record):
        """
        格式化日志记录
        
        Args:
            record: 日志记录
            
        Returns:
            格式化后的字符串
        """
        # 保存原始颜色
        if self.colors and record.levelname in self.COLORS:
            record.color = self.COLORS[record.levelname]
            record.reset = self.COLORS['RESET']
        else:
            record.color = ''
            record.reset = ''
        
        # 格式化消息
        message = super().format(record)
        
        return message


class JSONFormatter(logging.Formatter):
    """
    JSON格式化器
    将日志记录格式化为JSON
    """
    
    def format(self, record):
        """
        格式化日志记录为JSON
        
        Args:
            record: 日志记录
            
        Returns:
            JSON字符串
        """
        log_data = {
            'timestamp': datetime.fromtimestamp(record.created).isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
            'thread': record.thread,
            'thread_name': record.threadName,
            'process': record.process
        }
        
        # 添加异常信息
        if record.exc_info:
            log_data['exception'] = self.formatException(record.exc_info)
        
        # 添加额外字段
        if hasattr(record, 'extra_data'):
            log_data['extra'] = record.extra_data
        
        return json.dumps(log_data, ensure_ascii=False)


class PerformanceFilter(logging.Filter):
    """
    性能过滤器
    记录性能相关的日志
    """
    
    def __init__(self, threshold_ms: float = 100.0):
        """
        初始化性能过滤器
        
        Args:
            threshold_ms: 阈值 (毫秒)
        """
        super().__init__()
        self.threshold_ms = threshold_ms
    
    def filter(self, record):
        """
        过滤日志记录
        
        Args:
            record: 日志记录
            
        Returns:
            是否通过过滤
        """
        # 检查是否有性能数据
        if hasattr(record, 'execution_time_ms'):
            if record.execution_time_ms > self.threshold_ms:
                record.slow_execution = True
                return True
        
        return True


class LogFileManager:
    """
    日志文件管理器
    管理日志文件的轮转和清理
    """
    
    def __init__(self, 
                 log_dir: str = "logs",
                 max_bytes: int = 10*1024*1024,  # 10MB
                 backup_count: int = 5,
                 compress: bool = True):
        """
        初始化日志文件管理器
        
        Args:
            log_dir: 日志目录
            max_bytes: 单个日志文件最大大小 (字节)
            backup_count: 保留的备份文件数量
            compress: 是否压缩备份文件
        """
        self.log_dir = log_dir
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.compress = compress
        
        # 创建日志目录
        os.makedirs(log_dir, exist_ok=True)
    
    def get_log_filepath(self, name: str) -> str:
        """
        获取日志文件路径
        
        Args:
            name: 日志文件名
            
        Returns:
            完整文件路径
        """
        return os.path.join(self.log_dir, f"{name}.log")
    
    def create_rotating_handler(self, name: str) -> logging.handlers.RotatingFileHandler:
        """
        创建轮转文件处理器
        
        Args:
            name: 日志文件名
            
        Returns:
            轮转文件处理器
        """
        filepath = self.get_log_filepath(name)
        
        handler = logging.handlers.RotatingFileHandler(
            filepath,
            maxBytes=self.max_bytes,
            backupCount=self.backup_count,
            encoding='utf-8'
        )
        
        return handler
    
    def create_timed_handler(self, name: str, 
                            when: str = 'midnight',
                            interval: int = 1) -> logging.handlers.TimedRotatingFileHandler:
        """
        创建基于时间的轮转处理器
        
        Args:
            name: 日志文件名
            when: 轮转时间 ('S', 'M', 'H', 'D', 'midnight')
            interval: 轮转间隔
            
        Returns:
            基于时间的轮转处理器
        """
        filepath = self.get_log_filepath(name)
        
        handler = logging.handlers.TimedRotatingFileHandler(
            filepath,
            when=when,
            interval=interval,
            backupCount=self.backup_count,
            encoding='utf-8'
        )
        
        return handler
    
    def cleanup_old_logs(self, days: int = 30):
        """
        清理旧日志文件
        
        Args:
            days: 保留天数
        """
        if not os.path.exists(self.log_dir):
            return
        
        current_time = time.time()
        cutoff_time = current_time - (days * 24 * 60 * 60)
        
        for filename in os.listdir(self.log_dir):
            filepath = os.path.join(self.log_dir, filename)
            
            if os.path.isfile(filepath):
                file_time = os.path.getmtime(filepath)
                
                if file_time < cutoff_time:
                    try:
                        os.remove(filepath)
                        print(f"已删除旧日志文件: {filename}")
                    except Exception as e:
                        print(f"删除日志文件失败: {filename}, 错误: {e}")
    
    def get_log_size(self) -> Dict[str, int]:
        """
        获取日志文件大小
        
        Returns:
            文件名字典到大小的映射
        """
        if not os.path.exists(self.log_dir):
            return {}
        
        sizes = {}
        for filename in os.listdir(self.log_dir):
            filepath = os.path.join(self.log_dir, filename)
            if os.path.isfile(filepath):
                sizes[filename] = os.path.getsize(filepath)
        
        return sizes


class SystemLogger:
    """
    系统日志器
    提供统一的日志接口
    """
    
    def __init__(self, 
                 name: str = "System",
                 log_dir: str = "logs",
                 console_level: LogLevel = LogLevel.INFO,
                 file_level: LogLevel = LogLevel.DEBUG,
                 log_format: LogFormat = LogFormat.DETAILED,
                 enable_colors: bool = True,
                 enable_json: bool = False,
                 max_bytes: int = 10*1024*1024,
                 backup_count: int = 5):
        """
        初始化系统日志器
        
        Args:
            name: 日志器名称
            log_dir: 日志目录
            console_level: 控制台日志级别
            file_level: 文件日志级别
            log_format: 日志格式
            enable_colors: 是否启用颜色
            enable_json: 是否启用JSON格式
            max_bytes: 单个日志文件最大大小
            backup_count: 保留的备份文件数量
        """
        self.name = name
        self.log_dir = log_dir
        self.console_level = console_level
        self.file_level = file_level
        self.log_format = log_format
        self.enable_colors = enable_colors
        self.enable_json = enable_json
        
        # 创建日志器
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        
        # 避免重复添加处理器
        if not self.logger.handlers:
            self._setup_handlers(max_bytes, backup_count)
        
        # 性能监控
        self.performance_data = {}
        self._lock = threading.Lock()
    
    def _setup_handlers(self, max_bytes: int, backup_count: int):
        """
        设置日志处理器
        
        Args:
            max_bytes: 单个日志文件最大大小
            backup_count: 保留的备份文件数量
        """
        # 控制台处理器
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(self.console_level.value)
        
        # 文件处理器
        file_manager = LogFileManager(
            log_dir=self.log_dir,
            max_bytes=max_bytes,
            backup_count=backup_count
        )
        
        # 普通日志文件
        file_handler = file_manager.create_rotating_handler(self.name.lower())
        file_handler.setLevel(self.file_level.value)
        
        # 错误日志文件
        error_handler = file_manager.create_rotating_handler(f"{self.name.lower()}_error")
        error_handler.setLevel(logging.ERROR)
        
        # 设置格式化器
        if self.enable_json:
            json_formatter = JSONFormatter()
            file_handler.setFormatter(json_formatter)
            error_handler.setFormatter(json_formatter)
            
            # 控制台使用简单格式
            console_formatter = self._create_formatter(LogFormat.SIMPLE)
            console_handler.setFormatter(console_formatter)
        else:
            formatter = self._create_formatter(self.log_format)
            console_handler.setFormatter(formatter)
            file_handler.setFormatter(formatter)
            error_handler.setFormatter(formatter)
        
        # 添加处理器
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)
        self.logger.addHandler(error_handler)
        
        # 添加性能过滤器
        perf_filter = PerformanceFilter(threshold_ms=100.0)
        self.logger.addFilter(perf_filter)
    
    def _create_formatter(self, log_format: LogFormat) -> logging.Formatter:
        """
        创建格式化器
        
        Args:
            log_format: 日志格式
            
        Returns:
            格式化器实例
        """
        if log_format == LogFormat.SIMPLE:
            fmt = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            datefmt = "%Y-%m-%d %H:%M:%S"
            
            if self.enable_colors:
                return ColoredFormatter(fmt, datefmt, colors=True)
            else:
                return logging.Formatter(fmt, datefmt)
        
        elif log_format == LogFormat.DETAILED:
            fmt = "%(asctime)s - %(name)s - %(levelname)s - [%(module)s:%(funcName)s:%(lineno)d] - %(message)s"
            datefmt = "%Y-%m-%d %H:%M:%S"
            
            if self.enable_colors:
                return ColoredFormatter(fmt, datefmt, colors=True)
            else:
                return logging.Formatter(fmt, datefmt)
        
        elif log_format == LogFormat.JSON:
            return JSONFormatter()
        
        else:  # CUSTOM
            fmt = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            datefmt = "%Y-%m-%d %H:%M:%S"
            return logging.Formatter(fmt, datefmt)
    
    def debug(self, message: str, **kwargs):
        """
        记录调试日志
        
        Args:
            message: 日志消息
            **kwargs: 额外数据
        """
        self.logger.debug(message, extra=kwargs)
    
    def info(self, message: str, **kwargs):
        """
        记录信息日志
        
        Args:
            message: 日志消息
            **kwargs: 额外数据
        """
        self.logger.info(message, extra=kwargs)
    
    def warning(self, message: str, **kwargs):
        """
        记录警告日志
        
        Args:
            message: 日志消息
            **kwargs: 额外数据
        """
        self.logger.warning(message, extra=kwargs)
    
    def error(self, message: str, exc_info: bool = False, **kwargs):
        """
        记录错误日志
        
        Args:
            message: 日志消息
            exc_info: 是否记录异常信息
            **kwargs: 额外数据
        """
        self.logger.error(message, exc_info=exc_info, extra=kwargs)
    
    def critical(self, message: str, exc_info: bool = True, **kwargs):
        """
        记录严重错误日志
        
        Args:
            message: 日志消息
            exc_info: 是否记录异常信息
            **kwargs: 额外数据
        """
        self.logger.critical(message, exc_info=exc_info, extra=kwargs)
    
    def exception(self, message: str, **kwargs):
        """
        记录异常日志
        
        Args:
            message: 日志消息
            **kwargs: 额外数据
        """
        self.logger.exception(message, extra=kwargs)
    
    def log_performance(self, operation: str, execution_time_ms: float, **kwargs):
        """
        记录性能日志
        
        Args:
            operation: 操作名称
            execution_time_ms: 执行时间 (毫秒)
            **kwargs: 额外数据
        """
        with self._lock:
            if operation not in self.performance_data:
                self.performance_data[operation] = {
                    'count': 0,
                    'total_time': 0.0,
                    'min_time': float('inf'),
                    'max_time': 0.0,
                    'avg_time': 0.0
                }
            
            data = self.performance_data[operation]
            data['count'] += 1
            data['total_time'] += execution_time_ms
            data['min_time'] = min(data['min_time'], execution_time_ms)
            data['max_time'] = max(data['max_time'], execution_time_ms)
            data['avg_time'] = data['total_time'] / data['count']
        
        # 记录到日志
        message = f"性能监控 - {operation}: {execution_time_ms:.2f}ms"
        self.debug(message, extra={'execution_time_ms': execution_time_ms, **kwargs})
    
    def get_performance_stats(self) -> Dict[str, Dict]:
        """
        获取性能统计信息
        
        Returns:
            性能统计字典
        """
        with self._lock:
            return self.performance_data.copy()
    
    def reset_performance_stats(self):
        """重置性能统计"""
        with self._lock:
            self.performance_data.clear()
    
    def set_level(self, level: LogLevel, handler_type: str = 'all'):
        """
        设置日志级别
        
        Args:
            level: 日志级别
            handler_type: 处理器类型 ('console', 'file', 'all')
        """
        for handler in self.logger.handlers:
            if handler_type == 'all':
                handler.setLevel(level.value)
            elif handler_type == 'console' and isinstance(handler, logging.StreamHandler):
                handler.setLevel(level.value)
            elif handler_type == 'file' and isinstance(handler, logging.FileHandler):
                handler.setLevel(level.value)
    
    def add_handler(self, handler: logging.Handler):
        """
        添加自定义处理器
        
        Args:
            handler: 日志处理器
        """
        self.logger.addHandler(handler)
    
    def remove_handler(self, handler: logging.Handler):
        """
        移除处理器
        
        Args:
            handler: 日志处理器
        """
        self.logger.removeHandler(handler)
    
    def cleanup(self):
        """清理资源"""
        # 关闭所有处理器
        for handler in self.logger.handlers[:]:
            handler.close()
            self.logger.removeHandler(handler)


class PerformanceMonitor:
    """
    性能监控器
    监控代码执行性能
    """
    
    def __init__(self, logger: SystemLogger = None):
        """
        初始化性能监控器
        
        Args:
            logger: 系统日志器
        """
        self.logger = logger
        self.metrics = {}
        self._lock = threading.Lock()
    
    def start_timer(self, name: str) -> float:
        """
        开始计时
        
        Args:
            name: 计时器名称
            
        Returns:
            开始时间
        """
        return time.perf_counter()
    
    def stop_timer(self, name: str, start_time: float) -> float:
        """
        停止计时
        
        Args:
            name: 计时器名称
            start_time: 开始时间
            
        Returns:
            执行时间 (毫秒)
        """
        end_time = time.perf_counter()
        execution_time_ms = (end_time - start_time) * 1000
        
        # 记录指标
        self.record_metric(name, execution_time_ms)
        
        # 记录到日志
        if self.logger:
            self.logger.log_performance(name, execution_time_ms)
        
        return execution_time_ms
    
    def record_metric(self, name: str, value: float):
        """
        记录指标
        
        Args:
            name: 指标名称
            value: 指标值
        """
        with self._lock:
            if name not in self.metrics:
                self.metrics[name] = {
                    'count': 0,
                    'sum': 0.0,
                    'min': float('inf'),
                    'max': float('-inf'),
                    'avg': 0.0,
                    'values': []
                }
            
            metric = self.metrics[name]
            metric['count'] += 1
            metric['sum'] += value
            metric['min'] = min(metric['min'], value)
            metric['max'] = max(metric['max'], value)
            metric['avg'] = metric['sum'] / metric['count']
            
            # 保留最近1000个值
            metric['values'].append(value)
            if len(metric['values']) > 1000:
                metric['values'].pop(0)
    
    def get_metric(self, name: str) -> Optional[Dict]:
        """
        获取指标
        
        Args:
            name: 指标名称
            
        Returns:
            指标数据或None
        """
        with self._lock:
            return self.metrics.get(name)
    
    def get_all_metrics(self) -> Dict[str, Dict]:
        """
        获取所有指标
        
        Returns:
            所有指标数据
        """
        with self._lock:
            return self.metrics.copy()
    
    def reset_metrics(self):
        """重置所有指标"""
        with self._lock:
            self.metrics.clear()
    
    def print_summary(self):
        """打印性能摘要"""
        with self._lock:
            if not self.metrics:
                print("没有性能数据")
                return
            
            print("\n" + "="*80)
            print("性能监控摘要")
            print("="*80)
            print(f"{'指标名称':<30} {'调用次数':<10} {'平均时间(ms)':<15} {'最小时间(ms)':<15} {'最大时间(ms)':<15}")
            print("-"*80)
            
            for name, metric in self.metrics.items():
                print(f"{name:<30} {metric['count']:<10} {metric['avg']:<15.2f} {metric['min']:<15.2f} {metric['max']:<15.2f}")
            
            print("="*80)


class DebugLogger:
    """
    调试日志器
    提供更详细的调试信息
    """
    
    def __init__(self, 
                 name: str = "Debug",
                 log_dir: str = "logs/debug",
                 enable_trace: bool = True):
        """
        初始化调试日志器
        
        Args:
            name: 日志器名称
            log_dir: 日志目录
            enable_trace: 是否启用调用追踪
        """
        self.name = name
        self.log_dir = log_dir
        self.enable_trace = enable_trace
        
        # 创建日志目录
        os.makedirs(log_dir, exist_ok=True)
        
        # 创建日志器
        self.logger = logging.getLogger(f"{name}.debug")
        self.logger.setLevel(logging.DEBUG)
        
        # 避免重复添加处理器
        if not self.logger.handlers:
            self._setup_handlers()
        
        # 调用栈追踪
        self.call_stack = []
        self._lock = threading.Lock()
    
    def _setup_handlers(self):
        """设置处理器"""
        # 调试日志文件
        debug_handler = logging.handlers.RotatingFileHandler(
            os.path.join(self.log_dir, f"{self.name.lower()}_debug.log"),
            maxBytes=5*1024*1024,  # 5MB
            backupCount=3,
            encoding='utf-8'
        )
        debug_handler.setLevel(logging.DEBUG)
        
        # 使用详细格式
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - [%(module)s:%(funcName)s:%(lineno)d] - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        debug_handler.setFormatter(formatter)
        
        self.logger.addHandler(debug_handler)
    
    def trace(self, message: str):
        """
        记录追踪信息
        
        Args:
            message: 追踪消息
        """
        if self.enable_trace:
            self.logger.debug(f"[TRACE] {message}")
    
    def enter_function(self, func_name: str, args: tuple = None, kwargs: dict = None):
        """
        记录函数进入
        
        Args:
            func_name: 函数名
            args: 参数
            kwargs: 关键字参数
        """
        with self._lock:
            self.call_stack.append(func_name)
        
        indent = "  " * len(self.call_stack)
        message = f"{indent}>>> 进入 {func_name}"
        if args:
            message += f" (args={args})"
        if kwargs:
            message += f" (kwargs={kwargs})"
        
        self.trace(message)
    
    def exit_function(self, func_name: str, return_value=None):
        """
        记录函数退出
        
        Args:
            func_name: 函数名
            return_value: 返回值
        """
        with self._lock:
            if self.call_stack and self.call_stack[-1] == func_name:
                self.call_stack.pop()
        
        indent = "  " * len(self.call_stack)
        message = f"{indent}<<< 退出 {func_name}"
        if return_value is not None:
            message += f" (返回={return_value})"
        
        self.trace(message)
    
    def log_variable(self, name: str, value: Any, level: int = logging.DEBUG):
        """
        记录变量值
        
        Args:
            name: 变量名
            value: 变量值
            level: 日志级别
        """
        self.logger.log(level, f"变量 {name} = {value}")
    
    def log_state(self, state_name: str, state_data: Dict):
        """
        记录状态信息
        
        Args:
            state_name: 状态名称
            state_data: 状态数据
        """
        self.logger.debug(f"状态 {state_name}: {json.dumps(state_data, ensure_ascii=False, default=str)}")
    
    def log_event(self, event_name: str, event_data: Dict = None):
        """
        记录事件
        
        Args:
            event_name: 事件名称
            event_data: 事件数据
        """
        message = f"事件: {event_name}"
        if event_data:
            message += f" - {json.dumps(event_data, ensure_ascii=False, default=str)}"
        
        self.logger.info(message)
    
    def cleanup(self):
        """清理资源"""
        for handler in self.logger.handlers[:]:
            handler.close()
            self.logger.removeHandler(handler)


# 便捷函数
def create_logger(name: str = "System", 
                  log_dir: str = "logs",
                  console_level: LogLevel = LogLevel.INFO,
                  file_level: LogLevel = LogLevel.DEBUG) -> SystemLogger:
    """
    创建系统日志器的便捷函数
    
    Args:
        name: 日志器名称
        log_dir: 日志目录
        console_level: 控制台日志级别
        file_level: 文件日志级别
        
    Returns:
        系统日志器实例
    """
    return SystemLogger(
        name=name,
        log_dir=log_dir,
        console_level=console_level,
        file_level=file_level
    )


def create_debug_logger(name: str = "Debug",
                        log_dir: str = "logs/debug",
                        enable_trace: bool = True) -> DebugLogger:
    """
    创建调试日志器的便捷函数
    
    Args:
        name: 日志器名称
        log_dir: 日志目录
        enable_trace: 是否启用调用追踪
        
    Returns:
        调试日志器实例
    """
    return DebugLogger(
        name=name,
        log_dir=log_dir,
        enable_trace=enable_trace
    )


def create_performance_monitor(logger: SystemLogger = None) -> PerformanceMonitor:
    """
    创建性能监控器的便捷函数
    
    Args:
        logger: 系统日志器
        
    Returns:
        性能监控器实例
    """
    return PerformanceMonitor(logger=logger)


def log_function_call(logger: DebugLogger = None):
    """
    记录函数调用的装饰器
    
    Args:
        logger: 调试日志器
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            if logger:
                logger.enter_function(func.__name__, args, kwargs)
            
            start_time = time.perf_counter()
            
            try:
                result = func(*args, **kwargs)
                
                execution_time_ms = (time.perf_counter() - start_time) * 1000
                
                if logger:
                    logger.exit_function(func.__name__, result)
                    logger.trace(f"{func.__name__} 执行时间: {execution_time_ms:.2f}ms")
                
                return result
                
            except Exception as e:
                if logger:
                    logger.error(f"{func.__name__} 发生异常: {e}")
                    logger.exit_function(func.__name__, f"异常: {e}")
                raise
        
        return wrapper
    return decorator


def log_performance(logger: SystemLogger = None):
    """
    记录性能的装饰器
    
    Args:
        logger: 系统日志器
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.perf_counter()
            
            result = func(*args, **kwargs)
            
            execution_time_ms = (time.perf_counter() - start_time) * 1000
            
            if logger:
                logger.log_performance(func.__name__, execution_time_ms)
            
            return result
        
        return wrapper
    return decorator


# 全局日志器实例
_global_logger = None
_global_debug_logger = None
_global_performance_monitor = None


def get_logger(name: str = "System") -> SystemLogger:
    """
    获取全局日志器
    
    Args:
        name: 日志器名称
        
    Returns:
        系统日志器实例
    """
    global _global_logger
    if _global_logger is None:
        _global_logger = create_logger(name)
    return _global_logger


def get_debug_logger(name: str = "Debug") -> DebugLogger:
    """
    获取全局调试日志器
    
    Args:
        name: 日志器名称
        
    Returns:
        调试日志器实例
    """
    global _global_debug_logger
    if _global_debug_logger is None:
        _global_debug_logger = create_debug_logger(name)
    return _global_debug_logger


def get_performance_monitor() -> PerformanceMonitor:
    """
    获取全局性能监控器
    
    Returns:
        性能监控器实例
    """
    global _global_performance_monitor
    if _global_performance_monitor is None:
        _global_performance_monitor = create_performance_monitor(get_logger())
    return _global_performance_monitor


# 测试代码
if __name__ == "__main__":
    # 测试系统日志器
    print("测试系统日志器...")
    
    logger = create_logger("TestSystem", log_dir="test_logs")
    
    logger.debug("这是一条调试日志")
    logger.info("这是一条信息日志")
    logger.warning("这是一条警告日志")
    logger.error("这是一条错误日志")
    
    # 测试性能日志
    logger.log_performance("test_operation", 150.5)
    logger.log_performance("test_operation", 50.2)
    logger.log_performance("test_operation", 200.0)
    
    print(f"性能统计: {logger.get_performance_stats()}")
    
    # 测试调试日志器
    print("\n测试调试日志器...")
    
    debug_logger = create_debug_logger("TestDebug", log_dir="test_logs/debug")
    
    debug_logger.enter_function("test_function", args=(1, 2), kwargs={'key': 'value'})
    debug_logger.log_variable("x", 42)
    debug_logger.log_state("test_state", {"status": "running", "count": 10})
    debug_logger.log_event("test_event", {"data": "test"})
    debug_logger.exit_function("test_function", return_value=42)
    
    # 测试性能监控器
    print("\n测试性能监控器...")
    
    monitor = create_performance_monitor(logger)
    
    # 模拟一些性能监控
    for i in range(10):
        start = monitor.start_timer("test_loop")
        time.sleep(0.01)  # 模拟工作
        monitor.stop_timer("test_loop", start)
    
    monitor.print_summary()
    
    # 测试装饰器
    print("\n测试装饰器...")
    
    @log_function_call(debug_logger)
    @log_performance(logger)
    def example_function(x, y):
        time.sleep(0.05)
        return x + y
    
    result = example_function(3, 4)
    print(f"函数结果: {result}")
    
    # 测试全局日志器
    print("\n测试全局日志器...")
    
    global_logger = get_logger("Global")
    global_logger.info("这是全局日志器的消息")
    
    # 清理
    logger.cleanup()
    debug_logger.cleanup()
    
    print("\n所有测试完成!")