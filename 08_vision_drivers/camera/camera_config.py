"""
相机参数配置 - 不同光照环境最佳参数表
针对Orange Pi 5 + USB3.0全局快门相机优化
"""
from dataclasses import dataclass, field
from typing import Dict, Optional
import json
import os
import logging

logger = logging.getLogger(__name__)


@dataclass
class CameraParams:
    """相机参数集合"""
    exposure: int = 500           # 曝光值（100us为单位）
    gain: int = 16                # 增益
    brightness: int = 128         # 亮度
    contrast: int = 128           # 对比度
    saturation: int = 128         # 饱和度
    sharpness: int = 128          # 锐度
    white_balance: int = 4600     # 白平衡温度
    auto_exposure: bool = False   # 自动曝光
    auto_wb: bool = False         # 自动白平衡
    gamma: int = 100              # gamma值
    fps: float = 60.0             # 帧率
    resolution: tuple = (640, 480)  # 分辨率

    def to_dict(self) -> dict:
        return {
            'exposure': self.exposure, 'gain': self.gain,
            'brightness': self.brightness, 'contrast': self.contrast,
            'saturation': self.saturation, 'sharpness': self.sharpness,
            'white_balance': self.white_balance, 'auto_exposure': self.auto_exposure,
            'auto_wb': self.auto_wb, 'gamma': self.gamma,
            'fps': self.fps, 'resolution': list(self.resolution),
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'CameraParams':
        p = cls()
        for k, v in d.items():
            if hasattr(p, k):
                setattr(p, k, tuple(v) if k == 'resolution' and isinstance(v, list) else v)
        return p


# ==================== 预设参数表 ====================

PRESETS: Dict[str, CameraParams] = {
    # ---- 室内标准 ----
    "indoor_normal": CameraParams(
        exposure=800, gain=20, brightness=130, contrast=128,
        saturation=120, sharpness=128, white_balance=4000,
        auto_exposure=False, auto_wb=False, fps=60.0,
        resolution=(640, 480),
    ),
    # ---- 室内弱光 ----
    "indoor_low_light": CameraParams(
        exposure=2000, gain=48, brightness=140, contrast=135,
        saturation=100, sharpness=100, white_balance=3500,
        auto_exposure=False, auto_wb=False, fps=30.0,
        resolution=(640, 480),
    ),
    # ---- 室内强光（靠近窗户/灯光） ----
    "indoor_bright": CameraParams(
        exposure=300, gain=8, brightness=120, contrast=120,
        saturation=128, sharpness=128, white_balance=5000,
        auto_exposure=False, auto_wb=False, fps=60.0,
        resolution=(640, 480),
    ),
    # ---- 室外晴天 ----
    "outdoor_sunny": CameraParams(
        exposure=100, gain=0, brightness=110, contrast=115,
        saturation=140, sharpness=135, white_balance=5600,
        auto_exposure=False, auto_wb=False, fps=120.0,
        resolution=(640, 480),
    ),
    # ---- 室外阴天 ----
    "outdoor_cloudy": CameraParams(
        exposure=300, gain=8, brightness=125, contrast=128,
        saturation=130, sharpness=128, white_balance=6000,
        auto_exposure=False, auto_wb=False, fps=90.0,
        resolution=(640, 480),
    ),
    # ---- 室外阴影 ----
    "outdoor_shadow": CameraParams(
        exposure=600, gain=16, brightness=130, contrast=130,
        saturation=120, sharpness=120, white_balance=6500,
        auto_exposure=False, auto_wb=False, fps=60.0,
        resolution=(640, 480),
    ),
    # ---- 高速抓拍（最小曝光） ----
    "high_speed": CameraParams(
        exposure=10, gain=32, brightness=128, contrast=128,
        saturation=128, sharpness=100, white_balance=4600,
        auto_exposure=False, auto_wb=False, fps=120.0,
        resolution=(640, 480),
    ),
    # ---- 条码/二维码扫描 ----
    "barcode_scan": CameraParams(
        exposure=200, gain=8, brightness=128, contrast=180,
        saturation=0, sharpness=180, white_balance=5000,
        auto_exposure=False, auto_wb=False, fps=60.0,
        resolution=(640, 480),
    ),
    # ---- 颜色检测（饱和度拉满） ----
    "color_detect": CameraParams(
        exposure=400, gain=8, brightness=128, contrast=135,
        saturation=200, sharpness=100, white_balance=4600,
        auto_exposure=False, auto_wb=False, fps=60.0,
        resolution=(640, 480),
    ),
    # ---- 低延迟竞技模式 ----
    "low_latency": CameraParams(
        exposure=500, gain=16, brightness=128, contrast=128,
        saturation=128, sharpness=128, white_balance=4600,
        auto_exposure=False, auto_wb=False, fps=120.0,
        resolution=(320, 240),
    ),
}


class CameraConfig:
    """
    相机配置管理器
    - 预设参数表管理
    - 动态参数调整
    - 配置文件持久化
    - 自适应环境参数推荐
    """

    def __init__(self, config_path: str = None):
        self._presets = dict(PRESETS)
        self._current = CameraParams()
        self._config_path = config_path

        if config_path and os.path.exists(config_path):
            self.load(config_path)

    @property
    def current(self) -> CameraParams:
        return self._current

    def get_preset(self, name: str) -> Optional[CameraParams]:
        """获取预设参数"""
        return self._presets.get(name)

    def apply_preset(self, name: str) -> bool:
        """应用预设参数"""
        preset = self._presets.get(name)
        if preset is None:
            logger.warning(f"预设 '{name}' 不存在，可用: {list(self._presets.keys())}")
            return False
        self._current = CameraParams(**preset.to_dict())
        logger.info(f"已应用预设: {name}")
        return True

    def list_presets(self) -> list:
        """列出所有预设"""
        return list(self._presets.keys())

    def register_preset(self, name: str, params: CameraParams):
        """注册自定义预设"""
        self._presets[name] = params

    def recommend_for_lighting(self, lux: float) -> str:
        """根据光照强度推荐预设"""
        if lux < 50:
            return "indoor_low_light"
        elif lux < 200:
            return "indoor_normal"
        elif lux < 500:
            return "indoor_bright"
        elif lux < 2000:
            return "outdoor_cloudy"
        elif lux < 10000:
            return "outdoor_sunny"
        else:
            return "outdoor_sunny"  # 极强光下用最小曝光

    def recommend_for_scene(self, scene: str) -> str:
        """根据场景推荐预设"""
        scene_map = {
            '竞赛': 'high_speed',
            '比赛': 'high_speed',
            '高速': 'high_speed',
            '条码': 'barcode_scan',
            '二维码': 'barcode_scan',
            '颜色': 'color_detect',
            '色块': 'color_detect',
            '低延迟': 'low_latency',
            '室内': 'indoor_normal',
            '室外': 'outdoor_sunny',
        }
        for keyword, preset in scene_map.items():
            if keyword in scene:
                return preset
        return 'indoor_normal'

    def apply_to_camera(self, camera):
        """将当前参数应用到相机驱动"""
        p = self._current
        if hasattr(camera, 'set_exposure'):
            camera.set_exposure(p.exposure)
        if hasattr(camera, 'set_gain'):
            camera.set_gain(p.gain)
        if hasattr(camera, 'set_brightness'):
            camera.set_brightness(p.brightness)
        if hasattr(camera, 'set_contrast'):
            camera.set_contrast(p.contrast)
        if hasattr(camera, 'set_saturation'):
            camera.set_saturation(p.saturation)
        if hasattr(camera, 'set_auto_white_balance'):
            camera.set_auto_white_balance(p.auto_wb)
        logger.info("参数已应用到相机")

    def save(self, path: str = None):
        """保存配置到文件"""
        path = path or self._config_path
        if not path:
            raise ValueError("未指定保存路径")
        data = {
            'current': self._current.to_dict(),
            'presets': {k: v.to_dict() for k, v in self._presets.items()},
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        logger.info(f"配置已保存: {path}")

    def load(self, path: str = None):
        """从文件加载配置"""
        path = path or self._config_path
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if 'current' in data:
            self._current = CameraParams.from_dict(data['current'])
        if 'presets' in data:
            for name, params in data['presets'].items():
                self._presets[name] = CameraParams.from_dict(params)
        logger.info(f"配置已加载: {path}")
