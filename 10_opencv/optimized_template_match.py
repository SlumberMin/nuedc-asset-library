#!/usr/bin/env python3
"""
optimized_template_match.py - 优化版模板匹配模块
适用于 RK3588S 嵌入式平台，针对电赛场景优化

优化策略:
1. 图像金字塔加速: 先在低分辨率粗搜索，再在高分辨率精定位
2. 多尺度匹配: 自动缩放模板适应不同距离
3. 多方法融合: TM_CCOEFF_NORMED + TM_SQDIFF_NORMED 加权
4. NMS后处理: 非最大抑制去除重叠检测
5. ROI限定: 只在感兴趣区域搜索
6. 缓存机制: 模板预处理后缓存
"""

import cv2
import numpy as np
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from collections import deque


@dataclass
class MatchResult:
    """模板匹配结果"""
    x: int             # 左上角x
    y: int             # 左上角y
    w: int             # 宽度
    h: int             # 高度
    score: float       # 匹配分数
    scale: float       # 匹配时的缩放比例
    method: str        # 使用的匹配方法
    angle: float = 0.0 # 旋转角度(如果使用旋转匹配)

    @property
    def center(self) -> Tuple[int, int]:
        return (self.x + self.w // 2, self.y + self.h // 2)

    @property
    def bbox(self) -> Tuple[int, int, int, int]:
        return (self.x, self.y, self.w, self.h)


class OptimizedTemplateMatcher:
    """
    优化版模板匹配器
    
    金字塔加速策略:
    Level 0 (原始): 精确定位
    Level 1 (1/2):  粗搜索
    Level 2 (1/4):  初始搜索
    
    多尺度策略:
    在每个尺度上都进行匹配，取最佳结果
    """

    def __init__(self,
                 template: Optional[np.ndarray] = None,
                 template_path: Optional[str] = None,
                 scale_range: Tuple[float, float] = (0.5, 2.0),
                 scale_steps: int = 10,
                 match_threshold: float = 0.7,
                 method: str = 'ccoeff_normed',
                 use_pyramid: bool = True,
                 pyramid_levels: int = 3,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 use_rotation: bool = False,
                 rotation_range: Tuple[float, float] = (-30, 30),
                 rotation_step: float = 5.0,
                 nms_threshold: float = 0.3,
                 max_results: int = 10):
        """
        Args:
            template: 模板图像 (numpy数组)
            template_path: 模板文件路径
            scale_range: 缩放范围
            scale_steps: 缩放步数
            match_threshold: 匹配阈值
            method: 匹配方法 ('ccoeff_normed', 'sqdiff_normed', 'ccorr_normed')
            use_pyramid: 是否使用金字塔加速
            pyramid_levels: 金字塔层数
            roi: 感兴趣区域
            use_rotation: 是否使用旋转匹配
            rotation_range: 旋转角度范围
            rotation_step: 旋转角度步长
            nms_threshold: NMS IoU阈值
            max_results: 最大结果数
        """
        # 加载模板
        if template is not None:
            self._template = template.copy()
        elif template_path is not None:
            self._template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            if self._template is None:
                raise FileNotFoundError(f"无法加载模板: {template_path}")
        else:
            self._template = None
        
        self.scale_range = scale_range
        self.scale_steps = scale_steps
        self.match_threshold = match_threshold
        self.use_pyramid = use_pyramid
        self.pyramid_levels = pyramid_levels
        self.roi = roi
        self.use_rotation = use_rotation
        self.rotation_range = rotation_range
        self.rotation_step = rotation_step
        self.nms_threshold = nms_threshold
        self.max_results = max_results
        
        # 匹配方法映射
        self._methods = {
            'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
            'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
            'ccorr_normed': cv2.TM_CCORR_NORMED,
        }
        self._cv_method = self._methods.get(method, cv2.TM_CCOEFF_NORMED)
        self._method_name = method
        self._is_sqdiff = method == 'sqdiff_normed'
        
        # 预处理模板
        self._template_pyramid = []
        self._template_scales = []
        self._template_rotations = {}
        
        if self._template is not None:
            self._preprocess_template()
        
        # 缩放因子列表
        self._scales = np.linspace(scale_range[0], scale_range[1], scale_steps)
        
        # 性能统计
        self._perf = deque(maxlen=50)

    def _preprocess_template(self):
        """预处理模板（构建金字塔+旋转缓存）"""
        if self._template is None:
            return
        
        # 灰度化
        if len(self._template.shape) == 3:
            gray = cv2.cvtColor(self._template, cv2.COLOR_BGR2GRAY)
        else:
            gray = self._template.copy()
        
        # 金字塔
        self._template_pyramid = [gray]
        for i in range(self.pyramid_levels - 1):
            self._template_pyramid.append(cv2.pyrDown(self._template_pyramid[-1]))
        
        # 旋转缓存
        if self.use_rotation:
            angles = np.arange(self.rotation_range[0],
                               self.rotation_range[1],
                               self.rotation_step)
            for angle in angles:
                rotated = self._rotate_image(gray, angle)
                self._template_rotations[angle] = rotated

    def _rotate_image(self, img: np.ndarray, angle: float) -> np.ndarray:
        """旋转图像（保持完整内容）"""
        h, w = img.shape[:2]
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        
        # 计算旋转后尺寸
        cos = abs(M[0, 0])
        sin = abs(M[0, 1])
        new_w = int(h * sin + w * cos)
        new_h = int(h * cos + w * sin)
        
        M[0, 2] += (new_w - w) / 2
        M[1, 2] += (new_h - h) / 2
        
        return cv2.warpAffine(img, M, (new_w, new_h),
                              flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT,
                              borderValue=0)

    def _match_single_scale(self, gray: np.ndarray,
                            template: np.ndarray,
                            scale: float) -> List[MatchResult]:
        """单尺度模板匹配"""
        # 缩放模板
        th, tw = template.shape[:2]
        new_w = int(tw * scale)
        new_h = int(th * scale)
        
        if new_w < 5 or new_h < 5 or new_w > gray.shape[1] or new_h > gray.shape[0]:
            return []
        
        scaled_tmpl = cv2.resize(template, (new_w, new_h),
                                  interpolation=cv2.INTER_AREA if scale < 1 
                                  else cv2.INTER_LINEAR)
        
        # 执行匹配
        result = cv2.matchTemplate(gray, scaled_tmpl, self._cv_method)
        
        # 提取满足阈值的位置
        results = []
        
        if self._is_sqdiff:
            # SQDIFF: 值越小越好
            locations = np.where(result <= (1.0 - self.match_threshold))
            scores = 1.0 - result[locations]
        else:
            # CCOEFF/CCORR: 值越大越好
            locations = np.where(result >= self.match_threshold)
            scores = result[locations]
        
        for i in range(len(locations[0])):
            y = int(locations[0][i])
            x = int(locations[1][i])
            
            results.append(MatchResult(
                x=x, y=y,
                w=new_w, h=new_h,
                score=float(scores[i]),
                scale=scale,
                method=self._method_name,
            ))
        
        return results

    def _match_pyramid(self, gray: np.ndarray,
                       template: np.ndarray) -> List[MatchResult]:
        """
        金字塔加速匹配
        
        策略:
        1. 在金字塔顶层粗搜索，找到候选区域
        2. 在候选区域对应的下层精确搜索
        3. 逐层细化直到原始分辨率
        """
        # 构建搜索图像金字塔
        search_pyramid = [gray]
        for i in range(self.pyramid_levels - 1):
            search_pyramid.append(cv2.pyrDown(search_pyramid[-1]))
        
        # 从顶层开始
        top_level = len(search_pyramid) - 1
        
        # 顶层粗搜索（使用较小的缩放范围）
        coarse_results = []
        for scale in self._scales:
            results = self._match_single_scale(
                search_pyramid[top_level],
                self._template_pyramid[top_level],
                scale
            )
            coarse_results.extend(results)
        
        if not coarse_results:
            return []
        
        # 取top-N候选
        coarse_results.sort(key=lambda r: r.score, reverse=True)
        candidates = coarse_results[:20]
        
        # 逐层细化
        for level in range(top_level - 1, -1, -1):
            refined = []
            scale_factor = 2 ** level
            
            for cand in candidates:
                # 映射到当前层坐标
                region_x = int(cand.x * scale_factor)
                region_y = int(cand.y * scale_factor)
                region_w = int(cand.w * scale_factor) + 50  # 稍大一点的搜索区域
                region_h = int(cand.h * scale_factor) + 50
                
                # 边界检查
                region_x = max(0, region_x - 25)
                region_y = max(0, region_y - 25)
                region_w = min(region_w, search_pyramid[level].shape[1] - region_x)
                region_h = min(region_h, search_pyramid[level].shape[0] - region_y)
                
                if region_w < 10 or region_h < 10:
                    continue
                
                # 在候选区域精确匹配
                region = search_pyramid[level][region_y:region_y+region_h,
                                                region_x:region_x+region_w]
                
                level_results = self._match_single_scale(
                    region, self._template_pyramid[level], cand.scale
                )
                
                # 坐标偏移回全局
                for r in level_results:
                    r.x += region_x
                    r.y += region_y
                    refined.append(r)
            
            if refined:
                refined.sort(key=lambda r: r.score, reverse=True)
                candidates = refined[:20]
        
        # 最终映射回原始分辨率
        final_results = []
        for cand in candidates:
            cand.x *= 1  # Level 0 = 原始分辨率
            cand.y *= 1
            final_results.append(cand)
        
        return final_results

    def _nms(self, results: List[MatchResult]) -> List[MatchResult]:
        """非最大抑制(NMS)去重"""
        if not results:
            return []
        
        # 按分数降序排列
        results.sort(key=lambda r: r.score, reverse=True)
        
        keep = []
        suppressed = set()
        
        for i, ri in enumerate(results):
            if i in suppressed:
                continue
            
            keep.append(ri)
            
            for j in range(i + 1, len(results)):
                if j in suppressed:
                    continue
                
                rj = results[j]
                
                # 计算IoU
                x1 = max(ri.x, rj.x)
                y1 = max(ri.y, rj.y)
                x2 = min(ri.x + ri.w, rj.x + rj.w)
                y2 = min(ri.y + ri.h, rj.y + rj.h)
                
                if x2 > x1 and y2 > y1:
                    intersection = (x2 - x1) * (y2 - y1)
                    area_i = ri.w * ri.h
                    area_j = rj.w * rj.h
                    union = area_i + area_j - intersection
                    iou = intersection / union if union > 0 else 0
                    
                    if iou > self.nms_threshold:
                        suppressed.add(j)
            
            if len(keep) >= self.max_results:
                break
        
        return keep

    def set_template(self, template: np.ndarray):
        """设置/更新模板"""
        self._template = template.copy()
        self._preprocess_template()

    def detect(self, frame: np.ndarray,
               use_multi_scale: bool = True) -> List[MatchResult]:
        """
        主检测函数
        
        Args:
            frame: BGR输入图像
            use_multi_scale: 是否使用多尺度匹配
        
        Returns:
            匹配结果列表（按分数降序）
        """
        if self._template is None:
            return []
        
        t0 = cv2.getTickCount()
        
        # ROI裁剪
        img = frame
        roi_offset = (0, 0)
        if self.roi is not None:
            x, y, w, h = self.roi
            img = img[y:y+h, x:x+w]
            roi_offset = (x, y)
        
        # 灰度化
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        
        all_results = []
        
        if use_multi_scale:
            if self.use_pyramid:
                # 金字塔加速 + 多尺度
                for scale in self._scales:
                    results = self._match_pyramid(gray, self._template_pyramid[0])
                    all_results.extend(results)
                
                # 也直接在原图上匹配（最高精度）
                for scale in self._scales:
                    results = self._match_single_scale(
                        gray, self._template_pyramid[0], scale)
                    all_results.extend(results)
            else:
                # 直接多尺度匹配
                for scale in self._scales:
                    results = self._match_single_scale(
                        gray, self._template_pyramid[0], scale)
                    all_results.extend(results)
        else:
            # 单尺度匹配 (scale=1.0)
            results = self._match_single_scale(
                gray, self._template_pyramid[0], 1.0)
            all_results.extend(results)
        
        # 旋转匹配
        if self.use_rotation and self._template_rotations:
            for angle, rotated_tmpl in self._template_rotations.items():
                results = self._match_single_scale(gray, rotated_tmpl, 1.0)
                for r in results:
                    r.angle = angle
                all_results.extend(results)
        
        # NMS去重
        all_results = self._nms(all_results)
        
        # ROI坐标补偿
        if self.roi is not None:
            for r in all_results:
                r.x += roi_offset[0]
                r.y += roi_offset[1]
        
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf.append(elapsed)
        
        return all_results

    def detect_best(self, frame: np.ndarray) -> Optional[MatchResult]:
        """只返回最佳匹配"""
        results = self.detect(frame)
        return results[0] if results else None

    def get_fps(self) -> float:
        if not self._perf:
            return 0.0
        avg = sum(self._perf) / len(self._perf)
        return 1000.0 / avg if avg > 0 else 0.0

    def draw_results(self, frame: np.ndarray,
                     results: List[MatchResult]) -> np.ndarray:
        """绘制匹配结果"""
        vis = frame.copy()
        
        for i, r in enumerate(results):
            # 匹配框
            color = (0, 255, 0) if i == 0 else (0, 200, 200)
            cv2.rectangle(vis, (r.x, r.y), (r.x + r.w, r.y + r.h), color, 2)
            
            # 中心点
            cv2.circle(vis, r.center, 5, (0, 0, 255), -1)
            
            # 标注
            label = f"Score:{r.score:.3f} Scale:{r.scale:.2f}"
            if r.angle != 0:
                label += f" Rot:{r.angle:.0f}"
            cv2.putText(vis, label, (r.x, r.y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return vis


# ============================================================
# 快捷接口
# ============================================================

def match_template(frame: np.ndarray,
                   template: np.ndarray,
                   threshold: float = 0.7,
                   multi_scale: bool = True) -> List[MatchResult]:
    """快速模板匹配"""
    matcher = OptimizedTemplateMatcher(
        template=template,
        match_threshold=threshold,
        scale_range=(0.5, 2.0) if multi_scale else (1.0, 1.0),
        scale_steps=1 if not multi_scale else 10,
    )
    return matcher.detect(frame, use_multi_scale=multi_scale)


def find_template(frame: np.ndarray,
                  template_path: str,
                  threshold: float = 0.7) -> Optional[MatchResult]:
    """查找模板（返回最佳匹配）"""
    matcher = OptimizedTemplateMatcher(
        template_path=template_path,
        match_threshold=threshold,
    )
    return matcher.detect_best(frame)


# ============================================================
# 演示
# ============================================================

def main():
    """演示 + 性能测试"""
    # 创建测试场景
    scene = np.zeros((600, 800, 3), dtype=np.uint8)
    
    # 在场景中放置一些目标
    target = np.zeros((60, 80, 3), dtype=np.uint8)
    cv2.rectangle(target, (5, 5), (75, 55), (0, 200, 255), -1)
    cv2.circle(target, (40, 30), 15, (255, 200, 0), -1)
    
    # 放置多个不同大小的目标
    targets = [
        (100, 100, 1.0),
        (350, 200, 0.7),
        (500, 400, 1.3),
        (200, 350, 0.9),
    ]
    
    for tx, ty, scale in targets:
        w = int(80 * scale)
        h = int(60 * scale)
        scaled = cv2.resize(target, (w, h))
        scene[ty:ty+h, tx:tx+w] = scaled
    
    # 添加噪声
    noise = np.random.randint(0, 20, scene.shape, dtype=np.uint8)
    scene = cv2.add(scene, noise)
    
    # 创建模板
    template = target
    
    # 创建匹配器
    matcher = OptimizedTemplateMatcher(
        template=template,
        scale_range=(0.5, 1.5),
        scale_steps=8,
        match_threshold=0.6,
        use_pyramid=True,
    )
    
    # 预热
    for _ in range(5):
        matcher.detect(scene)
    
    # 性能测试
    iterations = 30
    times = []
    for _ in range(iterations):
        t0 = cv2.getTickCount()
        results = matcher.detect(scene)
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        times.append(elapsed)
    
    print(f"模板匹配性能测试 ({scene.shape[1]}x{scene.shape[0]})")
    print(f"  模板大小: {template.shape[1]}x{template.shape[0]}")
    print(f"  缩放范围: {matcher.scale_range}")
    print(f"  金字塔层数: {matcher.pyramid_levels}")
    print(f"  平均耗时: {np.mean(times):.2f} ms")
    print(f"  FPS: {1000/np.mean(times):.1f}")
    print(f"  检测到 {len(results)} 个匹配:")
    for r in results:
        print(f"    pos=({r.x},{r.y}) size=({r.w}x{r.h}) "
              f"score={r.score:.3f} scale={r.scale:.2f}")
    
    # 可视化
    vis = matcher.draw_results(scene, results)
    cv2.imshow('Template Matching', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
