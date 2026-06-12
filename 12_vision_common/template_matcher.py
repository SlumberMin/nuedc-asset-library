"""
模板匹配模块 - 多尺度+旋转+金字塔加速
适用于电赛中目标识别、定位、对齐等场景

功能:
- 单尺度模板匹配 (TM_CCOEFF_NORMED / TM_SQDIFF_NORMED)
- 多尺度金字塔加速匹配
- 多角度旋转模板匹配
- 多模板批量匹配
- 匹配结果可视化与NMS过滤
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict


class TemplateMatcher:
    """模板匹配器"""

    def __init__(self, method: int = cv2.TM_CCOEFF_NORMED):
        """
        初始化模板匹配器
        Args:
            method: 匹配方法
                cv2.TM_CCOEFF_NORMED  - 归一化相关系数 (推荐, 越大越匹配)
                cv2.TM_SQDIFF_NORMED  - 归一化平方差 (越小越匹配)
        """
        self.method = method
        self.is_sqdiff = method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]

    def match_single(self, image: np.ndarray, template: np.ndarray,
                     mask: Optional[np.ndarray] = None) -> Dict:
        """
        单尺度模板匹配
        Args:
            image: 搜索图像 (灰度或彩色)
            template: 模板图像
            mask: 掩码 (可选, 仅TM_CCOEFF_NORMED支持)
        Returns:
            dict: {location, score, top_left, bottom_right}
        """
        result = cv2.matchTemplate(image, template, self.method, mask=mask)
        if self.is_sqdiff:
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            score, loc = 1.0 - min_val, min_loc
        else:
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            score, loc = max_val, max_loc

        h, w = template.shape[:2]
        return {
            'location': loc,
            'score': float(score),
            'top_left': loc,
            'bottom_right': (loc[0] + w, loc[1] + h),
            'size': (w, h),
            'confidence': float(score)
        }

    def match_multi_scale(self, image: np.ndarray, template: np.ndarray,
                          scale_range: Tuple[float, float] = (0.5, 2.0),
                          scale_steps: int = 20,
                          pyramid_levels: int = 0) -> List[Dict]:
        """
        多尺度模板匹配 (带金字塔加速)
        Args:
            image: 搜索图像
            template: 模板图像
            scale_range: 缩放范围 (最小, 最大)
            scale_steps: 缩放步数
            pyramid_levels: 金字塔层数 (0=不使用, 1=一级加速, 2=二级加速)
        Returns:
            list[dict]: 匹配结果列表, 按score降序
        """
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY) if len(template.shape) == 3 else template

        results = []
        scales = np.linspace(scale_range[0], scale_range[1], scale_steps)

        for scale in scales:
            # 缩放模板
            new_w = int(tpl_gray.shape[1] * scale)
            new_h = int(tpl_gray.shape[0] * scale)
            if new_w < 1 or new_h < 1:
                continue
            if new_w > img_gray.shape[1] or new_h > img_gray.shape[0]:
                continue

            scaled_tpl = cv2.resize(tpl_gray, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # 金字塔加速: 在缩小图上粗搜索, 原图上精搜索
            if pyramid_levels > 0:
                coarse_match = self._pyramid_search(img_gray, scaled_tpl, pyramid_levels)
                if coarse_match is None:
                    continue
                results.append({
                    'location': coarse_match['location'],
                    'score': coarse_match['score'],
                    'scale': scale,
                    'top_left': coarse_match['top_left'],
                    'bottom_right': coarse_match['bottom_right'],
                    'size': (new_w, new_h)
                })
            else:
                match = self.match_single(img_gray, scaled_tpl)
                match['scale'] = scale
                results.append(match)

        # 按score排序
        results.sort(key=lambda x: x['score'], reverse=True)
        return results

    def _pyramid_search(self, image: np.ndarray, template: np.ndarray,
                        levels: int) -> Optional[Dict]:
        """金字塔加速搜索"""
        # 构建图像金字塔
        img_pyr = [image]
        for _ in range(levels):
            img_pyr.append(cv2.pyrDown(img_pyr[-1]))

        # 在最粗层搜索
        scale_factor = 2 ** levels
        coarse_tpl = cv2.resize(template,
                                (template.shape[1] // scale_factor, template.shape[0] // scale_factor),
                                interpolation=cv2.INTER_AREA)
        if coarse_tpl.shape[0] < 2 or coarse_tpl.shape[1] < 2:
            return None

        coarse_result = self.match_single(img_pyr[-1], coarse_tpl)
        if coarse_result['score'] < 0.3:
            return None

        # 逐层细化
        loc = coarse_result['location']
        for level in range(levels - 1, -1, -1):
            loc = (loc[0] * 2, loc[1] * 2)
            # 在当前层搜索附近区域
            search_margin = 5
            x, y = loc
            h, w = template.shape[:2] if level == 0 else (template.shape[0] // (2 ** level),
                                                           template.shape[1] // (2 ** level))
            img = img_pyr[level]
            x1 = max(0, x - search_margin)
            y1 = max(0, y - search_margin)
            x2 = min(img.shape[1], x + w + search_margin)
            y2 = min(img.shape[0], y + h + search_margin)

            if level == 0:
                tpl_use = template
            else:
                tpl_use = cv2.resize(template, (w, h), interpolation=cv2.INTER_AREA)

            if x2 - x1 < tpl_use.shape[1] or y2 - y1 < tpl_use.shape[0]:
                continue

            roi = img[y1:y2, x1:x2]
            local_result = self.match_single(roi, tpl_use)
            loc = (x1 + local_result['location'][0], y1 + local_result['location'][1])

        # 在原图上精确匹配
        final = self.match_single(image, template)
        return final

    def match_rotation(self, image: np.ndarray, template: np.ndarray,
                       angle_range: Tuple[float, float] = (0, 360),
                       angle_step: float = 5.0,
                       scale_range: Tuple[float, float] = (0.8, 1.2),
                       scale_steps: int = 5) -> List[Dict]:
        """
        多角度+多尺度模板匹配
        Args:
            image: 搜索图像
            template: 模板图像
            angle_range: 角度范围
            angle_step: 角度步长
            scale_range: 缩放范围
            scale_steps: 缩放步数
        Returns:
            list[dict]: 匹配结果, 按score降序
        """
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY) if len(template.shape) == 3 else template

        results = []
        angles = np.arange(angle_range[0], angle_range[1], angle_step)
        scales = np.linspace(scale_range[0], scale_range[1], scale_steps)

        for scale in scales:
            new_w = int(tpl_gray.shape[1] * scale)
            new_h = int(tpl_gray.shape[0] * scale)
            if new_w < 1 or new_h < 1:
                continue
            scaled = cv2.resize(tpl_gray, (new_w, new_h))

            for angle in angles:
                rotated = self._rotate_image(scaled, angle)
                if rotated is None:
                    continue
                rh, rw = rotated.shape[:2]
                if rw > img_gray.shape[1] or rh > img_gray.shape[0]:
                    continue

                match = self.match_single(img_gray, rotated)
                match['angle'] = angle
                match['scale'] = scale
                match['rotated_size'] = (rw, rh)
                results.append(match)

        results.sort(key=lambda x: x['score'], reverse=True)
        return results

    def match_multi_template(self, image: np.ndarray,
                             templates: Dict[str, np.ndarray],
                             threshold: float = 0.8) -> List[Dict]:
        """
        多模板匹配
        Args:
            image: 搜索图像
            templates: {name: template_image} 字典
            threshold: 匹配阈值
        Returns:
            list[dict]: 所有超过阈值的匹配结果
        """
        all_results = []
        for name, tpl in templates.items():
            result = self.match_single(image, tpl)
            if result['score'] >= threshold:
                result['template_name'] = name
                all_results.append(result)

        all_results.sort(key=lambda x: x['score'], reverse=True)
        return all_results

    def match_multi_location(self, image: np.ndarray, template: np.ndarray,
                             threshold: float = 0.8,
                             nms_dist: int = 20) -> List[Dict]:
        """
        多位置匹配 (同一模板在图中多次出现)
        Args:
            image: 搜索图像
            template: 模板
            threshold: 匹配阈值
            nms_dist: NMS抑制距离
        Returns:
            list[dict]: 多个匹配位置
        """
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY) if len(template.shape) == 3 else template

        result = cv2.matchTemplate(img_gray, tpl_gray, self.method)
        h, w = tpl_gray.shape[:2]

        if self.is_sqdiff:
            locs = np.where(result <= (1.0 - threshold))
        else:
            locs = np.where(result >= threshold)

        # 提取所有匹配位置
        matches = []
        for pt in zip(*locs[::-1]):
            score = float(result[pt[1], pt[0]])
            if self.is_sqdiff:
                score = 1.0 - score
            matches.append({
                'location': pt,
                'score': score,
                'top_left': pt,
                'bottom_right': (pt[0] + w, pt[1] + h),
                'size': (w, h)
            })

        # NMS过滤
        matches = self._nms(matches, nms_dist)
        matches.sort(key=lambda x: x['score'], reverse=True)
        return matches

    @staticmethod
    def _rotate_image(image: np.ndarray, angle: float) -> Optional[np.ndarray]:
        """旋转图像, 自动裁剪到最小包围矩形"""
        h, w = image.shape[:2]
        center = (w / 2, h / 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)

        # 计算旋转后尺寸
        cos = abs(M[0, 0])
        sin = abs(M[0, 1])
        new_w = int(h * sin + w * cos)
        new_h = int(h * cos + w * sin)

        M[0, 2] += (new_w - w) / 2
        M[1, 2] += (new_h - h) / 2

        rotated = cv2.warpAffine(image, M, (new_w, new_h),
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT,
                                 borderValue=0)
        return rotated

    @staticmethod
    def _nms(matches: List[Dict], dist: int) -> List[Dict]:
        """非极大值抑制"""
        if not matches:
            return []

        matches.sort(key=lambda x: x['score'], reverse=True)
        keep = []
        suppressed = set()

        for i, m in enumerate(matches):
            if i in suppressed:
                continue
            keep.append(m)
            for j in range(i + 1, len(matches)):
                if j in suppressed:
                    continue
                dx = m['location'][0] - matches[j]['location'][0]
                dy = m['location'][1] - matches[j]['location'][1]
                if (dx * dx + dy * dy) < dist * dist:
                    suppressed.add(j)

        return keep

    @staticmethod
    def draw_results(image: np.ndarray, results: List[Dict],
                     max_draw: int = 10) -> np.ndarray:
        """可视化匹配结果"""
        vis = image.copy()
        for i, r in enumerate(results[:max_draw]):
            tl = r['top_left']
            br = r['bottom_right']
            color = (0, 255, 0) if i == 0 else (0, 200, 255)
            cv2.rectangle(vis, tl, br, color, 2)
            label = f"{r['score']:.3f}"
            if 'template_name' in r:
                label = f"{r['template_name']}: {r['score']:.3f}"
            if 'scale' in r:
                label += f" s={r['scale']:.2f}"
            if 'angle' in r:
                label += f" a={r['angle']:.0f}"
            cv2.putText(vis, label, (tl[0], tl[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return vis


# ==================== 快捷函数 ====================

def find_template(image: np.ndarray, template: np.ndarray,
                  threshold: float = 0.8) -> Optional[Tuple[int, int, float]]:
    """快速模板匹配, 返回 (x, y, score) 或 None"""
    matcher = TemplateMatcher()
    result = matcher.match_single(image, template)
    if result['score'] >= threshold:
        loc = result['location']
        return (loc[0], loc[1], result['score'])
    return None


def find_template_multiscale(image: np.ndarray, template: np.ndarray,
                              threshold: float = 0.7,
                              scale_range: Tuple[float, float] = (0.5, 2.0)) -> List[Dict]:
    """快速多尺度模板匹配"""
    matcher = TemplateMatcher()
    results = matcher.match_multi_scale(image, template, scale_range=scale_range)
    return [r for r in results if r['score'] >= threshold]


# ==================== 示例与测试 ====================

if __name__ == '__main__':
    # 创建测试图像
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.rectangle(img, (100, 100), (200, 200), (255, 255, 255), -1)
    cv2.circle(img, (400, 300), 50, (0, 255, 0), -1)
    cv2.rectangle(img, (450, 50), (550, 150), (0, 0, 255), -1)

    # 提取模板
    template = img[100:200, 100:200].copy()

    matcher = TemplateMatcher()

    # 单尺度匹配
    result = matcher.match_single(img, template)
    print(f"单尺度匹配: location={result['location']}, score={result['score']:.4f}")

    # 多尺度匹配
    results = matcher.match_multi_scale(img, template, scale_range=(0.5, 1.5), scale_steps=10)
    print(f"多尺度匹配: 找到 {len(results)} 个结果")
    for r in results[:3]:
        print(f"  scale={r['scale']:.2f}, score={r['score']:.4f}")

    # 多角度匹配
    results = matcher.match_rotation(img, template, angle_range=(0, 360), angle_step=30)
    print(f"多角度匹配: 找到 {len(results)} 个结果")
    for r in results[:3]:
        print(f"  angle={r['angle']:.0f}, scale={r['scale']:.2f}, score={r['score']:.4f}")

    # 多位置匹配
    img2 = img.copy()
    cv2.rectangle(img2, (300, 250), (400, 350), (255, 255, 255), -1)
    results = matcher.match_multi_location(img2, template, threshold=0.8)
    print(f"多位置匹配: 找到 {len(results)} 个结果")

    # 可视化
    vis = matcher.draw_results(img, [result])
    cv2.imshow("Template Match", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("\n模板匹配模块测试完成!")
