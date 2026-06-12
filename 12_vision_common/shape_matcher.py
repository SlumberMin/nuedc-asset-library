"""
形状匹配模块 - Hu矩匹配 + 轮廓匹配 + 模板匹配
====================================================
功能:
  - Hu矩匹配 (旋转/缩放/平移不变)
  - 轮廓形状匹配 (cv2.matchShapes)
  - 模板匹配 (多种方法)
  - 多模板批量匹配
  - 旋转不变模板匹配
  - 形状分类 (圆/矩形/三角形/多边形)

适用场景:
  - 识别已知形状的目标物
  - 零件/标志/图案识别
  - 分拣系统中的形状分类
  - 电赛中已知形状目标的匹配

用法:
  sm = ShapeMatcher()
  score = sm.hu_match(contour1, contour2)
  result = sm.template_match(img, template, method='ccoeff')
  shape = sm.classify_shape(contour)
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict


class ShapeMatcher:
    """形状匹配器"""

    # ──────────────── Hu矩匹配 ────────────────
    @staticmethod
    def hu_moments(img_or_contour) -> np.ndarray:
        """
        计算Hu矩 (7个不变矩)。
        输入: 灰度图 或 单个轮廓
        """
        if isinstance(img_or_contour, np.ndarray) and img_or_contour.ndim == 2:
            # 灰度图
            moments = cv2.moments(img_or_contour)
        else:
            # 轮廓
            moments = cv2.moments(img_or_contour)
        hu = cv2.HuMoments(moments).flatten()
        # 取对数使数值更易比较
        for i in range(len(hu)):
            if hu[i] != 0:
                hu[i] = -np.sign(hu[i]) * np.log10(abs(hu[i]))
            else:
                hu[i] = 0
        return hu

    @staticmethod
    def hu_match(contour1, contour2, method: int = cv2.CONTOURS_MATCH_I1) -> float:
        """
        Hu矩匹配得分。
        method: I1(默认)/I2/I3, 值越小越相似
        返回: 匹配距离 (0=完全相同, 越大越不同)
        """
        return cv2.matchShapes(contour1, contour2, method, 0)

    @staticmethod
    def hu_match_batch(target, candidates: list, method: int = cv2.CONTOURS_MATCH_I1) -> list:
        """
        批量Hu矩匹配, 返回按相似度排序的结果。
        返回: [(index, score, contour), ...] 按score升序
        """
        results = []
        for i, c in enumerate(candidates):
            score = cv2.matchShapes(target, c, method, 0)
            results.append((i, score, c))
        results.sort(key=lambda x: x[1])
        return results

    # ──────────────── 轮廓匹配 ────────────────
    @staticmethod
    def contour_match(contour1, contour2, method: int = cv2.CONTOURS_MATCH_I1) -> float:
        """
        轮廓形状匹配 (等价于Hu矩匹配, 底层都是Hu矩)。
        """
        return cv2.matchShapes(contour1, contour2, method, 0)

    @staticmethod
    def contour_match_template(contour, templates: dict) -> Tuple[str, float]:
        """
        与多个模板轮廓匹配, 返回最佳匹配。
        templates: {'name': contour, ...}
        返回: (最佳名称, 得分)
        """
        best_name = None
        best_score = float('inf')
        for name, tmpl in templates.items():
            score = cv2.matchShapes(contour, tmpl, cv2.CONTOURS_MATCH_I1, 0)
            if score < best_score:
                best_score = score
                best_name = name
        return best_name, best_score

    # ──────────────── 模板匹配 ────────────────
    @staticmethod
    def template_match(img: np.ndarray, template: np.ndarray,
                       method: str = 'ccoeff', multi: bool = False,
                       threshold: float = 0.8) -> dict:
        """
        模板匹配。
        method: 'sqdiff' | 'sqdiff_normed' | 'ccorr' | 'ccorr_normed' | 'ccoeff' | 'ccoeff_normed'
        multi: 是否返回多个匹配位置
        返回: {'locations': [(x,y), ...], 'scores': [...], 'max_val': float, 'max_loc': (x,y)}
        """
        methods = {
            'sqdiff': cv2.TM_SQDIFF,
            'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
            'ccorr': cv2.TM_CCORR,
            'ccorr_normed': cv2.TM_CCORR_NORMED,
            'ccoeff': cv2.TM_CCOEFF,
            'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
        }
        cv_method = methods.get(method, cv2.TM_CCOEFF_NORMED)

        # 确保灰度
        if img.ndim == 3:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img_gray = img
        if template.ndim == 3:
            tmpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        else:
            tmpl_gray = template

        result = cv2.matchTemplate(img_gray, tmpl_gray, cv_method)

        if method in ('sqdiff', 'sqdiff_normed'):
            # 越小越好
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            best_val = min_val
            best_loc = min_loc
        else:
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            best_val = max_val
            best_loc = max_loc

        output = {
            'max_val': best_val,
            'max_loc': best_loc,
            'locations': [best_loc],
            'scores': [best_val],
            'template_size': (tmpl_gray.shape[1], tmpl_gray.shape[0]),
        }

        if multi:
            # 多目标匹配
            if method in ('sqdiff', 'sqdiff_normed'):
                locs = np.where(result <= 1.0 - threshold)
            else:
                locs = np.where(result >= threshold)
            locations = []
            scores = []
            th, tw = tmpl_gray.shape[:2]
            for pt in zip(*locs[::-1]):
                locations.append(pt)
                scores.append(result[pt[1], pt[0]])
            output['locations'] = locations
            output['scores'] = scores

        return output

    # ──────────────── 旋转不变模板匹配 ────────────────
    @staticmethod
    def rotate_template_match(img: np.ndarray, template: np.ndarray,
                              angle_range: Tuple[int, int] = (-180, 180),
                              angle_step: int = 5,
                              scale_range: Tuple[float, float] = (0.8, 1.2),
                              scale_step: float = 0.1,
                              method: str = 'ccoeff_normed') -> dict:
        """
        旋转+缩放不变的模板匹配。
        遍历不同角度和缩放比例, 找到最佳匹配。
        """
        methods = {
            'sqdiff': cv2.TM_SQDIFF,
            'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
            'ccorr': cv2.TM_CCORR,
            'ccorr_normed': cv2.TM_CCORR_NORMED,
            'ccoeff': cv2.TM_CCOEFF,
            'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
        }
        cv_method = methods.get(method, cv2.TM_CCOEFF_NORMED)
        is_sqdiff = method in ('sqdiff', 'sqdiff_normed')

        if img.ndim == 3:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img_gray = img
        if template.ndim == 3:
            tmpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        else:
            tmpl_gray = template

        best_score = float('inf') if is_sqdiff else float('-inf')
        best_loc = None
        best_angle = 0
        best_scale = 1.0

        h_t, w_t = tmpl_gray.shape[:2]
        scales = np.arange(scale_range[0], scale_range[1] + scale_step, scale_step)
        angles = np.arange(angle_range[0], angle_range[1] + angle_step, angle_step)

        for scale in scales:
            new_w = int(w_t * scale)
            new_h = int(h_t * scale)
            if new_w < 2 or new_h < 2:
                continue
            if new_w > img_gray.shape[1] or new_h > img_gray.shape[0]:
                continue
            scaled = cv2.resize(tmpl_gray, (new_w, new_h))

            for angle in angles:
                # 旋转模板
                center = (new_w // 2, new_h // 2)
                M = cv2.getRotationMatrix2D(center, angle, 1.0)
                cos_a = abs(M[0, 0])
                sin_a = abs(M[0, 1])
                rw = int(new_h * sin_a + new_w * cos_a)
                rh = int(new_h * cos_a + new_w * sin_a)
                M[0, 2] += rw / 2 - center[0]
                M[1, 2] += rh / 2 - center[1]
                rotated = cv2.warpAffine(scaled, M, (rw, rh))

                if rw > img_gray.shape[1] or rh > img_gray.shape[0]:
                    continue

                result = cv2.matchTemplate(img_gray, rotated, cv_method)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)

                if is_sqdiff:
                    # sqdiff: min_val is at index 0
                    _, min_val, _, min_loc = cv2.minMaxLoc(result)
                    if min_val < best_score:
                        best_score = min_val
                        best_loc = min_loc
                        best_angle = angle
                        best_scale = scale
                else:
                    if max_val > best_score:
                        best_score = max_val
                        best_loc = max_loc
                        best_angle = angle
                        best_scale = scale

        return {
            'max_val': best_score,
            'max_loc': best_loc,
            'best_angle': best_angle,
            'best_scale': best_scale,
            'template_size': (int(w_t * best_scale), int(h_t * best_scale)),
        }

    # ──────────────── 形状分类 ────────────────
    @staticmethod
    def classify_shape(contour, epsilon_factor: float = 0.04) -> str:
        """
        对轮廓进行形状分类。
        返回: 'triangle' | 'square' | 'rectangle' | 'pentagon' |
               'hexagon' | 'circle' | 'ellipse' | 'unknown'
        """
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon_factor * peri, True)
        vertices = len(approx)
        area = cv2.contourArea(contour)

        # 圆度
        circularity = 4 * np.pi * area / (peri ** 2) if peri > 0 else 0

        if vertices == 3:
            return 'triangle'
        elif vertices == 4:
            # 判断是正方形还是矩形
            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / h if h > 0 else 0
            if 0.85 <= aspect <= 1.15:
                return 'square'
            else:
                return 'rectangle'
        elif vertices == 5:
            return 'pentagon'
        elif vertices == 6:
            return 'hexagon'
        elif vertices > 6:
            if circularity > 0.85:
                return 'circle'
            elif len(contour) >= 5:
                return 'ellipse'
        return 'unknown'

    @staticmethod
    def classify_shapes(contours: list) -> List[Tuple[str, float]]:
        """
        批量形状分类。
        返回: [(shape_name, circularity), ...]
        """
        results = []
        for c in contours:
            shape = ShapeMatcher.classify_shape(c)
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            circ = 4 * np.pi * area / (peri ** 2) if peri > 0 else 0
            results.append((shape, circ))
        return results

    # ──────────────── 特征点匹配 (ORB) ────────────────
    @staticmethod
    def orb_match(img1: np.ndarray, img2: np.ndarray,
                  n_features: int = 500, match_ratio: float = 0.75) -> dict:
        """
        ORB特征点匹配, 适合纹理丰富的形状匹配。
        返回匹配结果和变换矩阵。
        """
        orb = cv2.ORB_create(nfeatures=n_features)
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        if des1 is None or des2 is None or len(kp1) < 2 or len(kp2) < 2:
            return {'good_matches': [], 'homography': None, 'match_count': 0}

        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bf.knnMatch(des1, des2, k=2)

        # Lowe's ratio test
        good = []
        for pair in matches:
            if len(pair) == 2:
                m, n = pair
                if m.distance < match_ratio * n.distance:
                    good.append(m)

        # 估计变换矩阵
        H = None
        if len(good) >= 4:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        return {
            'good_matches': good,
            'homography': H,
            'match_count': len(good),
            'kp1': kp1,
            'kp2': kp2,
        }

    # ──────────────── 可视化 ────────────────
    @staticmethod
    def draw_match_result(img: np.ndarray, match_result: dict,
                          color: tuple = (0, 255, 0)) -> np.ndarray:
        """绘制模板匹配结果"""
        vis = img.copy()
        loc = match_result['max_loc']
        tw, th = match_result['template_size']
        cv2.rectangle(vis, loc, (loc[0] + tw, loc[1] + th), color, 2)
        score = match_result['max_val']
        cv2.putText(vis, f"{score:.3f}", (loc[0], loc[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return vis

    @staticmethod
    def draw_classified_shapes(img: np.ndarray, contours: list) -> np.ndarray:
        """绘制形状分类结果"""
        vis = img.copy()
        classifications = ShapeMatcher.classify_shapes(contours)
        color_map = {
            'triangle': (0, 0, 255), 'square': (0, 255, 0),
            'rectangle': (0, 255, 255), 'pentagon': (255, 0, 0),
            'hexagon': (255, 255, 0), 'circle': (255, 0, 255),
            'ellipse': (128, 255, 128), 'unknown': (128, 128, 128),
        }
        for c, (shape, circ) in zip(contours, classifications):
            color = color_map.get(shape, (200, 200, 200))
            cv2.drawContours(vis, [c], -1, color, 2)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.putText(vis, f"{shape} ({circ:.2f})", (cx - 30, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        return vis


# ──────────────── Demo ────────────────
if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("用法: python shape_matcher.py <image> [template]")
        sys.exit(1)

    img = cv2.imread(sys.argv[1])
    if img is None:
        print(f"无法读取图像: {sys.argv[1]}")
        sys.exit(1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sm = ShapeMatcher()

    # 形状分类
    vis = sm.draw_classified_shapes(img, contours)
    print(f"检测到 {len(contours)} 个轮廓")
    for i, (shape, circ) in enumerate(sm.classify_shapes(contours)):
        print(f"  轮廓{i}: {shape}, 圆度={circ:.3f}")

    # 如果提供了模板, 做Hu矩匹配
    if len(sys.argv) >= 3:
        tmpl = cv2.imread(sys.argv[2], cv2.IMREAD_GRAYSCALE)
        if tmpl is not None:
            _, tbin = cv2.threshold(tmpl, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            tcontours, _ = cv2.findContours(tbin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if tcontours:
                target = max(tcontours, key=cv2.contourArea)
                results = sm.hu_match_batch(target, contours)
                print(f"\nHu矩匹配结果 (vs 模板):")
                for idx, score, _ in results[:5]:
                    print(f"  轮廓{idx}: 得分={score:.4f}")

    cv2.imshow('Shape Classification', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
