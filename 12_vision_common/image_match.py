"""
模板匹配通用工具库
支持: 单尺度 / 多尺度 / 旋转不变 / 多目标匹配
"""
import cv2
import numpy as np


def match_template(img, template, method='ccoeff_normed'):
    """单尺度模板匹配
    method: sqdiff, sqdiff_normed, ccorr, ccorr_normed, ccoeff, ccoeff_normed
    """
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    if len(template.shape) == 3:
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    else:
        tpl_gray = template.copy()
    method_map = {
        'sqdiff': cv2.TM_SQDIFF, 'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
        'ccorr': cv2.TM_CCORR, 'ccorr_normed': cv2.TM_CCORR_NORMED,
        'ccoeff': cv2.TM_CCOEFF, 'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
    }
    m = method_map.get(method, cv2.TM_CCOEFF_NORMED)
    result = cv2.matchTemplate(gray, tpl_gray, m)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    if m in (cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED):
        top_left = min_loc
        score = 1 - min_val
    else:
        top_left = max_loc
        score = max_val
    h, w = tpl_gray.shape[:2]
    bottom_right = (top_left[0] + w, top_left[1] + h)
    return top_left, bottom_right, score, result


def match_template_visual(img, template, method='ccoeff_normed', color=(0, 255, 0)):
    """单尺度模板匹配可视化"""
    top_left, bottom_right, score, _ = match_template(img, template, method)
    canvas = img.copy()
    cv2.rectangle(canvas, top_left, bottom_right, color, 2)
    cv2.putText(canvas, f"{score:.2f}", (top_left[0], top_left[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return canvas, top_left, bottom_right, score


def match_template_multiscale(img, template, scale_range=(0.5, 2.0, 0.1),
                                method='ccoeff_normed'):
    """多尺度模板匹配 (搜索不同缩放比例)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    if len(template.shape) == 3:
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    else:
        tpl_gray = template.copy()
    method_map = {
        'sqdiff': cv2.TM_SQDIFF, 'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
        'ccorr': cv2.TM_CCORR, 'ccorr_normed': cv2.TM_CCORR_NORMED,
        'ccoeff': cv2.TM_CCOEFF, 'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
    }
    m = method_map.get(method, cv2.TM_CCOEFF_NORMED)
    best_score = -1 if m not in (cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED) else float('inf')
    best_loc = None
    best_scale = 1.0
    best_size = None
    start, end, step = scale_range
    scales = np.arange(start, end + step / 2, step)
    for scale in scales:
        w = int(tpl_gray.shape[1] * scale)
        h = int(tpl_gray.shape[0] * scale)
        if w < 1 or h < 1 or w > gray.shape[1] or h > gray.shape[0]:
            continue
        resized = cv2.resize(tpl_gray, (w, h))
        result = cv2.matchTemplate(gray, resized, m)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if m in (cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED):
            if min_val < best_score:
                best_score = min_val
                best_loc = min_loc
                best_scale = scale
                best_size = (w, h)
        else:
            if max_val > best_score:
                best_score = max_val
                best_loc = max_loc
                best_scale = scale
                best_size = (w, h)
    if best_loc is None:
        return None, None, 0, 1.0
    tl = best_loc
    br = (tl[0] + best_size[0], tl[1] + best_size[1])
    return tl, br, best_score, best_scale


def match_template_rotation(img, template, angle_range=(0, 360, 10),
                             method='ccoeff_normed'):
    """旋转不变模板匹配 (遍历旋转角度)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    if len(template.shape) == 3:
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    else:
        tpl_gray = template.copy()
    method_map = {
        'sqdiff': cv2.TM_SQDIFF, 'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
        'ccorr': cv2.TM_CCORR, 'ccorr_normed': cv2.TM_CCORR_NORMED,
        'ccoeff': cv2.TM_CCOEFF, 'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
    }
    m = method_map.get(method, cv2.TM_CCOEFF_NORMED)
    is_min = m in (cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED)
    best_score = float('inf') if is_min else -1
    best_loc = None
    best_angle = 0
    best_w, best_h = tpl_gray.shape[1], tpl_gray.shape[0]
    start, end, step = angle_range
    for angle in np.arange(start, end, step):
        h_img, w_img = tpl_gray.shape[:2]
        center = (w_img // 2, h_img // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])
        new_w = int(h_img * sin + w_img * cos)
        new_h = int(h_img * cos + w_img * sin)
        M[0, 2] += (new_w - w_img) / 2
        M[1, 2] += (new_h - h_img) / 2
        rotated = cv2.warpAffine(tpl_gray, M, (new_w, new_h),
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        if rotated.shape[0] > gray.shape[0] or rotated.shape[1] > gray.shape[1]:
            continue
        result = cv2.matchTemplate(gray, rotated, m)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if is_min:
            if min_val < best_score:
                best_score = min_val
                best_loc = min_loc
                best_angle = angle
                best_w, best_h = rotated.shape[1], rotated.shape[0]
        else:
            if max_val > best_score:
                best_score = max_val
                best_loc = max_loc
                best_angle = angle
                best_w, best_h = rotated.shape[1], rotated.shape[0]
    if best_loc is None:
        return None, None, 0, 0
    tl = best_loc
    br = (tl[0] + best_w, tl[1] + best_h)
    return tl, br, best_score, best_angle


def match_template_multi(img, template, threshold=0.8, method='ccoeff_normed',
                          nms_dist=10):
    """多目标模板匹配 (返回所有匹配位置, 含NMS去重)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    if len(template.shape) == 3:
        tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    else:
        tpl_gray = template.copy()
    method_map = {
        'sqdiff': cv2.TM_SQDIFF, 'sqdiff_normed': cv2.TM_SQDIFF_NORMED,
        'ccorr': cv2.TM_CCORR, 'ccorr_normed': cv2.TM_CCORR_NORMED,
        'ccoeff': cv2.TM_CCOEFF, 'ccoeff_normed': cv2.TM_CCOEFF_NORMED,
    }
    m = method_map.get(method, cv2.TM_CCOEFF_NORMED)
    result = cv2.matchTemplate(gray, tpl_gray, m)
    is_min = m in (cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED)
    if is_min:
        locs = np.where(result <= (1 - threshold))
    else:
        locs = np.where(result >= threshold)
    h, w = tpl_gray.shape[:2]
    raw_rects = []
    raw_scores = []
    for pt in zip(*locs[::-1]):
        score = result[pt[1], pt[0]]
        raw_rects.append((pt[0], pt[1], w, h))
        raw_scores.append(1 - score if is_min else score)
    if len(raw_rects) == 0:
        return []
    rects_with_scores = list(zip(raw_rects, raw_scores))
    rects_with_scores.sort(key=lambda x: x[1], reverse=True)
    picked = []
    while rects_with_scores:
        best_rect, best_score = rects_with_scores.pop(0)
        picked.append((best_rect, best_score))
        remaining = []
        for r, s in rects_with_scores:
            dx = abs(r[0] - best_rect[0])
            dy = abs(r[1] - best_rect[1])
            if dx > nms_dist or dy > nms_dist:
                remaining.append((r, s))
        rects_with_scores = remaining
    return picked


def match_template_multi_visual(img, template, threshold=0.8, method='ccoeff_normed',
                                 nms_dist=10, color=(0, 255, 0)):
    """多目标匹配可视化"""
    matches = match_template_multi(img, template, threshold, method, nms_dist)
    canvas = img.copy()
    results = []
    for (x, y, w, h), score in matches:
        cv2.rectangle(canvas, (x, y), (x + w, y + h), color, 2)
        cv2.putText(canvas, f"{score:.2f}", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        results.append(((x, y, x + w, y + h), score))
    return canvas, results


def normalize_brightness(img, template):
    """亮度归一化 (减少光照差异对匹配的影响)"""
    img_n = img.astype(np.float32)
    tpl_n = template.astype(np.float32)
    img_n = (img_n - img_n.mean()) / (img_n.std() + 1e-6)
    tpl_n = (tpl_n - tpl_n.mean()) / (tpl_n.std() + 1e-6)
    return (img_n * 64 + 128).clip(0, 255).astype(np.uint8), \
           (tpl_n * 64 + 128).clip(0, 255).astype(np.uint8)
