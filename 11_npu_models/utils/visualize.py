"""
检测结果可视化工具
画检测框 / 标签 / 置信度 / 关键点 / 分类结果
"""
import numpy as np
import cv2


# 预定义颜色表 (80类COCO)
_COLORS = [
    (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255),
    (49, 210, 207), (10, 249, 72), (23, 204, 146), (134, 219, 61),
    (52, 147, 26), (187, 212, 0), (168, 153, 44), (255, 194, 0),
    (147, 69, 52), (255, 115, 100), (236, 24, 0), (255, 56, 132),
    (133, 0, 82), (203, 56, 255), (111, 25, 255), (0, 52, 255),
]


def get_color(class_id):
    """获取类别对应颜色"""
    return _COLORS[class_id % len(_COLORS)]


def draw_box(img, x1, y1, x2, y2, color=(0, 255, 0), thickness=2):
    """画矩形框"""
    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
    return img


def draw_label(img, text, x, y, color=(0, 255, 0), font_scale=0.5, thickness=1):
    """画标签文字(带背景)"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    # 背景框
    cv2.rectangle(img, (int(x), int(y) - th - 6), (int(x) + tw + 4, int(y)), color, -1)
    # 文字
    cv2.putText(img, text, (int(x) + 2, int(y) - 3), font, font_scale, (0, 0, 0), thickness)

    return img


def draw_detection(img, box, score, class_id, class_names=None):
    """
    画单个检测结果: 框 + 标签(类别+置信度)
    Args:
        img: 图像
        box: [x1, y1, x2, y2]
        score: 置信度
        class_id: 类别ID
        class_names: 类别名称列表
    """
    x1, y1, x2, y2 = [int(v) for v in box]
    color = get_color(class_id)

    # 画框
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

    # 标签
    if class_names and 0 <= class_id < len(class_names):
        label = f"{class_names[class_id]} {score:.2f}"
    else:
        label = f"class{class_id} {score:.2f}"

    draw_label(img, label, x1, y1, color)

    return img


def draw_detections(img, boxes, scores, class_ids, class_names=None, 
                    show_conf=True, min_score=0.0):
    """
    批量画检测结果
    Args:
        img: 图像 (会原地修改)
        boxes: [[x1,y1,x2,y2], ...]
        scores: [score, ...]
        class_ids: [id, ...]
        class_names: 类别名称列表
        show_conf: 是否显示置信度
        min_score: 最小显示置信度
    Returns:
        绘制后的图像
    """
    img_out = img.copy()

    for box, score, cid in zip(boxes, scores, class_ids):
        if score < min_score:
            continue
        x1, y1, x2, y2 = [int(v) for v in box]
        color = get_color(cid)

        cv2.rectangle(img_out, (x1, y1), (x2, y2), color, 2)

        if class_names and 0 <= cid < len(class_names):
            name = class_names[cid]
        else:
            name = f"cls{cid}"

        if show_conf:
            label = f"{name} {score:.2f}"
        else:
            label = name

        draw_label(img_out, label, x1, y1, color)

    return img_out


def draw_classification(img, class_name, prob, top_k=3, top_probs=None, top_names=None):
    """
    画分类结果
    Args:
        img: 图像
        class_name: 最可能的类别名
        prob: 概率
        top_k: 显示top-k结果
        top_probs: top-k概率列表
        top_names: top-k类别名列表
    """
    img_out = img.copy()
    y_offset = 30

    # 主结果
    text = f"{class_name}: {prob:.2%}"
    cv2.putText(img_out, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 0), 2)
    y_offset += 35

    # Top-K
    if top_probs and top_names:
        for i in range(min(top_k, len(top_probs))):
            text = f"  {top_names[i]}: {top_probs[i]:.2%}"
            cv2.putText(img_out, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (200, 200, 200), 1)
            y_offset += 25

    return img_out


def draw_fps(img, fps, pos=(10, 10), color=(0, 255, 0)):
    """画FPS信息"""
    text = f"FPS: {fps:.1f}"
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    return img


def draw_count(img, count, label="Objects", pos=(10, 30)):
    """画检测数量"""
    text = f"{label}: {count}"
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return img


def draw_class_stats(img, class_ids, class_names=None):
    """画各类别统计"""
    img_out = img.copy()
    unique, counts = np.unique(class_ids, return_counts=True)
    y_offset = 60

    for cid, cnt in zip(unique, counts):
        name = class_names[cid] if class_names and cid < len(class_names) else f"cls{cid}"
        text = f"{name}: {cnt}"
        color = get_color(cid)
        cv2.putText(img_out, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 1)
        y_offset += 25

    return img_out


def create_result_panel(img, boxes, scores, class_ids, class_names=None,
                        fps=None, width=300):
    """创建带侧边栏的结果面板"""
    h, w = img.shape[:2]

    # 创建面板
    panel = np.zeros((h, width, 3), dtype=np.uint8)

    # 图像区
    img_vis = draw_detections(img.copy(), boxes, scores, class_ids, class_names)

    # 侧边栏信息
    y = 30
    if fps is not None:
        cv2.putText(panel, f"FPS: {fps:.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 1)
        y += 30

    cv2.putText(panel, f"Detected: {len(boxes)}", (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    y += 35

    # 各检测结果
    for i, (box, score, cid) in enumerate(zip(boxes, scores, class_ids)):
        if y > h - 20:
            break
        name = class_names[cid] if class_names and cid < len(class_names) else f"cls{cid}"
        color = get_color(cid)
        text = f"{name} {score:.2f}"
        cv2.putText(panel, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        y += 22

    # 拼接
    result = np.concatenate([img_vis, panel], axis=1)
    return result
