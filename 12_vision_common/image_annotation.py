"""
图像标注工具 - 画框/画圆/画线/文字/箭头
"""
import cv2
import numpy as np


def draw_rect(img, pt1, pt2, color=(0, 255, 0), thickness=2):
    """画矩形框
    Args:
        pt1: 左上角 (x, y)
        pt2: 右下角 (x, y)
    Returns:
        标注后图像
    """
    out = img.copy()
    cv2.rectangle(out, tuple(pt1), tuple(pt2), color, thickness)
    return out


def draw_rotated_rect(img, center, size, angle, color=(0, 255, 0), thickness=2):
    """画旋转矩形"""
    out = img.copy()
    box = cv2.boxPoints((center, size, angle))
    box = np.int32(box)
    cv2.drawContours(out, [box], 0, color, thickness)
    return out


def draw_circle(img, center, radius, color=(0, 255, 0), thickness=2):
    """画圆
    Args:
        center: 圆心 (x, y)
        radius: 半径
    Returns:
        标注后图像
    """
    out = img.copy()
    cv2.circle(out, tuple(center), radius, color, thickness)
    return out


def draw_ellipse(img, center, axes, angle, start_angle, end_angle,
                 color=(0, 255, 0), thickness=2):
    """画椭圆"""
    out = img.copy()
    cv2.ellipse(out, tuple(center), tuple(axes), angle,
                start_angle, end_angle, color, thickness)
    return out


def draw_line(img, pt1, pt2, color=(0, 255, 0), thickness=2):
    """画直线
    Args:
        pt1: 起点 (x, y)
        pt2: 终点 (x, y)
    Returns:
        标注后图像
    """
    out = img.copy()
    cv2.line(out, tuple(pt1), tuple(pt2), color, thickness)
    return out


def draw_arrow(img, pt1, pt2, color=(0, 255, 0), thickness=2, tip_length=0.1):
    """画箭头
    Args:
        pt1: 起点
        pt2: 终点
        tip_length: 箭头长度与线段长度比
    Returns:
        标注后图像
    """
    out = img.copy()
    cv2.arrowedLine(out, tuple(pt1), tuple(pt2), color, thickness,
                    tipLength=tip_length)
    return out


def draw_text(img, text, org, font_scale=1.0, color=(0, 255, 0),
              thickness=2, font=cv2.FONT_HERSHEY_SIMPLEX, bg_color=None):
    """画文字 (可选背景色)
    Args:
        text: 文字内容
        org: 文字左下角 (x, y)
        bg_color: 背景色 None则不画背景
    Returns:
        标注后图像
    """
    out = img.copy()
    if bg_color is not None:
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = org
        cv2.rectangle(out, (x, y + baseline), (x + tw, y - th), bg_color, -1)
    cv2.putText(out, text, tuple(org), font, font_scale, color, thickness)
    return out


def draw_crosshair(img, center, size=20, color=(0, 255, 0), thickness=1):
    """画十字准星"""
    out = img.copy()
    x, y = center
    cv2.line(out, (x - size, y), (x + size, y), color, thickness)
    cv2.line(out, (x, y - size), (x, y + size), color, thickness)
    return out


def draw_contour(img, contour, color=(0, 255, 0), thickness=2):
    """画轮廓"""
    out = img.copy()
    cv2.drawContours(out, [np.int32(contour)], -1, color, thickness)
    return out


def draw_bboxes(img, bboxes, labels=None, color=(0, 255, 0), thickness=2):
    """批量画多个框
    Args:
        bboxes: [(x1,y1,x2,y2), ...]
        labels: 框标签列表, None不画标签
    Returns:
        标注后图像
    """
    out = img.copy()
    for i, bbox in enumerate(bboxes):
        x1, y1, x2, y2 = [int(v) for v in bbox]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)
        if labels is not None and i < len(labels):
            cv2.putText(out, str(labels[i]), (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)
    return out


def draw_keypoints(img, kps, color=(0, 0, 255), radius=3):
    """画关键点"""
    out = img.copy()
    for kp in kps:
        x, y = int(kp[0]), int(kp[1])
        cv2.circle(out, (x, y), radius, color, -1)
    return out


if __name__ == '__main__':
    img = np.zeros((400, 600, 3), np.uint8)
    img = draw_rect(img, (50, 50), (200, 150))
    img = draw_circle(img, (300, 200), 50)
    img = draw_line(img, (10, 350), (590, 350))
    img = draw_arrow(img, (300, 100), (500, 50))
    img = draw_text(img, 'Test', (200, 300), bg_color=(0, 0, 128))
    img = draw_crosshair(img, (300, 200))
    cv2.imshow('annotation', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
