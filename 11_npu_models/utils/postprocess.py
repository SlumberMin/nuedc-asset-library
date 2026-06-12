"""
后处理工具集
NMS / 检测框解码 / 置信度过滤 / Softmax / TopK
"""
import numpy as np


def nms(boxes, scores, iou_threshold=0.45):
    """
    非极大值抑制
    Args:
        boxes: (N, 4) [x1, y1, x2, y2]
        scores: (N,) 置信度分数
        iou_threshold: IoU阈值
    Returns:
        keep_indices: 保留的索引
    """
    if len(boxes) == 0:
        return np.array([], dtype=np.int32)

    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return np.array(keep, dtype=np.int32)


def multiclass_nms(boxes, scores, class_ids, iou_threshold=0.45, max_dets=300):
    """
    多类别NMS: 按类别分别做NMS
    Args:
        boxes: (N, 4)
        scores: (N,) 
        class_ids: (N,) 类别ID
    Returns:
        keep_indices
    """
    unique_classes = np.unique(class_ids)
    keep_all = []

    for cls_id in unique_classes:
        mask = class_ids == cls_id
        cls_boxes = boxes[mask]
        cls_scores = scores[mask]
        cls_keep = nms(cls_boxes, cls_scores, iou_threshold)
        orig_indices = np.where(mask)[0]
        keep_all.extend(orig_indices[cls_keep].tolist())

    if len(keep_all) > max_dets:
        sorted_by_score = sorted(keep_all, key=lambda i: scores[i], reverse=True)
        keep_all = sorted_by_score[:max_dets]

    return np.array(keep_all, dtype=np.int32)


def xywh_to_xyxy(boxes):
    """xywh -> xyxy"""
    result = np.zeros_like(boxes)
    result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return result


def xyxy_to_xywh(boxes):
    """xyxy -> xywh"""
    result = np.zeros_like(boxes)
    result[:, 0] = (boxes[:, 0] + boxes[:, 2]) / 2
    result[:, 1] = (boxes[:, 1] + boxes[:, 3]) / 2
    result[:, 2] = boxes[:, 2] - boxes[:, 0]
    result[:, 3] = boxes[:, 3] - boxes[:, 1]
    return result


def clip_boxes(boxes, img_shape):
    """将框裁剪到图像范围内"""
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, img_shape[1])
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, img_shape[0])
    return boxes


def scale_boxes(boxes, scale_w, scale_h):
    """缩放框坐标"""
    boxes = boxes.copy().astype(np.float32)
    boxes[:, [0, 2]] *= scale_w
    boxes[:, [1, 3]] *= scale_h
    return boxes


def undo_letterbox(boxes, ratio, pad_info, orig_shape):
    """将letterbox坐标映射回原图坐标"""
    pad_w, pad_h = pad_info
    boxes = boxes.copy().astype(np.float32)
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / ratio
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / ratio
    boxes = clip_boxes(boxes, orig_shape)
    return boxes


def softmax(logits):
    """数值稳定的softmax"""
    exp_out = np.exp(logits - np.max(logits, axis=-1, keepdims=True))
    return exp_out / np.sum(exp_out, axis=-1, keepdims=True)


def topk(logits, k=5):
    """返回Top-K索引和概率"""
    probs = softmax(logits)
    if probs.ndim > 1:
        probs = probs.flatten()
    top_indices = np.argsort(probs)[::-1][:k]
    return top_indices, probs[top_indices]


def filter_by_confidence(scores, class_ids, conf_thres=0.25):
    """按置信度过滤"""
    mask = scores > conf_thres
    return scores[mask], class_ids[mask], mask


def decode_yolov8(output, conf_thres=0.25, nms_thres=0.45):
    """
    YOLOv8输出解码
    Args:
        output: (1, 84, 8400) 或 (84, 8400)
    Returns:
        boxes, scores, class_ids (已过滤+NMS)
    """
    if output.ndim == 3:
        output = output[0]
    if output.shape[0] < output.shape[1]:
        output = output.T  # -> (8400, 84)

    boxes = xywh_to_xyxy(output[:, :4])
    cls_scores = output[:, 4:]
    class_ids = np.argmax(cls_scores, axis=1)
    max_scores = np.max(cls_scores, axis=1)

    mask = max_scores > conf_thres
    boxes, max_scores, class_ids = boxes[mask], max_scores[mask], class_ids[mask]

    if len(boxes) == 0:
        return [], [], []

    keep = nms(boxes, max_scores, nms_thres)
    return boxes[keep], max_scores[keep], class_ids[keep]


def decode_yolov5(output, conf_thres=0.25, nms_thres=0.45):
    """
    YOLOv5输出解码
    Args:
        output: (1, 25200, 85) 或 (25200, 85)
    """
    if output.ndim == 3:
        output = output[0]

    obj_conf = output[:, 4]
    cls_scores = output[:, 5:]
    class_ids = np.argmax(cls_scores, axis=1)
    scores = obj_conf * np.max(cls_scores, axis=1)

    mask = scores > conf_thres
    boxes = xywh_to_xyxy(output[mask, :4])
    scores = scores[mask]
    class_ids = class_ids[mask]

    if len(boxes) == 0:
        return [], [], []

    keep = nms(boxes, scores, nms_thres)
    return boxes[keep], scores[keep], class_ids[keep]
