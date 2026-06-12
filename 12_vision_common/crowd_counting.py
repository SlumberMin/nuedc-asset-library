"""
人群计数模块 - 密度图估计 + 回归计数 + 多尺度检测
=================================================
功能：
  1. 基于前景检测的人群计数
  2. 头部检测的回归计数
  3. 密度图估计（简化版）
  4. 多尺度检测融合

依赖：opencv-python, numpy
"""

import cv2
import numpy as np


class CrowdCounter:
    """
    人群计数器
    组合多种方法进行人群计数
    """

    def __init__(self, method='hybrid'):
        """
        初始化人群计数器

        参数:
            method: 计数方法 'foreground'(前景检测) / 'head'(头部检测) /
                    'density'(密度图) / 'hybrid'(混合)
        """
        self.method = method

        # 背景减除器
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=50, detectShadows=True)

        # 头部Haar分类器（如有）
        self.head_cascade = None
        try:
            self.head_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        except:
            pass

        # HOG行人检测器
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # 计数历史（用于平滑）
        self.count_history = []

    def count_foreground(self, frame):
        """
        基于前景检测的人群计数
        通过分析前景连通区域估算人数

        参数:
            frame: BGR输入图像

        返回:
            count: 估计人数
            fg_mask: 前景掩码
        """
        # 背景减除
        fg_mask = self.bg_subtractor.apply(frame)

        # 去除阴影（值为127的像素）
        fg_mask[fg_mask == 127] = 0

        # 二值化
        _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)

        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 膨胀连接断裂区域
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)

        # 查找连通区域
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 过滤和计数
        count = 0
        valid_contours = []
        h, w = frame.shape[:2]
        min_area = (h * w) * 0.002  # 最小面积阈值
        max_area = (h * w) * 0.3    # 最大面积阈值

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area < area < max_area:
                count += 1
                valid_contours.append(cnt)

        return count, fg_mask, valid_contours

    def count_heads(self, frame):
        """
        基于头部检测的人群计数

        参数:
            frame: BGR输入图像

        返回:
            count: 检测到的头部数量
            heads: 头部位置列表
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # 多尺度检测
        heads = []
        scale_factors = [1.0, 0.75, 0.5]

        for scale in scale_factors:
            # 缩放图像
            resized = cv2.resize(gray, None, fx=scale, fy=scale)

            # 头部/人脸检测
            if self.head_cascade is not None:
                detections = self.head_cascade.detectMultiScale(
                    resized, scaleFactor=1.1, minNeighbors=5,
                    minSize=(int(20 * scale), int(20 * scale)),
                    maxSize=(int(150 * scale), int(150 * scale)))

                for (x, y, dw, dh) in detections:
                    # 转换回原图坐标
                    x_orig = int(x / scale)
                    y_orig = int(y / scale)
                    w_orig = int(dw / scale)
                    h_orig = int(dh / scale)

                    # 非极大值抑制（简化版）
                    is_duplicate = False
                    for (hx, hy, hw, hh) in heads:
                        cx1, cy1 = x_orig + w_orig / 2, y_orig + h_orig / 2
                        cx2, cy2 = hx + hw / 2, hy + hh / 2
                        dist = np.sqrt((cx1 - cx2) ** 2 + (cy1 - cy2) ** 2)
                        if dist < max(w_orig, hw) * 0.5:
                            is_duplicate = True
                            break
                    if not is_duplicate:
                        heads.append((x_orig, y_orig, w_orig, h_orig))

        return len(heads), heads

    def count_pedestrians(self, frame):
        """
        基于HOG+SVM的行人检测计数

        参数:
            frame: BGR输入图像

        返回:
            count: 行人数量
            bboxes: 行人边界框列表
        """
        # HOG行人检测
        bboxes, weights = self.hog.detectMultiScale(
            frame, winStride=(8, 8), padding=(4, 4), scale=1.05)

        # NMS去重
        if len(bboxes) > 0:
            # 转换为x1,y1,x2,y2格式
            rects = []
            for (x, y, w, h) in bboxes:
                rects.append([x, y, x + w, y + h])
            rects = np.array(rects)

            # 简单NMS
            keep = self._nms(rects, overlap_thresh=0.3)
            bboxes = bboxes[keep] if len(keep) > 0 else []
        else:
            bboxes = []

        return len(bboxes), bboxes

    def _nms(self, boxes, overlap_thresh=0.3):
        """非极大值抑制"""
        if len(boxes) == 0:
            return []

        x1 = boxes[:, 0].astype(float)
        y1 = boxes[:, 1].astype(float)
        x2 = boxes[:, 2].astype(float)
        y2 = boxes[:, 3].astype(float)

        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y2)

        keep = []
        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            overlap = (w * h) / area[idxs[:last]]

            idxs = np.delete(idxs, np.concatenate(([last], np.where(overlap > overlap_thresh)[0])))

        return keep

    def estimate_density(self, frame, foreground_mask=None):
        """
        密度图估计（简化版）
        通过局部密度分析生成人群密度分布图

        参数:
            frame: BGR图像
            foreground_mask: 前景掩码（可选）

        返回:
            density_map: 密度图（浮点，值越大越密集）
            total_count: 总估计人数
        """
        h, w = frame.shape[:2]

        if foreground_mask is None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 使用边缘密度作为替代
            edges = cv2.Canny(gray, 50, 150)
            foreground_mask = edges

        # 分块统计密度
        block_size = 32
        density_map = np.zeros((h // block_size, w // block_size), dtype=np.float32)

        for by in range(density_map.shape[0]):
            for bx in range(density_map.shape[1]):
                # 提取块
                y1 = by * block_size
                x1 = bx * block_size
                y2 = min(y1 + block_size, h)
                x2 = min(x1 + block_size, w)

                block = foreground_mask[y1:y2, x1:x2]
                # 密度 = 非零像素比例
                density = np.count_nonzero(block) / (block.size + 1e-6)
                density_map[by, bx] = density

        # 高斯平滑
        density_map = cv2.GaussianBlur(density_map, (5, 5), 1)

        # 估算总人数（简化：每个高密度区域约1人）
        threshold = 0.3
        total_count = int(np.sum(density_map > threshold) * 0.5)

        return density_map, total_count

    def process_frame(self, frame, background_model_ready=True):
        """
        处理单帧（混合方法）

        参数:
            frame: BGR输入图像
            background_model_ready: 背景模型是否已就绪

        返回:
            result: 计数结果字典
        """
        results = {}

        # 前景检测计数
        fg_count, fg_mask, fg_contours = self.count_foreground(frame)
        results['foreground_count'] = fg_count

        # 行人检测计数
        ped_count, ped_bboxes = self.count_pedestrians(frame)
        results['pedestrian_count'] = ped_count
        results['pedestrian_bboxes'] = ped_bboxes

        # 头部检测计数
        head_count, heads = self.count_heads(frame)
        results['head_count'] = head_count
        results['heads'] = heads

        # 混合计数（加权平均）
        if self.method == 'hybrid':
            # 背景模型需要预热
            if background_model_ready:
                weights = [0.3, 0.5, 0.2]
                counts = [fg_count, ped_count, head_count]
            else:
                weights = [0.0, 0.7, 0.3]
                counts = [0, ped_count, head_count]

            weighted_count = sum(c * w for c, w in zip(counts, weights))
            final_count = max(1, int(round(weighted_count)))
        elif self.method == 'foreground':
            final_count = fg_count
        elif self.method == 'head':
            final_count = head_count
        else:
            final_count = ped_count

        # 平滑处理
        self.count_history.append(final_count)
        if len(self.count_history) > 10:
            self.count_history.pop(0)
        smoothed_count = int(round(np.mean(self.count_history)))

        results['final_count'] = smoothed_count
        results['raw_count'] = final_count
        results['fg_mask'] = fg_mask
        results['fg_contours'] = fg_contours

        return results

    def draw_debug(self, frame, result):
        """
        绘制调试可视化

        参数:
            frame: 原始图像
            result: process_frame返回的结果

        返回:
            vis: 可视化图像
        """
        vis = frame.copy()

        # 绘制前景轮廓
        if 'fg_contours' in result:
            cv2.drawContours(vis, result['fg_contours'], -1, (0, 255, 0), 1)

        # 绘制行人检测框
        for (x, y, w, h) in result.get('pedestrian_bboxes', []):
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # 绘制头部检测框
        for (x, y, w, h) in result.get('heads', []):
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # 显示计数
        count = result['final_count']
        cv2.putText(vis, f"Count: {count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(vis, f"FG:{result['foreground_count']} "
                        f"PED:{result['pedestrian_count']} "
                        f"HEAD:{result['head_count']}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

        return vis

    def draw_density_map(self, density_map, output_size=None):
        """
        可视化密度图

        参数:
            density_map: 密度图
            output_size: 输出尺寸 (w, h)

        返回:
            vis: 彩色密度图
        """
        # 归一化到0-255
        norm = cv2.normalize(density_map, None, 0, 255, cv2.NORM_MINMAX)
        norm = norm.astype(np.uint8)

        # 应用颜色映射
        colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        if output_size:
            colored = cv2.resize(colored, output_size, interpolation=cv2.INTER_LINEAR)

        return colored


# ==================== 使用示例 ====================
def demo_video(video_path):
    """视频人群计数演示"""
    counter = CrowdCounter(method='hybrid')
    cap = cv2.VideoCapture(video_path if video_path else 0)

    if not cap.isOpened():
        print(f"无法打开视频: {video_path}")
        return

    print("人群计数演示 - 按ESC退出")
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        # 背景模型需要预热（前30帧）
        bg_ready = frame_idx > 30

        result = counter.process_frame(frame, bg_ready)
        vis = counter.draw_debug(frame, result)

        cv2.imshow("Crowd Counting", vis)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """单张图片人群计数"""
    counter = CrowdCounter(method='hybrid')
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    result = counter.process_frame(frame, background_model_ready=False)

    print(f"估计人数: {result['final_count']}")
    print(f"  前景检测: {result['foreground_count']}")
    print(f"  行人检测: {result['pedestrian_count']}")
    print(f"  头部检测: {result['head_count']}")

    vis = counter.draw_debug(frame, result)
    cv2.imshow("Result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        demo_video(sys.argv[1])
    else:
        demo_video(0)
