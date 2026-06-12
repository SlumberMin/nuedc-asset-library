"""
场景文字检测与识别模块
EAST文本检测 + CRNN文字识别
适用于电赛中的文字识别、车牌识别、标签读取等场景
"""

import cv2
import numpy as np
import time
import re


# ============================================================
# EAST 文本检测器
# ============================================================
class EASTTextDetector:
    """
    基于 EAST (Efficient and Accurate Scene Text) 的文本检测
    使用 OpenCV DNN 加载预训练模型
    
    模型下载:
      frozen_east_text_detection.pb
      https://github.com/argman/EAST (需自行导出)
      或: https://www.dropbox.com/s/r2ingd0l3zt8hxs/frozen_east_text_detection.pb
    """

    def __init__(self, model_path=None, input_size=(320, 320),
                 confidence_threshold=0.5, nms_threshold=0.4):
        """
        参数:
            model_path: EAST模型文件路径 (.pb)
            input_size: 网络输入尺寸 (宽, 高)，必须是32的倍数
            confidence_threshold: 文本检测置信度阈值
            nms_threshold: 非极大值抑制阈值
        """
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.input_size = input_size
        self.net = None

        if model_path:
            try:
                self.net = cv2.dnn.readNet(model_path)
                print(f"[EAST检测] 已加载模型: {model_path}")
            except Exception as e:
                print(f"[EAST检测] 模型加载失败: {e}")
        else:
            print("[EAST检测] 未提供模型路径，使用OpenCV形态学方案")

    def _decode_predictions(self, scores, geometry, score_threshold):
        """解码EAST网络输出"""
        rows, cols = scores.shape[2:4]
        detections = []
        confidences = []

        for y in range(rows):
            scores_data = scores[0, 0, y]
            # 4个距离 + 1个角度
            x_data0 = geometry[0, 0, y]  # top
            x_data1 = geometry[0, 1, y]  # right
            x_data2 = geometry[0, 2, y]  # bottom
            x_data3 = geometry[0, 3, y]  # left
            angles = geometry[0, 4, y]

            for x in range(cols):
                if scores_data[x] < score_threshold:
                    continue

                # 计算偏移
                offset_x = x * 4.0
                offset_y = y * 4.0

                angle = angles[x]
                cos_a = np.cos(angle)
                sin_a = np.sin(angle)

                h = x_data0[x] + x_data2[x]
                w = x_data1[x] + x_data3[x]

                # 计算边界框四个角点
                end_x = int(offset_x + cos_a * x_data1[x] + sin_a * x_data2[x])
                end_y = int(offset_y - sin_a * x_data1[x] + cos_a * x_data2[x])
                start_x = int(end_x - w)
                start_y = int(end_y - h)

                detections.append((start_x, start_y, int(w), int(h)))
                confidences.append(float(scores_data[x]))

        return detections, confidences

    def detect_east(self, frame):
        """
        使用EAST模型检测文本区域
        返回: list of rotated rects [(cx, cy, w, h, angle), ...]
        """
        if self.net is None:
            return []

        orig_h, orig_w = frame.shape[:2]
        inp_w, inp_h = self.input_size

        # 构建输入
        blob = cv2.dnn.blobFromImage(
            frame, 1.0, (inp_w, inp_h),
            (123.68, 116.78, 103.94), True, False
        )
        self.net.setInput(blob)

        # 输出层名称
        output_layers = ["feature_fusion/Conv_7/Sigmoid", "feature_fusion/concat_3"]

        # 前向推理
        outputs = self.net.forward(output_layers)
        scores = outputs[0]
        geometry = outputs[1]

        # 解码
        boxes, confidences = self._decode_predictions(
            scores, geometry, self.confidence_threshold)

        # NMS
        indices = cv2.dnn.NMSBoxesRotated(
            [(b[0] + b[2]/2, b[1] + b[3]/2, b[2], b[3], 0) for b in boxes],
            confidences,
            self.confidence_threshold,
            self.nms_threshold
        ) if len(boxes) > 0 else []

        # 映射回原图坐标
        results = []
        scale_x = orig_w / inp_w
        scale_y = orig_h / inp_h

        for i in indices:
            if isinstance(i, (list, np.ndarray)):
                i = i[0]
            x, y, w, h = boxes[i]
            x1 = max(0, int(x * scale_x))
            y1 = max(0, int(y * scale_y))
            x2 = min(orig_w, int((x + w) * scale_x))
            y2 = min(orig_h, int((y + h) * scale_y))
            results.append((x1, y1, x2, y2))

        return results

    def detect_morphology(self, frame):
        """
        使用形态学方法检测文本区域（无需模型）
        利用文字的高对比度和笔画特征
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # MSER特征检测
        mser = cv2.MSER_create()
        mser.setMinArea(60)
        mser.setMaxArea(14400)

        regions, _ = mser.detectRegions(gray)

        # 将MSER区域转为矩形
        bboxes = []
        for region in regions:
            x, y, w, h = cv2.boundingRect(region)
            # 过滤不合理的区域
            aspect = w / max(h, 1)
            if 0.1 < aspect < 10 and w > 10 and h > 8:
                bboxes.append((x, y, x + w, y + h))

        # 合并相近的矩形
        bboxes = self._merge_nearby_boxes(bboxes)

        return bboxes

    def _merge_nearby_boxes(self, bboxes, distance_threshold=20):
        """合并相近的边界框"""
        if len(bboxes) < 2:
            return bboxes

        merged = True
        result = list(bboxes)

        while merged:
            merged = False
            new_result = []
            used = [False] * len(result)

            for i in range(len(result)):
                if used[i]:
                    continue
                x1_i, y1_i, x2_i, y2_i = result[i]

                for j in range(i + 1, len(result)):
                    if used[j]:
                        continue
                    x1_j, y1_j, x2_j, y2_j = result[j]

                    # 检查是否可以合并（行高相近、水平距离近）
                    h_i = y2_i - y1_i
                    h_j = y2_j - y1_j
                    if min(h_i, h_j) > 0 and max(h_i, h_j) / min(h_i, h_j) < 2.0:
                        gap = max(0, x1_j - x2_i, x1_i - x2_j)
                        if gap < distance_threshold:
                            x1_i = min(x1_i, x1_j)
                            y1_i = min(y1_i, y1_j)
                            x2_i = max(x2_i, x2_j)
                            y2_i = max(y2_i, y2_j)
                            used[j] = True
                            merged = True

                new_result.append((x1_i, y1_i, x2_i, y2_i))

            result = new_result

        return result

    def detect(self, frame):
        """
        文本检测主入口
        返回: list of (x1, y1, x2, y2) 边界框
        """
        if self.net is not None:
            results = self.detect_east(frame)
        else:
            results = self.detect_morphology(frame)

        return results


# ============================================================
# 简易文字识别器（基于轮廓+模板匹配）
# ============================================================
class SimpleOCR:
    """
    简易文字识别器
    当没有CRNN模型时，使用Tesseract或简单的模板匹配
    """

    def __init__(self, use_tesseract=False):
        """
        参数:
            use_tesseract: 是否使用pytesseract进行OCR
        """
        self.use_tesseract = use_tesseract
        self.tesseract_available = False

        if use_tesseract:
            try:
                import pytesseract
                self.tesseract_available = True
                print("[OCR] 已启用Tesseract")
            except ImportError:
                print("[OCR] pytesseract未安装，使用简化方案")

    def preprocess_for_ocr(self, roi):
        """对ROI进行OCR预处理"""
        # 转灰度
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()

        # 自适应二值化
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 11, 2
        )

        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        return binary

    def recognize_tesseract(self, roi):
        """使用Tesseract识别文字"""
        if not self.tesseract_available:
            return ""

        import pytesseract
        preprocessed = self.preprocess_for_ocr(roi)

        # 配置Tesseract
        config = '--psm 7 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789'
        text = pytesseract.image_to_string(preprocessed, config=config)

        return text.strip()

    def recognize_chars(self, roi):
        """
        基于轮廓的简化字符分割与特征提取
        不做真正的识别，仅返回字符数量和基本特征
        """
        preprocessed = self.preprocess_for_ocr(roi)

        # 查找字符轮廓
        contours, _ = cv2.findContours(
            preprocessed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        chars = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if h > 10 and w > 3 and w < h * 3:
                chars.append({
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'area': cv2.contourArea(cnt),
                    'aspect': w / max(h, 1),
                    'extent': cv2.contourArea(cnt) / max(w * h, 1)
                })

        # 按x坐标排序（从左到右）
        chars.sort(key=lambda c: c['x'])

        return chars

    def recognize(self, roi):
        """
        文字识别主入口
        返回: 识别到的文字字符串或字符特征
        """
        if self.use_tesseract and self.tesseract_available:
            return self.recognize_tesseract(roi)
        else:
            chars = self.recognize_chars(roi)
            return f"[{len(chars)}个字符]" if chars else ""


# ============================================================
# 场景文字检测识别管线
# ============================================================
class SceneTextPipeline:
    """
    场景文字检测与识别完整管线
    流程: 检测文字区域 -> 预处理 -> 识别文字
    """

    def __init__(self, east_model_path=None, use_tesseract=False,
                 input_size=(320, 320), confidence=0.5):
        self.detector = EASTTextDetector(
            model_path=east_model_path,
            input_size=input_size,
            confidence_threshold=confidence
        )
        self.recognizer = SimpleOCR(use_tesseract=use_tesseract)

    def process(self, frame):
        """
        处理一帧图像
        参数:
            frame: BGR图像
        返回:
            results: list of dict, 包含 bbox, text, roi
        """
        # 检测文字区域
        text_regions = self.detector.detect(frame)

        results = []
        for bbox in text_regions:
            x1, y1, x2, y2 = bbox

            # 提取ROI
            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            # 文字识别
            text = self.recognizer.recognize(roi)

            results.append({
                'bbox': (x1, y1, x2, y2),
                'text': text,
                'roi': roi
            })

        return results


# ============================================================
# 可视化
# ============================================================
def draw_text_results(frame, results):
    """在图像上绘制文字检测和识别结果"""
    result_img = frame.copy()

    for i, item in enumerate(results):
        x1, y1, x2, y2 = item['bbox']
        text = item['text']

        # 随机颜色
        color = (
            (37 * (i + 1)) % 255,
            (17 * (i + 1) + 100) % 255,
            (53 * (i + 1) + 50) % 255
        )

        # 绘制边框
        cv2.rectangle(result_img, (x1, y1), (x2, y2), color, 2)

        # 绘制文字
        label = text if text else f"Text_{i}"
        cv2.putText(result_img, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return result_img


# ============================================================
# 使用示例
# ============================================================
def demo_camera():
    """摄像头实时文字检测演示"""
    pipeline = SceneTextPipeline(use_tesseract=False)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("场景文字检测演示")
    print("按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_start = time.time()
        results = pipeline.process(frame)
        t_cost = time.time() - t_start

        result_img = draw_text_results(frame, results)
        fps = 1.0 / max(t_cost, 1e-6)
        cv2.putText(result_img, f"FPS: {fps:.1f}  Texts: {len(results)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Scene Text Detection", result_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """静态图像文字检测演示"""
    pipeline = SceneTextPipeline(use_tesseract=False)

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图像: {image_path}")
        return

    results = pipeline.process(frame)
    print(f"检测到 {len(results)} 个文字区域:")
    for i, r in enumerate(results):
        print(f"  [{i}] bbox={r['bbox']} text='{r['text']}'")

    result_img = draw_text_results(frame, results)
    cv2.imshow("Text Detection", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def demo_with_east_model(east_model_path, image_path=None, camera=False):
    """
    使用EAST模型的文字检测演示
    
    参数:
        east_model_path: frozen_east_text_detection.pb 路径
        image_path: 测试图像路径
        camera: 是否使用摄像头
    """
    pipeline = SceneTextPipeline(
        east_model_path=east_model_path,
        input_size=(320, 320),
        confidence=0.5
    )

    if camera:
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            results = pipeline.process(frame)
            result_img = draw_text_results(frame, results)
            cv2.imshow("EAST Text Detection", result_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
    elif image_path:
        frame = cv2.imread(image_path)
        results = pipeline.process(frame)
        result_img = draw_text_results(frame, results)
        cv2.imshow("EAST Text Detection", result_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
