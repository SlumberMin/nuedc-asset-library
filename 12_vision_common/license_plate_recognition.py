"""
车牌识别模块 - 车牌定位 + 字符分割 + OCR识别
适用于：停车场管理、交通监控、电赛车牌识别题等场景
依赖：pip install opencv-python numpy
（高级OCR可选：pip install pytesseract，需安装Tesseract）
"""
import cv2
import numpy as np


class LicensePlateRecognizer:
    """中国车牌识别器（定位+分割+模板匹配/OCR）"""

    # 中国车牌字符集
    PROVINCES = ['京', '津', '沪', '渝', '冀', '豫', '云', '辽', '黑', '湘',
                 '皖', '鲁', '新', '苏', '浙', '赣', '鄂', '桂', '甘', '晋',
                 '蒙', '陕', '吉', '闽', '贵', '粤', '川', '青', '藏', '琼', '宁']
    LETTERS = list('ABCDEFGHJKLMNPQRSTUVWXYZ')  # 不含I和O
    DIGITS = list('0123456789')
    ALPHANUM = LETTERS + DIGITS

    def __init__(self, use_tesseract=False):
        """
        初始化
        Args:
            use_tesseract: 是否使用Tesseract OCR（需要单独安装）
        """
        self.use_tesseract = use_tesseract
        if use_tesseract:
            try:
                import pytesseract
                self.pytesseract = pytesseract
            except ImportError:
                print("pytesseract 未安装，回退到模板匹配模式")
                self.use_tesseract = False

    def preprocess(self, image):
        """
        图像预处理：灰度化、高斯模糊、边缘检测
        Args:
            image: BGR图像
        Returns:
            gray, blurred, edges
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 高斯模糊去噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Sobel边缘检测（水平方向，适合检测车牌竖直边缘）
        sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        sobel_x = cv2.convertScaleAbs(sobel_x)
        # 二值化
        _, binary = cv2.threshold(sobel_x, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # 形态学操作：闭运算连接车牌区域
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 5))
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        return gray, blurred, closed

    def locate_plate(self, image, aspect_ratio_range=(2.0, 5.5), area_range=(2000, 80000)):
        """
        车牌定位：基于轮廓分析筛选车牌候选区域
        Args:
            image: BGR图像
            aspect_ratio_range: 车牌长宽比范围
            area_range: 车牌面积范围
        Returns:
            plates: 车牌区域图像列表, rects: 对应矩形坐标
        """
        gray, blurred, processed = self.preprocess(image)
        contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        plates, rects = [], []
        h_img, w_img = image.shape[:2]

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / max(h, 1)
            area = w * h

            # 筛选条件：长宽比、面积、位置合理性
            if (aspect_ratio_range[0] <= aspect_ratio <= aspect_ratio_range[1]
                    and area_range[0] <= area <= area_range[1]
                    and w > h  # 车牌宽度大于高度
                    and w > w_img * 0.05):  # 宽度至少为图像5%
                # 扩展一点边界
                pad = 5
                x1 = max(0, x - pad)
                y1 = max(0, y - pad)
                x2 = min(w_img, x + w + pad)
                y2 = min(h_img, y + h + pad)
                plate_img = image[y1:y2, x1:x2]
                plates.append(plate_img)
                rects.append((x1, y1, x2 - x1, y2 - y1))

        return plates, rects

    def segment_characters(self, plate_img, char_ratio=0.2):
        """
        字符分割：将车牌图像中的单个字符切分出来
        Args:
            plate_img: 车牌区域图像
            char_ratio: 最小字符宽度占比
        Returns:
            char_images: 字符图像列表（已resize到统一大小）
        """
        if plate_img is None or plate_img.size == 0:
            return []

        h, w = plate_img.shape[:2]
        gray = cv2.cvtColor(plate_img, cv2.COLOR_BGR2GRAY) if len(plate_img.shape) == 3 else plate_img

        # 自适应二值化
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 31, 10
        )

        # 去除上下边框
        row_sum = np.sum(binary, axis=1) / 255
        row_threshold = w * 0.3
        top, bottom = 0, h
        for i in range(h):
            if row_sum[i] > row_threshold:
                top = i
                break
        for i in range(h - 1, -1, -1):
            if row_sum[i] > row_threshold:
                bottom = i
                break
        binary = binary[max(0, top - 2):min(h, bottom + 2), :]

        # 垂直投影分割字符
        col_sum = np.sum(binary, axis=0) / 255
        min_char_w = int(w * char_ratio)
        in_char = False
        char_boxes = []
        start = 0

        for i, val in enumerate(col_sum):
            if val > 0 and not in_char:
                start = i
                in_char = True
            elif val == 0 and in_char:
                if i - start >= min_char_w * 0.3:  # 最小字符宽度
                    char_boxes.append((start, i))
                in_char = False
        if in_char and w - start >= min_char_w * 0.3:
            char_boxes.append((start, w))

        # 提取字符并resize
        char_images = []
        plate_gray = binary
        for (x1, x2) in char_boxes:
            char = plate_gray[:, x1:x2]
            if char.size > 0:
                char_resized = cv2.resize(char, (20, 40), interpolation=cv2.INTER_AREA)
                char_images.append(char_resized)

        return char_images

    def match_template(self, char_img, templates_dir=None):
        """
        模板匹配识别单个字符（简易版本）
        Args:
            char_img: 字符灰度图像
            templates_dir: 模板目录（如果None则返回'#'）
        Returns:
            识别出的字符
        """
        if templates_dir is None:
            return '#'
        # 模板匹配逻辑（需准备模板库）
        best_match = '#'
        best_score = 0
        import os
        for fname in os.listdir(templates_dir):
            tpl = cv2.imread(os.path.join(templates_dir, fname), cv2.IMREAD_GRAYSCALE)
            if tpl is None:
                continue
            tpl = cv2.resize(tpl, (char_img.shape[1], char_img.shape[0]))
            score = cv2.matchTemplate(char_img, tpl, cv2.TM_CCOEFF_NORMED)[0][0]
            if score > best_score:
                best_score = score
                best_match = fname.split('.')[0]
        return best_match if best_score > 0.5 else '#'

    def recognize(self, image, templates_dir=None):
        """
        完整车牌识别流程
        Args:
            image: BGR图像
            templates_dir: 字符模板目录（可选）
        Returns:
            results: [{'plate_text': str, 'box': (x,y,w,h), 'plate_img': ndarray}, ...]
        """
        plates, rects = self.locate_plate(image)
        results = []

        for plate_img, rect in zip(plates, rects):
            char_images = self.segment_characters(plate_img)
            plate_text = ''

            if self.use_tesseract:
                # 使用Tesseract OCR
                try:
                    gray = cv2.cvtColor(plate_img, cv2.COLOR_BGR2GRAY)
                    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    text = self.pytesseract.image_to_string(
                        binary, lang='chi_sim+eng',
                        config='--psm 7 -c tessedit_char_whitelist='
                               + ''.join(self.ALPHANUM) + ''.join(self.PROVINCES)
                    )
                    plate_text = text.strip().replace(' ', '')
                except Exception:
                    plate_text = ''
            else:
                # 模板匹配
                for char_img in char_images:
                    plate_text += self.match_template(char_img, templates_dir)

            results.append({
                'plate_text': plate_text,
                'box': rect,
                'plate_img': plate_img
            })

        return results

    def draw_results(self, image, results):
        """在图像上绘制识别结果"""
        vis = image.copy()
        for res in results:
            x, y, w, h = res['box']
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(vis, res['plate_text'], (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return vis


# ============== 使用示例 ==============
if __name__ == '__main__':
    print("=== 车牌识别示例 ===")

    recognizer = LicensePlateRecognizer(use_tesseract=False)

    # 创建模拟车牌图像进行测试
    # 模拟一个蓝色车牌
    plate_sim = np.zeros((60, 200, 3), dtype=np.uint8)
    plate_sim[:] = (180, 80, 30)  # 蓝色背景
    cv2.rectangle(plate_sim, (2, 2), (198, 58), (255, 255, 255), 2)
    cv2.putText(plate_sim, "B A1234", (20, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    # 将模拟车牌放入大图
    test_img = np.zeros((480, 640, 3), dtype=np.uint8)
    test_img[200:260, 200:400] = plate_sim

    results = recognizer.recognize(test_img)
    print(f"检测到 {len(results)} 个车牌")
    for r in results:
        print(f"  车牌文本: '{r['plate_text']}', 位置: {r['box']}")

    vis = recognizer.draw_results(test_img, results)
    cv2.imwrite('plate_result.jpg', vis)
    print("结果已保存到 plate_result.jpg")

    # 对真实图片使用：
    # img = cv2.imread('car.jpg')
    # results = recognizer.recognize(img, templates_dir='plate_templates/')
    # for r in results:
    #     print(f"识别结果: {r['plate_text']}")
