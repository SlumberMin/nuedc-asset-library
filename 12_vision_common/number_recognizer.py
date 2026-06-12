"""
数字识别模块
支持模板匹配和特征提取两种识别方法
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
from pathlib import Path
import os

class TemplateMatcher:
    """模板匹配识别器"""
    def __init__(self, template_dir: str = None):
        """
        初始化模板匹配器
        Args:
            template_dir: 模板图像目录，目录结构应为：
                template_dir/
                ├── 0/
                │   ├── 0_1.png
                │   └── 0_2.png
                ├── 1/
                │   ├── 1_1.png
                │   └── 1_2.png
                └── ...
        """
        self.templates = {}
        self.template_dir = template_dir
        if template_dir:
            self.load_templates(template_dir)
    
    def load_templates(self, template_dir: str):
        """加载模板图像"""
        template_path = Path(template_dir)
        if not template_path.exists():
            print(f"模板目录不存在: {template_dir}")
            return
        
        for digit in range(10):
            digit_dir = template_path / str(digit)
            if digit_dir.exists():
                templates = []
                for img_file in digit_dir.glob("*.png"):
                    img = cv2.imread(str(img_file), cv2.IMREAD_GRAYSCALE)
                    if img is not None:
                        # 统一模板大小
                        img = cv2.resize(img, (28, 28))
                        templates.append(img)
                
                if templates:
                    self.templates[str(digit)] = templates
        
        print(f"已加载模板: {', '.join(f'{k}:{len(v)}个' for k, v in self.templates.items())}")
    
    def add_template(self, digit: int, image: np.ndarray):
        """添加单个模板"""
        digit_str = str(digit)
        if digit_str not in self.templates:
            self.templates[digit_str] = []
        
        # 预处理模板
        processed = self.preprocess(image)
        self.templates[digit_str].append(processed)
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """预处理图像"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 调整大小
        resized = cv2.resize(binary, (28, 28))
        
        return resized
    
    def match(self, image: np.ndarray, method: str = 'template') -> List[Tuple[int, float]]:
        """
        使用模板匹配识别数字
        Args:
            image: 输入图像
            method: 匹配方法，'template' 或 'correlation'
        Returns:
            匹配结果列表，每个元素为 (数字, 置信度)
        """
        processed = self.preprocess(image)
        results = []
        
        for digit_str, templates in self.templates.items():
            digit = int(digit_str)
            max_score = 0.0
            
            for template in templates:
                if method == 'template':
                    # 模板匹配
                    result = cv2.matchTemplate(processed, template, cv2.TM_CCOEFF_NORMED)
                    score = result[0][0]
                else:
                    # 相关系数
                    score = self._correlation_score(processed, template)
                
                max_score = max(max_score, score)
            
            results.append((digit, max_score))
        
        # 按置信度排序
        results.sort(key=lambda x: x[1], reverse=True)
        return results
    
    def _correlation_score(self, img1: np.ndarray, img2: np.ndarray) -> float:
        """计算两个图像的相关系数"""
        # 转换为浮点型
        img1 = img1.astype(np.float32)
        img2 = img2.astype(np.float32)
        
        # 计算相关系数
        mean1 = np.mean(img1)
        mean2 = np.mean(img2)
        
        numerator = np.sum((img1 - mean1) * (img2 - mean2))
        denominator = np.sqrt(np.sum((img1 - mean1)**2) * np.sum((img2 - mean2)**2))
        
        if denominator == 0:
            return 0.0
        
        return numerator / denominator


class FeatureExtractor:
    """特征提取识别器"""
    def __init__(self):
        self.feature_size = 64  # 特征向量大小
    
    def extract_features(self, image: np.ndarray) -> np.ndarray:
        """
        提取数字图像的特征
        Args:
            image: 预处理后的二值图像
        Returns:
            特征向量
        """
        features = []
        
        # 1. 网格特征（将图像分成4x4网格，计算每个网格的像素比例）
        grid_features = self._grid_features(image, grid_size=4)
        features.extend(grid_features)
        
        # 2. 投影特征（水平和垂直投影）
        projection_features = self._projection_features(image)
        features.extend(projection_features)
        
        # 3. 矩特征
        moment_features = self._moment_features(image)
        features.extend(moment_features)
        
        # 4. 轮廓特征
        contour_features = self._contour_features(image)
        features.extend(contour_features)
        
        return np.array(features)
    
    def _grid_features(self, image: np.ndarray, grid_size: int = 4) -> List[float]:
        """网格特征"""
        h, w = image.shape
        cell_h = h // grid_size
        cell_w = w // grid_size
        
        features = []
        for i in range(grid_size):
            for j in range(grid_size):
                cell = image[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]
                ratio = np.sum(cell > 0) / (cell_h * cell_w)
                features.append(ratio)
        
        return features
    
    def _projection_features(self, image: np.ndarray) -> List[float]:
        """投影特征"""
        # 水平投影
        h_proj = np.sum(image > 0, axis=1) / image.shape[1]
        # 垂直投影
        v_proj = np.sum(image > 0, axis=0) / image.shape[0]
        
        # 采样（每4个取一个）
        h_sampled = h_proj[::4].tolist()
        v_sampled = v_proj[::4].tolist()
        
        return h_sampled + v_sampled
    
    def _moment_features(self, image: np.ndarray) -> List[float]:
        """矩特征"""
        moments = cv2.moments(image)
        
        # 归一化中心矩
        hu_moments = cv2.HuMoments(moments).flatten()
        
        # 对数变换
        hu_moments = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)
        
        return hu_moments.tolist()
    
    def _contour_features(self, image: np.ndarray) -> List[float]:
        """轮廓特征"""
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return [0.0] * 5
        
        # 最大轮廓
        contour = max(contours, key=cv2.contourArea)
        
        # 面积
        area = cv2.contourArea(contour)
        
        # 周长
        perimeter = cv2.arcLength(contour, True)
        
        # 圆度
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        
        # 边界框宽高比
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / h if h > 0 else 0
        
        # 矩形度
        rect_area = w * h
        rectangularity = area / rect_area if rect_area > 0 else 0
        
        return [area / 1000, perimeter / 100, circularity, aspect_ratio, rectangularity]


class NumberRecognizer:
    """数字识别器"""
    def __init__(self, template_dir: str = None):
        """
        初始化数字识别器
        Args:
            template_dir: 模板目录
        """
        self.template_matcher = TemplateMatcher(template_dir)
        self.feature_extractor = FeatureExtractor()
        
        # 训练数据
        self.train_features = []
        self.train_labels = []
        
        # 分类器
        self.classifier = None
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """预处理图像"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # 二值化
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 形态学操作
        kernel = np.ones((2, 2), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        # 调整大小
        resized = cv2.resize(binary, (28, 28))
        
        return resized
    
    def extract_digits(self, image: np.ndarray) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        从图像中提取单个数字
        Args:
            image: 输入图像
        Returns:
            数字图像和边界框的列表
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 形态学操作
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        digits = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100:  # 过滤噪声
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            # 检查宽高比
            aspect_ratio = w / h if h > 0 else 0
            if 0.2 < aspect_ratio < 2.0 and h > 15:
                digit_img = gray[y:y+h, x:x+w]
                digits.append((digit_img, (x, y, w, h)))
        
        # 按x坐标排序
        digits.sort(key=lambda x: x[1][0])
        
        return digits
    
    def recognize(self, image: np.ndarray, method: str = 'template') -> List[Tuple[int, float]]:
        """
        识别图像中的数字
        Args:
            image: 输入图像
            method: 识别方法，'template' 或 'feature'
        Returns:
            识别结果列表，每个元素为 (数字, 置信度)
        """
        if method == 'template':
            return self.template_matcher.match(image)
        elif method == 'feature':
            return self._recognize_by_feature(image)
        else:
            raise ValueError(f"未知的识别方法: {method}")
    
    def _recognize_by_feature(self, image: np.ndarray) -> List[Tuple[int, float]]:
        """使用特征提取识别"""
        processed = self.preprocess(image)
        features = self.feature_extractor.extract_features(processed)
        
        if self.classifier is None:
            return []
        
        # 预测
        prediction = self.classifier.predict([features])[0]
        probabilities = self.classifier.predict_proba([features])[0]
        
        # 获取所有类别的概率
        results = []
        for i, prob in enumerate(probabilities):
            results.append((i, prob))
        
        results.sort(key=lambda x: x[1], reverse=True)
        return results
    
    def train(self, images: List[np.ndarray], labels: List[int]):
        """
        训练识别器
        Args:
            images: 训练图像列表
            labels: 对应标签列表
        """
        from sklearn.ensemble import RandomForestClassifier
        
        # 提取特征
        features = []
        for img in images:
            processed = self.preprocess(img)
            feature = self.feature_extractor.extract_features(processed)
            features.append(feature)
        
        # 训练分类器
        self.classifier = RandomForestClassifier(n_estimators=100, random_state=42)
        self.classifier.fit(features, labels)
        
        print(f"训练完成，使用 {len(images)} 个样本")
    
    def recognize_sequence(self, image: np.ndarray) -> str:
        """
        识别图像中的数字序列
        Args:
            image: 输入图像
        Returns:
            识别出的数字字符串
        """
        digits = self.extract_digits(image)
        
        result = ""
        for digit_img, _ in digits:
            # 预处理
            processed = self.preprocess(digit_img)
            
            # 识别
            results = self.template_matcher.match(processed)
            if results:
                digit, confidence = results[0]
                if confidence > 0.5:  # 置信度阈值
                    result += str(digit)
        
        return result
    
    def visualize(self, image: np.ndarray, results: List[Tuple[int, float]]) -> np.ndarray:
        """可视化识别结果"""
        vis_image = image.copy()
        
        # 提取数字位置
        digits = self.extract_digits(image)
        
        for i, ((digit_img, bbox), (recognized_digit, confidence)) in enumerate(zip(digits, results)):
            x, y, w, h = bbox
            
            # 绘制边界框
            color = (0, 255, 0) if confidence > 0.7 else (0, 255, 255)
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), color, 2)
            
            # 绘制识别结果
            text = f"{recognized_digit}: {confidence:.2f}"
            cv2.putText(vis_image, text, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return vis_image


# 使用示例
if __name__ == "__main__":
    # 创建识别器
    recognizer = NumberRecognizer()
    
    # 示例：从摄像头读取并识别
    cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 识别数字序列
            sequence = recognizer.recognize_sequence(frame)
            print(f"识别结果: {sequence}")
            
            # 可视化
            digits = recognizer.extract_digits(frame)
            for digit_img, bbox in digits:
                x, y, w, h = bbox
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            cv2.imshow("Number Recognition", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()