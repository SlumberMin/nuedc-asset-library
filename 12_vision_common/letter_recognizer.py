"""
字母识别模块
支持模板匹配、特征提取和KNN/SVM分类器
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
from pathlib import Path
import string


class LetterSegmenter:
    """字母分割器"""
    
    @staticmethod
    def segment_by_contour(image: np.ndarray, 
                           min_area: int = 100,
                           min_height: int = 15) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        基于轮廓的字母分割
        Args:
            image: 输入图像（灰度或彩色）
            min_area: 最小轮廓面积
            min_height: 最小字母高度
        Returns:
            (字母图像, 边界框) 列表，按x坐标排序
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
        
        letters = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            if h < min_height:
                continue
            
            aspect_ratio = w / h if h > 0 else 0
            if 0.1 < aspect_ratio < 3.0:
                letter_img = gray[y:y+h, x:x+w]
                letters.append((letter_img, (x, y, w, h)))
        
        # 按x坐标排序
        letters.sort(key=lambda x: x[1][0])
        return letters
    
    @staticmethod
    def segment_by_projection(image: np.ndarray, 
                              threshold: float = 0.1) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        基于垂直投影的字母分割
        Args:
            image: 输入图像
            threshold: 投影阈值（占最大投影的比例）
        Returns:
            (字母图像, 边界框) 列表
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 垂直投影
        v_proj = np.sum(binary > 0, axis=0)
        
        if len(v_proj) == 0:
            return []
        
        # 寻找分割点
        max_proj = np.max(v_proj)
        if max_proj == 0:
            return []
        
        threshold_val = max_proj * threshold
        in_letter = False
        start = 0
        segments = []
        
        for i, val in enumerate(v_proj):
            if not in_letter and val > threshold_val:
                in_letter = True
                start = i
            elif in_letter and val <= threshold_val:
                in_letter = False
                if i - start > 5:  # 最小宽度
                    segments.append((start, i))
        
        if in_letter and len(v_proj) - start > 5:
            segments.append((start, len(v_proj)))
        
        # 提取字母图像
        letters = []
        h = gray.shape[0]
        for x_start, x_end in segments:
            letter_img = gray[0:h, x_start:x_end]
            letters.append((letter_img, (x_start, 0, x_end - x_start, h)))
        
        return letters


class LetterRecognizer:
    """字母识别器"""
    
    # 支持的字母集
    LETTERS = string.ascii_uppercase  # A-Z
    
    def __init__(self, template_dir: str = None, method: str = 'template'):
        """
        初始化字母识别器
        Args:
            template_dir: 模板目录（可选）
            method: 识别方法 - 'template', 'knn', 'svm'
        """
        self.method = method
        self.segmenter = LetterSegmenter()
        
        # 模板存储
        self.templates: Dict[str, List[np.ndarray]] = {}
        
        # KNN/SVM 分类器
        self.classifier = None
        self.train_features = []
        self.train_labels = []
        
        # 加载模板
        if template_dir:
            self.load_templates(template_dir)
    
    def load_templates(self, template_dir: str):
        """
        加载模板
        目录结构：
            template_dir/
            ├── A/
            │   ├── A_1.png
            │   └── A_2.png
            ├── B/
            │   └── ...
        """
        template_path = Path(template_dir)
        if not template_path.exists():
            print(f"模板目录不存在: {template_dir}")
            return
        
        for letter in self.LETTERS:
            letter_dir = template_path / letter
            if letter_dir.exists():
                templates = []
                for img_file in letter_dir.glob("*.*"):
                    img = cv2.imread(str(img_file), cv2.IMREAD_GRAYSCALE)
                    if img is not None:
                        img = cv2.resize(img, (28, 28))
                        templates.append(img)
                
                if templates:
                    self.templates[letter] = templates
        
        total = sum(len(v) for v in self.templates.values())
        print(f"已加载 {total} 个字母模板")
    
    def preprocess(self, image: np.ndarray, size: Tuple[int, int] = (28, 28)) -> np.ndarray:
        """
        预处理字母图像
        Args:
            image: 输入图像
            size: 输出尺寸
        Returns:
            预处理后的图像
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 保持宽高比缩放
        h, w = gray.shape
        max_dim = max(h, w)
        scale = size[0] / max_dim * 0.8
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = cv2.resize(gray, (new_w, new_h))
        
        # 居中放置
        canvas = np.zeros(size, dtype=np.uint8)
        x_offset = (size[1] - new_w) // 2
        y_offset = (size[0] - new_h) // 2
        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        
        # 二值化
        _, binary = cv2.threshold(canvas, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        return binary
    
    def _extract_hog_features(self, image: np.ndarray) -> np.ndarray:
        """
        提取HOG特征
        Args:
            image: 预处理后的图像
        Returns:
            HOG特征向量
        """
        # HOG描述符
        win_size = (28, 28)
        block_size = (14, 14)
        block_stride = (7, 7)
        cell_size = (7, 7)
        nbins = 9
        
        hog = cv2.HOGDescriptor(win_size, block_size, block_stride, cell_size, nbins)
        features = hog.compute(image)
        return features.flatten()
    
    def _extract_features(self, image: np.ndarray) -> np.ndarray:
        """
        提取综合特征
        Args:
            image: 预处理后的图像
        Returns:
            特征向量
        """
        features = []
        
        # 1. HOG特征
        hog = self._extract_hog_features(image)
        features.extend(hog.tolist())
        
        # 2. 网格特征
        grid_size = 4
        h, w = image.shape
        cell_h = h // grid_size
        cell_w = w // grid_size
        for i in range(grid_size):
            for j in range(grid_size):
                cell = image[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]
                ratio = np.sum(cell > 0) / (cell_h * cell_w)
                features.append(ratio)
        
        # 3. Hu矩
        moments = cv2.moments(image)
        hu = cv2.HuMoments(moments).flatten()
        hu = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)
        features.extend(hu.tolist())
        
        # 4. 投影特征
        h_proj = np.sum(image > 0, axis=1) / w
        v_proj = np.sum(image > 0, axis=0) / h
        features.extend(h_proj[::4].tolist())
        features.extend(v_proj[::4].tolist())
        
        return np.array(features, dtype=np.float32)
    
    def train(self, images: List[np.ndarray], labels: List[str]):
        """
        训练字母识别器
        Args:
            images: 训练图像列表
            labels: 对应字母标签列表
        """
        # 提取特征
        features = []
        for img in images:
            processed = self.preprocess(img)
            feat = self._extract_features(processed)
            features.append(feat)
        
        features = np.array(features, dtype=np.float32)
        labels = np.array(labels)
        
        if self.method == 'knn':
            self.classifier = cv2.ml.KNearest_create()
            self.classifier.train(features, cv2.ml.ROW_SAMPLE, 
                                  np.array([ord(l) - ord('A') for l in labels], dtype=np.float32))
        elif self.method == 'svm':
            self.classifier = cv2.ml.SVM_create()
            self.classifier.setKernel(cv2.ml.SVM_RBF)
            self.classifier.setType(cv2.ml.SVM_C_SVC)
            self.classifier.setC(10)
            self.classifier.train(features, cv2.ml.ROW_SAMPLE,
                                  np.array([ord(l) - ord('A') for l in labels], dtype=np.int32))
        
        print(f"训练完成: {len(images)} 个样本, 方法: {self.method}")
    
    def recognize(self, image: np.ndarray) -> List[Tuple[str, float]]:
        """
        识别单个字母
        Args:
            image: 输入字母图像
        Returns:
            (字母, 置信度) 列表，按置信度降序
        """
        processed = self.preprocess(image)
        
        if self.method == 'template':
            return self._recognize_template(processed)
        elif self.method in ('knn', 'svm') and self.classifier is not None:
            return self._recognize_classifier(processed)
        else:
            return self._recognize_template(processed)
    
    def _recognize_template(self, image: np.ndarray) -> List[Tuple[str, float]]:
        """模板匹配识别"""
        results = []
        
        for letter, templates in self.templates.items():
            max_score = 0.0
            for template in templates:
                # 归一化互相关
                result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
                score = result[0][0]
                max_score = max(max_score, score)
            
            # 也计算相关系数
            max_corr = 0.0
            for template in templates:
                img_f = image.astype(np.float32)
                tpl_f = template.astype(np.float32)
                corr = np.corrcoef(img_f.flatten(), tpl_f.flatten())[0, 1]
                max_corr = max(max_corr, corr if not np.isnan(corr) else 0)
            
            combined = 0.6 * max_score + 0.4 * max_corr
            results.append((letter, combined))
        
        results.sort(key=lambda x: x[1], reverse=True)
        return results
    
    def _recognize_classifier(self, image: np.ndarray) -> List[Tuple[str, float]]:
        """分类器识别"""
        features = self._extract_features(image).reshape(1, -1)
        
        if self.method == 'knn':
            _, results, _, distances = self.classifier.findNearest(features, k=5)
            label_idx = int(results[0, 0])
            letter = chr(label_idx + ord('A'))
            confidence = 1.0 / (1.0 + distances[0, 0])
            return [(letter, confidence)]
        
        elif self.method == 'svm':
            _, results = self.classifier.predict(features)
            label_idx = int(results[0, 0])
            letter = chr(label_idx + ord('A'))
            return [(letter, 0.8)]  # SVM不直接给概率
        
        return []
    
    def recognize_string(self, image: np.ndarray) -> str:
        """
        识别图像中的字母字符串
        Args:
            image: 输入图像
        Returns:
            识别出的字母字符串
        """
        letters = self.segmenter.segment_by_contour(image)
        
        result = ""
        for letter_img, _ in letters:
            results = self.recognize(letter_img)
            if results and results[0][1] > 0.4:
                result += results[0][0]
        
        return result
    
    def visualize(self, image: np.ndarray) -> np.ndarray:
        """可视化识别结果"""
        vis_image = image.copy()
        if len(vis_image.shape) == 2:
            vis_image = cv2.cvtColor(vis_image, cv2.COLOR_GRAY2BGR)
        
        letters = self.segmenter.segment_by_contour(image)
        
        for letter_img, (x, y, w, h) in letters:
            results = self.recognize(letter_img)
            
            if results:
                letter, confidence = results[0]
                
                # 颜色根据置信度
                if confidence > 0.7:
                    color = (0, 255, 0)
                elif confidence > 0.4:
                    color = (0, 255, 255)
                else:
                    color = (0, 0, 255)
                
                cv2.rectangle(vis_image, (x, y), (x + w, y + h), color, 2)
                text = f"{letter} ({confidence:.2f})"
                cv2.putText(vis_image, text, (x, y - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return vis_image


# 使用示例
if __name__ == "__main__":
    recognizer = LetterRecognizer(method='template')
    
    # 从摄像头读取
    cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 识别
            result = recognizer.recognize_string(frame)
            if result:
                print(f"识别结果: {result}")
            
            # 可视化
            vis_frame = recognizer.visualize(frame)
            cv2.imshow("Letter Recognition", vis_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
