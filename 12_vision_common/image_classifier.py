"""
图像分类器 - 特征提取 + KNN/SVM 分类
适用于: 物体分类、颜色/形状识别、质量检测
"""

import cv2
import numpy as np
import pickle
from pathlib import Path


class FeatureExtractor:
    """图像特征提取器"""

    @staticmethod
    def color_histogram(image, bins=32):
        """颜色直方图特征"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h_hist = cv2.calcHist([hsv], [0], None, [bins], [0, 180])
        s_hist = cv2.calcHist([hsv], [1], None, [bins], [0, 256])
        v_hist = cv2.calcHist([hsv], [2], None, [bins], [0, 256])
        hist = np.concatenate([h_hist, s_hist, v_hist]).flatten()
        return hist / (hist.sum() + 1e-7)

    @staticmethod
    def hu_moments(image):
        """Hu矩特征(旋转不变形状特征)"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        moments = cv2.moments(thresh)
        hu = cv2.HuMoments(moments).flatten()
        return -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)

    @staticmethod
    def hog_features(image, size=(64, 64)):
        """HOG特征(梯度方向直方图)"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        resized = cv2.resize(gray, size)
        hog = cv2.HOGDescriptor(size, (16, 16), (8, 8), (8, 8), 9)
        return hog.compute(resized).flatten()

    @staticmethod
    def combined_features(image, size=(64, 64)):
        """组合特征: 颜色直方图 + Hu矩 + 统计特征"""
        hist = FeatureExtractor.color_histogram(image)
        hu = FeatureExtractor.hu_moments(image)

        # 统计特征
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        resized = cv2.resize(gray, size)
        stat = np.array([
            resized.mean(), resized.std(),
            np.median(resized),
            float(np.percentile(resized, 25)),
            float(np.percentile(resized, 75)),
        ])

        return np.concatenate([hist, hu, stat])


class ImageClassifier:
    """图像分类器(KNN/SVM)"""

    def __init__(self, method='knn', feature_type='combined'):
        """
        Args:
            method: 'knn' 或 'svm'
            feature_type: 'color', 'hog', 'combined'
        """
        self.method = method
        self.feature_type = feature_type
        self.model = None
        self.label_names = []
        self.extractor = FeatureExtractor()

    def _extract(self, image):
        """提取特征"""
        if self.feature_type == 'color':
            return self.extractor.color_histogram(image)
        elif self.feature_type == 'hog':
            return self.extractor.hog_features(image)
        else:
            return self.extractor.combined_features(image)

    def _create_model(self):
        if self.method == 'knn':
            return cv2.ml.KNearest_create()
        elif self.method == 'svm':
            svm = cv2.ml.SVM_create()
            svm.setType(cv2.ml.SVM_C_SVC)
            svm.setKernel(cv2.ml.SVM_RBF)
            svm.setC(1.0)
            svm.setGamma(0.5)
            return svm
        else:
            raise ValueError(f"不支持的方法: {self.method}")

    def train(self, images, labels, label_names=None):
        """
        训练分类器
        Args:
            images: 图像列表
            labels: 标签列表(int)
            label_names: 标签名称列表
        """
        if label_names:
            self.label_names = label_names

        features = np.array([self._extract(img) for img in images], dtype=np.float32)
        labels = np.array(labels, dtype=np.int32)

        self.model = self._create_model()

        if self.method == 'knn':
            self.model.train(features, cv2.ml.ROW_SAMPLE, labels)
        elif self.method == 'svm':
            self.model.train(features, cv2.ml.ROW_SAMPLE, labels)
            self.model.save("svm_model.xml")

        print(f"训练完成: {len(images)}样本, {len(set(labels))}类别, 方法={self.method}")

    def predict(self, image):
        """
        预测单张图像
        Returns:
            (label_id, label_name, confidence)
        """
        if self.model is None:
            raise RuntimeError("模型未训练")

        feature = np.array([self._extract(image)], dtype=np.float32)

        if self.method == 'knn':
            _, results, _, dist = self.model.findNearest(feature, k=3)
            label_id = int(results[0][0])
            confidence = 1.0 / (1.0 + dist[0][0])
        elif self.method == 'svm':
            _, results = self.model.predict(feature)
            label_id = int(results[0][0])
            confidence = 0.8  # SVM不直接给confidence

        name = self.label_names[label_id] if label_id < len(self.label_names) else str(label_id)
        return label_id, name, confidence

    def predict_batch(self, images):
        """批量预测"""
        return [self.predict(img) for img in images]

    def evaluate(self, images, labels):
        """评估准确率"""
        correct = 0
        total = len(images)
        for img, label in zip(images, labels):
            pred_id, _, _ = self.predict(img)
            if pred_id == label:
                correct += 1
        acc = correct / total if total > 0 else 0
        print(f"准确率: {correct}/{total} = {acc:.2%}")
        return acc

    def save(self, path):
        """保存模型"""
        data = {
            'method': self.method,
            'feature_type': self.feature_type,
            'label_names': self.label_names,
            'model_data': self.model,
        }
        with open(path, 'wb') as f:
            pickle.dump(data, f)
        print(f"模型已保存: {path}")

    def load(self, path):
        """加载模型"""
        with open(path, 'rb') as f:
            data = pickle.load(f)
        self.method = data['method']
        self.feature_type = data['feature_type']
        self.label_names = data['label_names']
        self.model = data['model_data']
        print(f"模型已加载: {path}, {len(self.label_names)}类别")

    @staticmethod
    def load_from_folder(folder_path, image_size=(64, 64)):
        """
        从文件夹结构加载数据集
        folder/class1/*.jpg, folder/class2/*.jpg
        """
        folder = Path(folder_path)
        images, labels, names = [], [], []
        class_dirs = sorted([d for d in folder.iterdir() if d.is_dir()])

        for idx, class_dir in enumerate(class_dirs):
            names.append(class_dir.name)
            for img_path in class_dir.glob("*"):
                if img_path.suffix.lower() in ('.jpg', '.png', '.bmp', '.jpeg'):
                    img = cv2.imread(str(img_path))
                    if img is not None:
                        img = cv2.resize(img, image_size)
                        images.append(img)
                        labels.append(idx)

        print(f"加载数据集: {len(images)}图像, {len(names)}类别")
        return images, labels, names


def demo():
    """简单分类演示(合成数据)"""
    np.random.seed(42)
    images, labels, names = [], [], ["Red", "Green", "Blue"]

    for i in range(50):
        for color_bgr, label in [((0, 0, 200), 0), ((0, 200, 0), 1), ((200, 0, 0), 2)]:
            img = np.full((64, 64, 3), color_bgr, dtype=np.uint8)
            noise = np.random.randint(-30, 30, img.shape, dtype=np.int16)
            img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            images.append(img)
            labels.append(label)

    clf = ImageClassifier(method='knn', feature_type='combined')
    clf.train(images, labels, names)
    acc = clf.evaluate(images, labels)
    print(f"标签: {names}")


if __name__ == "__main__":
    demo()
