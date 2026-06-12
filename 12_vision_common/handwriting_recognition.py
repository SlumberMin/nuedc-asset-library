"""
手写识别模块 - MNIST风格CNN + 轮廓提取 + 分类
适用于：数字识别、手写输入、电赛手写数字识别题等场景
依赖：pip install opencv-python numpy
（深度学习：pip install tensorflow 或 pip install torch）
"""
import cv2
import numpy as np
import os


class HandwritingRecognizer:
    """手写数字/字符识别器"""

    def __init__(self, model_path=None, method='knn'):
        """
        初始化识别器
        Args:
            model_path: CNN模型路径（.h5 或 .pt），method='cnn'时需要
            method: 'knn' 使用KNN模板匹配, 'cnn' 使用CNN神经网络, 'contour' 使用轮廓特征
        """
        self.method = method
        self.model = None

        if method == 'cnn':
            if model_path and os.path.exists(model_path):
                self._load_cnn_model(model_path)
            else:
                print("CNN模型文件不存在，回退到KNN模式")
                self.method = 'knn'

        if self.method == 'knn':
            # 使用OpenCV内置KNN
            self.knn = cv2.ml.KNearest_create()
            self._train_knn_with_mnist_sample()

    def _load_cnn_model(self, model_path):
        """加载CNN模型"""
        try:
            import tensorflow as tf
            self.model = tf.keras.models.load_model(model_path)
            print(f"CNN模型加载成功: {model_path}")
        except ImportError:
            try:
                import torch
                self.model = torch.load(model_path)
                self.model.eval()
                print(f"PyTorch模型加载成功: {model_path}")
            except ImportError:
                print("需要安装 tensorflow 或 torch")
                self.method = 'knn'

    def _train_knn_with_mnist_sample(self):
        """用合成数据初始化KNN（实际使用时替换为真实MNIST数据）"""
        # 生成简单的合成训练数据
        # 每个数字生成几个样本特征向量
        samples = []
        labels = []
        np.random.seed(42)
        for digit in range(10):
            for _ in range(20):
                # 模拟28x28图像展平后的特征
                img = np.zeros((28, 28), dtype=np.uint8)
                # 简单图案
                if digit == 0:
                    cv2.circle(img, (14, 14), 10, 255, 2)
                elif digit == 1:
                    cv2.line(img, (14, 4), (14, 24), 255, 2)
                elif digit == 2:
                    cv2.ellipse(img, (14, 10), (8, 5), 0, 180, 360, 255, 2)
                    cv2.line(img, (6, 20), (22, 24), 255, 2)
                else:
                    # 其他数字用随机噪声模拟
                    img = np.random.randint(0, 2, (28, 28)).astype(np.uint8) * 255

                # 添加随机扰动
                noise = np.random.randint(-10, 10, img.shape).astype(np.int16)
                img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

                samples.append(img.flatten().astype(np.float32))
                labels.append(digit)

        samples = np.array(samples)
        labels = np.array(labels).reshape(-1, 1).astype(np.float32)

        self.knn.train(samples, cv2.ml.ROW_SAMPLE, labels)

    def preprocess_digit(self, image):
        """
        数字图像预处理（MNIST格式标准化）
        Args:
            image: 灰度图像（白底黑字或黑底白字）
        Returns:
            28x28标准化图像
        """
        # 确保灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # 确保白底黑字
        if np.mean(gray) < 127:
            gray = 255 - gray

        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 查找轮廓并裁剪到数字区域
        contours, _ = cv2.findContours(255 - binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            all_points = np.vstack(contours)
            x, y, w, h = cv2.boundingRect(all_points)
            digit = binary[y:y + h, x:x + w]

            # 保持宽高比resize到20x20
            max_dim = max(w, h)
            scale = 20.0 / max_dim
            new_w, new_h = int(w * scale), int(h * scale)
            digit = cv2.resize(digit, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # 居中放到28x28画布
            canvas = np.zeros((28, 28), dtype=np.uint8)
            x_offset = (28 - new_w) // 2
            y_offset = (28 - new_h) // 2
            canvas[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = digit
            return canvas
        else:
            return cv2.resize(binary, (28, 28))

    def extract_features(self, image):
        """
        提取特征向量（轮廓+Hu矩+像素分布）
        Args:
            image: 28x28灰度图
        Returns:
            特征向量
        """
        img = image.copy()
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 特征1：像素展平特征（降采样到14x14=196维）
        small = cv2.resize(img, (14, 14))
        pixel_feat = small.flatten().astype(np.float32) / 255.0

        # 特征2：Hu矩（7维，旋转不变）
        moments = cv2.moments(img)
        hu_moments = cv2.HuMoments(moments).flatten()
        hu_moments = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)

        # 特征3：投影特征（水平+垂直投影各28维）
        h_proj = np.sum(img, axis=1).astype(np.float32) / (255 * 28)
        v_proj = np.sum(img, axis=0).astype(np.float32) / (255 * 28)

        return np.concatenate([pixel_feat, hu_moments, h_proj, v_proj]).astype(np.float32)

    def recognize_digit(self, image):
        """
        识别单个手写数字
        Args:
            image: 数字图像（灰度或彩色）
        Returns:
            (digit, confidence): 识别结果和置信度
        """
        processed = self.preprocess_digit(image)

        if self.method == 'knn':
            sample = processed.flatten().astype(np.float32).reshape(1, -1)
            ret, results, neighbours, dist = self.knn.findNearest(sample, k=5)
            digit = int(results[0][0])
            # 置信度基于邻居距离
            conf = 1.0 / (1.0 + np.mean(dist))
            return digit, conf

        elif self.method == 'cnn':
            img_input = processed.reshape(1, 28, 28, 1).astype(np.float32) / 255.0
            if hasattr(self.model, 'predict'):
                pred = self.model.predict(img_input, verbose=0)
                digit = int(np.argmax(pred))
                conf = float(np.max(pred))
                return digit, conf

        elif self.method == 'contour':
            features = self.extract_features(processed)
            # 简单的特征距离匹配
            return self._contour_classify(features)

        return -1, 0.0

    def _contour_classify(self, features):
        """基于轮廓特征的简单分类"""
        # 使用Hu矩的前几个特征做简单判断
        # 实际应用中应训练分类器
        return 0, 0.5  # 占位

    def segment_digits(self, image, min_area=100):
        """
        从图像中分割出多个手写数字
        Args:
            image: 包含多个数字的图像
            min_area: 最小轮廓面积
        Returns:
            digit_images: 分割出的数字图像列表（按x坐标排序）
            bounding_boxes: 对应边界框
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # 二值化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 形态学操作去噪
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        digit_images = []
        bounding_boxes = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            # 过滤过宽或过窄的区域
            aspect_ratio = w / max(h, 1)
            if aspect_ratio > 3 or aspect_ratio < 0.1:
                continue

            digit = gray[y:y + h, x:x + w]
            digit_images.append(digit)
            bounding_boxes.append((x, y, w, h))

        # 按x坐标排序（从左到右）
        sorted_pairs = sorted(zip(bounding_boxes, digit_images), key=lambda p: p[0][0])
        if sorted_pairs:
            bounding_boxes, digit_images = zip(*sorted_pairs)
            return list(digit_images), list(bounding_boxes)
        return [], []

    def recognize_sequence(self, image):
        """
        识别图像中的多个手写数字序列
        Args:
            image: 包含多个数字的图像
        Returns:
            text: 识别结果字符串
            annotated: 标注图像
        """
        digit_images, boxes = self.segment_digits(image)
        text = ''
        annotated = image.copy() if len(image.shape) == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        for digit_img, (x, y, w, h) in zip(digit_images, boxes):
            digit, conf = self.recognize_digit(digit_img)
            text += str(digit)
            color = (0, 255, 0) if conf > 0.5 else (0, 0, 255)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            cv2.putText(annotated, f'{digit}({conf:.2f})', (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.putText(annotated, f'Result: {text}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return text, annotated


def build_mnist_cnn(save_path='mnist_cnn.h5'):
    """
    构建并训练MNIST CNN模型
    需要 tensorflow: pip install tensorflow
    Args:
        save_path: 模型保存路径
    Returns:
        训练好的模型
    """
    try:
        import tensorflow as tf
        from tensorflow.keras import layers, models

        # 加载MNIST数据集
        (x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()

        # 预处理
        x_train = x_train.reshape(-1, 28, 28, 1).astype('float32') / 255.0
        x_test = x_test.reshape(-1, 28, 28, 1).astype('float32') / 255.0

        # 构建CNN
        model = models.Sequential([
            layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)),
            layers.MaxPooling2D((2, 2)),
            layers.Conv2D(64, (3, 3), activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Conv2D(64, (3, 3), activation='relu'),
            layers.Flatten(),
            layers.Dense(128, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(10, activation='softmax')
        ])

        model.compile(optimizer='adam',
                      loss='sparse_categorical_crossentropy',
                      metrics=['accuracy'])

        model.fit(x_train, y_train, epochs=5, batch_size=128,
                  validation_data=(x_test, y_test))

        test_loss, test_acc = model.evaluate(x_test, y_test)
        print(f"测试准确率: {test_acc:.4f}")

        model.save(save_path)
        print(f"模型已保存: {save_path}")
        return model

    except ImportError:
        print("需要安装 tensorflow: pip install tensorflow")
        return None


# ============== 使用示例 ==============
if __name__ == '__main__':
    print("=== 手写数字识别示例 ===")

    # 创建识别器（KNN模式，无需额外模型）
    recognizer = HandwritingRecognizer(method='knn')

    # 创建测试图像：手写数字 "42"
    test_img = np.zeros((100, 120), dtype=np.uint8)

    # 绘制数字 "4"
    cv2.line(test_img, (10, 60), (10, 20), 255, 3)
    cv2.line(test_img, (10, 20), (30, 50), 255, 3)
    cv2.line(test_img, (5, 40), (35, 40), 255, 3)
    cv2.line(test_img, (30, 20), (30, 70), 255, 3)

    # 绘制数字 "2"
    cv2.ellipse(test_img, (75, 25), (15, 10), 0, 180, 360, 255, 3)
    cv2.line(test_img, (90, 30), (60, 60), 255, 3)
    cv2.line(test_img, (55, 60), (95, 60), 255, 3)

    # 单个数字识别
    digit_img = test_img[5:75, 5:40]
    digit, conf = recognizer.recognize_digit(digit_img)
    print(f"单个数字识别: {digit}, 置信度: {conf:.3f}")

    # 序列识别
    text, annotated = recognizer.recognize_sequence(test_img)
    print(f"序列识别结果: '{text}'")
    cv2.imwrite('handwriting_result.jpg', annotated)
    print("结果已保存到 handwriting_result.jpg")

    # 训练CNN模型（可选，需要tensorflow）
    # model = build_mnist_cnn('mnist_cnn.h5')
    # recognizer = HandwritingRecognizer(model_path='mnist_cnn.h5', method='cnn')
