"""
视觉异常检测模块
基于自编码器重建误差 + 阈值分割
适用于电赛中的产品缺陷检测、异常区域定位等场景
"""

import cv2
import numpy as np
import time
import os
from collections import deque


# ============================================================
# 自编码器模型（PyTorch 实现）
# ============================================================
class ConvAutoencoder:
    """
    卷积自编码器 - 用于异常检测
    训练时只学习正常样本的特征表示
    异常样本会产生较大的重建误差
    """

    def __init__(self, input_size=(128, 128), latent_dim=128):
        """
        参数:
            input_size: 输入图像尺寸 (H, W)
            latent_dim: 潜在空间维度
        """
        self.input_size = input_size
        self.latent_dim = latent_dim
        self.model = None
        self.use_pytorch = False

        try:
            import torch
            import torch.nn as nn
            self.use_pytorch = True
            self._build_model(nn)
            print("[自编码器] 使用PyTorch模型")
        except ImportError:
            print("[自编码器] PyTorch未安装，使用传统图像方法")

    def _build_model(self, nn):
        """构建PyTorch自编码器模型"""
        import torch

        class Encoder(nn.Module):
            def __init__(self):
                super().__init__()
                # 编码器: 逐步压缩
                self.conv = nn.Sequential(
                    # (3, 128, 128) -> (32, 64, 64)
                    nn.Conv2d(3, 32, 3, stride=2, padding=1),
                    nn.BatchNorm2d(32),
                    nn.ReLU(inplace=True),
                    # (32, 64, 64) -> (64, 32, 32)
                    nn.Conv2d(32, 64, 3, stride=2, padding=1),
                    nn.BatchNorm2d(64),
                    nn.ReLU(inplace=True),
                    # (64, 32, 32) -> (128, 16, 16)
                    nn.Conv2d(64, 128, 3, stride=2, padding=1),
                    nn.BatchNorm2d(128),
                    nn.ReLU(inplace=True),
                    # (128, 16, 16) -> (256, 8, 8)
                    nn.Conv2d(128, 256, 3, stride=2, padding=1),
                    nn.BatchNorm2d(256),
                    nn.ReLU(inplace=True),
                )
                # 全连接层映射到潜在空间
                self.fc = nn.Linear(256 * 8 * 8, 128)

            def forward(self, x):
                x = self.conv(x)
                x = x.view(x.size(0), -1)
                x = self.fc(x)
                return x

        class Decoder(nn.Module):
            def __init__(self):
                super().__init__()
                self.fc = nn.Linear(128, 256 * 8 * 8)
                self.deconv = nn.Sequential(
                    # (256, 8, 8) -> (128, 16, 16)
                    nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),
                    nn.BatchNorm2d(128),
                    nn.ReLU(inplace=True),
                    # (128, 16, 16) -> (64, 32, 32)
                    nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),
                    nn.BatchNorm2d(64),
                    nn.ReLU(inplace=True),
                    # (64, 32, 32) -> (32, 64, 64)
                    nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),
                    nn.BatchNorm2d(32),
                    nn.ReLU(inplace=True),
                    # (32, 64, 64) -> (3, 128, 128)
                    nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
                    nn.Sigmoid(),
                )

            def forward(self, x):
                x = self.fc(x)
                x = x.view(x.size(0), 256, 8, 8)
                x = self.deconv(x)
                return x

        class AE(nn.Module):
            def __init__(self):
                super().__init__()
                self.encoder = Encoder()
                self.decoder = Decoder()

            def forward(self, x):
                z = self.encoder(x)
                out = self.decoder(z)
                return out

        self.model = AE()

    def load_weights(self, weight_path):
        """加载预训练权重"""
        if self.use_pytorch and self.model is not None:
            import torch
            state_dict = torch.load(weight_path, map_location='cpu')
            self.model.load_state_dict(state_dict)
            self.model.eval()
            print(f"[自编码器] 已加载权重: {weight_path}")

    def preprocess(self, image):
        """图像预处理"""
        h, w = self.input_size
        img = cv2.resize(image, (w, h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        return img

    def predict_pytorch(self, image):
        """
        PyTorch推理，返回重建图像和误差图
        返回:
            reconstructed: 重建图像 (H, W, 3), float32, 0~1
            error_map: 重建误差图 (H, W), float32, 0~1
        """
        import torch

        orig_h, orig_w = image.shape[:2]
        img = self.preprocess(image)

        tensor = torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0).float()

        self.model.eval()
        with torch.no_grad():
            reconstructed = self.model(tensor)

        recon_np = reconstructed.squeeze().numpy().transpose(1, 2, 0)  # (H, W, 3)

        # 计算逐像素重建误差 (L2距离)
        error = np.mean((img - recon_np) ** 2, axis=2)  # (H, W)

        # 归一化
        if error.max() > 0:
            error = error / error.max()

        # 缩放回原始尺寸
        error_map = cv2.resize(error, (orig_w, orig_h))

        return recon_np, error_map


# ============================================================
# 传统图像异常检测（无需深度学习）
# ============================================================
class TraditionalAnomalyDetector:
    """
    基于传统图像处理的异常检测
    方法: 统计模型 + 纹理分析 + 形态学
    """

    def __init__(self, patch_size=32, threshold_method='otsu'):
        """
        参数:
            patch_size: 分析块大小
            threshold_method: 阈值方法 'otsu' / 'adaptive' / 'fixed'
        """
        self.patch_size = patch_size
        self.threshold_method = threshold_method
        self.reference_stats = None  # 正常样本的统计特征

    def fit(self, normal_images):
        """
        用正常样本拟合统计模型
        参数:
            normal_images: 正常图像列表 [image1, image2, ...]
        """
        all_features = []

        for img in normal_images:
            features = self._extract_features(img)
            all_features.append(features)

        all_features = np.array(all_features)

        # 计算正常样本的均值和标准差
        self.reference_stats = {
            'mean': np.mean(all_features, axis=0),
            'std': np.std(all_features, axis=0) + 1e-6,
            'min': np.min(all_features, axis=0),
            'max': np.max(all_features, axis=0),
        }

        print(f"[异常检测] 已用 {len(normal_images)} 张正常样本拟合模型")

    def _extract_features(self, image):
        """提取图像特征向量"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        features = []

        # 1. 灰度统计特征
        features.append(np.mean(gray))
        features.append(np.std(gray))
        features.append(np.median(gray))

        # 2. 直方图特征（简化）
        hist = cv2.calcHist([gray], [0], None, [32], [0, 256]).flatten()
        hist = hist / max(hist.sum(), 1)
        features.extend(hist.tolist())

        # 3. 纹理特征（LBP简化版）
        lbp = self._simple_lbp(gray)
        lbp_hist = cv2.calcHist([lbp.astype(np.uint8)], [0], None, [16], [0, 256]).flatten()
        lbp_hist = lbp_hist / max(lbp_hist.sum(), 1)
        features.extend(lbp_hist.tolist())

        # 4. 梯度特征
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        grad_mag = np.sqrt(gx**2 + gy**2)
        features.append(np.mean(grad_mag))
        features.append(np.std(grad_mag))

        return np.array(features, dtype=np.float32)

    def _simple_lbp(self, gray):
        """简化的LBP纹理特征"""
        h, w = gray.shape
        lbp = np.zeros_like(gray)

        for i in range(1, h - 1):
            for j in range(1, w - 1):
                center = gray[i, j]
                code = 0
                code |= (1 << 7) if gray[i-1, j-1] >= center else 0
                code |= (1 << 6) if gray[i-1, j] >= center else 0
                code |= (1 << 5) if gray[i-1, j+1] >= center else 0
                code |= (1 << 4) if gray[i, j+1] >= center else 0
                code |= (1 << 3) if gray[i+1, j+1] >= center else 0
                code |= (1 << 2) if gray[i+1, j] >= center else 0
                code |= (1 << 1) if gray[i+1, j-1] >= center else 0
                code |= (1 << 0) if gray[i, j-1] >= center else 0
                lbp[i, j] = code

        return lbp

    def detect_statistical(self, image):
        """
        基于统计模型的异常检测
        计算图像特征与正常分布的马氏距离
        返回: 异常得分 (越大越异常)
        """
        features = self._extract_features(image)

        if self.reference_stats is not None:
            # 马氏距离
            diff = features - self.reference_stats['mean']
            z_scores = diff / self.reference_stats['std']
            anomaly_score = np.sqrt(np.mean(z_scores ** 2))
        else:
            # 没有参考模型时，用简单的对比度/噪声评估
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            anomaly_score = np.std(gray) / 128.0

        return float(anomaly_score)

    def detect_local(self, image, kernel_size=15):
        """
        局部异常检测 - 基于局部统计
        返回: 异常热力图 (H, W), float32, 0~1
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        gray = gray.astype(np.float32)

        # 局部均值和标准差
        local_mean = cv2.blur(gray, (kernel_size, kernel_size))
        local_sq_mean = cv2.blur(gray ** 2, (kernel_size, kernel_size))
        local_std = np.sqrt(np.maximum(local_sq_mean - local_mean ** 2, 0))

        # 偏离局部统计的程度
        deviation = np.abs(gray - local_mean) / (local_std + 1e-6)

        # 归一化
        if deviation.max() > 0:
            deviation = deviation / deviation.max()

        return deviation.astype(np.float32)

    def detect_edge_anomaly(self, image):
        """
        基于边缘异常的检测 - 寻找异常的边缘模式
        返回: 异常边缘图 (H, W), uint8
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        # Canny边缘
        edges = cv2.Canny(gray, 50, 150)

        # 形态学处理：连接断裂边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=1)

        # 查找封闭轮廓（可能是缺陷边界）
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        anomaly_map = np.zeros_like(gray)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 50 < area < 5000:  # 中等大小的区域可能是缺陷
                # 计算区域内的边缘密度
                mask = np.zeros_like(gray)
                cv2.drawContours(mask, [cnt], -1, 255, -1)
                edge_density = np.count_nonzero(cv2.bitwise_and(edges, mask)) / max(np.count_nonzero(mask), 1)

                if edge_density > 0.1:  # 边缘密集的区域
                    cv2.drawContours(anomaly_map, [cnt], -1, 255, -1)

        return anomaly_map


# ============================================================
# 异常检测管线
# ============================================================
class AnomalyDetectionPipeline:
    """
    视觉异常检测完整管线
    整合多种异常检测方法
    """

    def __init__(self, method='traditional', autoencoder=None):
        """
        参数:
            method: 'traditional' / 'autoencoder' / 'combined'
            autoencoder: ConvAutoencoder 实例（当 method='autoencoder' 时需要）
        """
        self.method = method
        self.autoencoder = autoencoder
        self.trad_detector = TraditionalAnomalyDetector()

        # 阈值参数
        self.error_threshold = 0.3      # 重建误差阈值
        self.local_threshold = 0.5      # 局部异常阈值
        self.min_defect_area = 50       # 最小缺陷面积

    def fit_normal(self, normal_images):
        """用正常样本拟合模型"""
        self.trad_detector.fit(normal_images)

    def detect_anomalies(self, image):
        """
        异常检测主入口
        参数:
            image: BGR图像
        返回:
            result: dict, 包含:
                - anomaly_map: 异常热力图 (H, W), float32, 0~1
                - binary_mask: 二值化异常掩码 (H, W), uint8
                - defect_regions: 缺陷区域列表
                - anomaly_score: 整体异常得分
        """
        h, w = image.shape[:2]

        if self.method == 'autoencoder' and self.autoencoder:
            # 自编码器方法
            _, error_map = self.autoencoder.predict_pytorch(image)
            anomaly_map = error_map

        elif self.method == 'traditional':
            # 传统方法：局部统计异常
            local_map = self.trad_detector.detect_local(image)
            edge_map = self.trad_detector.detect_edge_anomaly(image).astype(np.float32) / 255.0

            # 融合多种异常指标
            anomaly_map = 0.6 * local_map + 0.4 * edge_map

        elif self.method == 'combined' and self.autoencoder:
            # 组合方法
            _, ae_error = self.autoencoder.predict_pytorch(image)
            local_map = self.trad_detector.detect_local(image)

            # 融合
            anomaly_map = 0.5 * ae_error + 0.5 * local_map

        else:
            anomaly_map = self.trad_detector.detect_local(image)

        # 归一化
        if anomaly_map.max() > anomaly_map.min():
            anomaly_map = (anomaly_map - anomaly_map.min()) / (anomaly_map.max() - anomaly_map.min())

        # 二值化
        binary_mask = self._threshold(anomaly_map)

        # 查找缺陷区域
        defect_regions = self._find_defect_regions(binary_mask, anomaly_map)

        # 整体异常得分
        anomaly_score = float(np.mean(anomaly_map))

        return {
            'anomaly_map': anomaly_map,
            'binary_mask': binary_mask,
            'defect_regions': defect_regions,
            'anomaly_score': anomaly_score
        }

    def _threshold(self, anomaly_map):
        """
        对异常热力图进行阈值分割
        返回: 二值掩码 (H, W), uint8
        """
        map_uint8 = (anomaly_map * 255).astype(np.uint8)

        if self.trad_detector.threshold_method == 'otsu':
            _, binary = cv2.threshold(map_uint8, 0, 255,
                                       cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        elif self.trad_detector.threshold_method == 'adaptive':
            binary = cv2.adaptiveThreshold(
                map_uint8, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2
            )
        else:
            thresh_val = int(self.error_threshold * 255)
            _, binary = cv2.threshold(map_uint8, thresh_val, 255, cv2.THRESH_BINARY)

        # 形态学后处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

        return binary

    def _find_defect_regions(self, binary_mask, anomaly_map):
        """从二值掩码中提取缺陷区域"""
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        regions = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_defect_area:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)

            # 计算该区域的平均异常得分
            mask_roi = np.zeros_like(anomaly_map)
            cv2.drawContours(mask_roi, [cnt], -1, 1, -1)
            region_score = float(np.sum(anomaly_map * mask_roi) / max(np.sum(mask_roi), 1))

            # 计算形状特征
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * area / max(perimeter * perimeter, 1e-6)

            regions.append({
                'bbox': (x, y, x + bw, y + bh),
                'area': area,
                'score': region_score,
                'circularity': circularity,
                'contour': cnt,
                'severity': 'high' if region_score > 0.7 else 'medium' if region_score > 0.4 else 'low'
            })

        # 按异常得分排序
        regions.sort(key=lambda r: r['score'], reverse=True)

        return regions


# ============================================================
# 可视化
# ============================================================
def draw_anomaly_result(image, result, show_heatmap=True):
    """
    绘制异常检测结果
    参数:
        image: 原图
        result: detect_anomalies 返回的结果
        show_heatmap: 是否显示热力图叠加
    """
    h, w = image.shape[:2]

    if show_heatmap:
        # 将热力图叠加到原图上
        heatmap = (result['anomaly_map'] * 255).astype(np.uint8)
        heatmap_color = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(image, 0.6, heatmap_color, 0.4, 0)
    else:
        overlay = image.copy()

    # 绘制缺陷区域
    for i, region in enumerate(result['defect_regions']):
        x1, y1, x2, y2 = region['bbox']
        severity = region['severity']
        score = region['score']

        # 根据严重程度选择颜色
        if severity == 'high':
            color = (0, 0, 255)     # 红色
        elif severity == 'medium':
            color = (0, 165, 255)   # 橙色
        else:
            color = (0, 255, 255)   # 黄色

        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        label = f"{severity.upper()} S:{score:.2f}"
        cv2.putText(overlay, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # 显示整体信息
    score = result['anomaly_score']
    n_defects = len(result['defect_regions'])
    status = "NG" if n_defects > 0 else "OK"
    status_color = (0, 0, 255) if status == "NG" else (0, 255, 0)

    cv2.putText(overlay, f"Status: {status}  Score: {score:.3f}  Defects: {n_defects}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

    return overlay


def draw_side_by_side(image, result):
    """原图和结果并排显示"""
    h, w = image.shape[:2]

    # 创建并排图像
    canvas = np.zeros((h, w * 2, 3), dtype=np.uint8)
    canvas[:, :w] = image

    result_img = draw_anomaly_result(image, result)
    canvas[:, w:] = result_img

    # 中间分隔线
    cv2.line(canvas, (w, 0), (w, h), (255, 255, 255), 2)

    cv2.putText(canvas, "Original", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(canvas, "Anomaly Detection", (w + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    return canvas


# ============================================================
# 使用示例
# ============================================================
def demo_camera():
    """摄像头实时异常检测演示"""
    pipeline = AnomalyDetectionPipeline(method='traditional')

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 采集正常样本进行校准
    print("正在采集正常样本进行校准...")
    print("请将摄像头对准正常场景，按 'f' 完成校准")

    normal_frames = []
    calibrated = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()

        if not calibrated:
            cv2.putText(display, f"Calibrating... Collected: {len(normal_frames)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(display, "Press 'f' to finish calibration",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if len(normal_frames) > 0 and len(normal_frames) % 10 == 0:
                print(f"  已采集 {len(normal_frames)} 帧")
        else:
            # 异常检测
            t_start = time.time()
            result = pipeline.detect_anomalies(frame)
            t_cost = time.time() - t_start

            display = draw_anomaly_result(frame, result)

            fps = 1.0 / max(t_cost, 1e-6)
            cv2.putText(display, f"FPS: {fps:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Anomaly Detection", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' ') and not calibrated:
            normal_frames.append(frame.copy())
        elif key == ord('f') and not calibrated:
            if len(normal_frames) > 0:
                pipeline.fit_normal(normal_frames)
                calibrated = True
                print(f"校准完成！共使用 {len(normal_frames)} 帧正常样本")
            else:
                print("请先采集正常样本（按空格键）")

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path, normal_dir=None):
    """
    静态图像异常检测演示
    参数:
        image_path: 待检测图像路径
        normal_dir: 正常样本目录（用于拟合模型，可选）
    """
    pipeline = AnomalyDetectionPipeline(method='traditional')

    # 如果有正常样本目录，先拟合模型
    if normal_dir and os.path.isdir(normal_dir):
        normal_images = []
        for fname in os.listdir(normal_dir):
            fpath = os.path.join(normal_dir, fname)
            img = cv2.imread(fpath)
            if img is not None:
                normal_images.append(img)

        if normal_images:
            pipeline.fit_normal(normal_images)

    # 检测
    image = cv2.imread(image_path)
    if image is None:
        print(f"无法读取图像: {image_path}")
        return

    t_start = time.time()
    result = pipeline.detect_anomalies(image)
    t_cost = time.time() - t_start

    print(f"检测耗时: {t_cost*1000:.1f}ms")
    print(f"异常得分: {result['anomaly_score']:.4f}")
    print(f"缺陷数量: {len(result['defect_regions'])}")

    for i, r in enumerate(result['defect_regions']):
        print(f"  缺陷[{i}]: bbox={r['bbox']}, area={r['area']}, "
              f"score={r['score']:.3f}, severity={r['severity']}")

    # 可视化
    result_img = draw_anomaly_result(image, result)
    side_by_side = draw_side_by_side(image, result)

    cv2.imshow("Anomaly Result", result_img)
    cv2.imshow("Side by Side", side_by_side)

    # 显示异常热力图
    heatmap = (result['anomaly_map'] * 255).astype(np.uint8)
    heatmap_color = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
    cv2.imshow("Anomaly Heatmap", heatmap_color)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        normal_dir = sys.argv[2] if len(sys.argv) > 2 else None
        demo_image(sys.argv[1], normal_dir)
    else:
        demo_camera()
