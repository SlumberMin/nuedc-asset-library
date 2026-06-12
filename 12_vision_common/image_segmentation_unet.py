"""
U-Net 图像分割模块
轻量级编码解码器 + 跳跃连接
适用于电赛中的语义分割、前景提取、缺陷检测等场景
"""

import cv2
import numpy as np
import time


# ============================================================
# U-Net 轻量级模型（PyTorch 实现）
# ============================================================
class UNetLite:
    """
    轻量级 U-Net 模型
    编码器: 4层下采样（卷积 + 最大池化）
    解码器: 4层上采样（转置卷积 + 跳跃连接）
    """

    def __init__(self, in_channels=3, out_channels=1, features=[16, 32, 64, 128]):
        """
        参数:
            in_channels: 输入通道数（3=RGB, 1=灰度）
            out_channels: 输出通道数（分割类别数）
            features: 每层特征图数量
        """
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.features = features
        self.model = None
        self.use_pytorch = False

        # 尝试加载PyTorch模型
        try:
            import torch
            import torch.nn as nn
            self.use_pytorch = True
            self._build_pytorch_model(nn, features, in_channels, out_channels)
            print("[U-Net] 使用PyTorch模型")
        except ImportError:
            print("[U-Net] PyTorch未安装，使用OpenCV传统分割方案")

    def _build_pytorch_model(self, nn, features, in_ch, out_ch):
        """构建PyTorch U-Net模型"""
        import torch

        class DoubleConv(nn.Module):
            """双层卷积块: Conv-BN-ReLU x 2"""
            def __init__(self, in_c, out_c):
                super().__init__()
                self.conv = nn.Sequential(
                    nn.Conv2d(in_c, out_c, 3, padding=1, bias=False),
                    nn.BatchNorm2d(out_c),
                    nn.ReLU(inplace=True),
                    nn.Conv2d(out_c, out_c, 3, padding=1, bias=False),
                    nn.BatchNorm2d(out_c),
                    nn.ReLU(inplace=True),
                )

            def forward(self, x):
                return self.conv(x)

        class UNetModel(nn.Module):
            def __init__(self, in_c=3, out_c=1, f=None):
                super().__init__()
                if f is None:
                    f = [16, 32, 64, 128]

                # 编码器（下采样路径）
                self.encoders = nn.ModuleList()
                self.pool = nn.MaxPool2d(2, 2)

                prev_ch = in_c
                for feat in f:
                    self.encoders.append(DoubleConv(prev_ch, feat))
                    prev_ch = feat

                # 瓶颈层
                self.bottleneck = DoubleConv(f[-1], f[-1] * 2)

                # 解码器（上采样路径 + 跳跃连接）
                self.decoders = nn.ModuleList()
                self.upsamples = nn.ModuleList()

                prev_ch = f[-1] * 2
                for feat in reversed(f):
                    self.upsamples.append(
                        nn.ConvTranspose2d(prev_ch, feat, kernel_size=2, stride=2)
                    )
                    self.decoders.append(DoubleConv(prev_ch, feat))
                    prev_ch = feat

                # 输出层
                self.output_conv = nn.Conv2d(f[0], out_c, kernel_size=1)

            def forward(self, x):
                skip_connections = []

                # 编码
                for encoder in self.encoders:
                    x = encoder(x)
                    skip_connections.append(x)
                    x = self.pool(x)

                # 瓶颈
                x = self.bottleneck(x)

                # 解码
                skip_connections = skip_connections[::-1]
                for i in range(len(self.decoders)):
                    x = self.upsamples[i](x)
                    skip = skip_connections[i]

                    # 处理尺寸不匹配
                    if x.shape != skip.shape:
                        x = nn.functional.interpolate(x, size=skip.shape[2:])

                    # 跳跃连接：拼接
                    x = torch.cat([skip, x], dim=1)
                    x = self.decoders[i](x)

                return self.output_conv(x)

        self.model = UNetModel(in_ch, out_ch, features)

    def load_weights(self, weight_path):
        """加载预训练权重"""
        if self.use_pytorch and self.model is not None:
            import torch
            state_dict = torch.load(weight_path, map_location='cpu')
            self.model.load_state_dict(state_dict)
            self.model.eval()
            print(f"[U-Net] 已加载权重: {weight_path}")

    def predict_pytorch(self, image, target_size=None):
        """
        PyTorch推理
        参数:
            image: BGR图像 (H, W, 3), uint8
            target_size: 输出尺寸 (H, W)
        返回:
            mask: 分割掩码 (H, W), uint8, 0~255
        """
        import torch

        if target_size is None:
            target_size = image.shape[:2]

        # 预处理
        h, w = image.shape[:2]
        # 缩放到32的倍数
        new_h = (h // 32) * 32
        new_w = (w // 32) * 32

        img_resized = cv2.resize(image, (new_w, new_h))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_norm = img_rgb.astype(np.float32) / 255.0

        # 转为 tensor: (1, C, H, W)
        tensor = torch.from_numpy(img_norm.transpose(2, 0, 1)).unsqueeze(0).float()

        # 推理
        self.model.eval()
        with torch.no_grad():
            output = self.model(tensor)
            output = torch.sigmoid(output)

        # 后处理
        mask = output.squeeze().numpy()
        mask = (mask * 255).astype(np.uint8)
        mask = cv2.resize(mask, (target_size[1], target_size[0]))

        return mask

    def segment_opencv(self, image, method='grabcut'):
        """
        基于OpenCV的传统分割方案（无需深度学习模型）
        参数:
            image: BGR图像
            method: 分割方法
                'grabcut' - GrabCut前景分割
                'watershed' - 分水岭分割
                'kmeans' - K-means聚类分割
                'color' - 颜色空间分割
        返回:
            mask: 分割掩码 (H, W), uint8
        """
        if method == 'grabcut':
            return self._grabcut_segment(image)
        elif method == 'watershed':
            return self._watershed_segment(image)
        elif method == 'kmeans':
            return self._kmeans_segment(image)
        elif method == 'color':
            return self._color_segment(image)
        else:
            return self._grabcut_segment(image)

    def _grabcut_segment(self, image):
        """GrabCut前景分割"""
        h, w = image.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        bgd_model = np.zeros((1, 65), np.float64)
        fgd_model = np.zeros((1, 65), np.float64)

        # 使用图像中心区域作为前景初始估计
        margin = int(min(h, w) * 0.15)
        rect = (margin, margin, w - 2 * margin, h - 2 * margin)

        cv2.grabCut(image, mask, rect, bgd_model, fgd_model, 5, cv2.GC_INIT_WITH_RECT)

        # 将 GrabCut 掩码转为二值掩码
        mask2 = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)

        return mask2

    def _watershed_segment(self, image):
        """分水岭分割"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Otsu 二值化
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 形态学开运算去除噪声
        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

        # 确定背景区域
        sure_bg = cv2.dilate(opening, kernel, iterations=3)

        # 确定前景区域（距离变换）
        dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(dist_transform, 0.5 * dist_transform.max(), 255, 0)
        sure_fg = sure_fg.astype(np.uint8)

        # 未知区域
        unknown = cv2.subtract(sure_bg, sure_fg)

        # 标记连通分量
        _, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown == 255] = 0

        # 分水岭
        markers = cv2.watershed(image, markers)

        # 提取前景（标记为1的区域）
        mask = np.where(markers == 1, 255, 0).astype(np.uint8)

        return mask

    def _kmeans_segment(self, image, n_clusters=3):
        """K-means聚类分割"""
        h, w = image.shape[:2]

        # 准备数据
        data = image.reshape(-1, 3).astype(np.float32)

        # K-means
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
        _, labels, centers = cv2.kmeans(data, n_clusters, None, criteria, 10,
                                         cv2.KMEANS_PP_CENTERS)

        # 选择亮度最高/最低的聚类作为前景
        brightness = np.mean(centers, axis=1)
        # 最暗的聚类作为前景
        fg_cluster = np.argmin(brightness)

        mask = np.where(labels.flatten() == fg_cluster, 255, 0).reshape(h, w).astype(np.uint8)

        return mask

    def _color_segment(self, image, target_h_range=(0, 30), target_s_range=(50, 255)):
        """基于颜色范围的分割（HSV空间）"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 暖色范围（可自定义）
        lower = np.array([target_h_range[0], target_s_range[0], 50])
        upper = np.array([target_h_range[1], target_s_range[1], 255])

        mask = cv2.inRange(hsv, lower, upper)

        # 形态学后处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        return mask

    def segment(self, image, method='grabcut', target_size=None):
        """
        分割主入口
        参数:
            image: BGR图像
            method: 'pytorch' 使用深度学习, 或 'grabcut'/'watershed'/'kmeans'/'color'
            target_size: 输出尺寸
        返回:
            mask: 分割掩码 (H, W), uint8
        """
        if method == 'pytorch' and self.use_pytorch:
            return self.predict_pytorch(image, target_size)
        else:
            return self.segment_opencv(image, method)


# ============================================================
# 后处理与分析工具
# ============================================================
def refine_mask(mask, kernel_size=5, iterations=2):
    """精细化分割掩码"""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    # 填充孔洞
    refined = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    # 去除小噪点
    refined = cv2.morphologyEx(refined, cv2.MORPH_OPEN, kernel, iterations=1)

    return refined


def find_defects(mask, min_area=50, max_area=10000):
    """
    在分割掩码中查找缺陷区域
    参数:
        mask: 二值掩码
        min_area: 最小缺陷面积
        max_area: 最大缺陷面积
    返回:
        defects: list of dict, 包含 bbox, area, contour
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    defects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area <= area <= max_area:
            x, y, w, h = cv2.boundingRect(cnt)
            # 计算圆形度
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * area / max(perimeter * perimeter, 1e-6)

            defects.append({
                'bbox': (x, y, x + w, y + h),
                'area': area,
                'contour': cnt,
                'circularity': circularity,
                'aspect_ratio': w / max(h, 1)
            })

    return defects


def compute_mask_stats(mask):
    """计算掩码统计信息"""
    total_pixels = mask.shape[0] * mask.shape[1]
    fg_pixels = np.count_nonzero(mask)

    return {
        'total_pixels': total_pixels,
        'foreground_pixels': fg_pixels,
        'background_pixels': total_pixels - fg_pixels,
        'fg_ratio': fg_pixels / max(total_pixels, 1),
    }


# ============================================================
# 可视化
# ============================================================
def draw_segmentation_result(image, mask, alpha=0.4):
    """
    将分割掩码叠加到原图上
    参数:
        image: 原图
        mask: 分割掩码
        alpha: 叠加透明度
    """
    result = image.copy()

    # 创建彩色掩码
    color_mask = np.zeros_like(image)
    color_mask[mask > 127] = [0, 255, 0]  # 绿色前景

    # 叠加
    mask_bool = mask > 127
    result[mask_bool] = cv2.addWeighted(
        image, 1 - alpha, color_mask, alpha, 0
    )[mask_bool]

    return result


def draw_defects(image, defects):
    """绘制检测到的缺陷区域"""
    result = image.copy()

    for i, d in enumerate(defects):
        x1, y1, x2, y2 = d['bbox']
        area = d['area']
        circ = d['circularity']

        color = (0, 0, 255) if circ > 0.7 else (0, 165, 255)  # 圆形缺陷红色，否则橙色

        cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
        cv2.putText(result, f"A:{area} C:{circ:.2f}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    return result


# ============================================================
# 使用示例
# ============================================================
def demo_camera():
    """摄像头实时分割演示"""
    seg = UNetLite()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 分割方法选择
    methods = ['grabcut', 'watershed', 'kmeans']
    method_idx = 0

    print("U-Net 图像分割演示")
    print("按键: 'm'切换方法, 'q'退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        method = methods[method_idx % len(methods)]

        t_start = time.time()
        mask = seg.segment(frame, method=method)
        t_cost = time.time() - t_start

        # 细化掩码
        mask = refine_mask(mask)

        # 可视化
        result = draw_segmentation_result(frame, mask)

        fps = 1.0 / max(t_cost, 1e-6)
        stats = compute_mask_stats(mask)

        cv2.putText(result, f"Method: {method}  FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f"FG: {stats['fg_ratio']*100:.1f}%", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Segmentation", result)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            method_idx += 1

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path, method='kmeans'):
    """静态图像分割演示"""
    seg = UNetLite()

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图像: {image_path}")
        return

    t_start = time.time()
    mask = seg.segment(frame, method=method)
    t_cost = time.time() - t_start

    mask = refine_mask(mask)

    # 查找缺陷
    defects = find_defects(mask, min_area=100)
    print(f"分割耗时: {t_cost*1000:.1f}ms")
    print(f"检测到 {len(defects)} 个区域")

    stats = compute_mask_stats(mask)
    print(f"前景占比: {stats['fg_ratio']*100:.1f}%")

    result = draw_segmentation_result(frame, mask)
    result = draw_defects(result, defects)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
