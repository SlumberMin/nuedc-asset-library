"""
实时风格迁移模块 - 前馈网络 + 多风格 + 视频处理
功能：基于OpenCV DNN实现快速风格迁移（无需PyTorch/TensorFlow）
依赖：opencv-python, numpy
适用：电赛中艺术效果、图像风格化、视觉演示等场景

注意：需要下载预训练ONNX模型才能使用DNN模式
      本模块同时提供纯OpenCV的轻量级风格模拟作为替代
"""

import cv2
import numpy as np
import os


# ============================================================
# 纯OpenCV风格迁移（无需深度学习模型）
# ============================================================

class StyleTransferCV:
    """
    基于传统图像处理的风格迁移
    不依赖深度学习框架，纯OpenCV实现
    支持多种艺术风格效果
    """

    # 风格列表
    STYLES = ['oil_painting', 'sketch', 'watercolor', 'cartoon',
              'mosaic', 'emboss', 'vintage', 'pencil', 'pop_art']

    def __init__(self):
        """初始化风格迁移器"""
        self.style_params = {
            'oil_painting': {'kernel_size': 5, 'sigma': 0.6},
            'sketch': {'sigma': 21, 'strength': 256},
            'watercolor': {'sigma_s': 60, 'sigma_r': 0.4},
            'cartoon': {'sigma_s': 150, 'sigma_r': 0.15},
            'mosaic': {'block_size': 10},
            'emboss': {'direction': 'se'},
            'vintage': {'sepia_strength': 0.8, 'vignette': 0.5},
            'pencil': {'sigma': 25, 'blend': 0.8},
            'pop_art': {'quantize': 4},
        }

    def transfer(self, frame, style, **kwargs):
        """
        执行风格迁移
        Args:
            frame: BGR输入图像
            style: 风格名称
            **kwargs: 风格参数覆盖
        Returns:
            result: 风格化图像
        """
        method = getattr(self, f'_style_{style}', None)
        if method is None:
            print(f"未知风格: {style}, 可选: {self.STYLES}")
            return frame
        return method(frame, **kwargs)

    def _style_oil_painting(self, frame, kernel_size=5, sigma=0.6, **kw):
        """油画效果：边缘保持平滑 + 色彩增强"""
        # 双边滤波（保边平滑）
        d = kernel_size * 2 + 1
        smooth = frame.copy()
        for _ in range(3):
            smooth = cv2.bilateralFilter(smooth, d, d * 2, d // 2)

        # 边缘检测
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.adaptiveThreshold(
            cv2.medianBlur(gray, 7), 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 2
        )
        edges_3ch = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # 合并
        result = cv2.bitwise_and(smooth, edges_3ch)

        # 增强色彩饱和度
        hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV).astype(np.float64)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * 1.3, 0, 255)
        result = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        return result

    def _style_sketch(self, frame, sigma=21, strength=256, **kw):
        """素描效果：铅笔画风格"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        inv = 255 - gray
        blur = cv2.GaussianBlur(inv, (sigma, sigma), 0)
        sketch = cv2.divide(gray, 255 - blur, scale=strength)
        return cv2.cvtColor(sketch, cv2.COLOR_GRAY2BGR)

    def _style_watercolor(self, frame, sigma_s=60, sigma_r=0.4, **kw):
        """水彩效果：边缘保持平滑 + 色彩扩散"""
        # 多次风格化滤波
        stylized = cv2.stylization(frame, sigma_s=sigma_s, sigma_s_color=sigma_s,
                                    sigma_r=sigma_r)
        stylized = cv2.stylization(stylized, sigma_s=sigma_s, sigma_s_color=sigma_s,
                                    sigma_r=sigma_r * 0.5)

        # 边缘叠加
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.adaptiveThreshold(
            cv2.medianBlur(gray, 3), 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 7, 3
        )
        edges_3ch = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        result = cv2.bitwise_and(stylized, edges_3ch)

        # 轻微模糊模拟水彩扩散
        result = cv2.GaussianBlur(result, (3, 3), 0)

        return result

    def _style_cartoon(self, frame, sigma_s=150, sigma_r=0.15, **kw):
        """卡通效果：色块化 + 粗边缘"""
        # 颜色量化
        data = frame.reshape((-1, 3)).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        _, labels, centers = cv2.kmeans(data, 8, None, criteria, 10,
                                         cv2.KMEANS_RANDOM_CENTERS)
        centers = np.uint8(centers)
        quantized = centers[labels.flatten()].reshape(frame.shape)

        # 双边滤波平滑
        smooth = cv2.bilateralFilter(quantized, 9, 75, 75)

        # 边缘
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 7)
        edges = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 5
        )
        edges_3ch = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        result = cv2.bitwise_and(smooth, edges_3ch)
        return result

    def _style_mosaic(self, frame, block_size=10, **kw):
        """马赛克效果"""
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (w // block_size, h // block_size),
                           interpolation=cv2.INTER_LINEAR)
        result = cv2.resize(small, (w, h), interpolation=cv2.INTER_NEAREST)
        return result

    def _style_emboss(self, frame, direction='se', **kw):
        """浮雕效果"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kernels = {
            'se': np.array([[-2, -1, 0], [-1, 1, 1], [0, 1, 2]]),
            'nw': np.array([[0, 1, 2], [-1, 1, 1], [-2, -1, 0]]),
            'e': np.array([[0, 0, 0], [-1, 1, 1], [0, 0, 0]]),
        }
        kernel = kernels.get(direction, kernels['se'])

        emboss = cv2.filter2D(gray, -1, kernel) + 128
        result = cv2.cvtColor(emboss, cv2.COLOR_GRAY2BGR)

        # 可选：与原图融合保留颜色
        result = cv2.addWeighted(frame, 0.3, result, 0.7, 0)

        return result

    def _style_vintage(self, frame, sepia_strength=0.8, vignette=0.5, **kw):
        """复古效果：褐色调 + 暗角 + 噪声"""
        # 棕褐色调
        sepia_kernel = np.array([
            [0.272, 0.534, 0.131],
            [0.349, 0.686, 0.168],
            [0.393, 0.769, 0.189]
        ])
        sepia = cv2.transform(frame.astype(np.float64), sepia_kernel.T)
        sepia = np.clip(sepia, 0, 255).astype(np.uint8)
        result = cv2.addWeighted(frame, 1 - sepia_strength, sepia, sepia_strength, 0)

        # 暗角
        h, w = result.shape[:2]
        Y, X = np.ogrid[:h, :w]
        cx, cy = w / 2, h / 2
        dist = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2)
        max_dist = np.sqrt(cx ** 2 + cy ** 2)
        vignette_mask = 1 - vignette * (dist / max_dist) ** 2
        vignette_mask = np.clip(vignette_mask, 0, 1)
        for c in range(3):
            result[:, :, c] = np.clip(
                result[:, :, c].astype(float) * vignette_mask, 0, 255
            ).astype(np.uint8)

        # 胶片噪声
        noise = np.random.normal(0, 15, result.shape).astype(np.int16)
        result = np.clip(result.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        return result

    def _style_pencil(self, frame, sigma=25, blend=0.8, **kw):
        """铅笔画效果（带纸张纹理）"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 反转
        inv = 255 - gray
        # 高斯模糊
        blur = cv2.GaussianBlur(inv, (sigma | 1, sigma | 1), 0)
        # 颜色减淡
        pencil = cv2.divide(gray, 255 - blur, scale=256)

        # 添加纸张纹理
        h, w = gray.shape
        texture = np.random.randint(220, 255, (h, w), dtype=np.uint8)
        texture = cv2.GaussianBlur(texture, (5, 5), 0)

        pencil_3ch = cv2.cvtColor(pencil, cv2.COLOR_GRAY2BGR).astype(float)
        texture_3ch = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR).astype(float)

        result = np.clip(pencil_3ch * texture_3ch / 255.0, 0, 255).astype(np.uint8)

        return result

    def _style_pop_art(self, frame, quantize=4, **kw):
        """波普艺术效果：高对比度 + 色彩量化"""
        # 色彩量化
        step = 256 // quantize
        quantized = (frame // step) * step + step // 2

        # 增强对比度
        lab = cv2.cvtColor(quantized, cv2.COLOR_BGR2LAB).astype(np.float64)
        lab[:, :, 0] = np.clip(lab[:, :, 0] * 1.2, 0, 255)
        result = cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_LAB2BGR)

        # 边缘
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        edges_3ch = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        result = cv2.bitwise_and(result, cv2.bitwise_not(edges_3ch))

        return result

    def list_styles(self):
        """列出所有可用风格"""
        return self.STYLES.copy()


# ============================================================
# DNN风格迁移（需要ONNX模型文件）
# ============================================================

class StyleTransferDNN:
    """
    基于OpenCV DNN的快速风格迁移
    需要预训练的ONNX模型文件

    推荐模型来源：
    - ONNX Model Zoo: https://github.com/onnx/models
    - 自行导出：PyTorch/TensorFlow → ONNX

    输入: (1, 3, H, W) float32, 归一化到 [0, 1] 或 ImageNet标准化
    输出: (1, 3, H, W) float32
    """

    def __init__(self, model_path=None, input_size=(256, 256)):
        """
        初始化DNN风格迁移
        Args:
            model_path: ONNX模型文件路径
            input_size: 网络输入尺寸 (w, h)
        """
        self.input_size = input_size
        self.net = None

        if model_path and os.path.exists(model_path):
            self.net = cv2.dnn.readNetFromONNX(model_path)
            print(f"已加载风格迁移模型: {model_path}")
        else:
            print("未提供模型文件，将使用纯OpenCV风格迁移")

    def transfer(self, frame, style_idx=0):
        """
        执行DNN风格迁移
        Args:
            frame: BGR输入图像
            style_idx: 风格索引（多风格模型）
        Returns:
            result: 风格化图像
        """
        if self.net is None:
            print("模型未加载，使用默认风格")
            cv_style = StyleTransferCV()
            return cv_style.transfer(frame, 'oil_painting')

        h, w = frame.shape[:2]
        inp_w, inp_h = self.input_size

        # 预处理
        blob = cv2.dnn.blobFromImage(
            frame, 1.0 / 255.0, (inp_w, inp_h), swapRB=False, crop=False
        )

        # 推理
        self.net.setInput(blob)
        output = self.net.forward()

        # 后处理
        output = output[0]  # 去batch维
        output = np.clip(output * 255, 0, 255).astype(np.uint8)
        output = output.transpose(1, 2, 0)  # CHW -> HWC

        # 缩放回原尺寸
        result = cv2.resize(output, (w, h), interpolation=cv2.INTER_CUBIC)

        return result


# ============================================================
# 组合式风格迁移器
# ============================================================

class RealtimeStyleProcessor:
    """
    实时风格迁移处理器
    支持摄像头/视频流的逐帧风格化
    """

    def __init__(self, style='oil_painting', fps_limit=30):
        """
        Args:
            style: 初始风格
            fps_limit: 帧率限制
        """
        self.cv_transfer = StyleTransferCV()
        self.dnn_transfer = StyleTransferDNN()
        self.current_style = style
        self.fps_limit = fps_limit

        # 多风格混合
        self.blend_styles = {}  # style_name -> weight
        self.blend_alpha = 1.0  # 与原图混合比例

    def set_style(self, style):
        """切换风格"""
        if style in StyleTransferCV.STYLES:
            self.current_style = style
            print(f"已切换到: {style}")
        elif style == 'original':
            self.current_style = 'original'
        else:
            print(f"未知风格: {style}")

    def set_blend(self, styles_dict, alpha=0.7):
        """
        设置多风格混合
        Args:
            styles_dict: {style_name: weight} 字典
            alpha: 与原图混合比例 (0=原图, 1=完全风格化)
        """
        self.blend_styles = styles_dict
        self.blend_alpha = alpha

    def process_frame(self, frame):
        """
        处理一帧
        Args:
            frame: BGR输入帧
        Returns:
            result: 风格化结果
        """
        if self.current_style == 'original':
            return frame

        # 多风格混合模式
        if self.blend_styles:
            result = np.zeros_like(frame, dtype=np.float64)
            total_weight = sum(self.blend_styles.values())
            for style_name, weight in self.blend_styles.items():
                styled = self.cv_transfer.transfer(frame, style_name)
                result += styled.astype(np.float64) * (weight / total_weight)
            result = np.clip(result, 0, 255).astype(np.uint8)
            result = cv2.addWeighted(frame, 1 - self.blend_alpha,
                                      result, self.blend_alpha, 0)
            return result

        # 单风格模式
        result = self.cv_transfer.transfer(frame, self.current_style)
        result = cv2.addWeighted(frame, 1 - self.blend_alpha,
                                  result, self.blend_alpha, 0)
        return result

    def process_video(self, input_path, output_path=None):
        """
        处理视频文件
        Args:
            input_path: 输入视频路径
            output_path: 输出视频路径（None则实时显示）
        """
        cap = cv2.VideoCapture(input_path)
        if not cap.isOpened():
            print(f"无法打开视频: {input_path}")
            return

        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        writer = None
        if output_path:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

        print(f"处理视频: {input_path} ({w}x{h} @ {fps:.1f}fps)")
        print(f"当前风格: {self.current_style}")
        print("按 q 中断")

        frame_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            result = self.process_frame(frame)

            if writer:
                writer.write(result)
            else:
                cv2.imshow('Style Transfer', result)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            frame_count += 1
            if frame_count % 100 == 0:
                print(f"  已处理 {frame_count} 帧")

        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        print(f"处理完成，共 {frame_count} 帧")


# ============================================================
# 使用示例
# ============================================================

def demo_camera():
    """摄像头实时风格迁移演示"""
    processor = RealtimeStyleProcessor(style='oil_painting', fps_limit=30)
    processor.blend_alpha = 0.75

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("实时风格迁移演示")
    print("按键切换风格:")
    print("  1-油彩 2-素描 3-水彩 4-卡通 5-马赛克")
    print("  6-浮雕 7-复古 8-铅笔 9-波普 0-原图")
    print("  +/- 调整强度  q-退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = processor.process_frame(frame)

        # 并排显示
        h, w = frame.shape[:2]
        display = np.zeros((h, w * 2, 3), dtype=np.uint8)
        display[:, :w] = frame
        display[:, w:] = result
        cv2.putText(display, f"Style: {processor.current_style} "
                             f"(alpha={processor.blend_alpha:.1f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('Style Transfer', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            processor.set_style('oil_painting')
        elif key == ord('2'):
            processor.set_style('sketch')
        elif key == ord('3'):
            processor.set_style('watercolor')
        elif key == ord('4'):
            processor.set_style('cartoon')
        elif key == ord('5'):
            processor.set_style('mosaic')
        elif key == ord('6'):
            processor.set_style('emboss')
        elif key == ord('7'):
            processor.set_style('vintage')
        elif key == ord('8'):
            processor.set_style('pencil')
        elif key == ord('9'):
            processor.set_style('pop_art')
        elif key == ord('0'):
            processor.set_style('original')
        elif key == ord('+') or key == ord('='):
            processor.blend_alpha = min(1.0, processor.blend_alpha + 0.1)
        elif key == ord('-'):
            processor.blend_alpha = max(0.0, processor.blend_alpha - 0.1)

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """单张图片所有风格预览"""
    cv_style = StyleTransferCV()
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取: {image_path}")
        return

    h, w = frame.shape[:2]
    # 缩小以便展示
    scale = min(1.0, 400 / max(h, w))
    small = cv2.resize(frame, None, fx=scale, fy=scale)
    sh, sw = small.shape[:2]

    # 网格布局
    styles = cv_style.list_styles()
    cols = 3
    rows = (len(styles) + cols) // cols + 1  # +1 for original
    grid = np.zeros((sh * rows, sw * cols, 3), dtype=np.uint8)

    # 原图
    grid[:sh, :sw] = small
    cv2.putText(grid, "Original", (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    for i, style in enumerate(styles):
        row = (i + 1) // cols
        col = (i + 1) % cols
        styled = cv_style.transfer(small, style)
        grid[row*sh:(row+1)*sh, col*sw:(col+1)*sw] = styled
        cv2.putText(grid, style, (col*sw + 5, row*sh + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    cv2.imshow('Style Gallery', grid)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
