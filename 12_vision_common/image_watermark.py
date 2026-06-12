"""
图像水印模块 - 文字水印 / 图片水印 / 水印检测
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class WatermarkEmbedder:
    """图像水印嵌入器"""

    # ---- 文字水印 ----

    @staticmethod
    def add_text_watermark(image, text, position='bottom-right',
                           font_scale=1.0, color=(255, 255, 255),
                           thickness=2, opacity=0.5,
                           font=cv2.FONT_HERSHEY_SIMPLEX):
        """
        添加文字水印
        :param image: BGR 图像
        :param text: 水印文字
        :param position: 位置 'top-left'/'top-right'/'bottom-left'/'bottom-right'/'center'
        :param font_scale: 字体大小
        :param color: BGR 颜色 (255,255,255)
        :param thickness: 线条粗细
        :param opacity: 不透明度 0~1
        :param font: OpenCV 字体
        :return: 带水印图像
        """
        h, w = image.shape[:2]
        overlay = image.copy()

        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        margin = 10

        pos_map = {
            'top-left':     (margin, margin + th),
            'top-right':    (w - tw - margin, margin + th),
            'bottom-left':  (margin, h - margin),
            'bottom-right': (w - tw - margin, h - margin),
            'center':       ((w - tw) // 2, (h + th) // 2),
        }
        org = pos_map.get(position, pos_map['bottom-right'])

        cv2.putText(overlay, text, org, font, font_scale, color, thickness, cv2.LINE_AA)
        return cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0)

    @staticmethod
    def add_text_watermark_tiled(image, text, font_scale=0.8,
                                 color=(200, 200, 200), opacity=0.3,
                                 angle=30, spacing=150):
        """
        添加平铺倾斜文字水印 (满屏水印)
        :param angle: 倾斜角度 (度)
        :param spacing: 水印间距 (像素)
        """
        h, w = image.shape[:2]
        overlay = image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, 1)

        # 创建大画布放文字后旋转
        diag = int(np.sqrt(w ** 2 + h ** 2))
        canvas = np.zeros((diag, diag, 3), dtype=np.uint8)

        for y in range(0, diag, spacing):
            for x in range(0, diag, spacing):
                cv2.putText(canvas, text, (x, y + th), font, font_scale, (255, 255, 255), 1, cv2.LINE_AA)

        M = cv2.getRotationMatrix2D((diag // 2, diag // 2), angle, 1.0)
        rotated = cv2.warpAffine(canvas, M, (diag, diag))

        # 裁剪到原图大小
        cx, cy = diag // 2, diag // 2
        cropped = rotated[cy - h // 2:cy + h // 2, cx - w // 2:cx + w // 2]
        if cropped.shape[:2] != (h, w):
            cropped = cv2.resize(cropped, (w, h))

        mask = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        mask_bool = mask > 0

        result = overlay.copy()
        for c in range(3):
            result[:, :, c] = np.where(
                mask_bool,
                cv2.addWeighted(overlay[:, :, c], 1 - opacity,
                                np.full_like(overlay[:, :, c], color[c]), opacity, 0),
                overlay[:, :, c]
            )
        return result

    # ---- 图片水印 ----

    @staticmethod
    def add_image_watermark(image, watermark, position='bottom-right',
                            opacity=0.5, scale=0.2):
        """
        添加图片水印 (支持 PNG 透明通道)
        :param image: BGR 主图
        :param watermark: 水印图 (BGR 或 BGRA)
        :param position: 位置
        :param opacity: 不透明度
        :param scale: 水印缩放比例 (相对于主图宽度)
        :return: 带水印图像
        """
        h, w = image.shape[:2]
        wm = watermark.copy()

        # 缩放水印
        wm_w = int(w * scale)
        wm_h = int(wm.shape[0] * wm_w / wm.shape[1])
        wm = cv2.resize(wm, (wm_w, wm_h), interpolation=cv2.INTER_AREA)

        # 计算位置
        margin = 10
        pos_map = {
            'top-left':     (margin, margin),
            'top-right':    (w - wm_w - margin, margin),
            'bottom-left':  (margin, h - wm_h - margin),
            'bottom-right': (w - wm_w - margin, h - wm_h - margin),
            'center':       ((w - wm_w) // 2, (h - wm_h) // 2),
        }
        x, y = pos_map.get(position, pos_map['bottom-right'])
        x, y = max(0, x), max(0, y)

        roi = image[y:y + wm_h, x:x + wm_w]

        if wm.shape[2] == 4:
            # 有 alpha 通道
            alpha = wm[:, :, 3:].astype(np.float32) / 255.0 * opacity
            wm_bgr = wm[:, :, :3].astype(np.float32)
            roi_f = roi.astype(np.float32)
            blended = (1 - alpha) * roi_f + alpha * wm_bgr
            image[y:y + wm_h, x:x + wm_w] = blended.astype(np.uint8)
        else:
            # 无 alpha, 直接混合
            blended = cv2.addWeighted(roi, 1 - opacity, wm, opacity, 0)
            image[y:y + wm_h, x:x + wm_w] = blended

        return image

    # ---- LSB 水印 (隐写) ----

    @staticmethod
    def embed_lsb(image, watermark_gray, bits=2):
        """
        LSB 隐写嵌入
        :param image: BGR 宿主图
        :param watermark_gray: 灰度水印图 (需 <= 宿主图尺寸)
        :param bits: 使用低位数 (1-4)
        :return: 含隐写水印的图像
        """
        h, w = image.shape[:2]
        wm = cv2.resize(watermark_gray, (w, h))
        bits = int(np.clip(bits, 1, 4))
        mask = 0xFF << bits & 0xFF  # 高位掩码
        wm_shifted = (wm >> (8 - bits))  # 水印高位移到低位

        result = image.copy()
        for c in range(3):
            channel = result[:, :, c]
            result[:, :, c] = (channel & mask) | wm_shifted

        return result

    @staticmethod
    def extract_lsb(stego_image, bits=2):
        """
        LSB 隐写提取
        :param stego_image: 含隐写图像
        :param bits: 使用低位数
        :return: 提取的灰度水印
        """
        bits = int(np.clip(bits, 1, 4))
        # 取第一个通道的低位
        extracted = (stego_image[:, :, 0] & ((1 << bits) - 1)) << (8 - bits)
        return extracted.astype(np.uint8)


if __name__ == '__main__':
    img = np.random.randint(100, 200, (480, 640, 3), dtype=np.uint8)

    # 文字水印
    result1 = WatermarkEmbedder.add_text_watermark(img, "TEST", opacity=0.6)
    print(f"文字水印: {result1.shape}")

    # 平铺水印
    result2 = WatermarkEmbedder.add_text_watermark_tiled(img, "CONFIDENTIAL")
    print(f"平铺水印: {result2.shape}")

    # LSB 隐写
    wm = np.random.randint(0, 256, (480, 640), dtype=np.uint8)
    stego = WatermarkEmbedder.embed_lsb(img, wm, bits=2)
    extracted = WatermarkEmbedder.extract_lsb(stego, bits=2)
    print(f"LSB 提取: {extracted.shape}")
