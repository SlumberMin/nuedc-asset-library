"""
图像超分辨率模块 - 插值放大、学习方法(OpenCV DNN)、自适应上采样
适用场景：图像放大、低分辨率图像增强、实时画面提升
"""

import cv2
import numpy as np
import os


def bicubic_upscale(image, scale_factor=2):
    """
    双三次插值上采样 (基线方法)
    :param image: 输入图像
    :param scale_factor: 放大倍数
    :return: 放大后的图像
    """
    h, w = image.shape[:2]
    return cv2.resize(image, (w * scale_factor, h * scale_factor), interpolation=cv2.INTER_CUBIC)


def lanczos_upscale(image, scale_factor=2):
    """
    Lanczos插值上采样 (边缘更锐利)
    :param image: 输入图像
    :param scale_factor: 放大倍数
    :return: 放大后的图像
    """
    h, w = image.shape[:2]
    return cv2.resize(image, (w * scale_factor, h * scale_factor), interpolation=cv2.INTER_LANCZOS4)


def area_downscale(image, scale_factor=4):
    """
    区域插值下采样 (适合缩小，抗锯齿)
    :param image: 输入图像
    :param scale_factor: 缩小倍数
    :return: 缩小后的图像
    """
    h, w = image.shape[:2]
    return cv2.resize(image, (w // scale_factor, h // scale_factor), interpolation=cv2.INTER_AREA)


def upscale_with_sharpen(image, scale_factor=2, sharpen_strength=0.5):
    """
    插值放大 + 后处理锐化
    :param image: 输入图像
    :param scale_factor: 放大倍数
    :param sharpen_strength: 锐化强度
    :return: 放大且锐化后的图像
    """
    upscaled = bicubic_upscale(image, scale_factor)
    blurred = cv2.GaussianBlur(upscaled, (0, 0), 1.0)
    sharpened = cv2.addWeighted(upscaled, 1.0 + sharpen_strength, blurred, -sharpen_strength, 0)
    return np.clip(sharpened, 0, 255).astype(np.uint8)


def upscale_edge_guided(image, scale_factor=2):
    """
    边缘引导上采样 (保持边缘锐度)
    :param image: 输入图像
    :param scale_factor: 放大倍数
    :return: 边缘增强的放大图像
    """
    h, w = image.shape[:2]
    new_h, new_w = h * scale_factor, w * scale_factor

    # 双三次插值
    upscaled = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

    # 边缘检测
    gray = cv2.cvtColor(upscaled, cv2.COLOR_BGR2GRAY) if len(upscaled.shape) == 3 else upscaled
    edges = cv2.Canny(gray, 50, 150)
    edges = cv2.GaussianBlur(edges, (3, 3), 0.5)
    edge_mask = edges.astype(np.float32) / 255.0

    if len(upscaled.shape) == 3:
        edge_mask = edge_mask[:, :, np.newaxis]

    # 边缘区域额外锐化
    blurred = cv2.GaussianBlur(upscaled, (3, 3), 1.0)
    sharpened = cv2.addWeighted(upscaled, 1.5, blurred, -0.5, 0)

    result = upscaled * (1 - edge_mask * 0.5) + sharpened * (edge_mask * 0.5)
    return np.clip(result, 0, 255).astype(np.uint8)


def upscale_dnn_espcn(image, scale_factor=2, model_path=None):
    """
    ESPCN超分辨率 (基于CNN的OpenCV DNN实现，需预训练模型)
    支持 x2/x3/x4 倍率
    :param image: 输入图像(BGR)
    :param scale_factor: 放大倍数 (2/3/4)
    :param model_path: 模型文件路径，None则尝试自动查找
    :return: 超分辨率结果
    """
    # 模型下载URL
    model_urls = {
        2: "https://github.com/fannymonori/TF-ESPCN/raw/master/export/ESPCN_x2.pb",
        3: "https://github.com/fannymonori/TF-ESPCN/raw/master/export/ESPCN_x3.pb",
        4: "https://github.com/fannymonori/TF-ESPCN/raw/master/export/ESPCN_x4.pb",
    }

    if model_path is None:
        model_path = f"ESPCN_x{scale_factor}.pb"

    if not os.path.exists(model_path):
        print(f"ESPCN模型未找到: {model_path}")
        print(f"请从 {model_urls.get(scale_factor)} 下载")
        print("回退到双三次插值 + 锐化")
        return upscale_with_sharpen(image, scale_factor)

    sr = cv2.dnn_superres.DnnSuperResImpl_create()
    sr.readModel(model_path)
    sr.setModel("espcn", scale_factor)
    result = sr.upsample(image)
    return result


def upscale_dnn_edsr(image, scale_factor=2, model_path=None):
    """
    EDSR超分辨率 (精度高于ESPCN，速度较慢)
    :param image: 输入图像
    :param scale_factor: 放大倍数
    :param model_path: 模型路径
    :return: 超分辨率结果
    """
    if model_path is None:
        model_path = f"EDSR_x{scale_factor}.pb"

    if not os.path.exists(model_path):
        print(f"EDSR模型未找到: {model_path}，回退到双三次插值")
        return bicubic_upscale(image, scale_factor)

    sr = cv2.dnn_superres.DnnSuperResImpl_create()
    sr.readModel(model_path)
    sr.setModel("edsr", scale_factor)
    result = sr.upsample(image)
    return result


def upscale_multi_pass(image, target_scale=4, pass_scale=2):
    """
    多次小幅放大 (例如4x分两次2x，质量通常优于单次4x)
    :param image: 输入图像
    :param target_scale: 目标倍数
    :param pass_scale: 每次放大倍数
    :return: 放大后的图像
    """
    result = image.copy()
    current_scale = 1
    while current_scale < target_scale:
        result = upscale_with_sharpen(result, pass_scale, sharpen_strength=0.3)
        current_scale *= pass_scale
    return result


def compute_psnr(img1, img2):
    """
    计算PSNR (评估超分辨率质量)
    :param img1: 参考图像
    :param img2: 待评估图像
    :return: PSNR值(dB)
    """
    mse = np.mean((img1.astype(np.float64) - img2.astype(np.float64)) ** 2)
    if mse == 0:
        return float('inf')
    return 10 * np.log10(255.0 ** 2 / mse)


def create_test_degraded(image, scale_factor=4, noise_sigma=5):
    """
    创建退化测试图像 (先缩小再加噪声)
    :param image: 原始高分辨率图像
    :param scale_factor: 缩小倍数
    :param noise_sigma: 噪声标准差
    :return: (低分辨率图像, 原图)
    """
    h, w = image.shape[:2]
    lr = cv2.resize(image, (w // scale_factor, h // scale_factor), interpolation=cv2.INTER_AREA)

    if noise_sigma > 0:
        noise = np.random.randn(*lr.shape) * noise_sigma
        lr = np.clip(lr.astype(np.float64) + noise, 0, 255).astype(np.uint8)

    return lr, image


# ===================== 示例与测试 =====================
if __name__ == "__main__":
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else None
    if img_path:
        img = cv2.imread(img_path)
    else:
        print("生成测试图像...")
        img = np.zeros((512, 512, 3), dtype=np.uint8)
        for i in range(0, 512, 20):
            cv2.line(img, (i, 0), (512 - i, 512), (0, 200, 255), 1)
            cv2.line(img, (0, i), (512, 512 - i), (255, 200, 0), 1)
        cv2.putText(img, "4x SR", (80, 300), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 8)
        cv2.circle(img, (256, 100), 60, (0, 0, 255), -1)

    if img is None:
        print("无法读取图像")
        sys.exit(1)

    scale = 4
    lr_img, hr_img = create_test_degraded(img, scale)

    # 各方法对比
    sr_bicubic = bicubic_upscale(lr_img, scale)
    sr_lanczos = lanczos_upscale(lr_img, scale)
    sr_sharpen = upscale_with_sharpen(lr_img, scale)
    sr_edge = upscale_edge_guided(lr_img, scale)
    sr_multi = upscale_multi_pass(lr_img, scale)

    # PSNR评估
    h, w = hr_img.shape[:2]
    for name, sr in [("Bicubic", sr_bicubic), ("Lanczos", sr_lanczos),
                      ("Sharpen", sr_sharpen), ("EdgeGuided", sr_edge),
                      ("MultiPass", sr_multi)]:
        sr_resized = cv2.resize(sr, (w, h))
        psnr = compute_psnr(hr_img, sr_resized)
        print(f"{name}: PSNR = {psnr:.2f} dB")

    # 尝试DNN方法
    sr_dnn = upscale_dnn_espcn(lr_img, scale)
    sr_dnn_resized = cv2.resize(sr_dnn, (w, h))
    psnr_dnn = compute_psnr(hr_img, sr_dnn_resized)
    print(f"ESPCN DNN: PSNR = {psnr_dnn:.2f} dB")

    # 显示结果
    cv2.imshow("Ground Truth", hr_img)
    cv2.imshow("Low Resolution (x{})".format(scale), cv2.resize(lr_img, (w, h)))
    cv2.imshow("Bicubic", sr_bicubic)
    cv2.imshow("Sharpened", sr_sharpen)

    print("按任意键关闭...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
