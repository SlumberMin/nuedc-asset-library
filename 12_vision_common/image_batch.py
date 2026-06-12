"""
图像批处理工具 - 批量读取/处理/保存
"""
import cv2
import os
import glob
import numpy as np


def list_images(folder, exts=('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp')):
    """列出目录下所有图像文件
    Args:
        folder: 目录路径
        exts: 支持的扩展名
    Returns:
        文件路径列表
    """
    files = []
    for f in os.listdir(folder):
        if os.path.splitext(f)[1].lower() in exts:
            files.append(os.path.join(folder, f))
    return sorted(files)


def batch_read(folder, flag=cv2.IMREAD_COLOR, recursive=False):
    """批量读取图像
    Args:
        folder: 目录路径
        flag: 读取标志
        recursive: 是否递归子目录
    Returns:
        [(路径, 图像), ...] 列表
    """
    if recursive:
        files = []
        for root, _, filenames in os.walk(folder):
            for f in filenames:
                ext = os.path.splitext(f)[1].lower()
                if ext in ('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp'):
                    files.append(os.path.join(root, f))
        files.sort()
    else:
        files = list_images(folder)

    result = []
    for path in files:
        img = cv2.imread(path, flag)
        if img is not None:
            result.append((path, img))
    return result


def batch_save(images, output_dir, prefix='', ext='.jpg', quality=95):
    """批量保存图像
    Args:
        images: [(路径, 图像), ...] 或 [图像, ...]
        output_dir: 输出目录
        prefix: 文件名前缀
        ext: 扩展名
        quality: JPEG质量 (0-100)
    """
    os.makedirs(output_dir, exist_ok=True)
    params = []
    if ext in ('.jpg', '.jpeg'):
        params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    elif ext == '.png':
        params = [cv2.IMWRITE_PNG_COMPRESSION, 3]

    for i, item in enumerate(images):
        if isinstance(item, tuple):
            path, img = item
            name = prefix + os.path.basename(path)
        else:
            img = item
            name = f'{prefix}{i:06d}{ext}'
        save_path = os.path.join(output_dir, name)
        if not save_path.endswith(ext):
            save_path = os.path.splitext(save_path)[0] + ext
        cv2.imwrite(save_path, img, params)


def batch_process(folder, process_func, output_dir=None, **kwargs):
    """批量处理图像
    Args:
        folder: 输入目录
        process_func: 处理函数 process(img, **kwargs) -> img
        output_dir: 输出目录 None则覆盖原图
        **kwargs: 传递给处理函数的参数
    Returns:
        处理结果列表 [(路径, 原图, 结果图), ...]
    """
    images = batch_read(folder)
    results = []
    for path, img in images:
        result = process_func(img, **kwargs)
        results.append((path, img, result))

    if output_dir is not None:
        batch_save([(r[0], r[2]) for r in results], output_dir)
    return results


def batch_resize(folder, output_dir, width=None, height=None, scale=None):
    """批量缩放图像"""
    def _resize(img, **kw):
        w, h = kw.get('width'), kw.get('height')
        s = kw.get('scale')
        if s is not None:
            return cv2.resize(img, None, fx=s, fy=s)
        if w is not None and h is not None:
            return cv2.resize(img, (w, h))
        if w is not None:
            ratio = w / img.shape[1]
            return cv2.resize(img, (w, int(img.shape[0] * ratio)))
        if h is not None:
            ratio = h / img.shape[0]
            return cv2.resize(img, (int(img.shape[1] * ratio), h))
        return img

    return batch_process(folder, _resize, output_dir,
                         width=width, height=height, scale=scale)


def batch_convert(folder, output_dir, src_ext=None, dst_ext='.jpg'):
    """批量格式转换"""
    images = batch_read(folder)
    batch_save([(p, img) for p, img in images], output_dir, ext=dst_ext)


def batch_rename(folder, prefix='', start=0, digits=6):
    """批量重命名图像文件"""
    files = list_images(folder)
    for i, path in enumerate(files):
        ext = os.path.splitext(path)[1]
        new_name = f'{prefix}{i + start:0{digits}d}{ext}'
        new_path = os.path.join(folder, new_name)
        os.rename(path, new_path)


def create_video_from_folder(folder, output_path, fps=30, size=None, ext='.jpg'):
    """将目录下图像合成视频
    Args:
        folder: 图像目录
        output_path: 输出视频路径 (.avi / .mp4)
        fps: 帧率
        size: 帧尺寸 (w,h) None则用第一帧大小
    """
    files = sorted(glob.glob(os.path.join(folder, f'*{ext}')))
    if not files:
        return
    first = cv2.imread(files[0])
    if size is None:
        size = (first.shape[1], first.shape[0])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    writer = cv2.VideoWriter(output_path, fourcc, fps, size)
    for f in files:
        img = cv2.imread(f)
        if img.shape[:2][::-1] != size:
            img = cv2.resize(img, size)
        writer.write(img)
    writer.release()


def extract_frames(video_path, output_dir, frame_interval=1, prefix='frame'):
    """从视频中提取帧
    Args:
        video_path: 视频路径
        output_dir: 输出目录
        frame_interval: 每隔N帧提取一帧
    """
    os.makedirs(output_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    idx = 0
    count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        if idx % frame_interval == 0:
            path = os.path.join(output_dir, f'{prefix}_{count:06d}.jpg')
            cv2.imwrite(path, frame)
            count += 1
        idx += 1
    cap.release()
    return count


if __name__ == '__main__':
    # 批量读取示例
    images = batch_read('./test_images')
    print(f"读取 {len(images)} 张图像")

    # 批量处理: 灰度化
    def to_gray(img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

    # batch_process('./test_images', to_gray, './output_gray')

    # 批量缩放
    # batch_resize('./test_images', './output_small', width=640)

    # 从视频提取帧
    # extract_frames('video.avi', './frames', frame_interval=5)
