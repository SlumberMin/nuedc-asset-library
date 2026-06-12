"""
image_ocr.py - OCR文字识别模块
支持后端: Tesseract / EasyOCR / PaddleOCR
用法:
    text = ocr_recognize(img, backend='tesseract')
    results = ocr_recognize_detail(img, backend='easyocr')
"""

import cv2
import numpy as np

# ========================= 预处理 =========================

def preprocess_for_ocr(img, method='adaptive'):
    """为OCR优化的图像预处理"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img.copy()
    if method == 'adaptive':
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY, 31, 10)
    elif method == 'otsu':
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    elif method == 'denoise':
        binary = cv2.fastNlMeansDenoising(gray, None, 10, 7, 21)
    elif method == 'sharpen':
        kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
        binary = cv2.filter2D(gray, -1, kernel)
    else:
        binary = gray
    return binary

def deskew_image(img):
    """自动校正文字倾斜"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    coords = np.column_stack(np.where(gray > 0))
    if len(coords) < 10:
        return img
    angle = cv2.minAreaRect(coords)[-1]
    if angle < -45:
        angle = -(90 + angle)
    else:
        angle = -angle
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

# ========================= Tesseract =========================

def _ocr_tesseract(img, lang='chi_sim+eng', config='--psm 6'):
    """Tesseract OCR"""
    try:
        import pytesseract
        text = pytesseract.image_to_string(img, lang=lang, config=config)
        # 详细结果
        data = pytesseract.image_to_data(img, lang=lang, config=config, output_type=pytesseract.Output.DICT)
        results = []
        for i in range(len(data['text'])):
            if int(data['conf'][i]) > 0:
                results.append({
                    'text': data['text'][i],
                    'confidence': float(data['conf'][i]) / 100.0,
                    'bbox': (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
                })
        return text.strip(), results
    except ImportError:
        raise ImportError("需要安装: pip install pytesseract, 并安装Tesseract-OCR")

# ========================= EasyOCR =========================

def _ocr_easyocr(img, lang_list=None):
    """EasyOCR"""
    if lang_list is None:
        lang_list = ['ch_sim', 'en']
    try:
        import easyocr
        reader = easyocr.Reader(lang_list, gpu=False)
        raw = reader.readtext(img)
        results = []
        texts = []
        for (bbox, text, conf) in raw:
            results.append({
                'text': text,
                'confidence': conf,
                'bbox': [(int(p[0]), int(p[1])) for p in bbox]
            })
            texts.append(text)
        return ' '.join(texts), results
    except ImportError:
        raise ImportError("需要安装: pip install easyocr")

# ========================= PaddleOCR =========================

def _ocr_paddle(img, lang='ch'):
    """PaddleOCR"""
    try:
        from paddleocr import PaddleOCR
        ocr = PaddleOCR(use_angle_cls=True, lang=lang, show_log=False)
        raw = ocr.ocr(img, cls=True)
        results = []
        texts = []
        if raw and raw[0]:
            for line in raw[0]:
                bbox_pts, (text, conf) = line
                bbox = (int(min(p[0] for p in bbox_pts)), int(min(p[1] for p in bbox_pts)),
                        int(max(p[0] for p in bbox_pts)), int(max(p[1] for p in bbox_pts)))
                results.append({'text': text, 'confidence': conf, 'bbox': bbox})
                texts.append(text)
        return ' '.join(texts), results
    except ImportError:
        raise ImportError("需要安装: pip install paddlepaddle paddleocr")

# ========================= 统一接口 =========================

_BACKENDS = {
    'tesseract': _ocr_tesseract,
    'easyocr': _ocr_easyocr,
    'paddle': _ocr_paddle,
    'paddleocr': _ocr_paddle,
}

def ocr_recognize(img, backend='tesseract', preprocess=None, **kwargs):
    """
    OCR文字识别，返回识别文本
    :param img: 图像(numpy数组)或文件路径
    :param backend: 'tesseract' / 'easyocr' / 'paddle'
    :param preprocess: 预处理方法 'adaptive'/'otsu'/'denoise'/'sharpen'/None
    :return: str 识别文本
    """
    if isinstance(img, str):
        img = cv2.imread(img)
    if preprocess:
        img = preprocess_for_ocr(img, preprocess)
    func = _BACKENDS.get(backend)
    if func is None:
        raise ValueError(f"不支持的后端: {backend}, 可选: {list(_BACKENDS.keys())}")
    text, _ = func(img, **kwargs)
    return text

def ocr_recognize_detail(img, backend='tesseract', preprocess=None, **kwargs):
    """
    OCR详细识别，返回文本和位置信息
    :return: (text, results) results为[{text, confidence, bbox}, ...]
    """
    if isinstance(img, str):
        img = cv2.imread(img)
    if preprocess:
        img = preprocess_for_ocr(img, preprocess)
    func = _BACKENDS.get(backend)
    if func is None:
        raise ValueError(f"不支持的后端: {backend}")
    return func(img, **kwargs)

def ocr_region(img, x, y, w, h, backend='tesseract', preprocess='adaptive', **kwargs):
    """识别图像指定区域的文字"""
    if isinstance(img, str):
        img = cv2.imread(img)
    roi = img[y:y+h, x:x+w]
    return ocr_recognize(roi, backend=backend, preprocess=preprocess, **kwargs)

def ocr_numbers(img, backend='tesseract', **kwargs):
    """专门识别数字(如电压表读数)"""
    config = kwargs.pop('config', '')
    if backend == 'tesseract':
        config += ' --psm 7 -c tessedit_char_whitelist=0123456789.'
        kwargs['config'] = config
    text = ocr_recognize(img, backend=backend, preprocess='adaptive', **kwargs)
    # 提取数字
    import re
    numbers = re.findall(r'[\d]+\.?\d*', text)
    return numbers

# ========================= 测试 =========================

if __name__ == '__main__':
    # 创建测试图像
    test_img = np.ones((100, 400, 3), dtype=np.uint8) * 255
    cv2.putText(test_img, 'Hello OCR 12345', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)

    print("=== OCR Test ===")
    for b in ['tesseract']:
        try:
            text = ocr_recognize(test_img, backend=b)
            print(f"[{b}] => '{text}'")
        except Exception as e:
            print(f"[{b}] 跳过: {e}")
