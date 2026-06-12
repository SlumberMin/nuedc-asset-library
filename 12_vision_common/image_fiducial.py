"""
image_fiducial.py - 基准标记检测模块
支持: AprilTag / ArUco / 自定义基准标记
功能: 检测、姿态估计、标记生成
用法:
    markers = detect_fiducials(img, marker_type='aruco')
    pose = estimate_pose(img, camera_matrix, dist_coeffs, marker_type='apriltag')
    generate_aruco_board('DICT_4X4_50', 4, 3, 'board.png')
"""

import cv2
import numpy as np

# ========================= ArUco字典映射 =========================

_ARUCO_DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
    'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# ========================= ArUco检测 =========================

def detect_aruco(img, dict_name='DICT_4X4_50', estimate_pose=False,
                 marker_length=0.05, camera_matrix=None, dist_coeffs=None):
    """
    ArUco标记检测
    :param img: 输入图像
    :param dict_name: ArUco字典名
    :param marker_length: 标记物理边长(米), 用于姿态估计
    :return: [{id, corners, center, rvec, tvec}, ...]
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

    dict_id = _ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)

    # 兼容新旧API
    try:
        # OpenCV 4.7+
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, rejected = detector.detectMarkers(gray)
    except AttributeError:
        # OpenCV < 4.7
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    results = []
    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            marker_corners = corners[i].reshape(4, 2)
            cx = int(marker_corners[:, 0].mean())
            cy = int(marker_corners[:, 1].mean())
            result = {
                'id': int(ids[i][0]),
                'corners': marker_corners.tolist(),
                'center': (cx, cy),
                'rvec': None,
                'tvec': None,
            }
            results.append(result)

        if estimate_pose and camera_matrix is not None and dist_coeffs is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs)
            for i in range(len(results)):
                results[i]['rvec'] = rvecs[i].tolist()
                results[i]['tvec'] = tvecs[i].tolist()

    return results

# ========================= AprilTag检测 =========================

def detect_apriltag(img, tag_family='36h11', estimate_pose=False,
                    tag_size=0.05, camera_matrix=None, dist_coeffs=None):
    """
    AprilTag检测(通过OpenCV ArUco模块)
    :param tag_family: '36h11' / '25h9' / '16h5'
    :return: [{id, corners, center, rvec, tvec}, ...]
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

    # OpenCV 4.7+ 支持AprilTag通过arUco检测
    family_map = {
        '36h11': cv2.aruco.DICT_APRILTAG_36h11,
        '25h9': cv2.aruco.DICT_APRILTAG_25h9,
        '16h5': cv2.aruco.DICT_APRILTAG_16h5,
        '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    }
    dict_id = family_map.get(tag_family, cv2.aruco.DICT_APRILTAG_36h11)

    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, rejected = detector.detectMarkers(gray)
    except AttributeError:
        try:
            aruco_dict = cv2.aruco.Dictionary_get(dict_id)
            params = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        except Exception as e:
            raise RuntimeError(f"OpenCV不支持AprilTag检测(需要>=4.5.4): {e}")

    results = []
    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i].reshape(4, 2)
            cx = int(marker_corners[:, 0].mean())
            cy = int(marker_corners[:, 1].mean())
            results.append({
                'id': int(ids[i][0]),
                'corners': marker_corners.tolist(),
                'center': (cx, cy),
                'rvec': None,
                'tvec': None,
                'family': tag_family,
            })
        if estimate_pose and camera_matrix is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, tag_size, camera_matrix, dist_coeffs)
            for i in range(len(results)):
                results[i]['rvec'] = rvecs[i].tolist()
                results[i]['tvec'] = tvecs[i].tolist()

    return results

# ========================= 统一接口 =========================

def detect_fiducials(img, marker_type='aruco', dict_name='DICT_4X4_50',
                     tag_family='36h11', **kwargs):
    """
    基准标记检测统一接口
    :param marker_type: 'aruco' / 'apriltag'
    :return: [{id, corners, center, rvec, tvec}, ...]
    """
    if isinstance(img, str):
        img = cv2.imread(img)
    if marker_type.lower() == 'aruco':
        return detect_aruco(img, dict_name=dict_name, **kwargs)
    elif marker_type.lower() == 'apriltag':
        return detect_apriltag(img, tag_family=tag_family, **kwargs)
    else:
        raise ValueError(f"不支持的标记类型: {marker_type}")

def find_marker_by_id(markers, target_id):
    """按ID查找标记"""
    for m in markers:
        if m['id'] == target_id:
            return m
    return None

def get_marker_corners_ordered(marker):
    """获取按顺序排列的4个角点: [左上, 右上, 右下, 左下]"""
    corners = np.array(marker['corners'], dtype=np.float32)
    # 按y排序分为上下
    idx = np.argsort(corners[:, 1])
    top = corners[idx[:2]]
    bottom = corners[idx[2:]]
    # 上面两个按x排序
    top = top[np.argsort(top[:, 0])]
    bottom = bottom[np.argsort(bottom[:, 0])]
    return np.array([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)

def get_marker_center(marker):
    """获取标记中心"""
    return marker.get('center', None)

def marker_distance(marker1, marker2):
    """两个标记中心的像素距离"""
    c1 = np.array(marker1['center'])
    c2 = np.array(marker2['center'])
    return np.linalg.norm(c1 - c2)

# ========================= 标记生成 =========================

def generate_aruco_marker(dict_name='DICT_4X4_50', marker_id=0, size=200, output_path=None):
    """生成单个ArUco标记图像"""
    dict_id = _ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        marker = np.zeros((size, size), dtype=np.uint8)
        cv2.aruco.drawMarker(aruco_dict, marker_id, size, marker, 1)
    if output_path:
        cv2.imwrite(output_path, marker)
    return marker

def generate_aruco_board(dict_name='DICT_4X4_50', cols=4, rows=3,
                         marker_size=200, spacing=50, output_path=None):
    """生成ArUco标记板(多个标记排布)"""
    dict_id = _ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)

    img_w = cols * marker_size + (cols + 1) * spacing
    img_h = rows * marker_size + (rows + 1) * spacing
    board_img = np.ones((img_h, img_w), dtype=np.uint8) * 255

    marker_id = 0
    for row in range(rows):
        for col in range(cols):
            y = spacing + row * (marker_size + spacing)
            x = spacing + col * (marker_size + spacing)
            try:
                marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
            except AttributeError:
                marker = np.zeros((marker_size, marker_size), dtype=np.uint8)
                cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size, marker, 1)
            board_img[y:y + marker_size, x:x + marker_size] = marker
            marker_id += 1

    board_img_bgr = cv2.cvtColor(board_img, cv2.COLOR_GRAY2BGR)
    if output_path:
        cv2.imwrite(output_path, board_img_bgr)
    return board_img_bgr

# ========================= 姿态工具 =========================

def create_camera_matrix(fx, fy, cx, cy):
    """构造相机内参矩阵"""
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

def get_pose_euler(rvec):
    """将旋转向量转换为欧拉角(度)"""
    rmat, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
    sy = np.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)
    if sy > 1e-6:
        x = np.arctan2(rmat[2, 1], rmat[2, 2])
        y = np.arctan2(-rmat[2, 0], sy)
        z = np.arctan2(rmat[1, 0], rmat[0, 0])
    else:
        x = np.arctan2(-rmat[1, 2], rmat[1, 1])
        y = np.arctan2(-rmat[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])

def draw_fiducials(img, markers, draw_id=True, draw_axes=False,
                   camera_matrix=None, dist_coeffs=None, marker_length=0.05):
    """在图像上绘制检测到的基准标记"""
    vis = img.copy()
    for m in markers:
        pts = np.array(m['corners'], dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
        cx, cy = m['center']
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        if draw_id:
            cv2.putText(vis, f"ID:{m['id']}", (cx - 10, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        if draw_axes and m.get('rvec') is not None and camera_matrix is not None:
            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs,
                              np.array(m['rvec']), np.array(m['tvec']), marker_length)
    return vis

# ========================= 测试 =========================

if __name__ == '__main__':
    print("=== 基准标记检测模块 ===")
    print(f"支持ArUco字典: {list(_ARUCO_DICTS.keys())}")
    print(f"支持AprilTag: 36h11, 25h9, 16h5, 36h10")

    # 生成测试标记
    marker = generate_aruco_marker('DICT_4X4_50', 0, 200, 'test_aruco.png')
    print("生成测试标记: test_aruco.png")

    board = generate_aruco_board('DICT_4X4_50', 4, 3, output_path='test_board.png')
    print("生成标记板: test_board.png")

    # 测试检测
    results = detect_fiducials(board, marker_type='aruco')
    print(f"检测到 {len(results)} 个标记")
    for r in results:
        print(f"  ID={r['id']}, center={r['center']}")
