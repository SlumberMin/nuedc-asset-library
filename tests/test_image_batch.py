"""
图像批处理单元测试
覆盖: 列出图像/批量读取/批量保存/批量处理/批量缩放/格式转换/重命名
"""
import unittest
import numpy as np
import cv2
import os
import sys
import tempfile
import shutil

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from importlib import import_module

mod = import_module('10_视觉通用代码库.image_batch')


class TestListImages(unittest.TestCase):
    """列出图像文件测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        # 创建测试文件
        for name in ['a.jpg', 'b.png', 'c.txt', 'd.bmp', 'e.py']:
            with open(os.path.join(self.tmpdir, name), 'w') as f:
                f.write('x')

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_finds_images(self):
        files = mod.list_images(self.tmpdir)
        self.assertEqual(len(files), 3)  # jpg, png, bmp

    def test_returns_sorted(self):
        files = mod.list_images(self.tmpdir)
        self.assertEqual(files, sorted(files))

    def test_excludes_non_images(self):
        files = mod.list_images(self.tmpdir)
        for f in files:
            self.assertTrue(f.endswith(('.jpg', '.png', '.bmp')))

    def test_custom_extensions(self):
        files = mod.list_images(self.tmpdir, exts=('.txt',))
        self.assertEqual(len(files), 1)

    def test_empty_dir(self):
        empty = tempfile.mkdtemp()
        try:
            files = mod.list_images(empty)
            self.assertEqual(len(files), 0)
        finally:
            shutil.rmtree(empty)


class TestBatchRead(unittest.TestCase):
    """批量读取测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        for i, name in enumerate(['img1.jpg', 'img2.jpg', 'img3.png']):
            img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.tmpdir, name), img)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_reads_all_images(self):
        result = mod.batch_read(self.tmpdir)
        self.assertEqual(len(result), 3)

    def test_returns_path_image_tuples(self):
        result = mod.batch_read(self.tmpdir)
        for path, img in result:
            self.assertIsInstance(path, str)
            self.assertIsInstance(img, np.ndarray)

    def test_images_are_valid(self):
        result = mod.batch_read(self.tmpdir)
        for _, img in result:
            self.assertEqual(img.shape, (32, 32, 3))

    def test_recursive(self):
        sub = os.path.join(self.tmpdir, 'sub')
        os.makedirs(sub)
        img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
        cv2.imwrite(os.path.join(sub, 'sub_img.jpg'), img)
        result = mod.batch_read(self.tmpdir, recursive=True)
        self.assertEqual(len(result), 4)

    def test_non_recursive(self):
        sub = os.path.join(self.tmpdir, 'sub')
        os.makedirs(sub)
        img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
        cv2.imwrite(os.path.join(sub, 'sub_img.jpg'), img)
        result = mod.batch_read(self.tmpdir, recursive=False)
        self.assertEqual(len(result), 3)


class TestBatchSave(unittest.TestCase):
    """批量保存测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.outdir = os.path.join(self.tmpdir, 'output')

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_save_plain_images(self):
        images = [np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8) for _ in range(3)]
        mod.batch_save(images, self.outdir)
        self.assertTrue(os.path.exists(self.outdir))
        files = os.listdir(self.outdir)
        self.assertEqual(len(files), 3)

    def test_save_with_prefix(self):
        images = [np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)]
        mod.batch_save(images, self.outdir, prefix='test_')
        files = os.listdir(self.outdir)
        self.assertTrue(any(f.startswith('test_') for f in files))

    def test_save_as_png(self):
        images = [np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)]
        mod.batch_save(images, self.outdir, ext='.png')
        files = os.listdir(self.outdir)
        self.assertTrue(any(f.endswith('.png') for f in files))

    def test_save_path_image_tuples(self):
        img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
        items = [(os.path.join(self.tmpdir, 'test.jpg'), img)]
        mod.batch_save(items, self.outdir)
        files = os.listdir(self.outdir)
        self.assertEqual(len(files), 1)


class TestBatchProcess(unittest.TestCase):
    """批量处理测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.outdir = os.path.join(self.tmpdir, 'output')
        for name in ['a.jpg', 'b.jpg']:
            img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.tmpdir, name), img)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_process_returns_results(self):
        def to_gray(img):
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        results = mod.batch_process(self.tmpdir, to_gray)
        self.assertEqual(len(results), 2)
        for path, orig, result in results:
            self.assertEqual(len(result.shape), 2)  # 灰度图

    def test_process_with_output(self):
        def identity(img):
            return img

        mod.batch_process(self.tmpdir, identity, output_dir=self.outdir)
        self.assertTrue(os.path.exists(self.outdir))


class TestBatchResize(unittest.TestCase):
    """批量缩放测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.outdir = os.path.join(self.tmpdir, 'output')
        for name in ['a.jpg', 'b.jpg']:
            img = np.random.randint(0, 256, (100, 200, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.tmpdir, name), img)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_resize_by_width(self):
        results = mod.batch_resize(self.tmpdir, self.outdir, width=100)
        self.assertEqual(len(results), 2)

    def test_resize_by_scale(self):
        results = mod.batch_resize(self.tmpdir, self.outdir, scale=0.5)
        for _, orig, result in results:
            self.assertEqual(result.shape[0], orig.shape[0] // 2)


class TestBatchConvert(unittest.TestCase):
    """格式转换测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.outdir = os.path.join(self.tmpdir, 'output')
        for name in ['a.jpg', 'b.jpg']:
            img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.tmpdir, name), img)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_convert_to_png(self):
        mod.batch_convert(self.tmpdir, self.outdir, dst_ext='.png')
        files = os.listdir(self.outdir)
        self.assertTrue(all(f.endswith('.png') for f in files))


class TestBatchRename(unittest.TestCase):
    """批量重命名测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        for name in ['z.jpg', 'a.jpg', 'm.jpg']:
            img = np.random.randint(0, 256, (8, 8, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.tmpdir, name), img)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_rename(self):
        mod.batch_rename(self.tmpdir, prefix='img_', start=0, digits=3)
        files = sorted(os.listdir(self.tmpdir))
        self.assertEqual(len(files), 3)
        self.assertTrue(all(f.startswith('img_') for f in files))


class TestCreateVideoFromFolder(unittest.TestCase):
    """图像合成视频测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        for i in range(5):
            img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.tmpdir, f'{i:04d}.jpg'), img)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_creates_video(self):
        output = os.path.join(self.tmpdir, 'test.avi')
        mod.create_video_from_folder(self.tmpdir, output, fps=10)
        self.assertTrue(os.path.exists(output))
        self.assertGreater(os.path.getsize(output), 0)


class TestExtractFrames(unittest.TestCase):
    """视频提取帧测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.viddir = os.path.join(self.tmpdir, 'frames')
        self.outdir = os.path.join(self.tmpdir, 'extracted')
        os.makedirs(self.viddir)
        # 创建视频
        for i in range(10):
            img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
            cv2.imwrite(os.path.join(self.viddir, f'{i:04d}.jpg'), img)
        self.video_path = os.path.join(self.tmpdir, 'test.avi')
        mod.create_video_from_folder(self.viddir, self.video_path, fps=10)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_extract_frames(self):
        count = mod.extract_frames(self.video_path, self.outdir, frame_interval=2)
        self.assertGreater(count, 0)
        files = os.listdir(self.outdir)
        self.assertEqual(len(files), count)


if __name__ == '__main__':
    unittest.main()
