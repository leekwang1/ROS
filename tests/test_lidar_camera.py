import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rosm.lidar_viewer import (  # noqa: E402
    CameraCalibration, LidarSceneApp, camera_frustum_geometry,
    image_plane_geometry, snapshot_projection_bounds,
)


class SnapshotCameraTests(unittest.TestCase):
    def calibration(self, source):
        k = np.array([[180.7, 0, 207.2], [0, 180.7, 115.4], [0, 0, 1.]])
        p = np.array([[142.87, 0, 212.97], [0, 142.87, 102.94], [0, 0, 1.]])
        return CameraCalibration("Xacti", 400, 300, "fisheye", k,
                                 np.array([.0527, .0236, -.01594, .002977]), p, source)

    def test_off_axis_corners_fit_wide_and_tall_viewports(self):
        cal = self.calibration(Path("unused.json"))
        angle = 0.7
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1.]])
        origin = np.array([15., -2., 5.])
        corners = cal.frustum_corners(origin, rotation, .63)
        plane = image_plane_geometry(corners)
        frustum = camera_frustum_geometry(origin, rotation, .63, 70, 4 / 3, cal)
        np.testing.assert_allclose(plane.vertices, np.asarray(frustum.points)[1:])
        for width, height in [(1600, 900), (800, 600), (390, 844)]:
            with self.subTest(size=(width, height)):
                left, right, bottom, top = snapshot_projection_bounds(
                    corners, origin, rotation, width, height)
                offsets = np.asarray(corners) - origin
                depth = offsets @ rotation[:, 0]
                x = offsets @ np.cross(rotation[:, 0], rotation[:, 2]) / depth
                y = offsets @ rotation[:, 2] / depth
                u = (x - left) / (right - left) * width
                v = (top - y) / (top - bottom) * height
                self.assertTrue(np.all(u >= -1e-7) and np.all(u <= width + 1e-7))
                self.assertTrue(np.all(v >= -1e-7) and np.all(v <= height + 1e-7))
                self.assertAlmostEqual(float(u.min() + u.max()), width)
                self.assertAlmostEqual(float(v.min() + v.max()), height)
                self.assertAlmostEqual((right - left) / (top - bottom), width / height)

    def test_exif_is_not_applied_before_remap_and_old_cache_is_ignored(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "snapshot.jpg"
            pixels = np.zeros((300, 400, 3), dtype=np.uint8)
            pixels[:150, :200] = [240, 20, 20]
            pixels[:150, 200:] = [20, 240, 20]
            pixels[150:, :200] = [20, 20, 240]
            pixels[150:, 200:] = [240, 240, 20]
            exif = Image.Exif()
            exif[274] = 3
            Image.fromarray(pixels).save(source, exif=exif)
            cal = self.calibration(root / "unused.json")
            cache = root / "undistorted_images"
            cache.mkdir()
            old = cache / "poi_snapshot_undistorted.jpg"
            old.write_bytes(b"old cache must not be read")
            app = SimpleNamespace(camera_calibration=cal, calibration_dir=root)
            marker = SimpleNamespace(image_path=source, snapshot_id="poi")
            output = LidarSceneApp._undistorted_image_for_marker(app, marker)
            self.assertNotEqual(output, old)
            self.assertEqual(old.read_bytes(), b"old cache must not be read")
            raw = cv2.imdecode(np.fromfile(source, np.uint8), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                cal.k, cal.d, np.eye(3), cal.p, (400, 300), cv2.CV_16SC2)
            expected = cv2.remap(raw, map1, map2, cv2.INTER_LINEAR)
            decoded = cv2.imdecode(np.fromfile(output, np.uint8), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
            self.assertLess(float(np.abs(decoded.astype(float) - expected).mean()), 3.0)
            self.assertGreater(float(np.abs(decoded.astype(float) - cv2.rotate(expected, cv2.ROTATE_180)).mean()), 50.0)
            with Image.open(output) as corrected:
                self.assertEqual(corrected.getexif().get(274), 3)
            self.assertEqual(LidarSceneApp._undistorted_image_for_marker(app, marker), output)


if __name__ == "__main__":
    unittest.main()
