from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import laspy
import numpy as np


TOOL_ROOT = Path(__file__).resolve().parents[1]
if str(TOOL_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOL_ROOT))

from rosm.lidar_viewer import load_full_las_point_cloud  # noqa: E402


class FullLasLoadTests(unittest.TestCase):
    def test_loads_all_points_and_intensity(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "sample.las"
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.array([1.0, 2.0, 3.0])
            las.y = np.array([4.0, 5.0, 6.0])
            las.z = np.array([7.0, 8.0, 9.0])
            las.intensity = np.array([0, 50, 100], dtype=np.uint16)
            las.write(path)

            pcd = load_full_las_point_cloud(path)

            self.assertEqual(len(pcd.points), 3)
            np.testing.assert_allclose(np.asarray(pcd.points)[2], [3.0, 6.0, 9.0])
            np.testing.assert_allclose(np.asarray(pcd.colors)[:, 0], [0.0, 0.5, 1.0])

    def test_can_cancel_between_chunks(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "sample.las"
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.array([1.0])
            las.y = np.array([2.0])
            las.z = np.array([3.0])
            las.write(path)

            with self.assertRaises(InterruptedError):
                load_full_las_point_cloud(path, cancelled=lambda: True)


if __name__ == "__main__":
    unittest.main()
