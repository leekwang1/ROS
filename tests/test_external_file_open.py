from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch


TOOL_ROOT = Path(__file__).resolve().parents[1]
if str(TOOL_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOL_ROOT))

from rosm.lidar_viewer import start_file_in_background  # noqa: E402


class ExternalFileOpenTests(unittest.TestCase):
    def test_startfile_runs_outside_calling_thread(self) -> None:
        calling_thread_id = threading.get_ident()
        worker_thread_ids: list[int] = []
        opened_paths: list[str] = []

        def fake_startfile(path: str) -> None:
            worker_thread_ids.append(threading.get_ident())
            opened_paths.append(path)

        expected = Path(r"C:\temp\snapshot.jpg")
        with patch("rosm.lidar_viewer.os.startfile", side_effect=fake_startfile):
            thread = start_file_in_background(expected)
            thread.join(timeout=2.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(opened_paths, [str(expected)])
        self.assertEqual(len(worker_thread_ids), 1)
        self.assertNotEqual(worker_thread_ids[0], calling_thread_id)


if __name__ == "__main__":
    unittest.main()
