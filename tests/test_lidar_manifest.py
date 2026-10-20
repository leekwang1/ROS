from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
if str(TOOL_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOL_ROOT))

from rosm.lidar_viewer import project_root_from_manifest  # noqa: E402


class ProjectRootManifestTests(unittest.TestCase):
    def test_uses_existing_project_root_from_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            lidar_dir = root / "outputs" / "lidar"
            project_root = root / "sepoong2"
            fallback = root / "sepoong"
            lidar_dir.mkdir(parents=True)
            project_root.mkdir()
            fallback.mkdir()
            (lidar_dir / "scene_manifest.json").write_text(
                json.dumps({"project_root": str(project_root), "scenes": []}),
                encoding="utf-8",
            )

            self.assertEqual(project_root_from_manifest(lidar_dir, fallback), project_root.resolve())

    def test_keeps_fallback_for_missing_manifest_project(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            lidar_dir = root / "outputs" / "lidar"
            fallback = root / "sepoong"
            lidar_dir.mkdir(parents=True)
            fallback.mkdir()
            (lidar_dir / "scene_manifest.json").write_text(
                json.dumps({"project_root": str(root / "missing"), "scenes": []}),
                encoding="utf-8",
            )

            self.assertEqual(project_root_from_manifest(lidar_dir, fallback), fallback.resolve())


if __name__ == "__main__":
    unittest.main()
