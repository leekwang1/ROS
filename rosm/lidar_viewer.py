from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import laspy
from open3d.visualization import gui, rendering
from PIL import Image

try:
    import cv2
except Exception:  # noqa: BLE001
    cv2 = None


def read_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        return list(csv.DictReader(fp))


def write_csv_dicts(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def start_file_in_background(path: Path, on_error: Any = None) -> threading.Thread:
    """Open a file outside the Open3D native event callback."""
    def worker() -> None:
        try:
            os.startfile(str(path))  # noqa: S606
        except Exception as exc:  # noqa: BLE001
            if on_error is not None:
                on_error(exc)

    thread = threading.Thread(target=worker, daemon=True, name="snapshot-image-opener")
    thread.start()
    return thread


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2] / "sepoong"


def default_lidar_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "lidar"


def default_snapshot_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "snapshots"


def default_calibration_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "calibration"


def default_output_root(project_root: Path) -> Path:
    return project_root.parent / "outputs"


def looks_like_project_root(path: Path) -> bool:
    if not path.is_dir():
        return False
    return any((path / name).exists() for name in ("flights", "maps", "snapshots", "pois", "assets", "inspections"))


def infer_project_root_from_input(path: Path) -> Path:
    path = path.resolve()
    start = path.parent if path.is_file() else path
    for candidate in (start, *start.parents):
        if looks_like_project_root(candidate):
            return candidate
    return start


def nearest_existing_dir(path: Path) -> Path:
    path = path.resolve()
    start = path if path.is_dir() else path.parent
    for candidate in (start, *start.parents):
        if candidate.exists() and candidate.is_dir():
            return candidate
    return Path.home()


def short_process_output(stdout: str, stderr: str, max_chars: int = 2500) -> str:
    text = "\n".join(part.strip() for part in (stdout, stderr) if part and part.strip())
    if len(text) <= max_chars:
        return text
    return text[-max_chars:]


def filter_mcap_index_to_file(index_dir: Path, project_root: Path, selected_mcap: Path) -> int:
    csv_path = index_dir / "mcap_files.csv"
    rows = read_csv_dicts(csv_path)
    if not rows:
        raise RuntimeError(f"mcap_files.csv에 MCAP 파일 목록이 없습니다.\n{csv_path}")

    selected_abs = selected_mcap.resolve()
    selected_name = selected_mcap.name.lower()
    filtered = []
    for row in rows:
        rel_path = row.get("mcap_file", "")
        file_name = row.get("file_name", "").lower()
        candidate = (project_root / rel_path).resolve() if rel_path else None
        if candidate == selected_abs or file_name == selected_name:
            filtered.append(row)

    if not filtered:
        raise RuntimeError(f"선택한 MCAP 파일이 프로젝트 색인에서 발견되지 않았습니다.\n{selected_mcap}")

    write_csv_dicts(csv_path, filtered, list(rows[0].keys()))
    return len(filtered)


def default_korean_font_candidates() -> list[Path]:
    return [
        Path("C:/Windows/Fonts/malgun.ttf"),
        Path("C:/Windows/Fonts/malgunbd.ttf"),
        Path("C:/Windows/Fonts/gulim.ttc"),
    ]


def hangul_code_points() -> list[int]:
    ranges = [
        (0x1100, 0x11FF),  # Hangul Jamo
        (0x3130, 0x318F),  # Hangul Compatibility Jamo
        (0xA960, 0xA97F),  # Hangul Jamo Extended-A
        (0xAC00, 0xD7A3),  # Hangul Syllables
        (0xD7B0, 0xD7FF),  # Hangul Jamo Extended-B
    ]
    points: list[int] = []
    for start, end in ranges:
        points.extend(range(start, end + 1))
    return points


def configure_korean_font(font_path: Path | None = None) -> Path | None:
    candidates = [font_path] if font_path else default_korean_font_candidates()
    for candidate in candidates:
        if candidate and candidate.exists():
            font = gui.FontDescription(str(candidate))
            font.add_typeface_for_language(str(candidate), "ko")
            font.add_typeface_for_language(str(candidate), "ko_KR")
            font.add_typeface_for_code_points(str(candidate), hangul_code_points())
            gui.Application.instance.set_font(gui.Application.DEFAULT_FONT_ID, font)
            return candidate
    return None


def to_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        return None


def normalize_quat(q: tuple[float | None, float | None, float | None, float | None]) -> tuple[float, float, float, float] | None:
    if any(value is None for value in q):
        return None
    x, y, z, w = (float(value) for value in q if value is not None)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        return None
    return x / norm, y / norm, z / norm, w / norm


def quat_to_matrix(q: tuple[float | None, float | None, float | None, float | None]) -> np.ndarray:
    norm = normalize_quat(q)
    if norm is None:
        return np.eye(3)
    x, y, z, w = norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def line_set(points: list[list[float]], lines: list[list[int]], color: list[float]) -> o3d.geometry.LineSet:
    geom = o3d.geometry.LineSet()
    geom.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    geom.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    geom.colors = o3d.utility.Vector3dVector(np.tile(np.asarray(color, dtype=float), (len(lines), 1)))
    return geom


def matrix3_array(value: Any) -> np.ndarray | None:
    try:
        array = np.asarray(value, dtype=float)
    except Exception:  # noqa: BLE001
        return None
    if array.shape == (3, 3):
        return array
    if array.size == 9:
        return array.reshape((3, 3))
    return None


def scaled_camera_matrix(matrix: np.ndarray, source_width: int, source_height: int, width: int, height: int) -> np.ndarray:
    scaled = np.asarray(matrix, dtype=float).copy()
    sx = float(width) / max(1.0, float(source_width))
    sy = float(height) / max(1.0, float(source_height))
    scaled[0, 0] *= sx
    scaled[0, 1] *= sx
    scaled[0, 2] *= sx
    scaled[1, 0] *= sy
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


@dataclass
class CameraCalibration:
    camera_model: str
    image_width: int
    image_height: int
    distortion_model: str
    k: np.ndarray
    d: np.ndarray
    p: np.ndarray
    source_file: Path

    @classmethod
    def from_json(cls, path: Path) -> "CameraCalibration | None":
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        image_size = data.get("image_size") if isinstance(data, dict) else {}
        if not isinstance(image_size, dict):
            image_size = {}
        width = int(image_size.get("width") or image_size.get("x") or 0)
        height = int(image_size.get("height") or image_size.get("y") or 0)
        k = matrix3_array(data.get("K"))
        p = matrix3_array(data.get("P"))
        try:
            d = np.asarray(data.get("D"), dtype=float).reshape((-1,))
        except Exception:  # noqa: BLE001
            d = np.asarray([], dtype=float)
        if width <= 0 or height <= 0 or k is None or p is None or d.size < 4:
            return None
        return cls(
            camera_model=str(data.get("camera_model") or "Xacti"),
            image_width=width,
            image_height=height,
            distortion_model=str(data.get("distortion_model") or ""),
            k=k,
            d=d[:4].astype(float),
            p=p,
            source_file=path,
        )

    def projection_for_size(self, width: int, height: int) -> np.ndarray:
        return scaled_camera_matrix(self.p, self.image_width, self.image_height, width, height)

    def intrinsic_for_size(self, width: int, height: int) -> np.ndarray:
        return scaled_camera_matrix(self.k, self.image_width, self.image_height, width, height)

    def normalized_rectified_corners(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w = int(width or self.image_width)
        h = int(height or self.image_height)
        projection = self.projection_for_size(w, h)
        fx = float(projection[0, 0])
        fy = float(projection[1, 1])
        cx = float(projection[0, 2])
        cy = float(projection[1, 2])
        if abs(fx) <= 1e-9 or abs(fy) <= 1e-9:
            raise ValueError("Invalid camera projection matrix.")
        pixels = np.asarray([[0.0, 0.0], [float(w), 0.0], [float(w), float(h)], [0.0, float(h)]], dtype=float)
        return np.column_stack(((pixels[:, 0] - cx) / fx, (pixels[:, 1] - cy) / fy))

    def half_extent_ratios(self) -> tuple[float, float]:
        corners = self.normalized_rectified_corners()
        return float(np.max(np.abs(corners[:, 0]))), float(np.max(np.abs(corners[:, 1])))

    def frustum_corners(self, origin: np.ndarray, rotation: np.ndarray, length: float) -> list[np.ndarray]:
        safe_length = max(0.05, float(length))
        forward = rotation[:, 0]
        right = rotation[:, 1]
        up = rotation[:, 2]
        corners = self.normalized_rectified_corners()
        return [
            origin + forward * safe_length + right * (float(x) * safe_length) - up * (float(y) * safe_length)
            for x, y in corners
        ]


@dataclass
class SnapshotMarker:
    snapshot_id: str
    poi_name: str
    image_name: str
    image_path: Path | None
    thumbnail_path: Path | None
    origin: np.ndarray
    rotation: np.ndarray
    wall_distance: float | None
    frustum_length: float
    frustum_length_source: str
    pose_source: str
    frustum_source: str
    frustum_corners: list[np.ndarray]
    geometries: list[o3d.geometry.Geometry]

    @property
    def display_name(self) -> str:
        name = self.poi_name or self.snapshot_id
        if self.image_name:
            return f"{name} - {self.image_name}"
        return name

    @property
    def distance_text(self) -> str:
        if self.frustum_length_source == "wall":
            return f"wall_distance {self.frustum_length:.3f} m"
        if self.frustum_length_source == "lidar":
            return f"LiDAR hit {self.frustum_length:.3f} m"
        return f"fixed {self.frustum_length:.3f} m"

    @property
    def source_text(self) -> str:
        return f"pose source {self.pose_source} / frustum {self.frustum_source}"


def material(shader: str, point_size: float = 1.0, line_width: float = 1.0) -> rendering.MaterialRecord:
    mat = rendering.MaterialRecord()
    mat.shader = shader
    mat.point_size = max(1.0, float(point_size))
    mat.line_width = max(1.0, float(line_width))
    return mat


def downsample_point_cloud(pcd: o3d.geometry.PointCloud, max_points: int) -> o3d.geometry.PointCloud:
    if max_points <= 0 or len(pcd.points) <= max_points:
        return pcd
    every_k_points = max(1, math.ceil(len(pcd.points) / max_points))
    return pcd.uniform_down_sample(every_k_points)


def clone_point_cloud(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    clone = o3d.geometry.PointCloud()
    clone.points = o3d.utility.Vector3dVector(np.asarray(pcd.points).copy())
    if pcd.has_colors():
        clone.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors).copy())
    if pcd.has_normals():
        clone.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals).copy())
    return clone


def normalized_values(values: np.ndarray, lower_percentile: float = 2.0, upper_percentile: float = 98.0) -> np.ndarray:
    if values.size == 0:
        return values
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return np.zeros_like(values, dtype=float)
    lower = float(np.percentile(finite, lower_percentile))
    upper = float(np.percentile(finite, upper_percentile))
    if upper <= lower:
        return np.zeros_like(values, dtype=float)
    return np.clip((values - lower) / (upper - lower), 0.0, 1.0)


def color_ramp(values: np.ndarray, stops: list[tuple[float, tuple[float, float, float]]]) -> np.ndarray:
    values = np.clip(values, 0.0, 1.0)
    colors = np.zeros((len(values), 3), dtype=float)
    for index, value in enumerate(values):
        for stop_index in range(len(stops) - 1):
            left_t, left_color = stops[stop_index]
            right_t, right_color = stops[stop_index + 1]
            if value <= right_t or stop_index == len(stops) - 2:
                span = max(1e-9, right_t - left_t)
                mix = np.clip((value - left_t) / span, 0.0, 1.0)
                colors[index] = np.asarray(left_color) * (1.0 - mix) + np.asarray(right_color) * mix
                break
    return colors


def intensity_band_colors(values: np.ndarray) -> np.ndarray:
    values = np.clip(values, 0.0, 1.0)
    thresholds = np.array([0.12, 0.28, 0.45, 0.62, 0.78, 0.9], dtype=float)
    palette = np.asarray(
        [
            (0.08, 0.02, 0.18),  # very weak return
            (0.34, 0.06, 0.55),
            (0.88, 0.18, 0.95),
            (0.12, 0.85, 1.0),
            (0.62, 1.0, 1.0),
            (1.0, 1.0, 1.0),
            (1.0, 0.82, 0.18),  # strongest return
        ],
        dtype=float,
    )
    indices = np.digitize(values, thresholds, right=False)
    colors = palette[indices]
    brightness = 0.45 + 0.75 * np.power(values, 0.5)
    return np.clip(colors * brightness[:, None] + 0.08, 0.0, 1.0)


def original_intensity_values(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    point_count = len(pcd.points)
    if not pcd.has_colors():
        return np.full(point_count, 0.65, dtype=float)
    colors = np.asarray(pcd.colors, dtype=float)
    if len(colors) != point_count:
        return np.full(point_count, 0.65, dtype=float)
    return normalized_values(colors.mean(axis=1), 1.0, 99.0)


def apply_lidar_color_mode(pcd: o3d.geometry.PointCloud, mode: str) -> o3d.geometry.PointCloud:
    points = np.asarray(pcd.points, dtype=float)
    if len(points) == 0:
        return pcd

    if mode == "gray":
        pcd.paint_uniform_color([0.68, 0.68, 0.68])
        return pcd

    intensity = original_intensity_values(pcd)
    if mode == "intensity":
        values = np.power(np.clip(intensity, 0.0, 1.0), 0.55)
        pcd.colors = o3d.utility.Vector3dVector(np.column_stack((values, values, values)))
        return pcd

    z_values = normalized_values(points[:, 2], 2.0, 98.0)
    if mode == "height":
        pcd.colors = o3d.utility.Vector3dVector(
            color_ramp(
                z_values,
                [
                    (0.0, (0.05, 0.15, 0.95)),
                    (0.35, (0.0, 0.85, 1.0)),
                    (0.65, (0.25, 1.0, 0.35)),
                    (0.85, (1.0, 0.9, 0.15)),
                    (1.0, (1.0, 1.0, 1.0)),
                ],
            )
        )
        return pcd

    pcd.colors = o3d.utility.Vector3dVector(intensity_band_colors(intensity))
    return pcd


def tight_point_cloud_bbox(
    pcd: o3d.geometry.PointCloud,
    lower_percentile: float = 0.5,
    upper_percentile: float = 99.5,
) -> o3d.geometry.AxisAlignedBoundingBox:
    points = np.asarray(pcd.points, dtype=float)
    if points.size == 0:
        return o3d.geometry.AxisAlignedBoundingBox()

    finite_mask = np.all(np.isfinite(points), axis=1)
    points = points[finite_mask]
    if len(points) < 10:
        return pcd.get_axis_aligned_bounding_box()

    lower = np.percentile(points, lower_percentile, axis=0)
    upper = np.percentile(points, upper_percentile, axis=0)
    extent = upper - lower
    if not np.all(np.isfinite(extent)) or float(np.max(extent)) <= 1e-9:
        return pcd.get_axis_aligned_bounding_box()

    min_extent = max(float(np.max(extent)) * 0.01, 0.05)
    extent = np.maximum(extent, min_extent)
    center = (lower + upper) * 0.5
    padding = extent * 0.02
    min_bound = center - extent * 0.5 - padding
    max_bound = center + extent * 0.5 + padding
    return o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)


def load_manifest(lidar_dir: Path) -> dict[str, Any]:
    path = lidar_dir / "scene_manifest.json"
    if not path.exists():
        return {"scenes": []}
    return json.loads(path.read_text(encoding="utf-8"))


def load_scenes(lidar_dir: Path) -> list[dict[str, Any]]:
    manifest = load_manifest(lidar_dir)
    scenes = [scene for scene in manifest.get("scenes", []) if isinstance(scene, dict)]
    if not scenes:
        raise SystemExit(f"No scene found. Check lidar output folder: {lidar_dir}")
    return scenes


def empty_scene() -> dict[str, Any]:
    return {"flight_id": "", "flight_identifier": "출력 없음", "_empty_scene": True}


def is_empty_scene(scene: dict[str, Any]) -> bool:
    return bool(scene.get("_empty_scene"))


def load_scenes_or_empty(lidar_dir: Path) -> list[dict[str, Any]]:
    try:
        return load_scenes(lidar_dir)
    except (FileNotFoundError, SystemExit):
        return [empty_scene()]


def project_root_from_manifest(lidar_dir: Path, fallback: Path) -> Path:
    value = load_manifest(lidar_dir).get("project_root")
    if isinstance(value, str) and value.strip():
        candidate = Path(value).expanduser()
        if candidate.is_dir():
            return candidate.resolve()
    return fallback.resolve()


def resolve_lidar_dir(path: Path) -> Path:
    candidates = [
        path,
        path / "lidar",
        path / "outputs" / "lidar",
    ]
    for candidate in candidates:
        if (candidate / "scene_manifest.json").exists():
            return candidate.resolve()
    raise FileNotFoundError(f"scene_manifest.json not found under: {path}")


def resolve_lidar_dir_or_default(path: Path) -> Path:
    try:
        return resolve_lidar_dir(path)
    except FileNotFoundError:
        return path.resolve()


def sibling_snapshot_dir(lidar_dir: Path, fallback: Path) -> Path:
    if lidar_dir.name.lower() == "lidar":
        candidate = lidar_dir.parent / "snapshots"
        if candidate.exists():
            return candidate.resolve()
    return fallback.resolve()


def sibling_calibration_dir(lidar_dir: Path, fallback: Path) -> Path:
    if lidar_dir.name.lower() == "lidar":
        candidate = lidar_dir.parent / "calibration"
        if candidate.exists():
            return candidate.resolve()
    return fallback.resolve()


def load_xacti_calibration(calibration_dir: Path) -> CameraCalibration | None:
    return CameraCalibration.from_json(calibration_dir / "xacti_4000x3000_camera_spec.json")


def scene_display_name(scene: dict[str, Any]) -> str:
    return str(scene.get("flight_identifier") or scene.get("flight_id") or "unknown")


def scene_index(scenes: list[dict[str, Any]], selected_scene: dict[str, Any]) -> int:
    selected_flight_id = selected_scene.get("flight_id")
    selected_identifier = selected_scene.get("flight_identifier")
    for index, scene in enumerate(scenes):
        if scene.get("flight_id") == selected_flight_id or scene.get("flight_identifier") == selected_identifier:
            return index
    return 0


def las_point_count_by_scene(lidar_dir: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in read_csv_dicts(lidar_dir / "las_files.csv"):
        count = to_float(row.get("point_count"))
        if count is None:
            continue
        int_count = int(count)
        for key in (row.get("flight_id"), row.get("flight_identifier"), row.get("las_file")):
            if key:
                counts[key] = int_count
    return counts


def pick_scene(manifest: dict[str, Any], flight_identifier: str) -> dict[str, Any]:
    scenes = manifest.get("scenes", [])
    if flight_identifier:
        for scene in scenes:
            if scene.get("flight_identifier") == flight_identifier or scene.get("flight_id") == flight_identifier:
                return scene
        raise SystemExit(f"Flight not found in scene_manifest.json: {flight_identifier}")
    for scene in scenes:
        if scene.get("snapshot_count", 0):
            return scene
    if scenes:
        return scenes[0]
    raise SystemExit("No scene found. Run 06_index_lidar_trajectory.py first.")


def load_point_cloud(
    lidar_dir: Path,
    scene: dict[str, Any],
    max_display_points: int,
    voxel_size: float,
) -> tuple[o3d.geometry.PointCloud, int, int]:
    preview = scene.get("preview_ply", "")
    if not preview:
        raise SystemExit("Scene has no preview_ply. Run 06_index_lidar_trajectory.py first.")
    path = lidar_dir / preview
    if not path.exists():
        raise SystemExit(f"Preview PLY not found: {path}")
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise SystemExit(f"Preview PLY is empty: {path}")
    if not pcd.has_colors():
        pcd.paint_uniform_color([0.65, 0.65, 0.65])

    original_point_count = len(pcd.points)
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
    if max_display_points > 0 and len(pcd.points) > max_display_points:
        every_k_points = max(1, math.ceil(len(pcd.points) / max_display_points))
        pcd = pcd.uniform_down_sample(every_k_points)
    return pcd, original_point_count, len(pcd.points)


def load_full_las_point_cloud(las_path: Path, cancelled: Any = None) -> o3d.geometry.PointCloud:
    """Load every LAS point while keeping raw intensity in grayscale colors."""
    with laspy.open(las_path) as reader:
        point_count = int(reader.header.point_count)
        if point_count <= 0:
            raise ValueError(f"LAS 파일에 점이 없습니다: {las_path}")

        points = np.empty((point_count, 3), dtype=np.float64)
        has_intensity = "intensity" in set(reader.header.point_format.dimension_names)
        intensities = np.empty(point_count, dtype=np.uint16) if has_intensity else None
        offset = 0
        for chunk in reader.chunk_iterator(500_000):
            if cancelled is not None and cancelled():
                raise InterruptedError("LAS 전체 로드를 취소했습니다.")
            size = len(chunk.x)
            end = offset + size
            points[offset:end, 0] = np.asarray(chunk.x)
            points[offset:end, 1] = np.asarray(chunk.y)
            points[offset:end, 2] = np.asarray(chunk.z)
            if intensities is not None:
                intensities[offset:end] = np.asarray(chunk.intensity, dtype=np.uint16)
            offset = end

    points = points[:offset]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if intensities is not None:
        intensity = intensities[:offset].astype(np.float64)
        max_intensity = float(np.max(intensity)) if len(intensity) else 0.0
        if max_intensity > 0:
            intensity /= max_intensity
        else:
            intensity.fill(0.0)
        pcd.colors = o3d.utility.Vector3dVector(np.column_stack((intensity, intensity, intensity)))
    else:
        pcd.paint_uniform_color([0.65, 0.65, 0.65])
    return pcd


def load_trajectory_geometry(lidar_dir: Path, flight_id: str, topic: str) -> o3d.geometry.LineSet | None:
    rows = [
        row
        for row in read_csv_dicts(lidar_dir / "flight_trajectories.csv")
        if row.get("flight_id") == flight_id and row.get("topic") == topic
    ]
    rows.sort(key=lambda row: float(row.get("header_stamp_sec") or -1))
    points: list[list[float]] = []
    for row in rows:
        x = to_float(row.get("position_x"))
        y = to_float(row.get("position_y"))
        z = to_float(row.get("position_z"))
        if x is None or y is None or z is None:
            continue
        points.append([x, y, z])
    if len(points) < 2:
        return None
    lines = [[index, index + 1] for index in range(len(points) - 1)]
    return line_set(points, lines, [1.0, 0.25, 0.05])


def resolve_project_path(project_root: Path, value: str | None) -> Path | None:
    if not value:
        return None
    path = Path(value)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def camera_frustum_geometry(
    origin: np.ndarray,
    rotation: np.ndarray,
    length: float,
    horizontal_fov_deg: float,
    aspect_ratio: float,
    calibration: CameraCalibration | None = None,
) -> o3d.geometry.LineSet:
    corners = camera_frustum_corners(origin, rotation, length, horizontal_fov_deg, aspect_ratio, calibration)
    points = [origin.tolist(), *[corner.tolist() for corner in corners]]
    lines = [
        [0, 1],
        [0, 2],
        [0, 3],
        [0, 4],
        [1, 2],
        [2, 3],
        [3, 4],
        [4, 1],
    ]
    return line_set(points, lines, [0.1, 1.0, 0.15])


def camera_frustum_corners(
    origin: np.ndarray,
    rotation: np.ndarray,
    length: float,
    horizontal_fov_deg: float,
    aspect_ratio: float,
    calibration: CameraCalibration | None = None,
) -> list[np.ndarray]:
    if calibration is not None:
        try:
            return calibration.frustum_corners(origin, rotation, length)
        except Exception:  # noqa: BLE001
            pass
    safe_length = max(0.05, float(length))
    forward = rotation[:, 0]
    right = rotation[:, 1]
    up = rotation[:, 2]
    safe_aspect = max(0.1, float(aspect_ratio))
    half_width = math.tan(math.radians(max(1.0, min(170.0, horizontal_fov_deg))) / 2.0) * safe_length
    half_height = half_width / safe_aspect
    center = origin + forward * safe_length
    return [
        center - right * half_width + up * half_height,
        center + right * half_width + up * half_height,
        center + right * half_width - up * half_height,
        center - right * half_width - up * half_height,
    ]


def image_plane_geometry(corners: list[np.ndarray]) -> o3d.geometry.TriangleMesh | None:
    if len(corners) != 4:
        return None
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(corners, dtype=float))
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray([[0, 1, 2], [0, 2, 3]], dtype=np.int32))
    mesh.triangle_uvs = o3d.utility.Vector2dVector(
        np.asarray(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [1.0, 1.0],
                [0.0, 0.0],
                [1.0, 1.0],
                [0.0, 1.0],
            ],
            dtype=float,
        )
    )
    mesh.compute_triangle_normals()
    return mesh


def snapshot_projection_bounds(
    corners: list[np.ndarray], origin: np.ndarray, rotation: np.ndarray, width: int, height: int,
) -> tuple[float, float, float, float]:
    # OpenGL screen-right is forward cross up, opposite to the stored local Y.
    forward = rotation[:, 0]
    up = rotation[:, 2]
    screen_right = np.cross(forward, up)
    offsets = np.asarray(corners, dtype=float) - origin
    depth = offsets @ forward
    if np.any(depth <= 0) or width <= 0 or height <= 0:
        raise ValueError("Invalid snapshot projection bounds.")
    x = (offsets @ screen_right) / depth
    y = (offsets @ up) / depth
    left, right = float(x.min()), float(x.max())
    bottom, top = float(y.min()), float(y.max())
    center_x, center_y = (left + right) / 2, (bottom + top) / 2
    half_width, half_height = (right - left) / 2, (top - bottom) / 2
    aspect = width / height
    if half_width / half_height < aspect:
        half_width = half_height * aspect
    else:
        half_height = half_width / aspect
    return center_x - half_width, center_x + half_width, center_y - half_height, center_y + half_height


def estimate_lidar_frustum_length(
    pcd: o3d.geometry.PointCloud | None,
    origin: np.ndarray,
    rotation: np.ndarray,
    horizontal_fov_deg: float,
    aspect_ratio: float,
    max_distance: float,
    min_distance: float,
    calibration: CameraCalibration | None = None,
) -> float | None:
    if pcd is None or pcd.is_empty():
        return None
    points = np.asarray(pcd.points, dtype=float)
    if points.size == 0:
        return None

    forward = rotation[:, 0]
    right = rotation[:, 1]
    up = rotation[:, 2]
    rel = points - origin
    forward_distance = rel @ forward
    lateral_distance = rel @ right
    vertical_distance = rel @ up

    if calibration is not None:
        try:
            half_width_ratio, half_height_ratio = calibration.half_extent_ratios()
        except Exception:  # noqa: BLE001
            half_width_ratio = math.tan(np.radians(max(1.0, min(170.0, horizontal_fov_deg))) / 2.0)
            half_height_ratio = half_width_ratio / max(0.1, aspect_ratio)
    else:
        half_width_ratio = math.tan(np.radians(max(1.0, min(170.0, horizontal_fov_deg))) / 2.0)
        half_height_ratio = half_width_ratio / max(0.1, aspect_ratio)
    half_width_at_distance = half_width_ratio * forward_distance
    half_height_at_distance = half_height_ratio * forward_distance

    mask = (
        (forward_distance >= min_distance)
        & (forward_distance <= max_distance)
        & (np.abs(lateral_distance) <= half_width_at_distance)
        & (np.abs(vertical_distance) <= half_height_at_distance)
    )
    hits = forward_distance[mask]
    if hits.size == 0:
        return None
    return float(np.percentile(hits, 5))


def choose_frustum_length(
    mode: str,
    fallback_length: float,
    wall_distance: float | None,
    pcd: o3d.geometry.PointCloud | None,
    origin: np.ndarray,
    rotation: np.ndarray,
    horizontal_fov_deg: float,
    aspect_ratio: float,
    lidar_max_distance: float,
    lidar_min_distance: float,
    calibration: CameraCalibration | None = None,
) -> tuple[float, str]:
    if mode == "fixed":
        return fallback_length, "fixed"
    if mode == "lidar":
        lidar_distance = estimate_lidar_frustum_length(
            pcd,
            origin,
            rotation,
            horizontal_fov_deg,
            aspect_ratio,
            lidar_max_distance,
            lidar_min_distance,
            calibration,
        )
        if lidar_distance is not None and lidar_distance > 0:
            return lidar_distance, "lidar"
    if wall_distance is not None and wall_distance > 0:
        return wall_distance, "wall"
    return fallback_length, "fixed"


def point_segment_distance_2d(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    segment = end - start
    length_sq = float(np.dot(segment, segment))
    if length_sq <= 1e-9:
        return float(np.linalg.norm(point - start))
    t = float(np.clip(np.dot(point - start, segment) / length_sq, 0.0, 1.0))
    closest = start + segment * t
    return float(np.linalg.norm(point - closest))


def snapshot_metadata_by_id(snapshot_dir: Path) -> dict[str, dict[str, str]]:
    rows = read_csv_dicts(snapshot_dir / "snapshot_json_poses.csv")
    metadata: dict[str, dict[str, str]] = {}
    for row in rows:
        snapshot_id = row.get("snapshot_id", "")
        if snapshot_id:
            metadata[snapshot_id] = row
    return metadata


def load_snapshot_markers(
    project_root: Path,
    snapshot_dir: Path,
    flight_id: str,
    axis_length: float,
    frustum_length: float,
    frustum_fov_deg: float,
    frustum_length_mode: str,
    lidar_reference_pcd: o3d.geometry.PointCloud | None,
    lidar_frustum_max_distance: float,
    lidar_frustum_min_distance: float,
    camera_calibration: CameraCalibration | None,
) -> list[SnapshotMarker]:
    markers: list[SnapshotMarker] = []
    metadata = snapshot_metadata_by_id(snapshot_dir)
    seen_snapshot_ids: set[str] = set()
    for row in read_csv_dicts(snapshot_dir / "mcap_snapshot_events.csv"):
        if row.get("flight_id") != flight_id:
            continue
        snapshot_id = str(row.get("snapshot_id", ""))
        if snapshot_id and snapshot_id in seen_snapshot_ids:
            continue
        if snapshot_id:
            seen_snapshot_ids.add(snapshot_id)
        meta = metadata.get(snapshot_id, {})
        x = to_float(row.get("camera_x"))
        y = to_float(row.get("camera_y"))
        z = to_float(row.get("camera_z"))
        if x is None or y is None or z is None:
            continue
        q = (to_float(row.get("camera_qx")), to_float(row.get("camera_qy")), to_float(row.get("camera_qz")), to_float(row.get("camera_qw")))
        origin = np.array([x, y, z], dtype=float)
        rotation = quat_to_matrix(q)
        wall_distance = to_float(row.get("wall_distance"))
        marker_frustum_length, marker_frustum_source = choose_frustum_length(
            frustum_length_mode,
            frustum_length,
            wall_distance,
            lidar_reference_pcd,
            origin,
            rotation,
            frustum_fov_deg,
            4.0 / 3.0,
            lidar_frustum_max_distance,
            lidar_frustum_min_distance,
            camera_calibration,
        )
        geometries: list[o3d.geometry.Geometry] = []
        axes = [
            (rotation[:, 0], [1.0, 0.0, 0.0]),
            (rotation[:, 1], [0.0, 1.0, 0.0]),
            (rotation[:, 2], [0.1, 0.35, 1.0]),
        ]
        for vector, color in axes:
            points = [origin.tolist(), (origin + vector * axis_length).tolist()]
            geometries.append(line_set(points, [[0, 1]], color))
        frustum_corners = camera_frustum_corners(origin, rotation, marker_frustum_length, frustum_fov_deg, 4.0 / 3.0, camera_calibration)
        geometries.append(camera_frustum_geometry(origin, rotation, marker_frustum_length, frustum_fov_deg, 4.0 / 3.0, camera_calibration))
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=axis_length * 0.09)
        sphere.translate(origin)
        sphere.paint_uniform_color([0.1, 1.0, 0.25])
        geometries.append(sphere)

        image_path = resolve_project_path(project_root, meta.get("camera_image"))
        thumbnail_path = resolve_project_path(project_root, meta.get("thumbnail"))
        markers.append(
            SnapshotMarker(
                snapshot_id=snapshot_id,
                poi_name=str(meta.get("poi_name") or row.get("poi_id", "")),
                image_name=str(meta.get("camera_image_name", "")),
                image_path=image_path if image_path and image_path.exists() else None,
                thumbnail_path=thumbnail_path if thumbnail_path and thumbnail_path.exists() else None,
                origin=origin,
                rotation=rotation,
                wall_distance=wall_distance,
                frustum_length=marker_frustum_length,
                frustum_length_source=marker_frustum_source,
                pose_source="mcap",
                frustum_source=camera_calibration.camera_model if camera_calibration is not None else f"fov {frustum_fov_deg:.1f} deg",
                frustum_corners=frustum_corners,
                geometries=geometries,
            )
        )
    return markers


def snapshot_geometries(snapshot_dir: Path, flight_id: str, axis_length: float) -> list[o3d.geometry.Geometry]:
    geometries: list[o3d.geometry.Geometry] = []
    for row in read_csv_dicts(snapshot_dir / "snapshot_json_poses.csv"):
        if row.get("flight_id") != flight_id:
            continue
        x = to_float(row.get("camera_x"))
        y = to_float(row.get("camera_y"))
        z = to_float(row.get("camera_z"))
        if x is None or y is None or z is None:
            continue
        q = (to_float(row.get("camera_qx")), to_float(row.get("camera_qy")), to_float(row.get("camera_qz")), to_float(row.get("camera_qw")))
        origin = np.array([x, y, z], dtype=float)
        rotation = quat_to_matrix(q)
        axes = [
            (rotation[:, 0], [1.0, 0.0, 0.0]),
            (rotation[:, 1], [0.0, 1.0, 0.0]),
            (rotation[:, 2], [0.1, 0.35, 1.0]),
        ]
        for vector, color in axes:
            points = [origin.tolist(), (origin + vector * axis_length).tolist()]
            geometries.append(line_set(points, [[0, 1]], color))
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=axis_length * 0.075)
        sphere.translate(origin)
        sphere.paint_uniform_color([1.0, 0.9, 0.1])
        geometries.append(sphere)
    return geometries


class LidarSceneApp:
    POINT_GEOMETRY = "lidar_points"
    TRAJECTORY_GEOMETRY = "trajectory"
    FRAME_GEOMETRY = "origin_frame"
    IMAGE_PLANE_GEOMETRY = "selected_snapshot_image_plane"
    FRUSTUM_MODE_ITEMS = [
        ("wall", "wall_distance"),
        ("lidar", "LiDAR 점군 추정"),
        ("fixed", "고정 길이"),
    ]
    COLOR_MODE_ITEMS = [
        ("gray", "회색"),
        ("height", "높이 Z 컬러"),
        ("intensity", "intensity 컬러"),
        ("inspector", "Inspector 스타일 컬러"),
    ]

    def __init__(
        self,
        project_root: Path,
        lidar_dir: Path,
        snapshot_dir: Path,
        calibration_dir: Path,
        scenes: list[dict[str, Any]],
        initial_scene_index: int,
        topic: str,
        axis_length: float,
        frustum_length: float,
        frustum_fov_deg: float,
        frustum_length_mode: str,
        lidar_frustum_max_distance: float,
        lidar_frustum_min_distance: float,
        snapshot_hover_radius_px: float,
        voxel_size: float,
        initial_max_display_points: int,
        point_size: float,
        color_mode: str,
        line_width: float,
        show_trajectory: bool,
        window_width: int,
        window_height: int,
    ) -> None:
        self.project_root = project_root
        self.lidar_dir = lidar_dir
        self.snapshot_dir = snapshot_dir
        self.calibration_dir = calibration_dir
        self.camera_calibration = load_xacti_calibration(calibration_dir)
        self.scenes = scenes
        self.current_scene_index = max(0, min(initial_scene_index, len(scenes) - 1))
        self.topic = topic
        self.axis_length = axis_length
        self.frustum_length = max(0.05, float(frustum_length))
        self.frustum_fov_deg = max(1.0, min(170.0, float(frustum_fov_deg)))
        self.frustum_length_mode = frustum_length_mode if frustum_length_mode in {"wall", "lidar", "fixed"} else "wall"
        self.lidar_frustum_max_distance = max(0.1, float(lidar_frustum_max_distance))
        self.lidar_frustum_min_distance = max(0.0, float(lidar_frustum_min_distance))
        self.snapshot_hover_radius_px = max(4.0, float(snapshot_hover_radius_px))
        self.voxel_size = voxel_size
        self.target_display_points = max(0, int(initial_max_display_points))
        self.point_size = max(1.0, float(point_size))
        self.color_mode = color_mode if color_mode in {value for value, _ in self.COLOR_MODE_ITEMS} else "inspector"
        self.line_width = max(1.0, float(line_width))
        self.show_trajectory = show_trajectory
        self.base_pcd = o3d.geometry.PointCloud()
        self.preview_point_count = 0
        self.current_point_count = 0
        self.full_cloud_enabled = False
        self.full_cloud_loading = False
        self._point_cloud_load_generation = 0
        self.trajectory: o3d.geometry.LineSet | None = None
        self.snapshot_markers: list[SnapshotMarker] = []
        self.snapshot_geometry_names: list[tuple[str, o3d.geometry.Geometry]] = []
        self.selected_snapshot_index = -1
        self.hover_snapshot_index = -1
        self.las_point_counts = las_point_count_by_scene(lidar_dir)
        self.show_snapshot_locations = True
        self.show_image_plane = False
        self.use_undistorted_image = False
        self.snapshot_view_active = False
        self._last_image_open_time = 0.0
        self._updating_controls = False
        self.extract_input_path: Path | None = project_root
        self.extract_custom_output_root: Path | None = None
        self.extract_running = False
        self.extract_input_label: gui.Label | None = None
        self.extract_output_label: gui.Label | None = None
        self.extract_status_label: gui.Label | None = None
        self.extract_run_button: gui.Button | None = None

        self.window = gui.Application.instance.create_window(
            f"ROSM LiDAR - {scene_display_name(scenes[self.current_scene_index])}",
            window_width,
            window_height,
        )
        self.widget3d = gui.SceneWidget()
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self.widget3d.scene.set_background([0.02, 0.02, 0.02, 1.0])
        self.widget3d.scene.show_axes(False)

        em = self.window.theme.font_size
        self.panel = gui.Vert(0.6 * em, gui.Margins(0.8 * em, 0.8 * em, 0.8 * em, 0.8 * em))
        self.output_dir_label = gui.Label("")
        self.flight_combo = gui.Combobox()
        for scene in self.scenes:
            self.flight_combo.add_item(scene_display_name(scene))
        self.flight_combo.selected_index = self.current_scene_index
        self.flight_combo.set_on_selection_changed(self._on_flight_selected)
        self.snapshot_combo = gui.Combobox()
        self.snapshot_combo.set_on_selection_changed(self._on_snapshot_selected)
        self.snapshot_combo.enabled = False
        self.snapshot_hint_label = gui.Label("마우스 올림: 하이라이트 / 클릭: 원본 열기")
        self.selected_snapshot_label = gui.Label("선택된 스냅샷 없음")
        self.frustum_mode_combo = gui.Combobox()
        for _, label in self.FRUSTUM_MODE_ITEMS:
            self.frustum_mode_combo.add_item(label)
        self.frustum_mode_combo.selected_index = self._frustum_mode_index(self.frustum_length_mode)
        self.frustum_mode_combo.set_on_selection_changed(self._on_frustum_mode_selected)
        self.status_label = gui.Label("")
        self.color_mode_combo = gui.Combobox()
        for _, label in self.COLOR_MODE_ITEMS:
            self.color_mode_combo.add_item(label)
        self.color_mode_combo.selected_index = self._color_mode_index(self.color_mode)
        self.color_mode_combo.set_on_selection_changed(self._on_color_mode_selected)
        self.point_size_label = gui.Label("")
        self.point_count_label = gui.Label("")
        self.line_width_label = gui.Label("")
        self.trajectory_checkbox = gui.Checkbox("비행 궤적 표시")
        self.trajectory_checkbox.checked = show_trajectory
        self.trajectory_checkbox.set_on_checked(self._on_trajectory_checked)
        self.snapshot_locations_checkbox = gui.Checkbox("사진 위치 보기")
        self.snapshot_locations_checkbox.checked = self.show_snapshot_locations
        self.snapshot_locations_checkbox.set_on_checked(self._on_snapshot_locations_checked)
        self.full_cloud_checkbox = gui.Checkbox("원본 LAS 전체 로드")
        self.full_cloud_checkbox.checked = False
        self.full_cloud_checkbox.set_on_checked(self._on_full_cloud_checked)
        self.image_plane_checkbox = gui.Checkbox("사진 평면 표시")
        self.image_plane_checkbox.checked = self.show_image_plane
        self.image_plane_checkbox.set_on_checked(self._on_image_plane_checked)
        self.undistort_image_checkbox = gui.Checkbox("이미지 왜곡 보정")
        self.undistort_image_checkbox.checked = self.use_undistorted_image
        self.undistort_image_checkbox.enabled = self._can_undistort_images()
        self.undistort_image_checkbox.set_on_checked(self._on_undistort_image_checked)

        self.point_size_slider = gui.Slider(gui.Slider.DOUBLE)
        self.point_size_slider.set_limits(1.0, 12.0)
        self.point_size_slider.double_value = self.point_size
        self.point_size_slider.set_on_value_changed(self._on_point_size_changed)

        self.point_count_slider = gui.Slider(gui.Slider.INT)
        self.point_count_slider.set_on_value_changed(self._on_point_count_changed)

        self.line_width_slider = gui.Slider(gui.Slider.DOUBLE)
        self.line_width_slider.set_limits(1.0, 20.0)
        self.line_width_slider.double_value = self.line_width
        self.line_width_slider.set_on_value_changed(self._on_line_width_changed)

        fit_button = gui.Button("화면 맞춤")
        fit_button.set_on_clicked(self._fit_camera)
        output_button = gui.Button("출력 폴더 변경")
        output_button.set_on_clicked(self._open_output_folder_dialog)
        view_image_button = gui.Button("선택 이미지 보기")
        view_image_button.set_on_clicked(self._show_selected_snapshot_image)
        snapshot_view_button = gui.Button("선택 시점 보기")
        snapshot_view_button.set_on_clicked(self._view_from_selected_snapshot)
        extract_button = gui.Button("MCAP 정보 추출")
        extract_button.set_on_clicked(self._open_mcap_extract_dialog)

        self._build_panel(extract_button, output_button, fit_button, view_image_button, snapshot_view_button)
        self.window.add_child(self.widget3d)
        self.window.add_child(self.panel)
        self.window.set_on_layout(self._on_layout)
        self.widget3d.set_on_mouse(self._on_mouse_widget3d)

        self._load_scene(self.current_scene_index, reset_camera=True)

    def _build_panel(
        self,
        extract_button: gui.Button,
        output_button: gui.Button,
        fit_button: gui.Button,
        view_image_button: gui.Button,
        snapshot_view_button: gui.Button,
    ) -> None:
        self.panel.add_child(extract_button)
        self.panel.add_child(self.output_dir_label)
        self.panel.add_child(output_button)
        self.panel.add_fixed(8)
        self.panel.add_child(gui.Label("비행 선택"))
        self.panel.add_child(self.flight_combo)
        self.panel.add_fixed(8)
        self.panel.add_child(gui.Label("스냅샷 선택"))
        self.panel.add_child(self.snapshot_combo)
        self.panel.add_child(self.snapshot_hint_label)
        self.panel.add_child(self.selected_snapshot_label)
        self.panel.add_child(view_image_button)
        self.panel.add_child(snapshot_view_button)
        self.panel.add_child(self.image_plane_checkbox)
        self.panel.add_child(self.undistort_image_checkbox)
        self.panel.add_fixed(8)
        self.panel.add_child(gui.Label("프러스텀 길이 기준"))
        self.panel.add_child(self.frustum_mode_combo)
        self.panel.add_fixed(8)
        self.panel.add_child(gui.Label("점군 색상"))
        self.panel.add_child(self.color_mode_combo)
        self.panel.add_fixed(8)
        self.panel.add_child(self.status_label)
        self.panel.add_fixed(8)
        self.panel.add_child(self.trajectory_checkbox)
        self.panel.add_child(self.snapshot_locations_checkbox)
        self.panel.add_fixed(8)
        self.panel.add_child(self.point_size_label)
        self.panel.add_child(self.point_size_slider)
        self.panel.add_fixed(8)
        self.panel.add_child(self.full_cloud_checkbox)
        self.panel.add_child(self.point_count_label)
        self.panel.add_child(self.point_count_slider)
        self.panel.add_fixed(8)
        self.panel.add_child(self.line_width_label)
        self.panel.add_child(self.line_width_slider)
        self.panel.add_fixed(12)
        self.panel.add_child(fit_button)
        self.panel.add_stretch()

    def _add_scene_geometries(self, display_pcd: o3d.geometry.PointCloud) -> None:
        self.widget3d.scene.clear_geometry()
        self.widget3d.scene.add_geometry(
            self.POINT_GEOMETRY,
            display_pcd,
            material("defaultUnlit", point_size=self.point_size),
        )
        if self.trajectory is not None:
            self.widget3d.scene.add_geometry(
                self.TRAJECTORY_GEOMETRY,
                self.trajectory,
                material("unlitLine", line_width=self.line_width),
            )
            self.widget3d.scene.show_geometry(self.TRAJECTORY_GEOMETRY, self.show_trajectory)
        self.snapshot_geometry_names = []
        for marker_index, marker in enumerate(self.snapshot_markers):
            for geometry_index, geometry in enumerate(marker.geometries):
                name = f"snapshot_{marker_index}_{geometry_index}"
                mat = material("unlitLine" if isinstance(geometry, o3d.geometry.LineSet) else "defaultUnlit", line_width=self.line_width)
                self.widget3d.scene.add_geometry(name, geometry, mat)
                self.widget3d.scene.show_geometry(name, self.show_snapshot_locations)
                self.snapshot_geometry_names.append((name, geometry))
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=self.axis_length * 2.0)
        self.widget3d.scene.add_geometry(self.FRAME_GEOMETRY, frame, material("defaultUnlit"))

    def _on_layout(self, ctx: gui.LayoutContext) -> None:
        em = ctx.theme.font_size
        rect = self.window.content_rect
        panel_width = max(340, 24 * em)
        self.widget3d.frame = gui.Rect(rect.x, rect.y, rect.width - panel_width, rect.height)
        self.panel.frame = gui.Rect(rect.get_right() - panel_width, rect.y, panel_width, rect.height)
        if self.snapshot_view_active:
            self._apply_snapshot_projection()

    def _fit_camera(self) -> None:
        self.snapshot_view_active = False
        bbox = tight_point_cloud_bbox(self.base_pcd) if len(self.base_pcd.points) else self.widget3d.scene.bounding_box
        if not bbox.is_empty():
            self.widget3d.setup_camera(85.0, bbox, bbox.get_center())
        if self.show_snapshot_locations and 0 <= self.selected_snapshot_index < len(self.snapshot_markers):
            self._show_selection_marker(self.snapshot_markers[self.selected_snapshot_index].origin)

    def _refresh_labels(self) -> None:
        scene = self.scenes[self.current_scene_index]
        if is_empty_scene(scene):
            self.output_dir_label.text = f"출력 폴더\n{self.lidar_dir}"
            self.status_label.text = (
                "출력 데이터 없음\n"
                "MCAP 정보 추출을 실행하세요.\n"
                f"{self._calibration_status_text()}"
            )
            self.point_size_label.text = f"LiDAR 점 크기: {self.point_size:.1f}"
            self.point_count_label.text = "표시 점 수: 0"
            self.line_width_label.text = f"궤적/카메라 선 굵기: {self.line_width:.1f}"
            return
        flight = scene_display_name(scene)
        las_count = (
            self.las_point_counts.get(str(scene.get("flight_id", "")))
            or self.las_point_counts.get(str(scene.get("flight_identifier", "")))
            or self.las_point_counts.get(str(scene.get("las_file", "")))
        )
        las_text = f"{las_count:,}" if las_count else "확인 안 됨"
        self.output_dir_label.text = f"출력 폴더\n{self.lidar_dir}"
        self.status_label.text = (
            f"{flight}\n"
            f"표시 점 수 {self.current_point_count:,} / "
            f"{'원본 LAS' if self.full_cloud_enabled else 'preview'} {len(self.base_pcd.points):,}\n"
            f"원본 LAS 점 수 {las_text}\n"
            f"궤적 {self._trajectory_point_count():,} / 스냅샷 {len(self.snapshot_markers)}\n"
            f"{self._calibration_status_text()}"
        )
        if self.full_cloud_loading:
            self.status_label.text += "\n원본 LAS 전체 로드 중..."
        self.point_size_label.text = f"LiDAR 점 크기: {self.point_size:.1f}"
        self.point_count_label.text = f"표시 점 수: {self.current_point_count:,}"
        self.line_width_label.text = f"궤적/카메라 선 굵기: {self.line_width:.1f}"

    def _trajectory_point_count(self) -> int:
        if self.trajectory is None:
            return 0
        return len(self.trajectory.points)

    def _frustum_mode_index(self, mode: str) -> int:
        for index, (value, _) in enumerate(self.FRUSTUM_MODE_ITEMS):
            if value == mode:
                return index
        return 0

    def _frustum_mode_value(self, index: int) -> str:
        if index < 0 or index >= len(self.FRUSTUM_MODE_ITEMS):
            return self.FRUSTUM_MODE_ITEMS[0][0]
        return self.FRUSTUM_MODE_ITEMS[index][0]

    def _color_mode_index(self, mode: str) -> int:
        for index, (value, _) in enumerate(self.COLOR_MODE_ITEMS):
            if value == mode:
                return index
        return len(self.COLOR_MODE_ITEMS) - 1

    def _color_mode_value(self, index: int) -> str:
        if index < 0 or index >= len(self.COLOR_MODE_ITEMS):
            return "inspector"
        return self.COLOR_MODE_ITEMS[index][0]

    def _can_undistort_images(self) -> bool:
        return cv2 is not None and self.camera_calibration is not None

    def _calibration_status_text(self) -> str:
        if self.camera_calibration is None:
            return "Xacti 보정 없음"
        return f"Xacti 보정 {self.camera_calibration.image_width} x {self.camera_calibration.image_height}"

    def _display_point_cloud(self, max_points: int) -> o3d.geometry.PointCloud:
        display_pcd = downsample_point_cloud(self.base_pcd, max_points)
        display_pcd = clone_point_cloud(display_pcd)
        return apply_lidar_color_mode(display_pcd, self.color_mode)

    def _refresh_snapshot_combo(self) -> None:
        self.snapshot_combo.clear_items()
        self.selected_snapshot_index = -1
        self.hover_snapshot_index = -1
        if not self.snapshot_markers:
            self.snapshot_combo.add_item("스냅샷 없음")
            self.snapshot_combo.selected_index = 0
            self.snapshot_combo.enabled = False
            self.selected_snapshot_label.text = "선택된 스냅샷 없음"
            return
        for marker in self.snapshot_markers:
            self.snapshot_combo.add_item(marker.display_name)
        self.snapshot_combo.selected_index = 0
        self.snapshot_combo.enabled = True
        self.selected_snapshot_index = 0
        marker = self.snapshot_markers[0]
        self.selected_snapshot_label.text = f"선택: {marker.display_name}\n{marker.distance_text}\n{marker.source_text}"

    def _select_snapshot(self, index: int, open_image: bool) -> None:
        if not self.snapshot_markers:
            self.selected_snapshot_index = -1
            self.selected_snapshot_label.text = "선택된 스냅샷 없음"
            return
        self.selected_snapshot_index = max(0, min(index, len(self.snapshot_markers) - 1))
        marker = self.snapshot_markers[self.selected_snapshot_index]
        if self.snapshot_combo.selected_index != self.selected_snapshot_index:
            self.snapshot_combo.selected_index = self.selected_snapshot_index
        self.selected_snapshot_label.text = f"선택: {marker.display_name}\n{marker.distance_text}\n{marker.source_text}"
        self._show_selection_marker(marker.origin)
        self._refresh_image_plane()
        if self.snapshot_view_active:
            self._view_from_selected_snapshot()
        if open_image:
            self._show_snapshot_image(marker)

    def _remove_selection_marker(self) -> None:
        name = "selected_snapshot"
        if self.widget3d.scene.has_geometry(name):
            self.widget3d.scene.remove_geometry(name)

    def _show_selection_marker(self, origin: np.ndarray) -> None:
        name = "selected_snapshot"
        self._remove_selection_marker()
        if not self.show_snapshot_locations or self.snapshot_view_active:
            self.window.post_redraw()
            return
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=self.axis_length * 0.09)
        sphere.translate(origin)
        sphere.paint_uniform_color([1.0, 0.95, 0.05])
        self.widget3d.scene.add_geometry(name, sphere, material("defaultUnlit"))
        self.window.post_redraw()

    def _refresh_snapshot_location_visibility(self) -> None:
        for name, _ in self.snapshot_geometry_names:
            if self.widget3d.scene.has_geometry(name):
                self.widget3d.scene.show_geometry(name, self.show_snapshot_locations)
        if self.show_snapshot_locations and not self.snapshot_view_active and 0 <= self.selected_snapshot_index < len(self.snapshot_markers):
            self._show_selection_marker(self.snapshot_markers[self.selected_snapshot_index].origin)
        else:
            self._remove_selection_marker()
            self._clear_hover_marker()
            self.window.post_redraw()

    def _remove_image_plane(self) -> None:
        if self.widget3d.scene.has_geometry(self.IMAGE_PLANE_GEOMETRY):
            self.widget3d.scene.remove_geometry(self.IMAGE_PLANE_GEOMETRY)

    def _refresh_image_plane(self) -> None:
        self._remove_image_plane()
        if not self.show_image_plane:
            self.window.post_redraw()
            return
        if self.selected_snapshot_index < 0 or self.selected_snapshot_index >= len(self.snapshot_markers):
            self.window.post_redraw()
            return
        marker = self.snapshot_markers[self.selected_snapshot_index]
        try:
            path = self._image_path_for_marker(marker)
        except Exception as exc:  # noqa: BLE001
            self.window.show_message_box("이미지 보정 오류", f"{marker.display_name}\n{exc}")
            self.window.post_redraw()
            return
        if path is None or not path.exists():
            self.window.post_redraw()
            return
        mesh = image_plane_geometry(marker.frustum_corners)
        if mesh is None:
            self.window.post_redraw()
            return
        try:
            mat = material("defaultUnlit")
            mat.albedo_img = o3d.io.read_image(str(path))
            self.widget3d.scene.add_geometry(self.IMAGE_PLANE_GEOMETRY, mesh, mat)
        except Exception as exc:  # noqa: BLE001
            self.window.show_message_box("사진 평면 오류", f"{path}\n{exc}")
        self.window.post_redraw()

    def _view_from_selected_snapshot(self) -> None:
        if self.selected_snapshot_index < 0 or self.selected_snapshot_index >= len(self.snapshot_markers):
            self.window.show_message_box("스냅샷 없음", "선택된 스냅샷이 없습니다.")
            return
        marker = self.snapshot_markers[self.selected_snapshot_index]
        forward = marker.rotation[:, 0]
        up = marker.rotation[:, 2]
        look_distance = max(marker.frustum_length, self.frustum_length, 0.5)
        self.widget3d.look_at(
            (marker.origin + forward * look_distance).astype(np.float32),
            marker.origin.astype(np.float32),
            up.astype(np.float32),
        )
        self.snapshot_view_active = True
        self._remove_selection_marker()
        self._clear_hover_marker()
        self._apply_snapshot_projection()
        self.window.post_redraw()

    def _apply_snapshot_projection(self) -> None:
        if not 0 <= self.selected_snapshot_index < len(self.snapshot_markers):
            return
        frame = self.widget3d.frame
        if frame.width <= 0 or frame.height <= 0:
            return
        marker = self.snapshot_markers[self.selected_snapshot_index]
        bounds = snapshot_projection_bounds(
            marker.frustum_corners, marker.origin, marker.rotation, frame.width, frame.height,
        )
        near = max(0.001, min(0.05, marker.frustum_length * 0.05))
        bbox = self.widget3d.scene.bounding_box
        far = max(10.0, marker.frustum_length * 2.0)
        if not bbox.is_empty():
            far = max(far, float(np.max(np.linalg.norm(np.asarray(bbox.get_box_points()) - marker.origin, axis=1))) * 1.1)
        # Keep an intrinsic projection so Open3D can resize it without losing the off-axis center.
        left, right, bottom, top = bounds
        fx = frame.width / (right - left)
        fy = frame.height / (top - bottom)
        projection = np.array([[fx, 0.0, -left * fx], [0.0, fy, top * fy], [0.0, 0.0, 1.0]])
        self.widget3d.scene.camera.set_projection(
            projection, near, far, float(frame.width), float(frame.height),
        )

    def _highlight_frustum_geometry(self, marker: SnapshotMarker) -> o3d.geometry.LineSet | None:
        for geometry in marker.geometries:
            if not isinstance(geometry, o3d.geometry.LineSet):
                continue
            points = np.asarray(geometry.points)
            lines = np.asarray(geometry.lines)
            if len(lines) < 8:
                continue
            return line_set(points.tolist(), lines.tolist(), [1.0, 1.0, 1.0])
        return None

    def _show_hover_marker(self, index: int) -> None:
        if index < 0 or index >= len(self.snapshot_markers):
            self._clear_hover_marker()
            return
        if (
            self.hover_snapshot_index == index
            and self.widget3d.scene.has_geometry("hover_snapshot")
            and self.widget3d.scene.has_geometry("hover_frustum")
        ):
            return
        self.hover_snapshot_index = index
        if self.widget3d.scene.has_geometry("hover_snapshot"):
            self.widget3d.scene.remove_geometry("hover_snapshot")
        if self.widget3d.scene.has_geometry("hover_frustum"):
            self.widget3d.scene.remove_geometry("hover_frustum")
        marker = self.snapshot_markers[index]
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=self.axis_length * 0.09)
        sphere.translate(marker.origin)
        sphere.paint_uniform_color([1.0, 1.0, 1.0])
        self.widget3d.scene.add_geometry("hover_snapshot", sphere, material("defaultUnlit"))
        frustum = self._highlight_frustum_geometry(marker)
        if frustum is not None:
            self.widget3d.scene.add_geometry(
                "hover_frustum",
                frustum,
                material("unlitLine", line_width=max(self.line_width * 1.8, self.line_width + 2.0)),
            )
        self.selected_snapshot_label.text = f"가리킴: {marker.display_name}\n{marker.distance_text}\n{marker.source_text}"
        self.window.post_redraw()

    def _clear_hover_marker(self) -> None:
        if self.hover_snapshot_index < 0:
            return
        self.hover_snapshot_index = -1
        if self.widget3d.scene.has_geometry("hover_snapshot"):
            self.widget3d.scene.remove_geometry("hover_snapshot")
        if self.widget3d.scene.has_geometry("hover_frustum"):
            self.widget3d.scene.remove_geometry("hover_frustum")
        if self.selected_snapshot_index >= 0 and self.selected_snapshot_index < len(self.snapshot_markers):
            marker = self.snapshot_markers[self.selected_snapshot_index]
            self.selected_snapshot_label.text = f"선택: {marker.display_name}\n{marker.distance_text}\n{marker.source_text}"
        else:
            self.selected_snapshot_label.text = "선택된 스냅샷 없음"
        self.window.post_redraw()

    def _image_path_for_marker(self, marker: SnapshotMarker) -> Path | None:
        if self.use_undistorted_image and marker.image_path is not None:
            undistorted = self._undistorted_image_for_marker(marker)
            if undistorted is not None and undistorted.exists():
                return undistorted
        return marker.image_path or marker.thumbnail_path

    def _undistorted_image_for_marker(self, marker: SnapshotMarker) -> Path | None:
        if cv2 is None or self.camera_calibration is None or marker.image_path is None:
            return None
        source_path = marker.image_path
        if not source_path.exists():
            return None

        cache_dir = self.calibration_dir / "undistorted_images"
        cache_dir.mkdir(parents=True, exist_ok=True)
        cache_path = cache_dir / f"{marker.snapshot_id}_{source_path.stem}_undistorted_raw_v3.jpg"
        try:
            calibration_mtime = self.camera_calibration.source_file.stat().st_mtime
        except OSError:
            calibration_mtime = 0.0
        required_mtime = max(source_path.stat().st_mtime, calibration_mtime)
        if cache_path.exists() and cache_path.stat().st_mtime >= required_mtime:
            return cache_path

        data = np.fromfile(str(source_path), dtype=np.uint8)
        # Match Open3D's raw JPEG pixel coordinates; EXIF must not rotate K/P's input.
        image = cv2.imdecode(data, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        if image is None:
            raise RuntimeError(f"이미지 파일을 읽을 수 없습니다: {source_path}")
        height, width = image.shape[:2]
        intrinsic = self.camera_calibration.intrinsic_for_size(width, height)
        projection = self.camera_calibration.projection_for_size(width, height)
        distortion = self.camera_calibration.d.reshape((4, 1))
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            intrinsic,
            distortion,
            np.eye(3, dtype=np.float64),
            projection,
            (width, height),
            cv2.CV_16SC2,
        )
        undistorted = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # Open3D uses raw pixels; external photo viewers still need the source display orientation.
        with Image.open(source_path) as source_image:
            orientation = source_image.getexif().get(274, 1)
        exif = Image.Exif()
        exif[274] = orientation if orientation in range(1, 9) else 1
        Image.fromarray(cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB)).save(
            cache_path, format="JPEG", quality=95, exif=exif,
        )
        return cache_path

    def _show_snapshot_image(self, marker: SnapshotMarker) -> None:
        try:
            path = self._image_path_for_marker(marker)
            if path is None or not path.exists():
                self.window.show_message_box("이미지 없음", f"이미지 파일을 찾을 수 없습니다.\n{marker.display_name}")
                return
        except Exception as exc:  # noqa: BLE001
            self.window.show_message_box("이미지 열기 오류", f"{marker.display_name}\n{exc}")
            return

        now = time.monotonic()
        if now - self._last_image_open_time < 0.75:
            return
        self._last_image_open_time = now

        def report_error(exc: Exception) -> None:
            message = f"{marker.display_name}\n{exc}"
            gui.Application.instance.post_to_main_thread(
                self.window,
                lambda message=message: self.window.show_message_box("이미지 열기 오류", message),
            )

        start_file_in_background(path, report_error)

    def _show_selected_snapshot_image(self) -> None:
        if self.selected_snapshot_index < 0 or self.selected_snapshot_index >= len(self.snapshot_markers):
            self.window.show_message_box("스냅샷 없음", "선택된 스냅샷이 없습니다.")
            return
        self._show_snapshot_image(self.snapshot_markers[self.selected_snapshot_index])

    def _selected_display_point_limit(self) -> int:
        point_count_limit = max(1, len(self.base_pcd.points))
        if self.target_display_points <= 0:
            return point_count_limit
        return max(1, min(self.target_display_points, point_count_limit))

    def _show_empty_outputs(self) -> None:
        self._point_cloud_load_generation += 1
        self.snapshot_view_active = False
        self.base_pcd = o3d.geometry.PointCloud()
        self.preview_point_count = 0
        self.current_point_count = 0
        self.full_cloud_enabled = False
        self.full_cloud_loading = False
        self.trajectory = None
        self.snapshot_markers = []
        self.snapshot_geometry_names = []
        self.selected_snapshot_index = -1
        self.hover_snapshot_index = -1

        self.widget3d.scene.clear_geometry()
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=self.axis_length * 2.0)
        self.widget3d.scene.add_geometry(self.FRAME_GEOMETRY, frame, material("defaultUnlit"))

        self._updating_controls = True
        self.flight_combo.selected_index = self.current_scene_index
        self.point_count_slider.set_limits(1, 1)
        self.point_count_slider.int_value = 1
        self.trajectory_checkbox.enabled = False
        self.trajectory_checkbox.checked = False
        self.snapshot_locations_checkbox.enabled = False
        self.snapshot_locations_checkbox.checked = False
        self.full_cloud_checkbox.enabled = False
        self.full_cloud_checkbox.checked = False
        self.undistort_image_checkbox.enabled = self._can_undistort_images()
        self.undistort_image_checkbox.checked = self.use_undistorted_image and self._can_undistort_images()
        self.frustum_mode_combo.selected_index = self._frustum_mode_index(self.frustum_length_mode)
        self.color_mode_combo.selected_index = self._color_mode_index(self.color_mode)
        self._refresh_snapshot_combo()
        self._updating_controls = False

        self.window.title = "ROSM LiDAR - 출력 없음"
        self._refresh_labels()
        self.window.post_redraw()

    def _load_scene(self, index: int, reset_camera: bool) -> None:
        self._point_cloud_load_generation += 1
        self.full_cloud_enabled = False
        self.full_cloud_loading = False
        self.current_scene_index = max(0, min(index, len(self.scenes) - 1))
        scene = self.scenes[self.current_scene_index]
        if is_empty_scene(scene):
            self._show_empty_outputs()
            return
        try:
            flight_id = str(scene.get("flight_id", ""))
            pcd, preview_point_count, _ = load_point_cloud(self.lidar_dir, scene, 0, self.voxel_size)
            self.base_pcd = pcd
            self.preview_point_count = preview_point_count
            self.trajectory = load_trajectory_geometry(self.lidar_dir, flight_id, self.topic)
            self.snapshot_markers = load_snapshot_markers(
                self.project_root,
                self.snapshot_dir,
                flight_id,
                self.axis_length,
                self.frustum_length,
                self.frustum_fov_deg,
                self.frustum_length_mode,
                self.base_pcd,
                self.lidar_frustum_max_distance,
                self.lidar_frustum_min_distance,
                self.camera_calibration,
            )
        except Exception as exc:  # noqa: BLE001
            self.window.show_message_box("로드 오류", str(exc))
            return

        display_limit = self._selected_display_point_limit()
        display_pcd = self._display_point_cloud(display_limit)
        self.current_point_count = len(display_pcd.points)

        self._updating_controls = True
        self.flight_combo.selected_index = self.current_scene_index
        self.point_count_slider.set_limits(1, max(1, len(self.base_pcd.points)))
        self.point_count_slider.int_value = display_limit
        self.trajectory_checkbox.enabled = self.trajectory is not None
        self.trajectory_checkbox.checked = self.show_trajectory and self.trajectory is not None
        self.snapshot_locations_checkbox.enabled = bool(self.snapshot_markers)
        self.snapshot_locations_checkbox.checked = self.show_snapshot_locations and bool(self.snapshot_markers)
        self.full_cloud_checkbox.enabled = self._current_las_path() is not None
        self.full_cloud_checkbox.checked = False
        self.undistort_image_checkbox.enabled = self._can_undistort_images()
        self.undistort_image_checkbox.checked = self.use_undistorted_image and self._can_undistort_images()
        self.frustum_mode_combo.selected_index = self._frustum_mode_index(self.frustum_length_mode)
        self.color_mode_combo.selected_index = self._color_mode_index(self.color_mode)
        self._refresh_snapshot_combo()
        self._updating_controls = False

        self.window.title = f"ROSM LiDAR - {scene_display_name(scene)}"
        self._add_scene_geometries(display_pcd)
        self._refresh_labels()
        if self.selected_snapshot_index >= 0 and self.selected_snapshot_index < len(self.snapshot_markers):
            self._show_selection_marker(self.snapshot_markers[self.selected_snapshot_index].origin)
            self._refresh_image_plane()
        if reset_camera:
            self._fit_camera()
        elif self.snapshot_view_active:
            self._view_from_selected_snapshot()
        self.window.post_redraw()

    def _reload_manifest(self, lidar_dir: Path) -> None:
        scenes = load_scenes_or_empty(lidar_dir)

        self.lidar_dir = lidar_dir
        self.project_root = project_root_from_manifest(lidar_dir, self.project_root)
        self.extract_input_path = self.project_root
        self.snapshot_dir = sibling_snapshot_dir(lidar_dir, self.snapshot_dir)
        self.calibration_dir = sibling_calibration_dir(lidar_dir, self.calibration_dir)
        self.camera_calibration = load_xacti_calibration(self.calibration_dir)
        if not self._can_undistort_images():
            self.use_undistorted_image = False
        self.scenes = scenes
        self.current_scene_index = 0
        self.las_point_counts = las_point_count_by_scene(lidar_dir)

        self._updating_controls = True
        self.flight_combo.clear_items()
        for scene in self.scenes:
            self.flight_combo.add_item(scene_display_name(scene))
        self.flight_combo.selected_index = 0
        self._updating_controls = False
        self._load_scene(0, reset_camera=True)

    def _current_extract_project_root(self) -> Path:
        path = self.extract_input_path or self.project_root
        return infer_project_root_from_input(path)

    def _current_extract_output_root(self) -> Path:
        if self.extract_custom_output_root is not None:
            return self.extract_custom_output_root.resolve()
        return default_output_root(self._current_extract_project_root()).resolve()

    def _describe_extract_input(self) -> str:
        path = self.extract_input_path or self.project_root
        project_root = self._current_extract_project_root()
        if path.is_file():
            return f"선택 MCAP 파일\n{path}\n\n추출 기준 프로젝트\n{project_root}"
        return f"선택 프로젝트 폴더\n{project_root}"

    def _describe_extract_output(self) -> str:
        mode = "지정 출력 폴더" if self.extract_custom_output_root is not None else "자동 출력 폴더"
        return f"{mode}\n{self._current_extract_output_root()}"

    def _refresh_extract_dialog_labels(self) -> None:
        if self.extract_input_label is not None:
            self.extract_input_label.text = self._describe_extract_input()
        if self.extract_output_label is not None:
            self.extract_output_label.text = self._describe_extract_output()

    def _set_extract_status(self, text: str) -> None:
        if self.extract_status_label is not None:
            self.extract_status_label.text = text

    def _post_extract_status(self, text: str) -> None:
        gui.Application.instance.post_to_main_thread(self.window, lambda: self._set_extract_status(text))

    def _open_mcap_extract_dialog(self) -> None:
        em = self.window.theme.font_size
        dialog = gui.Dialog("MCAP 정보 추출")
        layout = gui.Vert(0.6 * em, gui.Margins(em, em, em, em))

        self.extract_input_label = gui.Label(self._describe_extract_input())
        self.extract_output_label = gui.Label(self._describe_extract_output())
        self.extract_status_label = gui.Label("대기 중")

        select_row = gui.Horiz(0.5 * em)
        project_button = gui.Button("프로젝트 폴더 선택")
        project_button.set_on_clicked(self._open_extract_project_folder_dialog)
        mcap_button = gui.Button("MCAP 파일 선택")
        mcap_button.set_on_clicked(self._open_extract_mcap_file_dialog)
        select_row.add_child(project_button)
        select_row.add_child(mcap_button)

        output_row = gui.Horiz(0.5 * em)
        output_button = gui.Button("출력 폴더 지정")
        output_button.set_on_clicked(self._open_extract_output_folder_dialog)
        auto_output_button = gui.Button("자동 출력")
        auto_output_button.set_on_clicked(self._use_auto_extract_output)
        output_row.add_child(output_button)
        output_row.add_child(auto_output_button)

        action_row = gui.Horiz(0.5 * em)
        self.extract_run_button = gui.Button("추출 실행")
        self.extract_run_button.enabled = not self.extract_running
        self.extract_run_button.set_on_clicked(self._run_mcap_extraction)
        close_button = gui.Button("닫기")
        close_button.set_on_clicked(self.window.close_dialog)
        action_row.add_stretch()
        action_row.add_child(self.extract_run_button)
        action_row.add_child(close_button)

        layout.add_child(self.extract_input_label)
        layout.add_child(select_row)
        layout.add_fixed(8)
        layout.add_child(self.extract_output_label)
        layout.add_child(output_row)
        layout.add_fixed(8)
        layout.add_child(self.extract_status_label)
        layout.add_fixed(8)
        layout.add_child(action_row)
        dialog.add_child(layout)
        self.window.show_dialog(dialog)

    def _open_extract_project_folder_dialog(self) -> None:
        self.window.close_dialog()
        dialog = gui.FileDialog(gui.FileDialog.OPEN_DIR, "Inspector 프로젝트 폴더 선택", self.window.theme)
        dialog.set_path(str(nearest_existing_dir(self._current_extract_project_root())))
        dialog.set_on_cancel(lambda: (self.window.close_dialog(), self._open_mcap_extract_dialog()))

        def on_done(path: str) -> None:
            self.window.close_dialog()
            self.extract_input_path = Path(path).resolve()
            self.extract_custom_output_root = None
            self._open_mcap_extract_dialog()

        dialog.set_on_done(on_done)
        self.window.show_dialog(dialog)

    def _open_extract_mcap_file_dialog(self) -> None:
        self.window.close_dialog()
        dialog = gui.FileDialog(gui.FileDialog.OPEN, "MCAP 파일 선택", self.window.theme)
        dialog.add_filter(".mcap", "MCAP files (.mcap)")
        dialog.add_filter("", "All files")
        dialog.set_path(str(nearest_existing_dir(self._current_extract_project_root())))
        dialog.set_on_cancel(lambda: (self.window.close_dialog(), self._open_mcap_extract_dialog()))

        def on_done(path: str) -> None:
            self.window.close_dialog()
            selected = Path(path).resolve()
            if selected.suffix.lower() != ".mcap":
                self.window.show_message_box("MCAP 파일 오류", "확장자가 .mcap인 파일을 선택해야 합니다.")
                self._open_mcap_extract_dialog()
                return
            self.extract_input_path = selected
            self.extract_custom_output_root = None
            self._open_mcap_extract_dialog()

        dialog.set_on_done(on_done)
        self.window.show_dialog(dialog)

    def _open_extract_output_folder_dialog(self) -> None:
        self.window.close_dialog()
        dialog = gui.FileDialog(gui.FileDialog.OPEN_DIR, "출력 폴더 선택", self.window.theme)
        dialog.set_path(str(nearest_existing_dir(self._current_extract_output_root())))
        dialog.set_on_cancel(lambda: (self.window.close_dialog(), self._open_mcap_extract_dialog()))

        def on_done(path: str) -> None:
            self.window.close_dialog()
            self.extract_custom_output_root = Path(path).resolve()
            self._open_mcap_extract_dialog()

        dialog.set_on_done(on_done)
        self.window.show_dialog(dialog)

    def _use_auto_extract_output(self) -> None:
        self.extract_custom_output_root = None
        self._refresh_extract_dialog_labels()
        self._set_extract_status("자동 출력 폴더를 사용합니다.")

    def _mcap_extract_commands(self, project_root: Path, output_root: Path) -> list[tuple[str, list[str]]]:
        scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
        python = sys.executable
        return [
            (
                "프로젝트 파일 색인",
                [python, "-B", str(scripts_dir / "01_scan_project.py"), str(project_root), "--output-dir", str(output_root / "index")],
            ),
            (
                "MCAP 목록 추출",
                [
                    python,
                    "-B",
                    str(scripts_dir / "02_list_mcap.py"),
                    str(project_root),
                    "--index-dir",
                    str(output_root / "index"),
                    "--output-dir",
                    str(output_root / "mcap"),
                ],
            ),
            (
                "MCAP 샘플 추출",
                [
                    python,
                    "-B",
                    str(scripts_dir / "03_sample_mcap_topics.py"),
                    str(project_root),
                    "--mcap-dir",
                    str(output_root / "mcap"),
                    "--output-dir",
                    str(output_root / "mcap_samples"),
                ],
            ),
            (
                "스냅샷 포즈 색인",
                [
                    python,
                    "-B",
                    str(scripts_dir / "04_index_snapshots.py"),
                    str(project_root),
                    "--index-dir",
                    str(output_root / "index"),
                    "--mcap-dir",
                    str(output_root / "mcap"),
                    "--output-dir",
                    str(output_root / "snapshots"),
                ],
            ),
            (
                "캘리브레이션 색인",
                [
                    python,
                    "-B",
                    str(scripts_dir / "05_index_calibrations.py"),
                    str(project_root),
                    "--index-dir",
                    str(output_root / "index"),
                    "--output-dir",
                    str(output_root / "calibration"),
                ],
            ),
            (
                "LiDAR 장면 생성",
                [
                    python,
                    "-B",
                    str(scripts_dir / "06_index_lidar_trajectory.py"),
                    str(project_root),
                    "--index-dir",
                    str(output_root / "index"),
                    "--mcap-dir",
                    str(output_root / "mcap"),
                    "--snapshot-dir",
                    str(output_root / "snapshots"),
                    "--output-dir",
                    str(output_root / "lidar"),
                ],
            ),
            (
                "Raw LiDAR 메시지 색인",
                [
                    python,
                    "-B",
                    str(scripts_dir / "07_index_raw_lidar.py"),
                    str(project_root),
                    "--mcap-dir",
                    str(output_root / "mcap"),
                    "--output-dir",
                    str(output_root / "raw_lidar"),
                ],
            ),
        ]

    def _run_mcap_extraction(self) -> None:
        if self.extract_running:
            return
        input_path = self.extract_input_path or self.project_root
        if not input_path.exists():
            self.window.show_message_box("입력 경로 오류", f"선택한 경로가 없습니다.\n{input_path}")
            return
        project_root = self._current_extract_project_root()
        output_root = self._current_extract_output_root()
        if not project_root.exists():
            self.window.show_message_box("프로젝트 폴더 오류", f"프로젝트 폴더를 찾을 수 없습니다.\n{project_root}")
            return

        selected_mcap = input_path.resolve() if input_path.is_file() and input_path.suffix.lower() == ".mcap" else None
        commands = self._mcap_extract_commands(project_root, output_root)
        missing_scripts = [cmd[1][2] for cmd in commands if not Path(cmd[1][2]).exists()]
        if missing_scripts:
            self.window.show_message_box("스크립트 없음", "\n".join(missing_scripts))
            return

        self.extract_running = True
        if self.extract_run_button is not None:
            self.extract_run_button.enabled = False
        self._set_extract_status(f"추출 시작\n{output_root}")

        def worker() -> None:
            try:
                output_root.mkdir(parents=True, exist_ok=True)
                creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0) if os.name == "nt" else 0
                for index, (label, cmd) in enumerate(commands, start=1):
                    self._post_extract_status(f"{index}/{len(commands)} {label} 실행 중")
                    result = subprocess.run(
                        cmd,
                        cwd=str(Path(__file__).resolve().parents[1]),
                        capture_output=True,
                        text=True,
                        encoding="utf-8",
                        errors="replace",
                        creationflags=creationflags,
                        check=False,
                    )
                    if result.returncode != 0:
                        output = short_process_output(result.stdout, result.stderr)
                        raise RuntimeError(f"{label} 실패\n\n{output}")
                    if index == 1 and selected_mcap is not None:
                        self._post_extract_status("선택 MCAP 파일만 필터링 중")
                        kept_count = filter_mcap_index_to_file(output_root / "index", project_root, selected_mcap)
                        self._post_extract_status(f"선택 MCAP {kept_count}개로 필터링 완료")
                gui.Application.instance.post_to_main_thread(self.window, lambda: self._finish_mcap_extraction(output_root))
            except Exception as exc:  # noqa: BLE001
                gui.Application.instance.post_to_main_thread(self.window, lambda exc=exc: self._fail_mcap_extraction(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _finish_mcap_extraction(self, output_root: Path) -> None:
        self.extract_running = False
        if self.extract_run_button is not None:
            self.extract_run_button.enabled = True
        self._set_extract_status(f"추출 완료\n{output_root}")
        self._refresh_extract_dialog_labels()
        self._reload_manifest(output_root / "lidar")

    def _fail_mcap_extraction(self, exc: Exception) -> None:
        self.extract_running = False
        if self.extract_run_button is not None:
            self.extract_run_button.enabled = True
        self._set_extract_status("추출 실패")
        self.window.show_message_box("MCAP 정보 추출 실패", str(exc))

    def _open_output_folder_dialog(self) -> None:
        dialog = gui.FileDialog(gui.FileDialog.OPEN_DIR, "outputs 또는 outputs\\lidar 폴더 선택", self.window.theme)
        dialog.set_path(str(self.lidar_dir.parent if self.lidar_dir.name.lower() == "lidar" else self.lidar_dir))
        dialog.set_on_cancel(self.window.close_dialog)

        def on_done(path: str) -> None:
            self.window.close_dialog()
            try:
                self._reload_manifest(resolve_lidar_dir(Path(path)))
            except Exception as exc:  # noqa: BLE001
                self.window.show_message_box("출력 폴더 오류", str(exc))

        dialog.set_on_done(on_done)
        self.window.show_dialog(dialog)

    def _on_flight_selected(self, _: str, index: int) -> None:
        if self._updating_controls:
            return
        self._load_scene(index, reset_camera=True)

    def _on_snapshot_selected(self, _: str, index: int) -> None:
        if self._updating_controls or not self.snapshot_markers:
            return
        self._select_snapshot(index, open_image=False)

    def _on_frustum_mode_selected(self, _: str, index: int) -> None:
        if self._updating_controls:
            return
        self.frustum_length_mode = self._frustum_mode_value(index)
        self._load_scene(self.current_scene_index, reset_camera=False)

    def _on_color_mode_selected(self, _: str, index: int) -> None:
        if self._updating_controls:
            return
        self.color_mode = self._color_mode_value(index)
        self._replace_point_cloud(self._selected_display_point_limit())

    def _on_image_plane_checked(self, checked: bool) -> None:
        if self._updating_controls:
            return
        self.show_image_plane = bool(checked)
        self._refresh_image_plane()

    def _on_snapshot_locations_checked(self, checked: bool) -> None:
        if self._updating_controls:
            return
        self.show_snapshot_locations = bool(checked)
        self._refresh_snapshot_location_visibility()

    def _on_undistort_image_checked(self, checked: bool) -> None:
        if self._updating_controls:
            return
        if checked and not self._can_undistort_images():
            self.use_undistorted_image = False
            self.undistort_image_checkbox.checked = False
            self.window.show_message_box("이미지 보정 불가", "Xacti 보정 JSON 또는 OpenCV를 찾을 수 없습니다.")
            return
        self.use_undistorted_image = bool(checked)
        self._refresh_image_plane()

    def _event_xy_in_scene(self, event: gui.MouseEvent) -> tuple[float, float] | None:
        x = float(event.x - self.widget3d.frame.x)
        y = float(event.y - self.widget3d.frame.y)
        if x < 0 or y < 0 or x >= self.widget3d.frame.width or y >= self.widget3d.frame.height:
            return None
        return x, y

    def _world_to_screen(self, world: np.ndarray) -> np.ndarray | None:
        width = float(self.widget3d.frame.width)
        height = float(self.widget3d.frame.height)
        if width <= 1 or height <= 1:
            return None
        point = np.array([world[0], world[1], world[2], 1.0], dtype=float)
        view = np.asarray(self.widget3d.scene.camera.get_view_matrix(), dtype=float).reshape((4, 4))
        projection = np.asarray(self.widget3d.scene.camera.get_projection_matrix(), dtype=float).reshape((4, 4))
        candidates = []
        for view_matrix, projection_matrix in ((view, projection), (view.T, projection.T)):
            clip = projection_matrix @ view_matrix @ point
            if abs(float(clip[3])) <= 1e-9:
                continue
            ndc = clip[:3] / clip[3]
            if not np.all(np.isfinite(ndc)):
                continue
            if -1.2 <= ndc[2] <= 1.2:
                screen = np.array([(ndc[0] + 1.0) * 0.5 * width, (1.0 - ndc[1]) * 0.5 * height], dtype=float)
                candidates.append(screen)
        if candidates:
            return candidates[0]
        return None

    def _marker_screen_distance(self, marker: SnapshotMarker, cursor: np.ndarray) -> float:
        origin_screen = self._world_to_screen(marker.origin)
        if origin_screen is None:
            return math.inf
        return float(np.linalg.norm(cursor - origin_screen))

    def _nearest_snapshot_at_screen(self, x: float, y: float) -> tuple[int, float]:
        if not self.show_snapshot_locations or not self.snapshot_markers:
            return -1, math.inf
        cursor = np.array([x, y], dtype=float)
        distances = [self._marker_screen_distance(marker, cursor) for marker in self.snapshot_markers]
        index = int(np.argmin(distances))
        return index, float(distances[index])

    def _on_mouse_widget3d(self, event: gui.MouseEvent) -> gui.Widget.EventCallbackResult:
        if event.type in (gui.MouseEvent.Type.DRAG, gui.MouseEvent.Type.WHEEL) and self.snapshot_view_active:
            self.snapshot_view_active = False
            if self.show_snapshot_locations and 0 <= self.selected_snapshot_index < len(self.snapshot_markers):
                self._show_selection_marker(self.snapshot_markers[self.selected_snapshot_index].origin)
        xy = self._event_xy_in_scene(event)
        if xy is None:
            self._clear_hover_marker()
            return gui.Widget.EventCallbackResult.IGNORED

        if event.type == gui.MouseEvent.Type.MOVE:
            index, distance = self._nearest_snapshot_at_screen(*xy)
            if index >= 0 and distance <= self.snapshot_hover_radius_px:
                self._show_hover_marker(index)
            else:
                self._clear_hover_marker()
            return gui.Widget.EventCallbackResult.IGNORED

        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_button_down(gui.MouseButton.LEFT):
            index, distance = self._nearest_snapshot_at_screen(*xy)
            if index >= 0 and distance <= self.snapshot_hover_radius_px:
                self._show_hover_marker(index)
                self._select_snapshot(index, open_image=True)
                return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED

    def _replace_point_cloud(self, max_points: int) -> None:
        self.target_display_points = max(1, int(max_points))
        if len(self.base_pcd.points) == 0:
            self.current_point_count = 0
            self._refresh_labels()
            self.window.post_redraw()
            return
        pcd = self._display_point_cloud(max_points)
        self.current_point_count = len(pcd.points)
        if self.widget3d.scene.has_geometry(self.POINT_GEOMETRY):
            self.widget3d.scene.remove_geometry(self.POINT_GEOMETRY)
        self.widget3d.scene.add_geometry(
            self.POINT_GEOMETRY,
            pcd,
            material("defaultUnlit", point_size=self.point_size),
        )
        self._refresh_labels()
        self.window.post_redraw()

    def _current_las_path(self) -> Path | None:
        if not self.scenes or self.current_scene_index >= len(self.scenes):
            return None
        path = resolve_project_path(self.project_root, self.scenes[self.current_scene_index].get("las_file"))
        return path if path is not None and path.exists() else None

    def _on_full_cloud_checked(self, checked: bool) -> None:
        if self._updating_controls:
            return
        if not checked:
            self._load_scene(self.current_scene_index, reset_camera=False)
            return

        las_path = self._current_las_path()
        if las_path is None:
            self._updating_controls = True
            self.full_cloud_checkbox.checked = False
            self._updating_controls = False
            self.window.show_message_box("LAS 파일 없음", "현재 비행의 원본 LAS 파일을 찾을 수 없습니다.")
            return

        self._point_cloud_load_generation += 1
        generation = self._point_cloud_load_generation
        scene_index = self.current_scene_index
        display_limit = max(1, self.current_point_count)
        self.full_cloud_loading = True
        self.full_cloud_checkbox.enabled = False
        self.point_count_slider.enabled = False
        self._refresh_labels()
        self.window.post_redraw()

        def cancelled() -> bool:
            return generation != self._point_cloud_load_generation

        def worker() -> None:
            try:
                pcd = load_full_las_point_cloud(las_path, cancelled)
                gui.Application.instance.post_to_main_thread(
                    self.window,
                    lambda: self._finish_full_cloud_load(generation, scene_index, pcd, display_limit),
                )
            except InterruptedError:
                return
            except Exception as exc:  # noqa: BLE001
                gui.Application.instance.post_to_main_thread(
                    self.window,
                    lambda exc=exc: self._fail_full_cloud_load(generation, exc),
                )

        threading.Thread(target=worker, daemon=True, name="full-las-loader").start()

    def _finish_full_cloud_load(
        self,
        generation: int,
        scene_index: int,
        pcd: o3d.geometry.PointCloud,
        display_limit: int,
    ) -> None:
        if generation != self._point_cloud_load_generation or scene_index != self.current_scene_index:
            return
        self.base_pcd = pcd
        self.full_cloud_enabled = True
        self.full_cloud_loading = False
        display_limit = min(display_limit, len(self.base_pcd.points))
        self.target_display_points = display_limit
        self._updating_controls = True
        self.full_cloud_checkbox.enabled = True
        self.full_cloud_checkbox.checked = True
        self.point_count_slider.enabled = True
        self.point_count_slider.set_limits(1, max(1, len(self.base_pcd.points)))
        self.point_count_slider.int_value = display_limit
        self._updating_controls = False
        self._replace_point_cloud(display_limit)

    def _fail_full_cloud_load(self, generation: int, exc: Exception) -> None:
        if generation != self._point_cloud_load_generation:
            return
        self.full_cloud_loading = False
        self._updating_controls = True
        self.full_cloud_checkbox.enabled = self._current_las_path() is not None
        self.full_cloud_checkbox.checked = False
        self.point_count_slider.enabled = True
        self._updating_controls = False
        self._refresh_labels()
        self.window.show_message_box("원본 LAS 로드 실패", str(exc))

    def _on_trajectory_checked(self, checked: bool) -> None:
        if self._updating_controls:
            return
        self.show_trajectory = checked
        if self.trajectory is not None:
            self.widget3d.scene.show_geometry(self.TRAJECTORY_GEOMETRY, checked)
            self.window.post_redraw()

    def _on_point_size_changed(self, value: float) -> None:
        self.point_size = max(1.0, float(value))
        if self.widget3d.scene.has_geometry(self.POINT_GEOMETRY):
            self.widget3d.scene.modify_geometry_material(
                self.POINT_GEOMETRY,
                material("defaultUnlit", point_size=self.point_size),
            )
        self._refresh_labels()
        self.window.post_redraw()

    def _on_point_count_changed(self, value: int) -> None:
        if self._updating_controls:
            return
        self._replace_point_cloud(max(1, int(value)))

    def _on_line_width_changed(self, value: float) -> None:
        self.line_width = max(1.0, float(value))
        line_mat = material("unlitLine", line_width=self.line_width)
        if self.trajectory is not None:
            self.widget3d.scene.modify_geometry_material(self.TRAJECTORY_GEOMETRY, line_mat)
        for name, geometry in self.snapshot_geometry_names:
            if isinstance(geometry, o3d.geometry.LineSet):
                self.widget3d.scene.modify_geometry_material(name, line_mat)
        if self.widget3d.scene.has_geometry("hover_frustum"):
            self.widget3d.scene.modify_geometry_material(
                "hover_frustum",
                material("unlitLine", line_width=max(self.line_width * 1.8, self.line_width + 2.0)),
            )
        self._refresh_labels()
        self.window.post_redraw()

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="View LAS, MCAP trajectory, and snapshot camera poses with Open3D.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--lidar-dir", type=Path, default=None)
    parser.add_argument("--snapshot-dir", type=Path, default=None)
    parser.add_argument("--calibration-dir", type=Path, default=None)
    parser.add_argument("--flight-identifier", default="")
    parser.add_argument("--topic", default="/kalman_scan2map_node/odometry")
    parser.add_argument("--axis-length", type=float, default=0.5)
    parser.add_argument("--frustum-length", type=float, default=1.2)
    parser.add_argument("--frustum-fov-deg", type=float, default=70.0)
    parser.add_argument("--frustum-length-mode", choices=["wall", "lidar", "fixed"], default="wall")
    parser.add_argument("--lidar-frustum-max-distance", type=float, default=8.0)
    parser.add_argument("--lidar-frustum-min-distance", type=float, default=0.05)
    parser.add_argument("--snapshot-hover-radius-px", type=float, default=28.0)
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--color-mode", choices=["gray", "height", "intensity", "inspector"], default="inspector")
    parser.add_argument("--trajectory-line-width", "--line-width", dest="line_width", type=float, default=3.0)
    parser.add_argument("--no-trajectory", "--hide-trajectory", action="store_true", dest="no_trajectory")
    parser.add_argument("--max-display-points", type=int, default=0)
    parser.add_argument("--voxel-size", type=float, default=0.0)
    parser.add_argument("--window-width", type=int, default=1280)
    parser.add_argument("--window-height", type=int, default=840)
    parser.add_argument("--font-path", type=Path, default=None)
    args = parser.parse_args(argv)

    project_root = args.project_root.resolve()
    lidar_dir = resolve_lidar_dir_or_default(args.lidar_dir.resolve() if args.lidar_dir else default_lidar_dir(project_root).resolve())
    project_root = project_root_from_manifest(lidar_dir, project_root)
    snapshot_dir = (
        args.snapshot_dir.resolve()
        if args.snapshot_dir
        else sibling_snapshot_dir(lidar_dir, default_snapshot_dir(project_root)).resolve()
    )
    calibration_dir = args.calibration_dir.resolve() if args.calibration_dir else sibling_calibration_dir(lidar_dir, default_calibration_dir(project_root)).resolve()
    scenes = load_scenes_or_empty(lidar_dir)
    if is_empty_scene(scenes[0]):
        scene = scenes[0]
        initial_scene_index = 0
    else:
        scene = pick_scene({"scenes": scenes}, args.flight_identifier)
        initial_scene_index = scene_index(scenes, scene)

    print(f"Flight: {scene.get('flight_identifier')}")
    print(f"Output lidar dir: {lidar_dir}")
    print(f"Output calibration dir: {calibration_dir}")
    print(f"Scenes: {len(scenes)}")
    print(f"Trajectory: {'off' if args.no_trajectory else 'on'}")
    print(f"Point size: {max(1.0, args.point_size):.1f}")
    print(f"Color mode: {args.color_mode}")
    print(f"Line width: {max(1.0, args.line_width):.1f}")
    print("Point cloud: gray, trajectory: orange, snapshot camera axes: RGB/Y")
    print("Use the right control panel to select flights and change visibility, point size, point count, and line width.")

    gui.Application.instance.initialize()
    font = configure_korean_font(args.font_path.resolve() if args.font_path else None)
    print(f"Korean GUI font: {font if font else 'not found'}")
    LidarSceneApp(
        project_root=project_root,
        lidar_dir=lidar_dir,
        snapshot_dir=snapshot_dir,
        calibration_dir=calibration_dir,
        scenes=scenes,
        initial_scene_index=initial_scene_index,
        topic=args.topic,
        axis_length=args.axis_length,
        frustum_length=args.frustum_length,
        frustum_fov_deg=args.frustum_fov_deg,
        frustum_length_mode=args.frustum_length_mode,
        lidar_frustum_max_distance=args.lidar_frustum_max_distance,
        lidar_frustum_min_distance=args.lidar_frustum_min_distance,
        snapshot_hover_radius_px=args.snapshot_hover_radius_px,
        voxel_size=max(0.0, args.voxel_size),
        initial_max_display_points=max(0, args.max_display_points),
        point_size=args.point_size,
        color_mode=args.color_mode,
        line_width=args.line_width,
        show_trajectory=not args.no_trajectory,
        window_width=max(640, args.window_width),
        window_height=max(480, args.window_height),
    )
    gui.Application.instance.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
