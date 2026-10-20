from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import laspy
import numpy as np
import open3d as o3d
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


DEFAULT_ODOMETRY_TOPICS = ("/kalman_scan2map_node/odometry",)


def read_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        return list(csv.DictReader(fp))


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_value(row.get(key)) for key in fieldnames})


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list) and all(isinstance(item, (str, int, float, bool)) for item in value):
        return ";".join(str(item) for item in value)
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def rel_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2] / "sepoong"


def default_index_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "index"


def default_mcap_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "mcap"


def default_snapshot_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "snapshots"


def default_output_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "lidar"


def parse_json_list(value: str) -> list[str]:
    if not value:
        return []
    try:
        parsed = json.loads(value)
        if isinstance(parsed, list):
            return [str(item) for item in parsed if str(item)]
        if isinstance(parsed, str):
            return [parsed]
    except Exception:  # noqa: BLE001
        pass
    return [item.strip() for item in value.split(";") if item.strip()]


def to_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        return None


def to_int(value: Any) -> int | None:
    if value in (None, ""):
        return None
    try:
        return int(float(value))
    except Exception:  # noqa: BLE001
        return None


def ns_to_sec(value: Any) -> float | None:
    raw = to_int(value)
    if raw is None:
        return None
    return raw / 1_000_000_000


def attr(obj: Any, name: str, default: Any = None) -> Any:
    if obj is None:
        return default
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def stamp_to_ns(stamp: Any) -> int | None:
    sec = to_int(attr(stamp, "sec"))
    nanosec = to_int(attr(stamp, "nanosec"))
    if sec is None or nanosec is None:
        return None
    return sec * 1_000_000_000 + nanosec


def normalize_quat(q: tuple[float | None, float | None, float | None, float | None]) -> tuple[float, float, float, float] | None:
    if any(value is None for value in q):
        return None
    x, y, z, w = (float(value) for value in q if value is not None)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        return None
    return x / norm, y / norm, z / norm, w / norm


def quat_angle_error_deg(
    a: tuple[float | None, float | None, float | None, float | None],
    b: tuple[float | None, float | None, float | None, float | None],
) -> float | None:
    qa = normalize_quat(a)
    qb = normalize_quat(b)
    if qa is None or qb is None:
        return None
    dot = abs(sum(av * bv for av, bv in zip(qa, qb)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2 * math.acos(dot))


def position_distance(a: tuple[float | None, float | None, float | None], b: tuple[float | None, float | None, float | None]) -> float | None:
    if any(value is None for value in (*a, *b)):
        return None
    return math.sqrt(sum((float(av) - float(bv)) ** 2 for av, bv in zip(a, b)))


def load_flights(index_dir: Path) -> dict[str, dict[str, str]]:
    return {row.get("flight_id", ""): row for row in read_csv_dicts(index_dir / "flights.csv") if row.get("flight_id")}


def resolve_map_file(project_root: Path, map_row: dict[str, str], file_name: str) -> Path:
    folder = project_root / map_row.get("folder", "")
    return folder / file_name


def load_las_files(project_root: Path, index_dir: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for map_row in read_csv_dicts(index_dir / "maps.csv"):
        for file_name in parse_json_list(map_row.get("point_cloud_files", "")):
            las_path = resolve_map_file(project_root, map_row, file_name)
            flight = flights.get(map_row.get("parent_id", ""), {})
            base = {
                "map_id": map_row.get("map_id"),
                "map_parent_id": map_row.get("parent_id"),
                "map_parent_type": map_row.get("parent_type"),
                "map_source": map_row.get("source"),
                "flight_id": map_row.get("parent_id") if map_row.get("parent_type") == "Flight" else "",
                "flight_identifier": flight.get("identifier", ""),
                "flight_name": flight.get("name", ""),
                "las_file": rel_path(las_path, project_root),
                "file_name": las_path.name,
                "exists": las_path.exists(),
                "size_bytes": las_path.stat().st_size if las_path.exists() else "",
            }
            if not las_path.exists():
                errors.append({"path": str(las_path), "error_type": "FileNotFoundError", "message": "LAS file not found"})
                rows.append(base)
                continue
            try:
                with laspy.open(las_path) as las:
                    header = las.header
                    base.update(
                        {
                            "version": str(header.version),
                            "point_format": str(header.point_format.id),
                            "point_count": int(header.point_count),
                            "min_x": float(header.mins[0]),
                            "min_y": float(header.mins[1]),
                            "min_z": float(header.mins[2]),
                            "max_x": float(header.maxs[0]),
                            "max_y": float(header.maxs[1]),
                            "max_z": float(header.maxs[2]),
                            "span_x": float(header.maxs[0] - header.mins[0]),
                            "span_y": float(header.maxs[1] - header.mins[1]),
                            "span_z": float(header.maxs[2] - header.mins[2]),
                            "scale_x": float(header.scales[0]),
                            "scale_y": float(header.scales[1]),
                            "scale_z": float(header.scales[2]),
                            "offset_x": float(header.offsets[0]),
                            "offset_y": float(header.offsets[1]),
                            "offset_z": float(header.offsets[2]),
                            "dimensions": list(header.point_format.dimension_names),
                        }
                    )
            except Exception as exc:  # noqa: BLE001
                errors.append({"path": str(las_path), "error_type": type(exc).__name__, "message": str(exc)})
            rows.append(base)
    return rows


def points_from_las_sample(las_path: Path, max_points: int) -> tuple[np.ndarray, np.ndarray | None, int]:
    with laspy.open(las_path) as las:
        point_count = int(las.header.point_count)
        if point_count == 0:
            return np.empty((0, 3), dtype=np.float64), None, 1
        stride = max(1, math.ceil(point_count / max_points))
        points: list[np.ndarray] = []
        intensities: list[np.ndarray] = []
        seen = 0
        for chunk in las.chunk_iterator(500_000):
            size = len(chunk.x)
            mask = ((np.arange(size) + seen) % stride) == 0
            if np.any(mask):
                xyz = np.column_stack((np.asarray(chunk.x)[mask], np.asarray(chunk.y)[mask], np.asarray(chunk.z)[mask]))
                points.append(xyz)
                try:
                    intensities.append(np.asarray(chunk.intensity)[mask].astype(np.float64))
                except Exception:  # noqa: BLE001
                    pass
            seen += size
            if sum(len(item) for item in points) >= max_points:
                break
        if not points:
            return np.empty((0, 3), dtype=np.float64), None, stride
        xyz = np.vstack(points)[:max_points]
        intensity = np.concatenate(intensities)[: len(xyz)] if intensities else None
        return xyz, intensity, stride


def create_preview_ply(project_root: Path, output_dir: Path, las_rows: list[dict[str, Any]], max_points: int, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    preview_dir = output_dir / "preview_ply"
    preview_dir.mkdir(parents=True, exist_ok=True)
    for row in las_rows:
        if not row.get("exists"):
            continue
        las_path = project_root / str(row.get("las_file", ""))
        try:
            points, intensity, stride = points_from_las_sample(las_path, max_points=max_points)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            if intensity is not None and len(intensity) == len(points):
                max_intensity = float(np.max(intensity)) if len(intensity) else 0.0
                values = intensity / max_intensity if max_intensity > 0 else np.zeros_like(intensity)
                colors = np.column_stack((values, values, values))
                pcd.colors = o3d.utility.Vector3dVector(colors)
            ply_name = Path(str(row.get("las_file", ""))).with_suffix(".preview.ply").name
            if row.get("flight_identifier"):
                ply_name = f"{row['flight_identifier']}.preview.ply"
            ply_path = preview_dir / ply_name
            o3d.io.write_point_cloud(str(ply_path), pcd, write_ascii=False, compressed=False)
            rows.append(
                {
                    "las_file": row.get("las_file"),
                    "preview_ply": rel_path(ply_path, output_dir),
                    "flight_id": row.get("flight_id"),
                    "flight_identifier": row.get("flight_identifier"),
                    "source_points": row.get("point_count"),
                    "preview_points": len(points),
                    "sample_stride": stride,
                    "has_intensity_color": intensity is not None,
                    "size_bytes": ply_path.stat().st_size if ply_path.exists() else "",
                }
            )
        except Exception as exc:  # noqa: BLE001
            errors.append({"path": str(las_path), "error_type": type(exc).__name__, "message": str(exc)})
    return rows


def livetraj_summaries(project_root: Path, index_dir: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for map_row in read_csv_dicts(index_dir / "maps.csv"):
        for file_name in parse_json_list(map_row.get("trajectory_files", "")):
            path = resolve_map_file(project_root, map_row, file_name)
            flight = flights.get(map_row.get("parent_id", ""), {})
            base: dict[str, Any] = {
                "map_id": map_row.get("map_id"),
                "map_parent_id": map_row.get("parent_id"),
                "map_parent_type": map_row.get("parent_type"),
                "flight_id": map_row.get("parent_id") if map_row.get("parent_type") == "Flight" else "",
                "flight_identifier": flight.get("identifier", ""),
                "trajectory_file": rel_path(path, project_root),
                "exists": path.exists(),
                "row_count": 0,
            }
            if not path.exists():
                errors.append({"path": str(path), "error_type": "FileNotFoundError", "message": "Trajectory file not found"})
                rows.append(base)
                continue
            try:
                values: list[dict[str, float]] = []
                with path.open("r", encoding="utf-8-sig") as fp:
                    header = fp.readline().strip().split()
                    for line in fp:
                        parts = line.strip().split()
                        if len(parts) != len(header):
                            continue
                        item: dict[str, float] = {}
                        for key, part in zip(header, parts):
                            item[key] = float(part)
                        values.append(item)
                if values:
                    xs = [item["pos_x[m]"] for item in values]
                    ys = [item["pos_y[m]"] for item in values]
                    zs = [item["pos_z[m]"] for item in values]
                    ts = [item["timestamp[s]"] for item in values]
                    quality = Counter(str(int(item.get("quality", -1))) for item in values)
                    base.update(
                        {
                            "row_count": len(values),
                            "start_time_sec": min(ts),
                            "end_time_sec": max(ts),
                            "duration_sec": max(ts) - min(ts),
                            "min_x": min(xs),
                            "min_y": min(ys),
                            "min_z": min(zs),
                            "max_x": max(xs),
                            "max_y": max(ys),
                            "max_z": max(zs),
                            "quality_counts": dict(sorted(quality.items())),
                        }
                    )
            except Exception as exc:  # noqa: BLE001
                errors.append({"path": str(path), "error_type": type(exc).__name__, "message": str(exc)})
            rows.append(base)
    return rows


def odometry_topic_rows(mcap_dir: Path, topics: tuple[str, ...]) -> list[dict[str, Any]]:
    rows = []
    for row in read_csv_dicts(mcap_dir / "mcap_topic_candidates.csv"):
        if row.get("topic") in topics:
            rows.append(
                {
                    "mcap_file": row.get("mcap_file"),
                    "linked_flight_id": row.get("linked_flight_id"),
                    "linked_flight_identifier": row.get("linked_flight_identifier"),
                    "topic": row.get("topic"),
                    "schema_name": row.get("schema_name"),
                    "message_count": row.get("message_count"),
                }
            )
    return rows


def pose_from_odometry(decoded: Any) -> dict[str, Any]:
    header = attr(decoded, "header")
    stamp = attr(header, "stamp")
    pose = attr(decoded, "pose")
    position = attr(pose, "position")
    orientation = attr(pose, "orientation")
    twist = attr(decoded, "twist")
    linear = attr(twist, "linear")
    angular = attr(twist, "angular")
    return {
        "header_stamp_ns": stamp_to_ns(stamp),
        "header_stamp_sec": ns_to_sec(stamp_to_ns(stamp)),
        "frame_id": attr(header, "frame_id"),
        "child_frame_id": attr(decoded, "child_frame_id"),
        "position_x": attr(position, "x"),
        "position_y": attr(position, "y"),
        "position_z": attr(position, "z"),
        "orientation_qx": attr(orientation, "x"),
        "orientation_qy": attr(orientation, "y"),
        "orientation_qz": attr(orientation, "z"),
        "orientation_qw": attr(orientation, "w"),
        "linear_velocity_x": attr(linear, "x"),
        "linear_velocity_y": attr(linear, "y"),
        "linear_velocity_z": attr(linear, "z"),
        "angular_velocity_x": attr(angular, "x"),
        "angular_velocity_y": attr(angular, "y"),
        "angular_velocity_z": attr(angular, "z"),
        "confidence": attr(decoded, "confidence"),
    }


def extract_odometry(project_root: Path, mcap_dir: Path, topics: tuple[str, ...], errors: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    topic_rows = odometry_topic_rows(mcap_dir, topics)
    trajectories: list[dict[str, Any]] = []
    for topic_info in topic_rows:
        mcap_file = topic_info.get("mcap_file", "")
        topic = topic_info.get("topic", "")
        mcap_path = project_root / str(mcap_file)
        if not mcap_path.exists():
            errors.append({"path": str(mcap_path), "error_type": "FileNotFoundError", "message": "MCAP file not found"})
            continue
        try:
            with mcap_path.open("rb") as fp:
                reader = make_reader(fp, decoder_factories=[DecoderFactory()])
                for index, (_schema, _channel, message, decoded) in enumerate(reader.iter_decoded_messages(topics=[topic]), start=1):
                    row = {
                        "flight_id": topic_info.get("linked_flight_id"),
                        "flight_identifier": topic_info.get("linked_flight_identifier"),
                        "mcap_file": mcap_file,
                        "topic": topic,
                        "sample_index": index,
                        "sequence": attr(message, "sequence"),
                        "log_time_ns": attr(message, "log_time"),
                        "publish_time_ns": attr(message, "publish_time"),
                        "log_time_sec": ns_to_sec(attr(message, "log_time")),
                        "publish_time_sec": ns_to_sec(attr(message, "publish_time")),
                    }
                    row.update(pose_from_odometry(decoded))
                    trajectories.append(row)
        except Exception as exc:  # noqa: BLE001
            errors.append({"path": str(mcap_path), "error_type": type(exc).__name__, "message": f"{topic}: {exc}"})
    trajectories.sort(key=lambda row: (str(row.get("flight_identifier")), str(row.get("topic")), to_int(row.get("header_stamp_ns")) or -1))
    return topic_rows, trajectories


def snapshot_pose(row: dict[str, str], prefix: str) -> tuple[tuple[float | None, float | None, float | None], tuple[float | None, float | None, float | None, float | None]]:
    pos = to_float(row.get(f"{prefix}_x")), to_float(row.get(f"{prefix}_y")), to_float(row.get(f"{prefix}_z"))
    quat = to_float(row.get(f"{prefix}_qx")), to_float(row.get(f"{prefix}_qy")), to_float(row.get(f"{prefix}_qz")), to_float(row.get(f"{prefix}_qw"))
    return pos, quat


def trajectory_pose(row: dict[str, Any]) -> tuple[tuple[float | None, float | None, float | None], tuple[float | None, float | None, float | None, float | None]]:
    pos = to_float(row.get("position_x")), to_float(row.get("position_y")), to_float(row.get("position_z"))
    quat = to_float(row.get("orientation_qx")), to_float(row.get("orientation_qy")), to_float(row.get("orientation_qz")), to_float(row.get("orientation_qw"))
    return pos, quat


def load_snapshot_rows(snapshot_dir: Path) -> list[dict[str, str]]:
    rows = read_csv_dicts(snapshot_dir / "snapshot_json_poses.csv")
    match_rows = {row.get("snapshot_id"): row for row in read_csv_dicts(snapshot_dir / "snapshot_mcap_matches.csv")}
    for row in rows:
        match = match_rows.get(row.get("snapshot_id"), {})
        if match.get("mcap_stamp_ns"):
            row["match_time_ns"] = match["mcap_stamp_ns"]
        else:
            row["match_time_ns"] = row.get("time_ns_guess", "")
    return rows


def nearest_trajectory_rows(snapshot_dir: Path, trajectories: list[dict[str, Any]]) -> list[dict[str, Any]]:
    snapshots = load_snapshot_rows(snapshot_dir)
    by_flight_topic: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in trajectories:
        by_flight_topic[(str(row.get("flight_id")), str(row.get("topic")))].append(row)
    for rows in by_flight_topic.values():
        rows.sort(key=lambda row: to_int(row.get("header_stamp_ns")) or -1)

    output: list[dict[str, Any]] = []
    for snapshot in snapshots:
        flight_id = str(snapshot.get("flight_id") or "")
        target_ns = to_int(snapshot.get("match_time_ns"))
        if target_ns is None:
            continue
        for (candidate_flight_id, topic), rows in sorted(by_flight_topic.items()):
            if candidate_flight_id != flight_id or not rows:
                continue
            times = [to_int(row.get("header_stamp_ns")) or -1 for row in rows]
            index = bisect.bisect_left(times, target_ns)
            candidates = []
            if 0 <= index < len(rows):
                candidates.append(rows[index])
            if 0 <= index - 1 < len(rows):
                candidates.append(rows[index - 1])
            best = min(candidates, key=lambda row: abs((to_int(row.get("header_stamp_ns")) or 0) - target_ns))
            best_time = to_int(best.get("header_stamp_ns"))
            snapshot_drone_pos, snapshot_drone_quat = snapshot_pose(snapshot, "drone")
            traj_pos, traj_quat = trajectory_pose(best)
            output.append(
                {
                    "snapshot_id": snapshot.get("snapshot_id"),
                    "flight_id": flight_id,
                    "flight_identifier": snapshot.get("flight_identifier"),
                    "topic": topic,
                    "snapshot_time_ns": target_ns,
                    "nearest_odom_time_ns": best_time,
                    "time_delta_ns": (best_time - target_ns) if best_time is not None else "",
                    "time_delta_sec": ((best_time - target_ns) / 1_000_000_000) if best_time is not None else "",
                    "snapshot_drone_x": snapshot.get("drone_x"),
                    "snapshot_drone_y": snapshot.get("drone_y"),
                    "snapshot_drone_z": snapshot.get("drone_z"),
                    "odom_x": best.get("position_x"),
                    "odom_y": best.get("position_y"),
                    "odom_z": best.get("position_z"),
                    "position_error_m": position_distance(snapshot_drone_pos, traj_pos),
                    "orientation_error_deg": quat_angle_error_deg(snapshot_drone_quat, traj_quat),
                    "odom_frame_id": best.get("frame_id"),
                    "odom_child_frame_id": best.get("child_frame_id"),
                    "camera_image": snapshot.get("camera_image"),
                }
            )
    return output


def trajectory_stats(trajectories: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in trajectories:
        groups[(str(row.get("flight_identifier")), str(row.get("topic")))].append(row)
    rows: list[dict[str, Any]] = []
    for (flight_identifier, topic), items in sorted(groups.items()):
        xs = [to_float(row.get("position_x")) for row in items]
        ys = [to_float(row.get("position_y")) for row in items]
        zs = [to_float(row.get("position_z")) for row in items]
        ts = [to_int(row.get("header_stamp_ns")) for row in items]
        xs2 = [value for value in xs if value is not None]
        ys2 = [value for value in ys if value is not None]
        zs2 = [value for value in zs if value is not None]
        ts2 = [value for value in ts if value is not None]
        rows.append(
            {
                "flight_identifier": flight_identifier,
                "topic": topic,
                "row_count": len(items),
                "start_time_sec": ns_to_sec(min(ts2)) if ts2 else "",
                "end_time_sec": ns_to_sec(max(ts2)) if ts2 else "",
                "duration_sec": ((max(ts2) - min(ts2)) / 1_000_000_000) if ts2 else "",
                "min_x": min(xs2) if xs2 else "",
                "min_y": min(ys2) if ys2 else "",
                "min_z": min(zs2) if zs2 else "",
                "max_x": max(xs2) if xs2 else "",
                "max_y": max(ys2) if ys2 else "",
                "max_z": max(zs2) if zs2 else "",
                "frame_id": items[0].get("frame_id", "") if items else "",
                "child_frame_id": items[0].get("child_frame_id", "") if items else "",
            }
        )
    return rows


def build_scene_manifest(project_root: Path, output_dir: Path, las_rows: list[dict[str, Any]], preview_rows: list[dict[str, Any]], trajectory_rows: list[dict[str, Any]], snapshot_rows: list[dict[str, str]]) -> dict[str, Any]:
    previews_by_flight = {str(row.get("flight_id")): row for row in preview_rows if row.get("flight_id")}
    snapshots_by_flight = Counter(row.get("flight_id", "") for row in snapshot_rows)
    trajectories_by_flight = Counter(row.get("flight_id", "") for row in trajectory_rows)
    flights = sorted(set(previews_by_flight) | set(snapshots_by_flight) | set(trajectories_by_flight))
    scenes = []
    for flight_id in flights:
        las = next((row for row in las_rows if row.get("flight_id") == flight_id), {})
        preview = previews_by_flight.get(flight_id, {})
        flight_identifier = las.get("flight_identifier") or preview.get("flight_identifier") or next((row.get("flight_identifier") for row in trajectory_rows if row.get("flight_id") == flight_id), "")
        scenes.append(
            {
                "flight_id": flight_id,
                "flight_identifier": flight_identifier,
                "las_file": las.get("las_file", ""),
                "preview_ply": preview.get("preview_ply", ""),
                "trajectory_rows": trajectories_by_flight.get(flight_id, 0),
                "snapshot_count": snapshots_by_flight.get(flight_id, 0),
            }
        )
    return {
        "project_root": str(project_root),
        "lidar_output_dir": str(output_dir),
        "default_trajectory_file": "flight_trajectories.csv",
        "default_snapshot_file": "../snapshots/snapshot_json_poses.csv",
        "scenes": scenes,
    }


def build_lidar_outputs(
    project_root: Path,
    index_dir: Path,
    mcap_dir: Path,
    snapshot_dir: Path,
    output_dir: Path,
    odometry_topics: tuple[str, ...],
    max_preview_points: int,
) -> dict[str, Any]:
    project_root = project_root.resolve()
    index_dir = index_dir.resolve()
    mcap_dir = mcap_dir.resolve()
    snapshot_dir = snapshot_dir.resolve()
    output_dir = output_dir.resolve()
    errors: list[dict[str, Any]] = []
    flights = load_flights(index_dir)

    las_rows = load_las_files(project_root, index_dir, flights, errors)
    preview_rows = create_preview_ply(project_root, output_dir, las_rows, max_preview_points, errors)
    livetraj_rows = livetraj_summaries(project_root, index_dir, flights, errors)
    odom_topic_rows, trajectory_rows = extract_odometry(project_root, mcap_dir, odometry_topics, errors)
    nearest_rows = nearest_trajectory_rows(snapshot_dir, trajectory_rows)
    stats_rows = trajectory_stats(trajectory_rows)
    snapshot_rows = load_snapshot_rows(snapshot_dir)
    manifest = build_scene_manifest(project_root, output_dir, las_rows, preview_rows, trajectory_rows, snapshot_rows)

    write_csv(
        output_dir / "las_files.csv",
        las_rows,
        [
            "map_id",
            "map_parent_id",
            "map_parent_type",
            "map_source",
            "flight_id",
            "flight_identifier",
            "flight_name",
            "las_file",
            "file_name",
            "exists",
            "size_bytes",
            "version",
            "point_format",
            "point_count",
            "min_x",
            "min_y",
            "min_z",
            "max_x",
            "max_y",
            "max_z",
            "span_x",
            "span_y",
            "span_z",
            "scale_x",
            "scale_y",
            "scale_z",
            "offset_x",
            "offset_y",
            "offset_z",
            "dimensions",
        ],
    )
    write_csv(output_dir / "las_preview_files.csv", preview_rows, ["las_file", "preview_ply", "flight_id", "flight_identifier", "source_points", "preview_points", "sample_stride", "has_intensity_color", "size_bytes"])
    write_csv(
        output_dir / "processed_livetraj_summary.csv",
        livetraj_rows,
        [
            "map_id",
            "map_parent_id",
            "map_parent_type",
            "flight_id",
            "flight_identifier",
            "trajectory_file",
            "exists",
            "row_count",
            "start_time_sec",
            "end_time_sec",
            "duration_sec",
            "min_x",
            "min_y",
            "min_z",
            "max_x",
            "max_y",
            "max_z",
            "quality_counts",
        ],
    )
    write_csv(output_dir / "odometry_topics.csv", odom_topic_rows, ["mcap_file", "linked_flight_id", "linked_flight_identifier", "topic", "schema_name", "message_count"])
    write_csv(
        output_dir / "flight_trajectory_stats.csv",
        stats_rows,
        ["flight_identifier", "topic", "row_count", "start_time_sec", "end_time_sec", "duration_sec", "min_x", "min_y", "min_z", "max_x", "max_y", "max_z", "frame_id", "child_frame_id"],
    )
    write_csv(
        output_dir / "flight_trajectories.csv",
        trajectory_rows,
        [
            "flight_id",
            "flight_identifier",
            "mcap_file",
            "topic",
            "sample_index",
            "sequence",
            "log_time_ns",
            "publish_time_ns",
            "log_time_sec",
            "publish_time_sec",
            "header_stamp_ns",
            "header_stamp_sec",
            "frame_id",
            "child_frame_id",
            "position_x",
            "position_y",
            "position_z",
            "orientation_qx",
            "orientation_qy",
            "orientation_qz",
            "orientation_qw",
            "linear_velocity_x",
            "linear_velocity_y",
            "linear_velocity_z",
            "angular_velocity_x",
            "angular_velocity_y",
            "angular_velocity_z",
            "confidence",
        ],
    )
    write_csv(
        output_dir / "snapshot_nearest_odometry.csv",
        nearest_rows,
        [
            "snapshot_id",
            "flight_id",
            "flight_identifier",
            "topic",
            "snapshot_time_ns",
            "nearest_odom_time_ns",
            "time_delta_ns",
            "time_delta_sec",
            "snapshot_drone_x",
            "snapshot_drone_y",
            "snapshot_drone_z",
            "odom_x",
            "odom_y",
            "odom_z",
            "position_error_m",
            "orientation_error_deg",
            "odom_frame_id",
            "odom_child_frame_id",
            "camera_image",
        ],
    )
    write_json(output_dir / "scene_manifest.json", manifest)
    write_csv(output_dir / "lidar_errors.csv", errors, ["path", "error_type", "message"])

    position_errors = [float(row["position_error_m"]) for row in nearest_rows if row.get("position_error_m") not in ("", None)]
    time_deltas = [abs(int(row["time_delta_ns"])) for row in nearest_rows if row.get("time_delta_ns") not in ("", None)]
    summary = {
        "project_root": str(project_root),
        "index_dir": str(index_dir),
        "mcap_dir": str(mcap_dir),
        "snapshot_dir": str(snapshot_dir),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "odometry_topics": list(odometry_topics),
        "max_preview_points": max_preview_points,
        "counts": {
            "las_files": len(las_rows),
            "las_preview_files": len(preview_rows),
            "processed_livetraj_files": len(livetraj_rows),
            "odometry_topics": len(odom_topic_rows),
            "trajectory_rows": len(trajectory_rows),
            "snapshot_nearest_odometry_rows": len(nearest_rows),
            "errors": len(errors),
        },
        "las_files_by_parent_type": dict(sorted(Counter(row.get("map_parent_type", "") for row in las_rows).items())),
        "trajectory_rows_by_flight": dict(sorted(Counter(row.get("flight_identifier", "") for row in trajectory_rows).items())),
        "max_abs_snapshot_time_delta_ns": max(time_deltas) if time_deltas else "",
        "max_snapshot_drone_position_error_m": max(position_errors) if position_errors else "",
        "scenes": manifest["scenes"],
    }
    write_json(output_dir / "lidar_trajectory_summary.json", summary)
    return summary


def parse_topics(text: str) -> tuple[str, ...]:
    items = tuple(item.strip() for item in text.split(",") if item.strip())
    return items or DEFAULT_ODOMETRY_TOPICS


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Index LAS maps and extract MCAP odometry trajectories.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--index-dir", type=Path, default=None)
    parser.add_argument("--mcap-dir", type=Path, default=None)
    parser.add_argument("--snapshot-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--odometry-topics", default=",".join(DEFAULT_ODOMETRY_TOPICS))
    parser.add_argument("--max-preview-points", type=int, default=200_000)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if not project_root.exists():
        raise SystemExit(f"Project root does not exist: {project_root}")
    if args.max_preview_points < 100:
        raise SystemExit("--max-preview-points must be >= 100")
    index_dir = args.index_dir.resolve() if args.index_dir else default_index_dir(project_root).resolve()
    mcap_dir = args.mcap_dir.resolve() if args.mcap_dir else default_mcap_dir(project_root).resolve()
    snapshot_dir = args.snapshot_dir.resolve() if args.snapshot_dir else default_snapshot_dir(project_root).resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else default_output_dir(project_root).resolve()
    summary = build_lidar_outputs(
        project_root=project_root,
        index_dir=index_dir,
        mcap_dir=mcap_dir,
        snapshot_dir=snapshot_dir,
        output_dir=output_dir,
        odometry_topics=parse_topics(args.odometry_topics),
        max_preview_points=args.max_preview_points,
    )
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
