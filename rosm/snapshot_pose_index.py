from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

try:
    import cv2
    import numpy as np
except Exception:  # noqa: BLE001
    cv2 = None
    np = None


SNAPSHOT_TOPIC = "/point_of_interest/snapshot"


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


def load_json(path: Path, errors: list[dict[str, Any]]) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        errors.append({"path": str(path), "error_type": type(exc).__name__, "message": str(exc)})
        return None


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


def default_output_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "snapshots"


def parse_json_field(value: str) -> Any:
    if not value:
        return {}
    try:
        return json.loads(value)
    except Exception:  # noqa: BLE001
        return {}


def attr(obj: Any, name: str, default: Any = None) -> Any:
    if obj is None:
        return default
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def optional_value(value: Any) -> Any:
    """Unwrap ROS optional messages while preserving legacy scalar fields."""
    has_value = attr(value, "has_value")
    if has_value is None:
        return value
    if not bool(has_value):
        return None
    return attr(value, "value")


def get_dict(data: dict[str, Any] | None, key: str) -> dict[str, Any]:
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, dict) else {}


def get_position_from_dict(pose: dict[str, Any]) -> tuple[float | None, float | None, float | None]:
    pos = get_dict(pose, "position")
    return to_float(pos.get("x")), to_float(pos.get("y")), to_float(pos.get("z"))


def get_quat_from_dict(pose: dict[str, Any]) -> tuple[float | None, float | None, float | None, float | None]:
    ori = get_dict(pose, "orientation")
    return to_float(ori.get("x")), to_float(ori.get("y")), to_float(ori.get("z")), to_float(ori.get("w"))


def get_position_from_obj(pose: Any) -> tuple[float | None, float | None, float | None]:
    pos = attr(pose, "position")
    return to_float(attr(pos, "x")), to_float(attr(pos, "y")), to_float(attr(pos, "z"))


def get_quat_from_obj(pose: Any) -> tuple[float | None, float | None, float | None, float | None]:
    ori = attr(pose, "orientation")
    return to_float(attr(ori, "x")), to_float(attr(ori, "y")), to_float(attr(ori, "z")), to_float(attr(ori, "w"))


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


def snapshot_time_ns_guess(time_raw: Any) -> int | None:
    raw = to_int(time_raw)
    if raw is None:
        return None
    return raw * 1000


def ns_to_sec(value: Any) -> float | None:
    raw = to_int(value)
    if raw is None:
        return None
    return raw / 1_000_000_000


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


def quat_to_matrix(q: tuple[float | None, float | None, float | None, float | None]) -> list[list[float]] | None:
    norm = normalize_quat(q)
    if norm is None:
        return None
    x, y, z, w = norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def quat_to_euler_deg(q: tuple[float | None, float | None, float | None, float | None]) -> tuple[float | None, float | None, float | None]:
    norm = normalize_quat(q)
    if norm is None:
        return None, None, None
    x, y, z, w = norm
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def pose_columns(prefix: str, position: tuple[float | None, float | None, float | None], quat: tuple[float | None, float | None, float | None, float | None]) -> dict[str, Any]:
    x, y, z = position
    qx, qy, qz, qw = quat
    roll, pitch, yaw = quat_to_euler_deg(quat)
    matrix = quat_to_matrix(quat)
    row: dict[str, Any] = {
        f"{prefix}_x": x,
        f"{prefix}_y": y,
        f"{prefix}_z": z,
        f"{prefix}_qx": qx,
        f"{prefix}_qy": qy,
        f"{prefix}_qz": qz,
        f"{prefix}_qw": qw,
        f"{prefix}_roll_deg": roll,
        f"{prefix}_pitch_deg": pitch,
        f"{prefix}_yaw_deg": yaw,
    }
    if matrix is not None:
        row.update(
            {
                f"{prefix}_x_axis_x": matrix[0][0],
                f"{prefix}_x_axis_y": matrix[1][0],
                f"{prefix}_x_axis_z": matrix[2][0],
                f"{prefix}_y_axis_x": matrix[0][1],
                f"{prefix}_y_axis_y": matrix[1][1],
                f"{prefix}_y_axis_z": matrix[2][1],
                f"{prefix}_z_axis_x": matrix[0][2],
                f"{prefix}_z_axis_y": matrix[1][2],
                f"{prefix}_z_axis_z": matrix[2][2],
            }
        )
    return row


def position_distance(a: tuple[float | None, float | None, float | None], b: tuple[float | None, float | None, float | None]) -> float | None:
    if any(value is None for value in (*a, *b)):
        return None
    return math.sqrt(sum((float(av) - float(bv)) ** 2 for av, bv in zip(a, b)))


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


def image_info(path: Path) -> dict[str, Any]:
    row: dict[str, Any] = {
        "image_exists": path.exists(),
        "image_size_bytes": path.stat().st_size if path.exists() else "",
        "image_width": "",
        "image_height": "",
        "image_channels": "",
    }
    if not path.exists() or cv2 is None or np is None:
        return row
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
        if image is None:
            return row
        row["image_height"] = int(image.shape[0])
        row["image_width"] = int(image.shape[1])
        row["image_channels"] = int(image.shape[2]) if len(image.shape) >= 3 else 1
    except Exception:  # noqa: BLE001
        pass
    return row


def load_flights(index_dir: Path) -> dict[str, dict[str, Any]]:
    flights: dict[str, dict[str, Any]] = {}
    for row in read_csv_dicts(index_dir / "flights.csv"):
        time_sync = parse_json_field(row.get("time_sync", ""))
        row["time_sync_parsed"] = time_sync
        flight_id = row.get("flight_id", "")
        if flight_id:
            flights[flight_id] = row
    return flights


def load_pois(index_dir: Path) -> dict[str, dict[str, str]]:
    return {row.get("poi_id", ""): row for row in read_csv_dicts(index_dir / "pois.csv") if row.get("poi_id")}


def load_snapshot_json_rows(project_root: Path, index_dir: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    flights = load_flights(index_dir)
    pois = load_pois(index_dir)
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "snapshots").glob("*/snapshot.json")):
        data = load_json(meta_path, errors)
        if data is None:
            continue
        folder = meta_path.parent
        flight = flights.get(str(data.get("flight_id")), {})
        poi = pois.get(str(data.get("poi_id")), {})
        image_name = data.get("camera_image") or ""
        thumbnail_name = data.get("thumbnail") or ""
        image_path = folder / str(image_name) if image_name else folder / ""
        time_raw = data.get("time")
        time_ns = snapshot_time_ns_guess(time_raw)
        time_sync = flight.get("time_sync_parsed", {})

        row: dict[str, Any] = {
            "snapshot_id": data.get("id") or folder.name,
            "snapshot_folder": rel_path(folder, project_root),
            "flight_id": data.get("flight_id"),
            "flight_identifier": flight.get("identifier", ""),
            "flight_name": flight.get("name", ""),
            "poi_id": data.get("poi_id"),
            "poi_name": poi.get("name", ""),
            "time_raw": time_raw,
            "time_unit_guess": "microseconds",
            "time_ns_guess": time_ns,
            "time_sec_guess": ns_to_sec(time_ns),
            "camera_image": rel_path(image_path, project_root) if image_name else "",
            "camera_image_name": image_name,
            "thumbnail": rel_path(folder / str(thumbnail_name), project_root) if thumbnail_name else "",
            "camera_pitch": data.get("camera_pitch"),
            "wall_distance": data.get("wall_distance"),
            "criticality": data.get("criticality"),
            "mcap_file": flight.get("mcap_file", ""),
            "phase_start_time": time_sync.get("phase_start_time") if isinstance(time_sync, dict) else "",
            "phase_end_time": time_sync.get("phase_end_time") if isinstance(time_sync, dict) else "",
            "video_offset": time_sync.get("video_offset") if isinstance(time_sync, dict) else "",
            "av_offset": time_sync.get("av_offset") if isinstance(time_sync, dict) else "",
            "obc_offset": time_sync.get("obc_offset") if isinstance(time_sync, dict) else "",
            "vio_offset": time_sync.get("vio_offset") if isinstance(time_sync, dict) else "",
        }
        row.update(image_info(image_path))
        camera_pose = get_dict(data, "camera_pose")
        drone_pose = get_dict(data, "drone_pose")
        row.update(pose_columns("camera", get_position_from_dict(camera_pose), get_quat_from_dict(camera_pose)))
        row.update(pose_columns("drone", get_position_from_dict(drone_pose), get_quat_from_dict(drone_pose)))
        rows.append(row)
    return rows


def load_snapshot_mcap_candidates(mcap_dir: Path) -> list[dict[str, str]]:
    rows = read_csv_dicts(mcap_dir / "mcap_topic_candidates.csv")
    return [row for row in rows if row.get("topic") == SNAPSHOT_TOPIC and str(row.get("message_count", "0")) not in ("", "0")]


def event_from_decoded(candidate: dict[str, str], message: Any, decoded: Any, event_index: int) -> dict[str, Any]:
    stamp = attr(decoded, "stamp")
    camera_pose = attr(decoded, "camera_pose")
    drone_pose = attr(decoded, "drone_pose")
    row: dict[str, Any] = {
        "event_index": event_index,
        "mcap_file": candidate.get("mcap_file"),
        "linked_flight_id": candidate.get("linked_flight_id"),
        "linked_flight_identifier": candidate.get("linked_flight_identifier"),
        "channel_id": candidate.get("channel_id"),
        "topic": candidate.get("topic"),
        "schema_name": candidate.get("schema_name"),
        "sequence": attr(message, "sequence"),
        "log_time_ns": attr(message, "log_time"),
        "publish_time_ns": attr(message, "publish_time"),
        "log_time_sec": ns_to_sec(attr(message, "log_time")),
        "publish_time_sec": ns_to_sec(attr(message, "publish_time")),
        "stamp_sec": attr(stamp, "sec"),
        "stamp_nanosec": attr(stamp, "nanosec"),
        "stamp_ns": stamp_to_ns(stamp),
        "stamp_sec_float": ns_to_sec(stamp_to_ns(stamp)),
        "snapshot_id": attr(decoded, "id"),
        "flight_id": attr(decoded, "flight_id"),
        "poi_id": attr(decoded, "poi_id"),
        "criticality": attr(decoded, "criticality"),
        "camera_pitch": attr(decoded, "camera_pitch"),
        "wall_distance": optional_value(attr(decoded, "wall_distance")),
    }
    row.update(pose_columns("camera", get_position_from_obj(camera_pose), get_quat_from_obj(camera_pose)))
    row.update(pose_columns("drone", get_position_from_obj(drone_pose), get_quat_from_obj(drone_pose)))
    return row


def load_mcap_snapshot_events(project_root: Path, mcap_dir: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for candidate in load_snapshot_mcap_candidates(mcap_dir):
        relative_mcap = candidate.get("mcap_file", "")
        if not relative_mcap:
            continue
        mcap_path = project_root / relative_mcap
        if not mcap_path.exists():
            errors.append({"path": str(mcap_path), "error_type": "FileNotFoundError", "message": relative_mcap})
            continue
        try:
            event_index = 0
            with mcap_path.open("rb") as fp:
                reader = make_reader(fp, decoder_factories=[DecoderFactory()])
                for _schema, _channel, message, decoded in reader.iter_decoded_messages(topics=[SNAPSHOT_TOPIC]):
                    event_index += 1
                    rows.append(event_from_decoded(candidate, message, decoded, event_index))
        except Exception as exc:  # noqa: BLE001
            errors.append({"path": str(mcap_path), "error_type": type(exc).__name__, "message": str(exc)})
    return rows


def row_pose(row: dict[str, Any], prefix: str) -> tuple[tuple[float | None, float | None, float | None], tuple[float | None, float | None, float | None, float | None]]:
    pos = to_float(row.get(f"{prefix}_x")), to_float(row.get(f"{prefix}_y")), to_float(row.get(f"{prefix}_z"))
    quat = to_float(row.get(f"{prefix}_qx")), to_float(row.get(f"{prefix}_qy")), to_float(row.get(f"{prefix}_qz")), to_float(row.get(f"{prefix}_qw"))
    return pos, quat


def delta(a: Any, b: Any) -> float | None:
    av = to_float(a)
    bv = to_float(b)
    if av is None or bv is None:
        return None
    return av - bv


def build_matches(json_rows: list[dict[str, Any]], event_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_id: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for event in event_rows:
        snapshot_id = str(event.get("snapshot_id") or "")
        if snapshot_id:
            by_id[snapshot_id].append(event)

    rows: list[dict[str, Any]] = []
    for snapshot in json_rows:
        snapshot_id = str(snapshot.get("snapshot_id") or "")
        events = by_id.get(snapshot_id, [])
        event = events[0] if events else {}
        time_ns = to_int(snapshot.get("time_ns_guess"))
        stamp_ns = to_int(event.get("stamp_ns"))
        log_time_ns = to_int(event.get("log_time_ns"))
        publish_time_ns = to_int(event.get("publish_time_ns"))
        json_camera_pos, json_camera_quat = row_pose(snapshot, "camera")
        json_drone_pos, json_drone_quat = row_pose(snapshot, "drone")
        event_camera_pos, event_camera_quat = row_pose(event, "camera")
        event_drone_pos, event_drone_quat = row_pose(event, "drone")
        row = {
            "snapshot_id": snapshot_id,
            "flight_id": snapshot.get("flight_id"),
            "flight_identifier": snapshot.get("flight_identifier"),
            "poi_id": snapshot.get("poi_id"),
            "poi_name": snapshot.get("poi_name"),
            "camera_image": snapshot.get("camera_image"),
            "json_time_raw": snapshot.get("time_raw"),
            "json_time_ns_guess": time_ns,
            "mcap_found": bool(event),
            "mcap_duplicate_count": len(events),
            "mcap_file": event.get("mcap_file", ""),
            "mcap_log_time_ns": log_time_ns,
            "mcap_publish_time_ns": publish_time_ns,
            "mcap_stamp_ns": stamp_ns,
            "delta_stamp_minus_json_time_ns": stamp_ns - time_ns if stamp_ns is not None and time_ns is not None else "",
            "delta_log_minus_json_time_ns": log_time_ns - time_ns if log_time_ns is not None and time_ns is not None else "",
            "delta_publish_minus_json_time_ns": publish_time_ns - time_ns if publish_time_ns is not None and time_ns is not None else "",
            "camera_position_error_m": position_distance(json_camera_pos, event_camera_pos),
            "camera_orientation_error_deg": quat_angle_error_deg(json_camera_quat, event_camera_quat),
            "drone_position_error_m": position_distance(json_drone_pos, event_drone_pos),
            "drone_orientation_error_deg": quat_angle_error_deg(json_drone_quat, event_drone_quat),
            "camera_pitch_delta": delta(snapshot.get("camera_pitch"), event.get("camera_pitch")),
            "wall_distance_delta": delta(snapshot.get("wall_distance"), event.get("wall_distance")),
            "json_camera_x": snapshot.get("camera_x"),
            "json_camera_y": snapshot.get("camera_y"),
            "json_camera_z": snapshot.get("camera_z"),
            "mcap_camera_x": event.get("camera_x", ""),
            "mcap_camera_y": event.get("camera_y", ""),
            "mcap_camera_z": event.get("camera_z", ""),
            "json_drone_x": snapshot.get("drone_x"),
            "json_drone_y": snapshot.get("drone_y"),
            "json_drone_z": snapshot.get("drone_z"),
            "mcap_drone_x": event.get("drone_x", ""),
            "mcap_drone_y": event.get("drone_y", ""),
            "mcap_drone_z": event.get("drone_z", ""),
        }
        rows.append(row)
    return rows


def build_timeline(json_rows: list[dict[str, Any]], matches: list[dict[str, Any]]) -> list[dict[str, Any]]:
    matched_by_id = {row.get("snapshot_id"): row for row in matches}
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in json_rows:
        grouped[str(row.get("flight_id") or "")].append(row)

    timeline: list[dict[str, Any]] = []
    for flight_id, snapshots in grouped.items():
        snapshots.sort(key=lambda row: to_int(row.get("time_ns_guess")) or -1)
        previous_time: int | None = None
        for index, snapshot in enumerate(snapshots, start=1):
            current_time = to_int(snapshot.get("time_ns_guess"))
            match = matched_by_id.get(snapshot.get("snapshot_id"), {})
            row = {
                "flight_id": flight_id,
                "flight_identifier": snapshot.get("flight_identifier"),
                "order_in_flight": index,
                "snapshot_id": snapshot.get("snapshot_id"),
                "poi_id": snapshot.get("poi_id"),
                "poi_name": snapshot.get("poi_name"),
                "camera_image": snapshot.get("camera_image"),
                "time_raw": snapshot.get("time_raw"),
                "time_ns_guess": current_time,
                "time_sec_guess": ns_to_sec(current_time),
                "delta_from_previous_sec": ((current_time - previous_time) / 1_000_000_000) if current_time is not None and previous_time is not None else "",
                "mcap_found": match.get("mcap_found", False),
                "mcap_stamp_ns": match.get("mcap_stamp_ns", ""),
                "delta_stamp_minus_json_time_ns": match.get("delta_stamp_minus_json_time_ns", ""),
                "camera_x": snapshot.get("camera_x"),
                "camera_y": snapshot.get("camera_y"),
                "camera_z": snapshot.get("camera_z"),
                "camera_yaw_deg": snapshot.get("camera_yaw_deg"),
                "camera_pitch_deg": snapshot.get("camera_pitch_deg"),
                "camera_roll_deg": snapshot.get("camera_roll_deg"),
            }
            timeline.append(row)
            previous_time = current_time
    return timeline


POSE_FIELDNAMES = [
    "snapshot_id",
    "snapshot_folder",
    "flight_id",
    "flight_identifier",
    "flight_name",
    "poi_id",
    "poi_name",
    "time_raw",
    "time_unit_guess",
    "time_ns_guess",
    "time_sec_guess",
    "camera_image",
    "camera_image_name",
    "image_exists",
    "image_size_bytes",
    "image_width",
    "image_height",
    "image_channels",
    "thumbnail",
    "camera_pitch",
    "wall_distance",
    "criticality",
    "mcap_file",
    "phase_start_time",
    "phase_end_time",
    "video_offset",
    "av_offset",
    "obc_offset",
    "vio_offset",
]


POSE_SUFFIXES = [
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "x_axis_x",
    "x_axis_y",
    "x_axis_z",
    "y_axis_x",
    "y_axis_y",
    "y_axis_z",
    "z_axis_x",
    "z_axis_y",
    "z_axis_z",
]


def pose_fieldnames(base: list[str]) -> list[str]:
    return base + [f"camera_{suffix}" for suffix in POSE_SUFFIXES] + [f"drone_{suffix}" for suffix in POSE_SUFFIXES]


def build_snapshot_outputs(project_root: Path, index_dir: Path, mcap_dir: Path, output_dir: Path) -> dict[str, Any]:
    project_root = project_root.resolve()
    index_dir = index_dir.resolve()
    mcap_dir = mcap_dir.resolve()
    output_dir = output_dir.resolve()
    errors: list[dict[str, Any]] = []

    json_rows = load_snapshot_json_rows(project_root, index_dir, errors)
    event_rows = load_mcap_snapshot_events(project_root, mcap_dir, errors)
    match_rows = build_matches(json_rows, event_rows)
    timeline_rows = build_timeline(json_rows, match_rows)

    write_csv(output_dir / "snapshot_json_poses.csv", json_rows, pose_fieldnames(POSE_FIELDNAMES))
    write_csv(
        output_dir / "mcap_snapshot_events.csv",
        event_rows,
        pose_fieldnames(
            [
                "event_index",
                "mcap_file",
                "linked_flight_id",
                "linked_flight_identifier",
                "channel_id",
                "topic",
                "schema_name",
                "sequence",
                "log_time_ns",
                "publish_time_ns",
                "log_time_sec",
                "publish_time_sec",
                "stamp_sec",
                "stamp_nanosec",
                "stamp_ns",
                "stamp_sec_float",
                "snapshot_id",
                "flight_id",
                "poi_id",
                "criticality",
                "camera_pitch",
                "wall_distance",
            ]
        ),
    )
    write_csv(
        output_dir / "snapshot_mcap_matches.csv",
        match_rows,
        [
            "snapshot_id",
            "flight_id",
            "flight_identifier",
            "poi_id",
            "poi_name",
            "camera_image",
            "json_time_raw",
            "json_time_ns_guess",
            "mcap_found",
            "mcap_duplicate_count",
            "mcap_file",
            "mcap_log_time_ns",
            "mcap_publish_time_ns",
            "mcap_stamp_ns",
            "delta_stamp_minus_json_time_ns",
            "delta_log_minus_json_time_ns",
            "delta_publish_minus_json_time_ns",
            "camera_position_error_m",
            "camera_orientation_error_deg",
            "drone_position_error_m",
            "drone_orientation_error_deg",
            "camera_pitch_delta",
            "wall_distance_delta",
            "json_camera_x",
            "json_camera_y",
            "json_camera_z",
            "mcap_camera_x",
            "mcap_camera_y",
            "mcap_camera_z",
            "json_drone_x",
            "json_drone_y",
            "json_drone_z",
            "mcap_drone_x",
            "mcap_drone_y",
            "mcap_drone_z",
        ],
    )
    write_csv(
        output_dir / "snapshot_timeline.csv",
        timeline_rows,
        [
            "flight_id",
            "flight_identifier",
            "order_in_flight",
            "snapshot_id",
            "poi_id",
            "poi_name",
            "camera_image",
            "time_raw",
            "time_ns_guess",
            "time_sec_guess",
            "delta_from_previous_sec",
            "mcap_found",
            "mcap_stamp_ns",
            "delta_stamp_minus_json_time_ns",
            "camera_x",
            "camera_y",
            "camera_z",
            "camera_yaw_deg",
            "camera_pitch_deg",
            "camera_roll_deg",
        ],
    )
    write_csv(output_dir / "snapshot_pose_errors.csv", errors, ["path", "error_type", "message"])

    matched = [row for row in match_rows if row.get("mcap_found")]
    missing = [row for row in match_rows if not row.get("mcap_found")]
    time_deltas = [abs(int(row["delta_stamp_minus_json_time_ns"])) for row in matched if row.get("delta_stamp_minus_json_time_ns") not in ("", None)]
    camera_errors = [float(row["camera_position_error_m"]) for row in matched if row.get("camera_position_error_m") not in ("", None)]
    orientation_errors = [float(row["camera_orientation_error_deg"]) for row in matched if row.get("camera_orientation_error_deg") not in ("", None)]

    summary = {
        "project_root": str(project_root),
        "index_dir": str(index_dir),
        "mcap_dir": str(mcap_dir),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "counts": {
            "snapshot_json_files": len(json_rows),
            "mcap_snapshot_events": len(event_rows),
            "matched_by_snapshot_id": len(matched),
            "missing_mcap_event": len(missing),
            "images_existing": sum(1 for row in json_rows if row.get("image_exists")),
            "errors": len(errors),
        },
        "snapshots_by_flight": dict(sorted(Counter(row.get("flight_identifier", "") for row in json_rows).items())),
        "mcap_events_by_flight": dict(sorted(Counter(row.get("linked_flight_identifier", "") for row in event_rows).items())),
        "max_abs_delta_stamp_minus_json_time_ns": max(time_deltas) if time_deltas else "",
        "max_camera_position_error_m": max(camera_errors) if camera_errors else "",
        "max_camera_orientation_error_deg": max(orientation_errors) if orientation_errors else "",
    }
    write_json(output_dir / "snapshot_pose_summary.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Index Inspector5 snapshots and match MCAP snapshot events.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--index-dir", type=Path, default=None)
    parser.add_argument("--mcap-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if not project_root.exists():
        raise SystemExit(f"Project root does not exist: {project_root}")
    index_dir = args.index_dir.resolve() if args.index_dir else default_index_dir(project_root).resolve()
    mcap_dir = args.mcap_dir.resolve() if args.mcap_dir else default_mcap_dir(project_root).resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else default_output_dir(project_root).resolve()
    summary = build_snapshot_outputs(project_root, index_dir, mcap_dir, output_dir)
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
