from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TOP_FOLDERS = (
    "assets",
    "flights",
    "inspections",
    "maps",
    "orgs",
    "pois",
    "snapshots",
)

MEDIA_EXTENSIONS = {".jpg", ".jpeg", ".png", ".mov", ".mp4"}
CALIBRATION_NAMES = {
    "camera_calibration_0.yaml",
    "camera_calibration_1.yaml",
    "camera_calibration_2.yaml",
    "imu_cam_extrinsics.yaml",
    "static_transforms.yaml",
    "geoslam_transforms.yaml",
}


def load_json(path: Path, errors: list[dict[str, Any]]) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        errors.append(
            {
                "path": str(path),
                "error_type": type(exc).__name__,
                "message": str(exc),
            }
        )
        return None


def ensure_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def get_nested(data: dict[str, Any] | None, *keys: str) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def file_ref(data: dict[str, Any] | None, key: str) -> str:
    value = get_nested(data, key)
    if isinstance(value, dict):
        return str(value.get("path") or "")
    if isinstance(value, str):
        return value
    return ""


def rel_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def iso_mtime(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat()


def epoch_to_utc(value: Any) -> str:
    if value in (None, ""):
        return ""
    try:
        return datetime.fromtimestamp(float(value), timezone.utc).isoformat()
    except Exception:  # noqa: BLE001
        return ""


def duration_seconds_guess(value: Any) -> str:
    if value in (None, ""):
        return ""
    try:
        number = float(value)
    except Exception:  # noqa: BLE001
        return ""
    if number > 1_000_000:
        return f"{number / 1_000_000:.3f}"
    return f"{number:.3f}"


def csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list) and all(isinstance(item, (str, int, float, bool)) for item in value):
        return ";".join(str(item) for item in value)
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


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


def infer_category(path: Path, project_root: Path) -> str:
    parts = path.relative_to(project_root).parts
    if parts and parts[0] in TOP_FOLDERS:
        return parts[0]
    return ""


def infer_kind(path: Path) -> str:
    suffix = path.suffix.lower()
    name = path.name.lower()
    if name.endswith(".backup"):
        return "backup"
    if suffix == ".mcap":
        return "mcap"
    if suffix == ".bag":
        return "bag"
    if suffix == ".las":
        return "point_cloud_las"
    if suffix in {".yaml", ".yml"}:
        return "yaml"
    if suffix == ".json":
        return "json"
    if suffix == ".csv":
        return "csv"
    if suffix in {".mov", ".mp4"}:
        return "video"
    if suffix in {".jpg", ".jpeg", ".png"}:
        return "image"
    if suffix in {".txt", ".log"}:
        return "text_log"
    if suffix in {".stn", ".meta", ".gcg"}:
        return suffix.lstrip(".")
    return "other"


def infer_role(path: Path) -> str:
    name = path.name.lower()
    suffix = path.suffix.lower()
    if name == "flight.json":
        return "flight_metadata"
    if name == "snapshot.json":
        return "snapshot_metadata"
    if name == "map.json":
        return "map_metadata"
    if name == "poi.json":
        return "poi_metadata"
    if name == "asset.json":
        return "asset_metadata"
    if name == "inspection.json":
        return "inspection_metadata"
    if name == "organization.json":
        return "organization_metadata"
    if name in CALIBRATION_NAMES:
        return name.removesuffix(".yaml")
    if name.endswith("_obcbag.mcap"):
        return "obc_mcap"
    if name.endswith("_livemap.las"):
        return "live_map_las"
    if name.endswith("_livetraj.csv"):
        return "live_trajectory_csv"
    if name.endswith("_lidar_metadata.json"):
        return "lidar_metadata"
    if name.endswith("_logbook.json"):
        return "logbook"
    if name.endswith("_nxstats.csv"):
        return "nxstats"
    if name.endswith("_gaston-stats.csv"):
        return "gaston_stats"
    if name.endswith("_syslog.txt"):
        return "syslog"
    if name.endswith("_thermal.mp4"):
        return "thermal_video"
    if name.endswith("_thermal.meta"):
        return "thermal_video_meta"
    if name.endswith("_thermal.stn"):
        return "thermal_log"
    if suffix in MEDIA_EXTENSIONS:
        return "media"
    return ""


def file_rows(project_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(project_root.rglob("*")):
        if not path.is_file():
            continue
        stat = path.stat()
        rows.append(
            {
                "category": infer_category(path, project_root),
                "container_id": path.parent.name,
                "relative_path": rel_path(path, project_root),
                "file_name": path.name,
                "extension": path.suffix.lower(),
                "kind": infer_kind(path),
                "role": infer_role(path),
                "size_bytes": stat.st_size,
                "modified_utc": iso_mtime(path),
            }
        )
    return rows


def linked_file(base: Path, project_root: Path, file_name: str) -> tuple[str, bool, int | str]:
    if not file_name:
        return "", False, ""
    path = base / file_name
    if path.exists():
        return rel_path(path, project_root), True, path.stat().st_size
    return rel_path(path, project_root), False, ""


def component_by_name(robot: dict[str, Any], component_name: str) -> dict[str, Any]:
    for component in ensure_list(robot.get("components")):
        if get_nested(component, "identifier", "component") == component_name:
            return component
    return {}


def index_flights(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "flights").glob("*/flight.json")):
        data = load_json(meta_path, errors) or {}
        folder = meta_path.parent
        files = data.get("files") if isinstance(data.get("files"), dict) else {}
        robot = data.get("robot") if isinstance(data.get("robot"), dict) else {}
        main_robot = robot.get("main") if isinstance(robot.get("main"), dict) else {}
        lidar = component_by_name(robot, "Lidar")
        location = data.get("location") if isinstance(data.get("location"), dict) else {}

        mcap_name = str(files.get("obc_bag") or file_ref(files, "obc_bag_file"))
        mcap_relative, mcap_exists, mcap_size = linked_file(folder, project_root, mcap_name)

        rows.append(
            {
                "flight_id": data.get("id") or folder.name,
                "folder": rel_path(folder, project_root),
                "name": data.get("name"),
                "identifier": data.get("identifier"),
                "inspection_id": data.get("inspection_id"),
                "phase": data.get("phase"),
                "datetime_utc_raw": data.get("datetime_utc"),
                "datetime_utc_iso": epoch_to_utc(data.get("datetime_utc")),
                "duration_raw": data.get("duration"),
                "duration_seconds_guess": duration_seconds_guess(data.get("duration")),
                "flight_count": data.get("flight_count"),
                "session_count": data.get("session_count"),
                "latitude": location.get("latitude"),
                "longitude": location.get("longitude"),
                "altitude": location.get("altitude"),
                "robot_model": robot.get("model") or main_robot.get("model"),
                "robot_serial": main_robot.get("serial_number"),
                "robot_fw": main_robot.get("firmware_version"),
                "lidar_model": lidar.get("model"),
                "lidar_serial": lidar.get("serial_number"),
                "lidar_fw": lidar.get("firmware_version"),
                "mcap_file": mcap_relative,
                "mcap_exists": mcap_exists,
                "mcap_size_bytes": mcap_size,
                "rgb_videos": files.get("rgb_videos"),
                "thermal_video": files.get("thermal_video"),
                "lidar_metadata": files.get("lidar_metadata"),
                "logbook": files.get("logbook"),
                "thumbnail": files.get("thumbnail"),
                "camera_calibration_0": file_ref(files, "calibration_camera_0_file"),
                "camera_calibration_1": file_ref(files, "calibration_camera_1_file"),
                "camera_calibration_2": file_ref(files, "calibration_camera_2_file"),
                "imu_cam_extrinsics": file_ref(files, "calibration_imu_cam_extrinsics_file"),
                "static_transforms": file_ref(files, "calibration_static_transforms_file"),
                "geoslam_transforms": file_ref(files, "calibration_geoslam_transforms_file"),
                "time_sync": data.get("time_sync"),
                "snapshot_count": 0,
                "map_count": 0,
            }
        )
    return rows


def pose_fields(prefix: str, pose: dict[str, Any] | None) -> dict[str, Any]:
    position = pose.get("position") if isinstance(pose, dict) and isinstance(pose.get("position"), dict) else {}
    orientation = pose.get("orientation") if isinstance(pose, dict) and isinstance(pose.get("orientation"), dict) else {}
    return {
        f"{prefix}_x": position.get("x"),
        f"{prefix}_y": position.get("y"),
        f"{prefix}_z": position.get("z"),
        f"{prefix}_qw": orientation.get("w"),
        f"{prefix}_qx": orientation.get("x"),
        f"{prefix}_qy": orientation.get("y"),
        f"{prefix}_qz": orientation.get("z"),
    }


def index_snapshots(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "snapshots").glob("*/snapshot.json")):
        data = load_json(meta_path, errors) or {}
        folder = meta_path.parent
        image_name = str(data.get("camera_image") or file_ref(data, "camera_image_file"))
        thumbnail_name = str(data.get("thumbnail") or file_ref(data, "thumbnail_file"))
        image_relative, image_exists, image_size = linked_file(folder, project_root, image_name)
        thumbnail_relative, thumbnail_exists, _ = linked_file(folder, project_root, thumbnail_name)
        row = {
            "snapshot_id": data.get("id") or folder.name,
            "folder": rel_path(folder, project_root),
            "flight_id": data.get("flight_id"),
            "poi_id": data.get("poi_id"),
            "time_raw": data.get("time"),
            "camera_image": image_relative,
            "camera_image_exists": image_exists,
            "camera_image_size_bytes": image_size,
            "thumbnail": thumbnail_relative,
            "thumbnail_exists": thumbnail_exists,
            "camera_pitch": data.get("camera_pitch"),
            "wall_distance": data.get("wall_distance"),
            "criticality": data.get("criticality"),
            "tags": data.get("tags"),
        }
        row.update(pose_fields("camera_pos", data.get("camera_pose")))
        row.update(pose_fields("drone_pos", data.get("drone_pose")))
        rows.append(row)
    return rows


def map_file_refs(items: Any) -> list[str]:
    refs: list[str] = []
    for item in ensure_list(items):
        if not isinstance(item, dict):
            continue
        value = item.get("file_path") or get_nested(item, "file", "path")
        if value:
            refs.append(str(value))
    return refs


def index_maps(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "maps").glob("*/map.json")):
        data = load_json(meta_path, errors) or {}
        folder = meta_path.parent
        point_clouds = map_file_refs(data.get("point_clouds"))
        trajectories = map_file_refs(data.get("trajectories"))
        rows.append(
            {
                "map_id": data.get("id") or folder.name,
                "folder": rel_path(folder, project_root),
                "parent_id": data.get("parent_id"),
                "parent_type": data.get("parent_type"),
                "source": data.get("source"),
                "point_cloud_files": point_clouds,
                "point_cloud_exists": all((folder / item).exists() for item in point_clouds) if point_clouds else False,
                "trajectory_files": trajectories,
                "trajectory_exists": all((folder / item).exists() for item in trajectories) if trajectories else False,
                "thumbnail": data.get("thumbnail"),
                "transform": data.get("transform"),
            }
        )
    return rows


def index_pois(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "pois").glob("*/poi.json")):
        data = load_json(meta_path, errors) or {}
        rows.append(
            {
                "poi_id": data.get("id") or meta_path.parent.name,
                "folder": rel_path(meta_path.parent, project_root),
                "asset_id": data.get("asset_id"),
                "name": data.get("name"),
                "description": data.get("description"),
                "tags": data.get("tags"),
                "snapshot_count": 0,
            }
        )
    return rows


def index_assets(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "assets").glob("*/asset.json")):
        data = load_json(meta_path, errors) or {}
        rows.append(
            {
                "asset_id": data.get("id") or meta_path.parent.name,
                "folder": rel_path(meta_path.parent, project_root),
                "organization_id": data.get("organization_id"),
                "name": data.get("name"),
                "description": data.get("description"),
                "tags": data.get("tags"),
                "relocalization": data.get("relocalization"),
            }
        )
    return rows


def index_inspections(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "inspections").glob("*/inspection.json")):
        data = load_json(meta_path, errors) or {}
        rows.append(
            {
                "inspection_id": data.get("id") or meta_path.parent.name,
                "folder": rel_path(meta_path.parent, project_root),
                "asset_id": data.get("asset_id"),
                "name": data.get("name"),
                "description": data.get("description"),
                "tags": data.get("tags"),
                "flight_count": 0,
            }
        )
    return rows


def index_organizations(project_root: Path, errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for meta_path in sorted((project_root / "orgs").glob("*/organization.json")):
        data = load_json(meta_path, errors) or {}
        rows.append(
            {
                "organization_id": data.get("id") or meta_path.parent.name,
                "folder": rel_path(meta_path.parent, project_root),
                "name": data.get("name"),
                "path": data.get("path"),
            }
        )
    return rows


def index_mcap_files(project_root: Path, file_index: list[dict[str, Any]], flights: list[dict[str, Any]]) -> list[dict[str, Any]]:
    flights_by_id = {str(row["flight_id"]): row for row in flights}
    rows: list[dict[str, Any]] = []
    for row in file_index:
        if row["kind"] != "mcap":
            continue
        path = project_root / row["relative_path"]
        linked_flight = flights_by_id.get(path.parent.name, {})
        rows.append(
            {
                "mcap_file": row["relative_path"],
                "file_name": row["file_name"],
                "folder": rel_path(path.parent, project_root),
                "linked_flight_id": linked_flight.get("flight_id", ""),
                "linked_flight_identifier": linked_flight.get("identifier", ""),
                "size_bytes": row["size_bytes"],
                "modified_utc": row["modified_utc"],
            }
        )
    return rows


def index_calibration_files(project_root: Path, file_index: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in file_index:
        if row["kind"] != "yaml":
            continue
        path = project_root / row["relative_path"]
        rows.append(
            {
                "file": row["relative_path"],
                "file_name": row["file_name"],
                "folder": rel_path(path.parent, project_root),
                "linked_flight_id": path.parent.name if infer_category(path, project_root) == "flights" else "",
                "role": row["role"],
                "size_bytes": row["size_bytes"],
                "modified_utc": row["modified_utc"],
            }
        )
    return rows


def index_media_files(file_index: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "file": row["relative_path"],
            "file_name": row["file_name"],
            "category": row["category"],
            "container_id": row["container_id"],
            "kind": row["kind"],
            "role": row["role"],
            "size_bytes": row["size_bytes"],
            "modified_utc": row["modified_utc"],
        }
        for row in file_index
        if row["extension"] in MEDIA_EXTENSIONS
    ]


def apply_counts(
    flights: list[dict[str, Any]],
    maps: list[dict[str, Any]],
    snapshots: list[dict[str, Any]],
    pois: list[dict[str, Any]],
    inspections: list[dict[str, Any]],
) -> None:
    snapshots_by_flight = Counter(str(row.get("flight_id")) for row in snapshots if row.get("flight_id"))
    maps_by_parent = Counter(str(row.get("parent_id")) for row in maps if row.get("parent_id"))
    snapshots_by_poi = Counter(str(row.get("poi_id")) for row in snapshots if row.get("poi_id"))
    flights_by_inspection = Counter(str(row.get("inspection_id")) for row in flights if row.get("inspection_id"))

    for row in flights:
        flight_id = str(row.get("flight_id"))
        row["snapshot_count"] = snapshots_by_flight.get(flight_id, 0)
        row["map_count"] = maps_by_parent.get(flight_id, 0)
    for row in pois:
        poi_id = str(row.get("poi_id"))
        row["snapshot_count"] = snapshots_by_poi.get(poi_id, 0)
    for row in inspections:
        inspection_id = str(row.get("inspection_id"))
        row["flight_count"] = flights_by_inspection.get(inspection_id, 0)


def scan_project(project_root: Path, output_dir: Path) -> dict[str, Any]:
    project_root = project_root.resolve()
    output_dir = output_dir.resolve()
    errors: list[dict[str, Any]] = []

    file_index = file_rows(project_root)
    flights = index_flights(project_root, errors)
    snapshots = index_snapshots(project_root, errors)
    maps = index_maps(project_root, errors)
    pois = index_pois(project_root, errors)
    assets = index_assets(project_root, errors)
    inspections = index_inspections(project_root, errors)
    organizations = index_organizations(project_root, errors)
    apply_counts(flights, maps, snapshots, pois, inspections)

    mcap_files = index_mcap_files(project_root, file_index, flights)
    calibration_files = index_calibration_files(project_root, file_index)
    media_files = index_media_files(file_index)

    write_csv(
        output_dir / "project_files.csv",
        file_index,
        ["category", "container_id", "relative_path", "file_name", "extension", "kind", "role", "size_bytes", "modified_utc"],
    )
    write_csv(
        output_dir / "flights.csv",
        flights,
        [
            "flight_id",
            "folder",
            "name",
            "identifier",
            "inspection_id",
            "phase",
            "datetime_utc_raw",
            "datetime_utc_iso",
            "duration_raw",
            "duration_seconds_guess",
            "flight_count",
            "session_count",
            "latitude",
            "longitude",
            "altitude",
            "robot_model",
            "robot_serial",
            "robot_fw",
            "lidar_model",
            "lidar_serial",
            "lidar_fw",
            "mcap_file",
            "mcap_exists",
            "mcap_size_bytes",
            "rgb_videos",
            "thermal_video",
            "lidar_metadata",
            "logbook",
            "thumbnail",
            "camera_calibration_0",
            "camera_calibration_1",
            "camera_calibration_2",
            "imu_cam_extrinsics",
            "static_transforms",
            "geoslam_transforms",
            "time_sync",
            "snapshot_count",
            "map_count",
        ],
    )
    write_csv(
        output_dir / "snapshots.csv",
        snapshots,
        [
            "snapshot_id",
            "folder",
            "flight_id",
            "poi_id",
            "time_raw",
            "camera_image",
            "camera_image_exists",
            "camera_image_size_bytes",
            "thumbnail",
            "thumbnail_exists",
            "camera_pitch",
            "wall_distance",
            "criticality",
            "camera_pos_x",
            "camera_pos_y",
            "camera_pos_z",
            "camera_pos_qw",
            "camera_pos_qx",
            "camera_pos_qy",
            "camera_pos_qz",
            "drone_pos_x",
            "drone_pos_y",
            "drone_pos_z",
            "drone_pos_qw",
            "drone_pos_qx",
            "drone_pos_qy",
            "drone_pos_qz",
            "tags",
        ],
    )
    write_csv(
        output_dir / "maps.csv",
        maps,
        [
            "map_id",
            "folder",
            "parent_id",
            "parent_type",
            "source",
            "point_cloud_files",
            "point_cloud_exists",
            "trajectory_files",
            "trajectory_exists",
            "thumbnail",
            "transform",
        ],
    )
    write_csv(output_dir / "pois.csv", pois, ["poi_id", "folder", "asset_id", "name", "description", "tags", "snapshot_count"])
    write_csv(output_dir / "assets.csv", assets, ["asset_id", "folder", "organization_id", "name", "description", "tags", "relocalization"])
    write_csv(output_dir / "inspections.csv", inspections, ["inspection_id", "folder", "asset_id", "name", "description", "tags", "flight_count"])
    write_csv(output_dir / "organizations.csv", organizations, ["organization_id", "folder", "name", "path"])
    write_csv(output_dir / "mcap_files.csv", mcap_files, ["mcap_file", "file_name", "folder", "linked_flight_id", "linked_flight_identifier", "size_bytes", "modified_utc"])
    write_csv(output_dir / "calibration_files.csv", calibration_files, ["file", "file_name", "folder", "linked_flight_id", "role", "size_bytes", "modified_utc"])
    write_csv(output_dir / "media_files.csv", media_files, ["file", "file_name", "category", "container_id", "kind", "role", "size_bytes", "modified_utc"])
    write_csv(output_dir / "scan_errors.csv", errors, ["path", "error_type", "message"])

    by_category = Counter(row["category"] or "(root)" for row in file_index)
    by_kind = Counter(row["kind"] for row in file_index)
    by_extension = Counter(row["extension"] or "(none)" for row in file_index)
    summary = {
        "project_root": str(project_root),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "counts": {
            "files": len(file_index),
            "bytes": sum(int(row["size_bytes"]) for row in file_index),
            "flights": len(flights),
            "mcap_files": len(mcap_files),
            "snapshots": len(snapshots),
            "maps": len(maps),
            "pois": len(pois),
            "assets": len(assets),
            "inspections": len(inspections),
            "organizations": len(organizations),
            "calibration_files": len(calibration_files),
            "media_files": len(media_files),
            "errors": len(errors),
        },
        "files_by_category": dict(sorted(by_category.items())),
        "files_by_kind": dict(sorted(by_kind.items())),
        "files_by_extension": dict(sorted(by_extension.items())),
    }
    write_json(output_dir / "project_summary.json", summary)
    return summary


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2] / "sepoong"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Index an Inspector5 project folder.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if args.output_dir is None:
        output_dir = project_root.parent / "outputs" / "index"
    else:
        output_dir = args.output_dir.resolve()
    if not project_root.exists():
        raise SystemExit(f"Project root does not exist: {project_root}")
    summary = scan_project(project_root, output_dir)
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
