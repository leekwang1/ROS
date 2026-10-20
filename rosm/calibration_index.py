from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


CAMERA_FILES = {
    "camera_calibration_0.yaml": 0,
    "camera_calibration_1.yaml": 1,
    "camera_calibration_2.yaml": 2,
}

XACTI_CAMERA_TYPE = 1
ELIOS3_ROBOT_TYPE = 3
XACTI_SNAPSHOT_WIDTH = 4000
XACTI_SNAPSHOT_HEIGHT = 3000
DEFAULT_INSPECTOR_DIR = Path("C:/Program Files/Flyability/Inspector 5")


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


def matrix3(values: list[float]) -> list[list[float]]:
    if len(values) != 9:
        raise ValueError(f"Expected 9 matrix values, got {len(values)}")
    return [values[0:3], values[3:6], values[6:9]]


def read_xacti_camera_spec_from_inspector(
    image_width: int = XACTI_SNAPSHOT_WIDTH,
    image_height: int = XACTI_SNAPSHOT_HEIGHT,
    inspector_dir: Path = DEFAULT_INSPECTOR_DIR,
) -> dict[str, Any]:
    if os.name != "nt":
        raise RuntimeError("Inspector robotspec.dll extraction is only available on Windows.")
    robotspec_path = inspector_dir / "robotspec.dll"
    if not robotspec_path.exists():
        raise FileNotFoundError(f"robotspec.dll not found: {robotspec_path}")

    dll_dir_handle = os.add_dll_directory(str(inspector_dir)) if hasattr(os, "add_dll_directory") else None
    dll = ctypes.CDLL(str(robotspec_path))

    class QSize(ctypes.Structure):
        _fields_ = [("wd", ctypes.c_int), ("ht", ctypes.c_int)]

    def export(name: str) -> Any:
        try:
            return getattr(dll, name)
        except AttributeError as exc:
            raise RuntimeError(f"robotspec.dll export not found: {name}") from exc

    xacti_camera = export("?xactiCamera@CameraSpecDatabase@flya@@SA?AVCameraSpec@2@AEBVQSize@@@Z")
    xacti_camera.argtypes = [ctypes.c_void_p, ctypes.POINTER(QSize)]
    xacti_camera.restype = ctypes.c_void_p

    camera_type = export("?type@CameraSpec@flya@@QEBA?AW4CameraType@2@XZ")
    camera_type.argtypes = [ctypes.c_void_p]
    camera_type.restype = ctypes.c_int

    camera_k = export("?K@CameraSpec@flya@@QEBAAEBV?$Matx@N$02$02@cv@@XZ")
    camera_k.argtypes = [ctypes.c_void_p]
    camera_k.restype = ctypes.c_void_p

    camera_d = export("?D@CameraSpec@flya@@QEBAAEBV?$Vec@N$03@cv@@XZ")
    camera_d.argtypes = [ctypes.c_void_p]
    camera_d.restype = ctypes.c_void_p

    camera_p = export("?P@CameraSpec@flya@@QEBAAEBV?$Matx@N$02$02@cv@@XZ")
    camera_p.argtypes = [ctypes.c_void_p]
    camera_p.restype = ctypes.c_void_p

    lens_to_focal_point = export("?lensToFocalPointDistanceMeters@CameraSpec@flya@@QEBANXZ")
    lens_to_focal_point.argtypes = [ctypes.c_void_p]
    lens_to_focal_point.restype = ctypes.c_double

    robot_main_camera = None
    camera_position_on_robot = None
    try:
        robot_get = export("?get@RobotSpecDatabase@flya@@SAPEAVRobotSpec@2@W4Type@RobotSpecType@2@@Z")
        robot_get.argtypes = [ctypes.c_int]
        robot_get.restype = ctypes.c_void_p
        main_camera = export("?mainCamera@RobotSpec@flya@@QEBA?AW4CameraType@2@XZ")
        main_camera.argtypes = [ctypes.c_void_p]
        main_camera.restype = ctypes.c_int
        camera_position = export("?cameraPositionOnRobot@RobotSpec@flya@@QEBAAEBVQVector3D@@XZ")
        camera_position.argtypes = [ctypes.c_void_p]
        camera_position.restype = ctypes.c_void_p
        robot = robot_get(ELIOS3_ROBOT_TYPE)
        if robot:
            robot_main_camera = int(main_camera(robot))
            pos_ptr = camera_position(robot)
            if pos_ptr:
                pos = list((ctypes.c_float * 3).from_address(pos_ptr))
                camera_position_on_robot = {"x": pos[0], "y": pos[1], "z": pos[2]}
    except Exception:  # noqa: BLE001
        robot_main_camera = None
        camera_position_on_robot = None

    spec = ctypes.create_string_buffer(256)
    size = QSize(int(image_width), int(image_height))
    xacti_camera(ctypes.byref(spec), ctypes.byref(size))

    k_values = [float(value) for value in (ctypes.c_double * 9).from_address(camera_k(ctypes.byref(spec)))]
    d_values = [float(value) for value in (ctypes.c_double * 4).from_address(camera_d(ctypes.byref(spec)))]
    p_values = [float(value) for value in (ctypes.c_double * 9).from_address(camera_p(ctypes.byref(spec)))]

    return {
        "schema_version": 1,
        "camera_model": "Xacti",
        "exif_model": "FlyaCam 1 V01",
        "image_size": {"width": int(image_width), "height": int(image_height)},
        "distortion_model": "opencv_fisheye_kannala_brandt",
        "source": {
            "type": "inspector_robotspec_dll",
            "path": str(robotspec_path),
            "function": "flya::CameraSpecDatabase::xactiCamera(QSize)",
        },
        "camera_type": {"name": "Xacti", "enum_value": int(camera_type(ctypes.byref(spec)))},
        "robot_type": {
            "name": "Elios3",
            "enum_value": ELIOS3_ROBOT_TYPE,
            "main_camera_type": robot_main_camera,
        },
        "camera_position_on_robot": camera_position_on_robot,
        "K": matrix3(k_values),
        "D": d_values,
        "P": matrix3(p_values),
        "lens_to_focal_point_distance_m": float(lens_to_focal_point(ctypes.byref(spec))),
        "undistort": {
            "opencv_module": "cv2.fisheye",
            "rectification_rotation": matrix3([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            "balance": 0.0,
            "fov_scale": 1.0,
            "uses_projection_matrix": "P",
        },
    }


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


def default_output_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "calibration"


def load_yaml(path: Path, errors: list[dict[str, Any]]) -> dict[str, Any] | None:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception as exc:  # noqa: BLE001
        errors.append({"path": str(path), "error_type": type(exc).__name__, "message": str(exc)})
        return None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def to_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        return None


def get_dict(data: dict[str, Any] | None, key: str) -> dict[str, Any]:
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, dict) else {}


def get_list(data: dict[str, Any] | None, key: str) -> list[Any]:
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, list) else []


def normalize_quat(qx: Any, qy: Any, qz: Any, qw: Any) -> tuple[float, float, float, float] | None:
    vals = [to_float(qx), to_float(qy), to_float(qz), to_float(qw)]
    if any(value is None for value in vals):
        return None
    x, y, z, w = (float(value) for value in vals if value is not None)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        return None
    return x / norm, y / norm, z / norm, w / norm


def quat_norm(qx: Any, qy: Any, qz: Any, qw: Any) -> float | None:
    vals = [to_float(qx), to_float(qy), to_float(qz), to_float(qw)]
    if any(value is None for value in vals):
        return None
    x, y, z, w = (float(value) for value in vals if value is not None)
    return math.sqrt(x * x + y * y + z * z + w * w)


def quat_to_matrix(qx: Any, qy: Any, qz: Any, qw: Any) -> list[list[float]] | None:
    norm = normalize_quat(qx, qy, qz, qw)
    if norm is None:
        return None
    x, y, z, w = norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def transform_matrix(tx: Any, ty: Any, tz: Any, qx: Any, qy: Any, qz: Any, qw: Any) -> list[list[float]] | None:
    rotation = quat_to_matrix(qx, qy, qz, qw)
    vals = [to_float(tx), to_float(ty), to_float(tz)]
    if rotation is None or any(value is None for value in vals):
        return None
    x, y, z = (float(value) for value in vals if value is not None)
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], x],
        [rotation[1][0], rotation[1][1], rotation[1][2], y],
        [rotation[2][0], rotation[2][1], rotation[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def quat_to_euler_deg(qx: Any, qy: Any, qz: Any, qw: Any) -> tuple[float | None, float | None, float | None]:
    norm = normalize_quat(qx, qy, qz, qw)
    if norm is None:
        return None, None, None
    x, y, z, w = norm
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def pose_columns(tx: Any, ty: Any, tz: Any, qx: Any, qy: Any, qz: Any, qw: Any) -> dict[str, Any]:
    roll, pitch, yaw = quat_to_euler_deg(qx, qy, qz, qw)
    return {
        "translation_x": tx,
        "translation_y": ty,
        "translation_z": tz,
        "rotation_qx": qx,
        "rotation_qy": qy,
        "rotation_qz": qz,
        "rotation_qw": qw,
        "rotation_norm": quat_norm(qx, qy, qz, qw),
        "roll_deg": roll,
        "pitch_deg": pitch,
        "yaw_deg": yaw,
        "matrix_4x4": transform_matrix(tx, ty, tz, qx, qy, qz, qw),
    }


def file_role(path: Path) -> str:
    name = path.name
    if name in CAMERA_FILES:
        return "camera_intrinsics"
    if name == "imu_cam_extrinsics.yaml":
        return "imu_camera_extrinsics"
    if name == "static_transforms.yaml":
        return "static_transforms"
    if name == "geoslam_transforms.yaml":
        return "geoslam_transforms"
    return "yaml"


def load_flights(index_dir: Path) -> dict[str, dict[str, str]]:
    rows = read_csv_dicts(index_dir / "flights.csv")
    return {row.get("flight_id", ""): row for row in rows if row.get("flight_id")}


def flight_info(flights: dict[str, dict[str, str]], flight_id: str) -> dict[str, str]:
    return flights.get(flight_id, {})


def yaml_file_rows(project_root: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted((project_root / "flights").glob("*/*.yaml")):
        flight_id = path.parent.name
        flight = flight_info(flights, flight_id)
        try:
            digest = sha256_file(path)
        except Exception as exc:  # noqa: BLE001
            digest = ""
            errors.append({"path": str(path), "error_type": type(exc).__name__, "message": str(exc)})
        rows.append(
            {
                "flight_id": flight_id,
                "flight_identifier": flight.get("identifier", ""),
                "flight_name": flight.get("name", ""),
                "file": rel_path(path, project_root),
                "file_name": path.name,
                "role": file_role(path),
                "size_bytes": path.stat().st_size,
                "sha256": digest,
            }
        )
    return rows


def camera_intrinsic_rows(project_root: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted((project_root / "flights").glob("*/camera_calibration_*.yaml")):
        data = load_yaml(path, errors)
        if data is None:
            continue
        flight_id = path.parent.name
        flight = flight_info(flights, flight_id)
        matrix = get_list(get_dict(data, "camera_matrix"), "data")
        distortion = get_list(get_dict(data, "distortion_coefficients"), "data")
        camera_index = CAMERA_FILES.get(path.name, "")
        rows.append(
            {
                "flight_id": flight_id,
                "flight_identifier": flight.get("identifier", ""),
                "flight_name": flight.get("name", ""),
                "file": rel_path(path, project_root),
                "camera_index": camera_index,
                "camera_name": data.get("camera_name"),
                "image_width": data.get("image_width"),
                "image_height": data.get("image_height"),
                "distortion_model": data.get("distortion_model"),
                "fx": matrix[0] if len(matrix) > 0 else "",
                "fy": matrix[4] if len(matrix) > 4 else "",
                "cx": matrix[2] if len(matrix) > 2 else "",
                "cy": matrix[5] if len(matrix) > 5 else "",
                "camera_matrix": matrix,
                "distortion_coefficients": distortion,
                "mcu_serial_number": get_dict(data, "header").get("mcu_serial_number"),
                "drone_part_id": get_dict(data, "header").get("drone_part_id"),
            }
        )
    return rows


def camera_extrinsic_rows(project_root: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted((project_root / "flights").glob("*/imu_cam_extrinsics.yaml")):
        data = load_yaml(path, errors)
        if data is None:
            continue
        flight_id = path.parent.name
        flight = flight_info(flights, flight_id)
        for key in ("Camera0", "Camera1", "Camera2"):
            camera = get_dict(data, key)
            quat = get_dict(camera, "quaternion")
            vector = get_dict(camera, "vector")
            camera_index = int(key.replace("Camera", ""))
            tx = vector.get("MrMC_x")
            ty = vector.get("MrMC_y")
            tz = vector.get("MrMC_z")
            qx = quat.get("qCM_x")
            qy = quat.get("qCM_y")
            qz = quat.get("qCM_z")
            qw = quat.get("qCM_w")
            row = {
                "flight_id": flight_id,
                "flight_identifier": flight.get("identifier", ""),
                "flight_name": flight.get("name", ""),
                "file": rel_path(path, project_root),
                "camera_index": camera_index,
                "parent_frame": "imu",
                "child_frame": f"camera_{camera_index}",
                "transform_direction": "imu_to_camera_from_yaml_comment",
                "mcu_serial_number": get_dict(data, "header").get("mcu_serial_number"),
                "drone_part_id": get_dict(data, "header").get("drone_part_id"),
            }
            row.update(pose_columns(tx, ty, tz, qx, qy, qz, qw))
            rows.append(row)
    return rows


def static_transform_rows(project_root: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted((project_root / "flights").glob("*/static_transforms.yaml")):
        data = load_yaml(path, errors)
        if data is None:
            continue
        flight_id = path.parent.name
        flight = flight_info(flights, flight_id)
        header = get_dict(data, "header")
        for group_index, group in enumerate(get_list(data, "transforms"), start=1):
            parent_frame = group.get("parent_frame")
            child_frame = group.get("child_frame")
            alternatives = group.get("transforms") if isinstance(group.get("transforms"), list) else []
            for transform_index, transform in enumerate(alternatives, start=1):
                translation = get_dict(transform, "translation")
                rotation = get_dict(transform, "rotation")
                row = {
                    "flight_id": flight_id,
                    "flight_identifier": flight.get("identifier", ""),
                    "flight_name": flight.get("name", ""),
                    "file": rel_path(path, project_root),
                    "group_index": group_index,
                    "transform_index": transform_index,
                    "parent_frame": parent_frame,
                    "child_frame": child_frame,
                    "conditions": transform.get("conditions", []),
                    "global_version": header.get("global_version"),
                    "bsp": header.get("bsp"),
                    "fw": header.get("fw"),
                    "drone_serial_number": header.get("drone_serial_number"),
                    "current_drone_part_id": header.get("current_drone_part_id"),
                    "original_drone_part_id": header.get("original_drone_part_id"),
                    "autocalib_commit": header.get("autocalib_commit"),
                    "calibration_date": header.get("calibration_date"),
                }
                row.update(
                    pose_columns(
                        translation.get("x"),
                        translation.get("y"),
                        translation.get("z"),
                        rotation.get("qx"),
                        rotation.get("qy"),
                        rotation.get("qz"),
                        rotation.get("qw"),
                    )
                )
                rows.append(row)
    return rows


def geoslam_transform_rows(project_root: Path, flights: dict[str, dict[str, str]], errors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted((project_root / "flights").glob("*/geoslam_transforms.yaml")):
        data = load_yaml(path, errors)
        if data is None:
            continue
        flight_id = path.parent.name
        flight = flight_info(flights, flight_id)
        header = get_dict(data, "header")
        hardware = get_dict(get_dict(get_dict(data, "parameters"), "hardware"), "hardware")
        if not hardware:
            hardware = get_dict(get_dict(data, "parameters"), "hardware")
        raw = hardware.get("imuToLaserTransform")
        raw_list = raw if isinstance(raw, list) else []
        tx = raw_list[0] if len(raw_list) > 0 else ""
        ty = raw_list[1] if len(raw_list) > 1 else ""
        tz = raw_list[2] if len(raw_list) > 2 else ""
        qw = raw_list[3] if len(raw_list) > 3 else ""
        qx = raw_list[4] if len(raw_list) > 4 else ""
        qy = raw_list[5] if len(raw_list) > 5 else ""
        qz = raw_list[6] if len(raw_list) > 6 else ""
        row = {
            "flight_id": flight_id,
            "flight_identifier": flight.get("identifier", ""),
            "flight_name": flight.get("name", ""),
            "file": rel_path(path, project_root),
            "parent_frame": "imu",
            "child_frame": "laser_geoslam",
            "transform_direction": "imu_to_laser_from_key_name",
            "raw_field": "parameters.hardware.imuToLaserTransform",
            "raw_value": raw_list,
            "raw_order_guess": "tx,ty,tz,qw,qx,qy,qz",
            "global_version": header.get("global_version"),
            "bsp": header.get("bsp"),
            "fw": header.get("fw"),
            "drone_serial_number": header.get("drone_serial_number"),
            "current_drone_part_id": header.get("current_drone_part_id"),
            "original_drone_part_id": header.get("original_drone_part_id"),
            "autocalib_commit": header.get("autocalib_commit"),
            "calibration_date": header.get("calibration_date"),
        }
        row.update(pose_columns(tx, ty, tz, qx, qy, qz, qw))
        rows.append(row)
    return rows


def transform_edge_rows(
    static_rows: list[dict[str, Any]],
    camera_rows: list[dict[str, Any]],
    geoslam_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for source, items in (
        ("static_transforms", static_rows),
        ("imu_cam_extrinsics", camera_rows),
        ("geoslam_transforms", geoslam_rows),
    ):
        for row in items:
            rows.append(
                {
                    "source": source,
                    "flight_id": row.get("flight_id"),
                    "flight_identifier": row.get("flight_identifier"),
                    "file": row.get("file"),
                    "parent_frame": row.get("parent_frame"),
                    "child_frame": row.get("child_frame"),
                    "conditions": row.get("conditions", ""),
                    "transform_direction": row.get("transform_direction", "parent_to_child"),
                    "translation_x": row.get("translation_x"),
                    "translation_y": row.get("translation_y"),
                    "translation_z": row.get("translation_z"),
                    "rotation_qx": row.get("rotation_qx"),
                    "rotation_qy": row.get("rotation_qy"),
                    "rotation_qz": row.get("rotation_qz"),
                    "rotation_qw": row.get("rotation_qw"),
                    "roll_deg": row.get("roll_deg"),
                    "pitch_deg": row.get("pitch_deg"),
                    "yaw_deg": row.get("yaw_deg"),
                    "matrix_4x4": row.get("matrix_4x4"),
                }
            )
    return rows


def frame_rows(edge_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    per_flight: dict[tuple[str, str], dict[str, Any]] = {}
    for edge in edge_rows:
        flight_id = str(edge.get("flight_id") or "")
        flight_identifier = edge.get("flight_identifier", "")
        for role, frame in (("parent", edge.get("parent_frame")), ("child", edge.get("child_frame"))):
            if not frame:
                continue
            key = (flight_id, str(frame))
            current = per_flight.setdefault(
                key,
                {
                    "flight_id": flight_id,
                    "flight_identifier": flight_identifier,
                    "frame": frame,
                    "as_parent_count": 0,
                    "as_child_count": 0,
                    "sources": set(),
                },
            )
            if role == "parent":
                current["as_parent_count"] += 1
            else:
                current["as_child_count"] += 1
            current["sources"].add(str(edge.get("source") or ""))
    rows: list[dict[str, Any]] = []
    for row in per_flight.values():
        rows.append(
            {
                "flight_id": row["flight_id"],
                "flight_identifier": row["flight_identifier"],
                "frame": row["frame"],
                "as_parent_count": row["as_parent_count"],
                "as_child_count": row["as_child_count"],
                "sources": sorted(row["sources"]),
            }
        )
    return sorted(rows, key=lambda item: (str(item["flight_identifier"]), str(item["frame"])))


BASE_TRANSFORM_FIELDS = [
    "flight_id",
    "flight_identifier",
    "flight_name",
    "file",
    "parent_frame",
    "child_frame",
    "translation_x",
    "translation_y",
    "translation_z",
    "rotation_qx",
    "rotation_qy",
    "rotation_qz",
    "rotation_qw",
    "rotation_norm",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "matrix_4x4",
]


def build_calibration_outputs(project_root: Path, index_dir: Path, output_dir: Path) -> dict[str, Any]:
    project_root = project_root.resolve()
    index_dir = index_dir.resolve()
    output_dir = output_dir.resolve()
    errors: list[dict[str, Any]] = []
    flights = load_flights(index_dir)

    yaml_rows = yaml_file_rows(project_root, flights, errors)
    intrinsic_rows = camera_intrinsic_rows(project_root, flights, errors)
    extrinsic_rows = camera_extrinsic_rows(project_root, flights, errors)
    static_rows = static_transform_rows(project_root, flights, errors)
    geoslam_rows = geoslam_transform_rows(project_root, flights, errors)
    edges = transform_edge_rows(static_rows, extrinsic_rows, geoslam_rows)
    frames = frame_rows(edges)
    xacti_camera_spec: dict[str, Any] | None = None
    try:
        xacti_camera_spec = read_xacti_camera_spec_from_inspector()
        write_json(output_dir / "xacti_4000x3000_camera_spec.json", xacti_camera_spec)
    except Exception as exc:  # noqa: BLE001
        errors.append({"path": str(DEFAULT_INSPECTOR_DIR / "robotspec.dll"), "error_type": type(exc).__name__, "message": str(exc)})

    write_csv(
        output_dir / "yaml_files.csv",
        yaml_rows,
        ["flight_id", "flight_identifier", "flight_name", "file", "file_name", "role", "size_bytes", "sha256"],
    )
    write_csv(
        output_dir / "camera_intrinsics.csv",
        intrinsic_rows,
        [
            "flight_id",
            "flight_identifier",
            "flight_name",
            "file",
            "camera_index",
            "camera_name",
            "image_width",
            "image_height",
            "distortion_model",
            "fx",
            "fy",
            "cx",
            "cy",
            "camera_matrix",
            "distortion_coefficients",
            "mcu_serial_number",
            "drone_part_id",
        ],
    )
    write_csv(
        output_dir / "camera_extrinsics.csv",
        extrinsic_rows,
        BASE_TRANSFORM_FIELDS
        + ["camera_index", "transform_direction", "mcu_serial_number", "drone_part_id"],
    )
    write_csv(
        output_dir / "static_transforms.csv",
        static_rows,
        BASE_TRANSFORM_FIELDS
        + [
            "group_index",
            "transform_index",
            "conditions",
            "global_version",
            "bsp",
            "fw",
            "drone_serial_number",
            "current_drone_part_id",
            "original_drone_part_id",
            "autocalib_commit",
            "calibration_date",
        ],
    )
    write_csv(
        output_dir / "geoslam_transforms.csv",
        geoslam_rows,
        BASE_TRANSFORM_FIELDS
        + [
            "transform_direction",
            "raw_field",
            "raw_value",
            "raw_order_guess",
            "global_version",
            "bsp",
            "fw",
            "drone_serial_number",
            "current_drone_part_id",
            "original_drone_part_id",
            "autocalib_commit",
            "calibration_date",
        ],
    )
    write_csv(
        output_dir / "transform_edges.csv",
        edges,
        [
            "source",
            "flight_id",
            "flight_identifier",
            "file",
            "parent_frame",
            "child_frame",
            "conditions",
            "transform_direction",
            "translation_x",
            "translation_y",
            "translation_z",
            "rotation_qx",
            "rotation_qy",
            "rotation_qz",
            "rotation_qw",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "matrix_4x4",
        ],
    )
    write_csv(output_dir / "frames.csv", frames, ["flight_id", "flight_identifier", "frame", "as_parent_count", "as_child_count", "sources"])
    write_csv(output_dir / "calibration_errors.csv", errors, ["path", "error_type", "message"])

    hashes_by_name: dict[str, set[str]] = defaultdict(set)
    for row in yaml_rows:
        if row.get("sha256"):
            hashes_by_name[str(row["file_name"])].add(str(row["sha256"]))

    summary = {
        "project_root": str(project_root),
        "index_dir": str(index_dir),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "counts": {
            "yaml_files": len(yaml_rows),
            "camera_intrinsics": len(intrinsic_rows),
            "camera_extrinsics": len(extrinsic_rows),
            "static_transforms": len(static_rows),
            "geoslam_transforms": len(geoslam_rows),
            "transform_edges": len(edges),
            "frames": len(frames),
            "xacti_4000x3000_camera_specs": 1 if xacti_camera_spec is not None else 0,
            "errors": len(errors),
        },
        "yaml_files_by_role": dict(sorted(Counter(row.get("role", "") for row in yaml_rows).items())),
        "unique_hash_count_by_file_name": {name: len(values) for name, values in sorted(hashes_by_name.items())},
        "all_flights_share_same_calibration_files": all(len(values) == 1 for values in hashes_by_name.values()) if hashes_by_name else False,
        "frames": sorted({str(row.get("frame")) for row in frames if row.get("frame")}),
        "xacti_4000x3000_camera_spec_file": "xacti_4000x3000_camera_spec.json" if xacti_camera_spec is not None else "",
    }
    write_json(output_dir / "calibration_summary.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Index calibration YAML and transform graph files.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--index-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if not project_root.exists():
        raise SystemExit(f"Project root does not exist: {project_root}")
    index_dir = args.index_dir.resolve() if args.index_dir else default_index_dir(project_root).resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else default_output_dir(project_root).resolve()
    summary = build_calibration_outputs(project_root, index_dir, output_dir)
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
