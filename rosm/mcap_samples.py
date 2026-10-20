from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


DEFAULT_ROLES = ("lidar", "pose_or_odometry", "tf_transform", "snapshot_or_poi", "imu", "camera")
MAX_JSON_CHARS = 8000


def ensure_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    return [value]


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


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def read_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        return list(csv.DictReader(fp))


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2] / "sepoong"


def default_mcap_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "mcap"


def default_output_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "mcap_samples"


def parse_roles(text: str) -> set[str]:
    return {item.strip() for item in text.split(",") if item.strip()}


def ns_to_seconds(value: Any) -> str:
    if value in (None, ""):
        return ""
    try:
        return f"{int(value) / 1_000_000_000:.9f}"
    except Exception:  # noqa: BLE001
        return ""


def public_field_names(obj: Any) -> list[str]:
    if obj is None:
        return []
    annotations = getattr(type(obj), "__annotations__", None)
    if isinstance(annotations, dict) and annotations:
        names = [name for name in annotations if not name.startswith("_")]
        if names:
            return names
    if hasattr(obj, "__dict__"):
        names = [name for name in vars(obj) if not name.startswith("_")]
        if names:
            return names
    names: list[str] = []
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception:  # noqa: BLE001
            continue
        if callable(value):
            continue
        names.append(name)
    return names


def attr(obj: Any, name: str, default: Any = None) -> Any:
    if obj is None:
        return default
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def scalar_type_name(value: Any) -> str:
    if isinstance(value, bytes):
        return f"bytes[{len(value)}]"
    if isinstance(value, (list, tuple)):
        return f"list[{len(value)}]"
    return type(value).__name__


def summarize_bytes(data: bytes, max_bytes: int = 64) -> dict[str, Any]:
    prefix = data[:max_bytes]
    ascii_prefix = "".join(chr(byte) if 32 <= byte <= 126 else "." for byte in prefix)
    return {
        "type": "bytes",
        "length": len(data),
        "hex_prefix": prefix.hex(" "),
        "ascii_prefix": ascii_prefix,
    }


def safe_to_dict(value: Any, depth: int = 0, max_depth: int = 5, max_items: int = 8, max_bytes: int = 64) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, bytes):
        return summarize_bytes(value, max_bytes=max_bytes)
    if depth >= max_depth:
        return f"<{type(value).__name__}>"
    if isinstance(value, dict):
        result: dict[str, Any] = {}
        for index, (key, item) in enumerate(value.items()):
            if index >= max_items:
                result["..."] = f"{len(value) - max_items} more"
                break
            result[str(key)] = safe_to_dict(item, depth + 1, max_depth, max_items, max_bytes)
        return result
    if isinstance(value, (list, tuple)):
        return {
            "type": "list",
            "length": len(value),
            "items": [safe_to_dict(item, depth + 1, max_depth, max_items, max_bytes) for item in list(value)[:max_items]],
        }
    fields = public_field_names(value)
    if fields:
        return {
            name: safe_to_dict(attr(value, name), depth + 1, max_depth, max_items, max_bytes)
            for name in fields[:max_items]
        }
    return repr(value)


def truncate_text(text: str, max_chars: int = MAX_JSON_CHARS) -> str:
    text = re.sub(r"\s+", " ", text).strip()
    if len(text) <= max_chars:
        return text
    return text[:max_chars] + "...<truncated>"


def field_paths(value: Any, prefix: str = "", depth: int = 0, max_depth: int = 5, max_items: int = 3) -> list[dict[str, Any]]:
    if value is None or isinstance(value, (str, int, float, bool, bytes)):
        return [{"field_path": prefix or ".", "field_type": scalar_type_name(value), "example": short_example(value)}]
    if depth >= max_depth:
        return [{"field_path": prefix or ".", "field_type": type(value).__name__, "example": f"<{type(value).__name__}>"}]
    if isinstance(value, dict):
        rows: list[dict[str, Any]] = []
        for key, item in list(value.items())[:max_items]:
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            rows.extend(field_paths(item, child_prefix, depth + 1, max_depth, max_items))
        return rows
    if isinstance(value, (list, tuple)):
        rows = [{"field_path": prefix or ".", "field_type": f"list[{len(value)}]", "example": ""}]
        for index, item in enumerate(list(value)[:max_items]):
            child_prefix = f"{prefix}[{index}]" if prefix else f"[{index}]"
            rows.extend(field_paths(item, child_prefix, depth + 1, max_depth, max_items))
        return rows
    rows = []
    for name in public_field_names(value)[:max_items * 3]:
        child_prefix = f"{prefix}.{name}" if prefix else name
        rows.extend(field_paths(attr(value, name), child_prefix, depth + 1, max_depth, max_items))
    return rows or [{"field_path": prefix or ".", "field_type": type(value).__name__, "example": short_example(value)}]


def short_example(value: Any, max_chars: int = 120) -> str:
    if isinstance(value, bytes):
        summary = summarize_bytes(value, max_bytes=16)
        return f"len={summary['length']} hex={summary['hex_prefix']}"
    if isinstance(value, (list, tuple)):
        return f"len={len(value)}"
    text = repr(value)
    if len(text) > max_chars:
        return text[:max_chars] + "..."
    return text


def stamp_to_ns(stamp: Any) -> int | None:
    sec = attr(stamp, "sec")
    nanosec = attr(stamp, "nanosec")
    if sec is None or nanosec is None:
        return None
    try:
        return int(sec) * 1_000_000_000 + int(nanosec)
    except Exception:  # noqa: BLE001
        return None


def vector_fields(prefix: str, vector: Any) -> dict[str, Any]:
    return {
        f"{prefix}_x": attr(vector, "x"),
        f"{prefix}_y": attr(vector, "y"),
        f"{prefix}_z": attr(vector, "z"),
    }


def quaternion_fields(prefix: str, quat: Any) -> dict[str, Any]:
    return {
        f"{prefix}_qx": attr(quat, "x"),
        f"{prefix}_qy": attr(quat, "y"),
        f"{prefix}_qz": attr(quat, "z"),
        f"{prefix}_qw": attr(quat, "w"),
    }


def extract_pose_like(decoded: Any) -> dict[str, Any]:
    row: dict[str, Any] = {}
    header = attr(decoded, "header")
    stamp = attr(header, "stamp")
    row["header_stamp_ns"] = stamp_to_ns(stamp)
    row["header_stamp_sec"] = attr(stamp, "sec")
    row["header_stamp_nanosec"] = attr(stamp, "nanosec")
    row["frame_id"] = attr(header, "frame_id")
    row["child_frame_id"] = attr(decoded, "child_frame_id")
    pose = attr(decoded, "pose")
    if pose is not None:
        row.update(vector_fields("position", attr(pose, "position")))
        row.update(quaternion_fields("orientation", attr(pose, "orientation")))
    twist = attr(decoded, "twist")
    if twist is not None:
        row.update(vector_fields("linear_velocity", attr(twist, "linear")))
        row.update(vector_fields("angular_velocity", attr(twist, "angular")))
    row["confidence"] = attr(decoded, "confidence")
    return row


def extract_tf_like(decoded: Any) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    transforms = ensure_list(attr(decoded, "transforms"))
    for index, transform_stamped in enumerate(transforms):
        header = attr(transform_stamped, "header")
        stamp = attr(header, "stamp")
        transform = attr(transform_stamped, "transform")
        row: dict[str, Any] = {
            "transform_index": index,
            "transform_count": len(transforms),
            "header_stamp_ns": stamp_to_ns(stamp),
            "header_stamp_sec": attr(stamp, "sec"),
            "header_stamp_nanosec": attr(stamp, "nanosec"),
            "frame_id": attr(header, "frame_id"),
            "child_frame_id": attr(transform_stamped, "child_frame_id"),
        }
        row.update(vector_fields("position", attr(transform, "translation")))
        row.update(quaternion_fields("orientation", attr(transform, "rotation")))
        rows.append(row)
    return rows


def extract_binary(decoded: Any) -> dict[str, Any]:
    for field_name in ("buf", "data"):
        value = attr(decoded, field_name)
        if isinstance(value, bytes):
            summary = summarize_bytes(value, max_bytes=32)
            return {
                "binary_field": field_name,
                "binary_length": summary["length"],
                "hex_prefix": summary["hex_prefix"],
                "ascii_prefix": summary["ascii_prefix"],
                "format": attr(decoded, "format"),
            }
    return {}


def load_candidate_rows(mcap_dir: Path, roles: set[str]) -> list[dict[str, str]]:
    path = mcap_dir / "mcap_topic_candidates.csv"
    rows = read_csv_dicts(path)
    if not rows:
        rows = read_csv_dicts(mcap_dir / "mcap_channels.csv")
    selected: list[dict[str, str]] = []
    for row in rows:
        if row.get("topic_role") not in roles:
            continue
        if str(row.get("has_messages", "True")).lower() == "false":
            continue
        selected.append(row)
    selected.sort(key=lambda row: (row.get("mcap_file", ""), row.get("topic_role", ""), row.get("topic", "")))
    return selected


def make_base_row(candidate: dict[str, str], message: Any, sample_index: int, decoded: Any) -> dict[str, Any]:
    return {
        "mcap_file": candidate.get("mcap_file"),
        "linked_flight_id": candidate.get("linked_flight_id"),
        "linked_flight_identifier": candidate.get("linked_flight_identifier"),
        "channel_id": candidate.get("channel_id"),
        "topic": candidate.get("topic"),
        "topic_role": candidate.get("topic_role"),
        "schema_id": candidate.get("schema_id"),
        "schema_name": candidate.get("schema_name"),
        "schema_encoding": candidate.get("schema_encoding"),
        "message_encoding": candidate.get("message_encoding"),
        "sample_index": sample_index,
        "sequence": getattr(message, "sequence", ""),
        "log_time_ns": getattr(message, "log_time", ""),
        "publish_time_ns": getattr(message, "publish_time", ""),
        "log_time_sec": ns_to_seconds(getattr(message, "log_time", "")),
        "publish_time_sec": ns_to_seconds(getattr(message, "publish_time", "")),
        "decoded_type": type(decoded).__name__,
        "top_level_fields": public_field_names(decoded),
    }


def sample_one_topic(
    project_root: Path,
    candidate: dict[str, str],
    samples_per_topic: int,
    rows: list[dict[str, Any]],
    jsonl_rows: list[dict[str, Any]],
    field_rows: list[dict[str, Any]],
    pose_rows: list[dict[str, Any]],
    binary_rows: list[dict[str, Any]],
    errors: list[dict[str, Any]],
) -> None:
    relative_mcap = candidate.get("mcap_file", "")
    topic = candidate.get("topic", "")
    if not relative_mcap or not topic:
        return
    mcap_path = project_root / relative_mcap
    if not mcap_path.exists():
        errors.append({"mcap_file": relative_mcap, "topic": topic, "error_type": "FileNotFoundError", "message": str(mcap_path)})
        return

    sample_count = 0
    try:
        with mcap_path.open("rb") as fp:
            reader = make_reader(fp, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages(topics=[topic]):
                sample_count += 1
                base = make_base_row(candidate, message, sample_count, decoded)
                decoded_summary = safe_to_dict(decoded)
                decoded_json = json.dumps(decoded_summary, ensure_ascii=False, separators=(",", ":"))
                sample_row = dict(base)
                sample_row["decoded_preview"] = truncate_text(decoded_json)
                rows.append(sample_row)
                jsonl_row = dict(base)
                jsonl_row["decoded"] = decoded_summary
                jsonl_rows.append(jsonl_row)

                for field in field_paths(decoded):
                    field_row = dict(base)
                    field_row.update(field)
                    field_rows.append(field_row)

                if candidate.get("topic_role") == "pose_or_odometry":
                    pose_row = dict(base)
                    pose_row.update(extract_pose_like(decoded))
                    pose_rows.append(pose_row)
                elif candidate.get("topic_role") == "tf_transform":
                    for tf_row in extract_tf_like(decoded):
                        pose_row = dict(base)
                        pose_row.update(tf_row)
                        pose_rows.append(pose_row)

                binary = extract_binary(decoded)
                if binary:
                    binary_row = dict(base)
                    binary_row.update(binary)
                    binary_rows.append(binary_row)

                if sample_count >= samples_per_topic:
                    break
    except Exception as exc:  # noqa: BLE001
        errors.append({"mcap_file": relative_mcap, "topic": topic, "error_type": type(exc).__name__, "message": str(exc)})
    if sample_count == 0:
        errors.append({"mcap_file": relative_mcap, "topic": topic, "error_type": "NoSampleFound", "message": "No decoded sample found for topic"})


def build_samples(project_root: Path, mcap_dir: Path, output_dir: Path, roles: set[str], samples_per_topic: int) -> dict[str, Any]:
    project_root = project_root.resolve()
    mcap_dir = mcap_dir.resolve()
    output_dir = output_dir.resolve()

    candidates = load_candidate_rows(mcap_dir, roles)
    rows: list[dict[str, Any]] = []
    jsonl_rows: list[dict[str, Any]] = []
    field_rows: list[dict[str, Any]] = []
    pose_rows: list[dict[str, Any]] = []
    binary_rows: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []

    for candidate in candidates:
        sample_one_topic(
            project_root=project_root,
            candidate=candidate,
            samples_per_topic=samples_per_topic,
            rows=rows,
            jsonl_rows=jsonl_rows,
            field_rows=field_rows,
            pose_rows=pose_rows,
            binary_rows=binary_rows,
            errors=errors,
        )

    write_csv(
        output_dir / "mcap_topic_samples.csv",
        rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "topic_role",
            "schema_id",
            "schema_name",
            "schema_encoding",
            "message_encoding",
            "sample_index",
            "sequence",
            "log_time_ns",
            "publish_time_ns",
            "log_time_sec",
            "publish_time_sec",
            "decoded_type",
            "top_level_fields",
            "decoded_preview",
        ],
    )
    write_jsonl(output_dir / "mcap_topic_samples.jsonl", jsonl_rows)
    write_csv(
        output_dir / "mcap_topic_fields.csv",
        field_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "topic_role",
            "schema_name",
            "sample_index",
            "field_path",
            "field_type",
            "example",
        ],
    )
    write_csv(
        output_dir / "mcap_pose_samples.csv",
        pose_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "topic_role",
            "schema_name",
            "sample_index",
            "sequence",
            "log_time_ns",
            "publish_time_ns",
            "header_stamp_ns",
            "header_stamp_sec",
            "header_stamp_nanosec",
            "frame_id",
            "child_frame_id",
            "transform_index",
            "transform_count",
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
        output_dir / "mcap_binary_samples.csv",
        binary_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "topic_role",
            "schema_name",
            "sample_index",
            "sequence",
            "log_time_ns",
            "publish_time_ns",
            "binary_field",
            "binary_length",
            "hex_prefix",
            "ascii_prefix",
            "format",
        ],
    )
    write_csv(output_dir / "mcap_sample_errors.csv", errors, ["mcap_file", "topic", "error_type", "message"])

    summary = {
        "project_root": str(project_root),
        "mcap_dir": str(mcap_dir),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "roles": sorted(roles),
        "samples_per_topic": samples_per_topic,
        "counts": {
            "candidate_topics": len(candidates),
            "sample_rows": len(rows),
            "field_rows": len(field_rows),
            "pose_rows": len(pose_rows),
            "binary_rows": len(binary_rows),
            "errors": len(errors),
        },
        "samples_by_role": dict(sorted(Counter(row.get("topic_role", "") for row in rows).items())),
        "samples_by_schema": dict(sorted(Counter(row.get("schema_name", "") for row in rows).items())),
        "samples_by_topic": dict(sorted(Counter(row.get("topic", "") for row in rows).items())),
        "errors_by_type": dict(sorted(Counter(row.get("error_type", "") for row in errors).items())),
    }
    write_json(output_dir / "mcap_sample_summary.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Decode sample messages from selected MCAP topics.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--mcap-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--roles", default=",".join(DEFAULT_ROLES))
    parser.add_argument("--samples-per-topic", type=int, default=1)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if not project_root.exists():
        raise SystemExit(f"Project root does not exist: {project_root}")
    if args.samples_per_topic < 1:
        raise SystemExit("--samples-per-topic must be >= 1")

    mcap_dir = args.mcap_dir.resolve() if args.mcap_dir else default_mcap_dir(project_root).resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else default_output_dir(project_root).resolve()
    roles = parse_roles(args.roles)
    summary = build_samples(project_root, mcap_dir, output_dir, roles, args.samples_per_topic)
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
