from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from mcap.reader import make_reader


def ensure_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
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


def rel_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def iso_mtime(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat()


def ns_to_seconds(value: Any) -> str:
    if value in (None, ""):
        return ""
    try:
        return f"{int(value) / 1_000_000_000:.9f}"
    except Exception:  # noqa: BLE001
        return ""


def ns_span_seconds(start_ns: Any, end_ns: Any) -> str:
    if start_ns in (None, "") or end_ns in (None, ""):
        return ""
    try:
        return f"{(int(end_ns) - int(start_ns)) / 1_000_000_000:.9f}"
    except Exception:  # noqa: BLE001
        return ""


def compression_ratio(compressed_size: int, uncompressed_size: int) -> str:
    if not compressed_size or not uncompressed_size:
        return ""
    return f"{compressed_size / uncompressed_size:.6f}"


def decode_preview(data: bytes, max_chars: int = 300) -> str:
    text = data[:4000].decode("utf-8", errors="replace")
    text = re.sub(r"\s+", " ", text).strip()
    return text[:max_chars]


def read_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        return list(csv.DictReader(fp))


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2] / "sepoong"


def default_index_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "index"


def default_output_dir(project_root: Path) -> Path:
    return project_root.parent / "outputs" / "mcap"


def topic_role(topic: str, schema_name: str = "") -> str:
    text = f"{topic} {schema_name}".lower()
    if any(token in text for token in ("alert", "log", "diagnostic")):
        return "log_or_alert"
    if "point_of_interest" in text or "snapshot" in text:
        return "snapshot_or_poi"
    if topic in {"/tf", "/tf_static"}:
        return "tf_transform"
    if any(token in text for token in ("image", "camera", "rgb", "video")):
        return "camera"
    if "/imu" in text or text.endswith("imu") or "lidar_imu" in text or "inertial" in text:
        return "imu"
    if any(token in text for token in ("pointcloud", "point_cloud", "points", "ouster", "lidar_scan", "os_node", "laser")):
        return "lidar"
    if any(token in text for token in ("pose", "odom", "odometry", "trajectory", "tf2")):
        return "pose_or_odometry"
    if any(token in text for token in ("vio", "slam", "localization", "geoslam", "map")):
        return "localization_or_map"
    if any(token in text for token in ("battery", "motor", "esc", "flight", "state", "status")):
        return "robot_state"
    if any(token in text for token in ("alert", "log", "diagnostic")):
        return "log_or_alert"
    if any(token in text for token in ("control", "cmd", "setpoint", "automation", "autonomous")):
        return "control"
    return "other"


def candidate_reason(role: str) -> str:
    reasons = {
        "lidar": "LiDAR scan/frame 후보",
        "pose_or_odometry": "드론 위치/자세 후보",
        "tf_transform": "좌표계 transform 후보",
        "imu": "IMU/동작 센서 후보",
        "camera": "카메라 영상/촬영 시간 후보",
        "snapshot_or_poi": "snapshot/POI 이벤트 후보",
        "localization_or_map": "SLAM/localization 보조 후보",
    }
    return reasons.get(role, "")


def load_flight_lookup(index_dir: Path) -> dict[str, dict[str, str]]:
    rows = read_csv_dicts(index_dir / "flights.csv")
    lookup: dict[str, dict[str, str]] = {}
    for row in rows:
        flight_id = row.get("flight_id", "")
        folder = row.get("folder", "")
        if flight_id:
            lookup[flight_id] = row
        if folder:
            lookup[Path(folder).name] = row
    return lookup


def load_mcap_paths(project_root: Path, index_dir: Path) -> list[dict[str, Any]]:
    indexed = read_csv_dicts(index_dir / "mcap_files.csv")
    rows: list[dict[str, Any]] = []
    if indexed:
        for row in indexed:
            rel = row.get("mcap_file") or row.get("file") or row.get("relative_path")
            if not rel:
                continue
            rows.append(
                {
                    "relative_path": rel.replace("\\", "/"),
                    "linked_flight_id": row.get("linked_flight_id", ""),
                    "linked_flight_identifier": row.get("linked_flight_identifier", ""),
                }
            )
        return rows

    for path in sorted(project_root.rglob("*.mcap")):
        rows.append(
            {
                "relative_path": rel_path(path, project_root),
                "linked_flight_id": path.parent.name,
                "linked_flight_identifier": "",
            }
        )
    return rows


def schema_message_counts(channels: dict[int, Any], counts_by_channel: dict[int, int]) -> Counter[int]:
    counts: Counter[int] = Counter()
    for channel_id, count in counts_by_channel.items():
        channel = channels.get(channel_id)
        if channel:
            counts[channel.schema_id] += count
    return counts


def summarize_mcap(
    project_root: Path,
    mcap_info: dict[str, Any],
    flight_lookup: dict[str, dict[str, str]],
    errors: list[dict[str, Any]],
) -> dict[str, list[dict[str, Any]]]:
    relative_path = str(mcap_info["relative_path"])
    mcap_path = project_root / relative_path
    file_rows: list[dict[str, Any]] = []
    channel_rows: list[dict[str, Any]] = []
    schema_rows: list[dict[str, Any]] = []
    chunk_rows: list[dict[str, Any]] = []
    metadata_rows: list[dict[str, Any]] = []

    if not mcap_path.exists():
        errors.append({"mcap_file": relative_path, "error_type": "FileNotFoundError", "message": str(mcap_path)})
        return {
            "files": file_rows,
            "channels": channel_rows,
            "schemas": schema_rows,
            "chunks": chunk_rows,
            "metadata": metadata_rows,
        }

    flight_id = mcap_info.get("linked_flight_id") or mcap_path.parent.name
    flight = flight_lookup.get(str(flight_id), {})
    flight_identifier = mcap_info.get("linked_flight_identifier") or flight.get("identifier", "")

    try:
        with mcap_path.open("rb") as fp:
            reader = make_reader(fp)
            header = reader.get_header()
            summary = reader.get_summary()
            schemas = summary.schemas or {}
            channels = summary.channels or {}
            statistics = summary.statistics
            chunk_indexes = ensure_list(summary.chunk_indexes)
            metadata_indexes = ensure_list(summary.metadata_indexes)
            attachment_indexes = ensure_list(summary.attachment_indexes)
            counts_by_channel = dict(statistics.channel_message_counts) if statistics else {}
            counts_by_schema = schema_message_counts(channels, counts_by_channel)
            channel_count_with_messages = sum(1 for count in counts_by_channel.values() if count > 0)
            schema_ids_with_messages = {channels[channel_id].schema_id for channel_id, count in counts_by_channel.items() if count > 0 and channel_id in channels}
            compression_types = sorted({chunk.compression or "none" for chunk in chunk_indexes})

            file_rows.append(
                {
                    "mcap_file": relative_path,
                    "file_name": mcap_path.name,
                    "folder": rel_path(mcap_path.parent, project_root),
                    "linked_flight_id": flight_id,
                    "linked_flight_identifier": flight_identifier,
                    "profile": header.profile,
                    "library": header.library,
                    "size_bytes": mcap_path.stat().st_size,
                    "modified_utc": iso_mtime(mcap_path),
                    "message_count": statistics.message_count if statistics else "",
                    "message_start_time_ns": statistics.message_start_time if statistics else "",
                    "message_end_time_ns": statistics.message_end_time if statistics else "",
                    "message_start_time_sec": ns_to_seconds(statistics.message_start_time if statistics else None),
                    "message_end_time_sec": ns_to_seconds(statistics.message_end_time if statistics else None),
                    "duration_sec": ns_span_seconds(
                        statistics.message_start_time if statistics else None,
                        statistics.message_end_time if statistics else None,
                    ),
                    "summary_channel_count": len(channels),
                    "stats_channel_count": statistics.channel_count if statistics else "",
                    "channel_count_with_messages": channel_count_with_messages,
                    "summary_schema_count": len(schemas),
                    "stats_schema_count": statistics.schema_count if statistics else "",
                    "schema_count_with_messages": len(schema_ids_with_messages),
                    "chunk_count": statistics.chunk_count if statistics else len(chunk_indexes),
                    "metadata_count": statistics.metadata_count if statistics else len(metadata_indexes),
                    "attachment_count": statistics.attachment_count if statistics else len(attachment_indexes),
                    "compression_types": compression_types,
                }
            )

            channel_usage_by_schema: Counter[int] = Counter()
            for channel in channels.values():
                channel_usage_by_schema[channel.schema_id] += 1

            for schema_id, schema in sorted(schemas.items()):
                schema_rows.append(
                    {
                        "mcap_file": relative_path,
                        "linked_flight_id": flight_id,
                        "linked_flight_identifier": flight_identifier,
                        "schema_id": schema_id,
                        "schema_name": schema.name,
                        "schema_encoding": schema.encoding,
                        "schema_size_bytes": len(schema.data),
                        "channel_count": channel_usage_by_schema.get(schema_id, 0),
                        "message_count": counts_by_schema.get(schema_id, 0),
                        "schema_preview": decode_preview(schema.data),
                    }
                )

            for channel_id, channel in sorted(channels.items()):
                schema = schemas.get(channel.schema_id)
                schema_name = schema.name if schema else ""
                schema_encoding = schema.encoding if schema else ""
                message_count = int(counts_by_channel.get(channel_id, 0))
                channel_rows.append(
                    {
                        "mcap_file": relative_path,
                        "linked_flight_id": flight_id,
                        "linked_flight_identifier": flight_identifier,
                        "channel_id": channel_id,
                        "topic": channel.topic,
                        "topic_role": topic_role(channel.topic, schema_name),
                        "message_encoding": channel.message_encoding,
                        "schema_id": channel.schema_id,
                        "schema_name": schema_name,
                        "schema_encoding": schema_encoding,
                        "schema_size_bytes": len(schema.data) if schema else "",
                        "message_count": message_count,
                        "has_messages": message_count > 0,
                        "channel_metadata": channel.metadata,
                    }
                )

            for index, chunk in enumerate(chunk_indexes, start=1):
                channel_ids = sorted(int(channel_id) for channel_id in chunk.message_index_offsets.keys())
                chunk_rows.append(
                    {
                        "mcap_file": relative_path,
                        "linked_flight_id": flight_id,
                        "linked_flight_identifier": flight_identifier,
                        "chunk_number": index,
                        "chunk_start_offset": chunk.chunk_start_offset,
                        "chunk_length": chunk.chunk_length,
                        "message_start_time_ns": chunk.message_start_time,
                        "message_end_time_ns": chunk.message_end_time,
                        "message_start_time_sec": ns_to_seconds(chunk.message_start_time),
                        "message_end_time_sec": ns_to_seconds(chunk.message_end_time),
                        "duration_sec": ns_span_seconds(chunk.message_start_time, chunk.message_end_time),
                        "compression": chunk.compression or "none",
                        "compressed_size": chunk.compressed_size,
                        "uncompressed_size": chunk.uncompressed_size,
                        "compression_ratio": compression_ratio(chunk.compressed_size, chunk.uncompressed_size),
                        "message_index_length": chunk.message_index_length,
                        "channel_ids": channel_ids,
                    }
                )

            for metadata_index in metadata_indexes:
                metadata_rows.append(
                    {
                        "mcap_file": relative_path,
                        "linked_flight_id": flight_id,
                        "linked_flight_identifier": flight_identifier,
                        "metadata_name": metadata_index.name,
                        "metadata_offset": metadata_index.offset,
                        "metadata_length": metadata_index.length,
                        "metadata_key_count": "",
                        "metadata": "",
                    }
                )

        try:
            with mcap_path.open("rb") as fp:
                reader = make_reader(fp)
                records = list(reader.iter_metadata())
            records_by_name = {record.name: record for record in records}
            for row in metadata_rows:
                record = records_by_name.get(str(row["metadata_name"]))
                if record:
                    row["metadata_key_count"] = len(record.metadata)
                    row["metadata"] = record.metadata
        except Exception as exc:  # noqa: BLE001
            errors.append({"mcap_file": relative_path, "error_type": type(exc).__name__, "message": f"metadata read failed: {exc}"})

    except Exception as exc:  # noqa: BLE001
        errors.append({"mcap_file": relative_path, "error_type": type(exc).__name__, "message": str(exc)})

    return {
        "files": file_rows,
        "channels": channel_rows,
        "schemas": schema_rows,
        "chunks": chunk_rows,
        "metadata": metadata_rows,
    }


def build_inventory(project_root: Path, index_dir: Path, output_dir: Path) -> dict[str, Any]:
    project_root = project_root.resolve()
    index_dir = index_dir.resolve()
    output_dir = output_dir.resolve()

    errors: list[dict[str, Any]] = []
    flight_lookup = load_flight_lookup(index_dir)
    mcap_infos = load_mcap_paths(project_root, index_dir)

    file_rows: list[dict[str, Any]] = []
    channel_rows: list[dict[str, Any]] = []
    schema_rows: list[dict[str, Any]] = []
    chunk_rows: list[dict[str, Any]] = []
    metadata_rows: list[dict[str, Any]] = []

    for mcap_info in mcap_infos:
        result = summarize_mcap(project_root, mcap_info, flight_lookup, errors)
        file_rows.extend(result["files"])
        channel_rows.extend(result["channels"])
        schema_rows.extend(result["schemas"])
        chunk_rows.extend(result["chunks"])
        metadata_rows.extend(result["metadata"])

    write_csv(
        output_dir / "mcap_files_detail.csv",
        file_rows,
        [
            "mcap_file",
            "file_name",
            "folder",
            "linked_flight_id",
            "linked_flight_identifier",
            "profile",
            "library",
            "size_bytes",
            "modified_utc",
            "message_count",
            "message_start_time_ns",
            "message_end_time_ns",
            "message_start_time_sec",
            "message_end_time_sec",
            "duration_sec",
            "summary_channel_count",
            "stats_channel_count",
            "channel_count_with_messages",
            "summary_schema_count",
            "stats_schema_count",
            "schema_count_with_messages",
            "chunk_count",
            "metadata_count",
            "attachment_count",
            "compression_types",
        ],
    )
    write_csv(
        output_dir / "mcap_channels.csv",
        channel_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "topic_role",
            "message_encoding",
            "schema_id",
            "schema_name",
            "schema_encoding",
            "schema_size_bytes",
            "message_count",
            "has_messages",
            "channel_metadata",
        ],
    )
    candidate_rows = [
        {
            "mcap_file": row.get("mcap_file"),
            "linked_flight_id": row.get("linked_flight_id"),
            "linked_flight_identifier": row.get("linked_flight_identifier"),
            "channel_id": row.get("channel_id"),
            "topic": row.get("topic"),
            "topic_role": row.get("topic_role"),
            "candidate_reason": candidate_reason(str(row.get("topic_role") or "")),
            "message_encoding": row.get("message_encoding"),
            "schema_id": row.get("schema_id"),
            "schema_name": row.get("schema_name"),
            "schema_encoding": row.get("schema_encoding"),
            "message_count": row.get("message_count"),
        }
        for row in channel_rows
        if row.get("has_messages")
        and row.get("topic_role") in {"lidar", "pose_or_odometry", "tf_transform", "imu", "camera", "snapshot_or_poi", "localization_or_map"}
    ]
    write_csv(
        output_dir / "mcap_topic_candidates.csv",
        candidate_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "topic_role",
            "candidate_reason",
            "message_encoding",
            "schema_id",
            "schema_name",
            "schema_encoding",
            "message_count",
        ],
    )
    write_csv(
        output_dir / "mcap_schemas.csv",
        schema_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "schema_id",
            "schema_name",
            "schema_encoding",
            "schema_size_bytes",
            "channel_count",
            "message_count",
            "schema_preview",
        ],
    )
    write_csv(
        output_dir / "mcap_chunks.csv",
        chunk_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "chunk_number",
            "chunk_start_offset",
            "chunk_length",
            "message_start_time_ns",
            "message_end_time_ns",
            "message_start_time_sec",
            "message_end_time_sec",
            "duration_sec",
            "compression",
            "compressed_size",
            "uncompressed_size",
            "compression_ratio",
            "message_index_length",
            "channel_ids",
        ],
    )
    write_csv(
        output_dir / "mcap_metadata.csv",
        metadata_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "metadata_name",
            "metadata_offset",
            "metadata_length",
            "metadata_key_count",
            "metadata",
        ],
    )
    write_csv(output_dir / "mcap_errors.csv", errors, ["mcap_file", "error_type", "message"])

    topic_role_counts = Counter(row.get("topic_role", "") for row in channel_rows)
    profile_counts = Counter(row.get("profile", "") for row in file_rows)
    message_encoding_counts = Counter(row.get("message_encoding", "") for row in channel_rows)
    schema_encoding_counts = Counter(row.get("schema_encoding", "") for row in schema_rows)
    compression_counts = Counter(row.get("compression", "") for row in chunk_rows)

    summary = {
        "project_root": str(project_root),
        "index_dir": str(index_dir),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "counts": {
            "mcap_files": len(file_rows),
            "channels": len(channel_rows),
            "channels_with_messages": sum(1 for row in channel_rows if row.get("has_messages")),
            "topic_candidates": len(candidate_rows),
            "schemas": len(schema_rows),
            "chunks": len(chunk_rows),
            "metadata_records": len(metadata_rows),
            "errors": len(errors),
            "total_messages": sum(int(row.get("message_count") or 0) for row in file_rows),
            "total_size_bytes": sum(int(row.get("size_bytes") or 0) for row in file_rows),
        },
        "profile_counts": dict(sorted(profile_counts.items())),
        "message_encoding_counts": dict(sorted(message_encoding_counts.items())),
        "schema_encoding_counts": dict(sorted(schema_encoding_counts.items())),
        "compression_counts": dict(sorted(compression_counts.items())),
        "topic_role_counts": dict(sorted(topic_role_counts.items())),
    }
    write_json(output_dir / "mcap_inventory_summary.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="List MCAP channels, schemas, chunks, and metadata.")
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
    summary = build_inventory(project_root, index_dir, output_dir)
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
