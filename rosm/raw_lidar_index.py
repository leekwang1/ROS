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


DEFAULT_LIDAR_TOPICS = (
    "/os_node/lidar_scan/compressed",
    "/os_node/lidar_scan_mask2d",
)


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
    return project_root.parent / "outputs" / "raw_lidar"


def parse_topics(text: str) -> set[str]:
    return {item.strip() for item in text.split(",") if item.strip()}


def ns_to_seconds(value: Any) -> str:
    if value in (None, ""):
        return ""
    try:
        return f"{int(value) / 1_000_000_000:.9f}"
    except Exception:  # noqa: BLE001
        return ""


def attr(obj: Any, name: str, default: Any = None) -> Any:
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def public_field_names(obj: Any) -> list[str]:
    annotations = getattr(type(obj), "__annotations__", None)
    if isinstance(annotations, dict) and annotations:
        return [name for name in annotations if not name.startswith("_")]
    if hasattr(obj, "__dict__"):
        return [name for name in vars(obj) if not name.startswith("_")]
    names: list[str] = []
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception:  # noqa: BLE001
            continue
        if not callable(value):
            names.append(name)
    return names


def binary_bytes(value: Any) -> bytes | None:
    if value is None:
        return None
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, memoryview):
        return value.tobytes()
    if isinstance(value, list) and all(isinstance(item, int) for item in value):
        try:
            return bytes(value)
        except ValueError:
            return None
    return None


def ascii_prefix(data: bytes, max_bytes: int = 32) -> str:
    return "".join(chr(byte) if 32 <= byte <= 126 else "." for byte in data[:max_bytes])


def hex_prefix(data: bytes, max_bytes: int = 64) -> str:
    return data[:max_bytes].hex(" ")


def u32_le_prefix(data: bytes, max_words: int = 8) -> str:
    words: list[str] = []
    limit = min(len(data) // 4, max_words)
    for index in range(limit):
        start = index * 4
        words.append(str(int.from_bytes(data[start : start + 4], "little", signed=False)))
    return ";".join(words)


def magic_name(data: bytes) -> str:
    if data.startswith(b"jfly"):
        return "jfly"
    if data.startswith(b"\x1f\x8b"):
        return "gzip"
    if data.startswith(b"\x28\xb5\x2f\xfd"):
        return "zstd"
    if data.startswith(b"\x04\x22\x4d\x18"):
        return "lz4_frame"
    if data.startswith(b"\x89PNG"):
        return "png"
    if data.startswith(b"\xff\xd8"):
        return "jpeg"
    return ""


def likely_lidar_kind(row: dict[str, str]) -> str:
    topic = row.get("topic", "").lower()
    schema = row.get("schema_name", "").lower()
    if schema.endswith("/packetbytes") or "packetbytes" in schema:
        return "packet_bytes"
    if schema.endswith("/mask2d") or "mask2d" in schema:
        return "mask2d"
    if schema.endswith("/pointcloud2") or "pointcloud2" in schema:
        return "pointcloud2"
    if "lidar_scan" in topic:
        return "lidar_scan"
    return "other_lidar"


def is_lidar_candidate(row: dict[str, str], topics: set[str]) -> bool:
    topic = row.get("topic", "")
    if topics and topic not in topics:
        return False
    if row.get("has_messages", "").lower() not in {"true", "1", "yes"}:
        return False
    text = f"{row.get('topic', '')} {row.get('topic_role', '')} {row.get('schema_name', '')}".lower()
    return any(token in text for token in ("lidar", "ouster", "pointcloud2", "packetbytes", "mask2d", "os_node"))


def load_candidate_rows(mcap_dir: Path, topics: set[str]) -> list[dict[str, str]]:
    rows = read_csv_dicts(mcap_dir / "mcap_channels.csv")
    return [row for row in rows if is_lidar_candidate(row, topics)]


def base_topic_row(row: dict[str, str]) -> dict[str, Any]:
    result = dict(row)
    result["lidar_kind"] = likely_lidar_kind(row)
    return result


def base_message_row(candidate: dict[str, str], message: Any, message_index: int, decoded: Any) -> dict[str, Any]:
    return {
        "mcap_file": candidate.get("mcap_file"),
        "linked_flight_id": candidate.get("linked_flight_id"),
        "linked_flight_identifier": candidate.get("linked_flight_identifier"),
        "channel_id": candidate.get("channel_id"),
        "topic": candidate.get("topic"),
        "lidar_kind": likely_lidar_kind(candidate),
        "schema_name": candidate.get("schema_name"),
        "message_index": message_index,
        "sequence": attr(message, "sequence"),
        "log_time_ns": attr(message, "log_time"),
        "publish_time_ns": attr(message, "publish_time"),
        "log_time_sec": ns_to_seconds(attr(message, "log_time")),
        "publish_time_sec": ns_to_seconds(attr(message, "publish_time")),
        "decoded_type": type(decoded).__name__,
        "top_level_fields": public_field_names(decoded),
    }


def extract_stamp_ns(stamp: Any) -> str:
    sec = attr(stamp, "sec")
    nanosec = attr(stamp, "nanosec")
    if sec in (None, "") or nanosec in (None, ""):
        return ""
    try:
        return str(int(sec) * 1_000_000_000 + int(nanosec))
    except Exception:  # noqa: BLE001
        return ""


def extract_binary_summary(decoded: Any) -> dict[str, Any]:
    fields = public_field_names(decoded)
    for field in ("buf", "data"):
        value = attr(decoded, field)
        data = binary_bytes(value)
        if data is None:
            continue
        return {
            "binary_field": field,
            "binary_length": len(data),
            "magic": magic_name(data),
            "ascii_prefix": ascii_prefix(data),
            "hex_prefix": hex_prefix(data),
            "u32_le_prefix": u32_le_prefix(data),
        }
    for field in fields:
        data = binary_bytes(attr(decoded, field))
        if data is None:
            continue
        return {
            "binary_field": field,
            "binary_length": len(data),
            "magic": magic_name(data),
            "ascii_prefix": ascii_prefix(data),
            "hex_prefix": hex_prefix(data),
            "u32_le_prefix": u32_le_prefix(data),
        }
    return {
        "binary_field": "",
        "binary_length": "",
        "magic": "",
        "ascii_prefix": "",
        "hex_prefix": "",
        "u32_le_prefix": "",
    }


def extract_message_details(decoded: Any) -> dict[str, Any]:
    details: dict[str, Any] = {}
    stamp = attr(decoded, "stamp")
    header = attr(decoded, "header")
    if header is not None:
        stamp = attr(header, "stamp", stamp)
        details["frame_id"] = attr(header, "frame_id")
    details["stamp_ns"] = extract_stamp_ns(stamp)
    details["stamp_sec"] = attr(stamp, "sec")
    details["stamp_nanosec"] = attr(stamp, "nanosec")
    for field in ("rows", "cols", "reserved", "height", "width", "point_step", "row_step", "is_bigendian", "is_dense"):
        value = attr(decoded, field)
        if value is not None:
            details[field] = value
    point_fields = attr(decoded, "fields")
    if point_fields is not None:
        details["point_fields"] = [
            {
                "name": attr(field, "name"),
                "offset": attr(field, "offset"),
                "datatype": attr(field, "datatype"),
                "count": attr(field, "count"),
            }
            for field in point_fields
        ]
    details.update(extract_binary_summary(decoded))
    return details


def summarize_numeric(values: list[int]) -> dict[str, Any]:
    if not values:
        return {"min": "", "max": "", "mean": ""}
    return {
        "min": min(values),
        "max": max(values),
        "mean": f"{sum(values) / len(values):.3f}",
    }


def sample_topic_messages(
    project_root: Path,
    candidate: dict[str, str],
    max_messages_per_topic: int,
    sample_rows: list[dict[str, Any]],
    jsonl_rows: list[dict[str, Any]],
    errors: list[dict[str, Any]],
) -> dict[str, Any]:
    relative_mcap = candidate.get("mcap_file", "")
    topic = candidate.get("topic", "")
    mcap_path = project_root / relative_mcap
    stats = {
        "scanned_messages": 0,
        "first_log_time_ns": "",
        "last_log_time_ns": "",
        "first_publish_time_ns": "",
        "last_publish_time_ns": "",
        "binary_length_min": "",
        "binary_length_max": "",
        "binary_length_mean": "",
        "magic_counts": {},
    }
    if not relative_mcap or not topic:
        return stats
    if not mcap_path.exists():
        errors.append({"mcap_file": relative_mcap, "topic": topic, "error_type": "FileNotFoundError", "message": str(mcap_path)})
        return stats

    lengths: list[int] = []
    magic_counter: Counter[str] = Counter()
    try:
        with mcap_path.open("rb") as fp:
            reader = make_reader(fp, decoder_factories=[DecoderFactory()])
            for _, _, message, decoded in reader.iter_decoded_messages(topics=[topic]):
                message_index = int(stats["scanned_messages"]) + 1
                stats["scanned_messages"] = message_index
                if not stats["first_log_time_ns"]:
                    stats["first_log_time_ns"] = attr(message, "log_time")
                    stats["first_publish_time_ns"] = attr(message, "publish_time")
                stats["last_log_time_ns"] = attr(message, "log_time")
                stats["last_publish_time_ns"] = attr(message, "publish_time")

                base = base_message_row(candidate, message, message_index, decoded)
                details = extract_message_details(decoded)
                length = details.get("binary_length")
                if isinstance(length, int):
                    lengths.append(length)
                magic = str(details.get("magic") or "")
                if magic:
                    magic_counter[magic] += 1

                if max_messages_per_topic <= 0 or message_index <= max_messages_per_topic:
                    row = dict(base)
                    row.update(details)
                    sample_rows.append(row)
                    jsonl_rows.append(row)
                if max_messages_per_topic > 0 and message_index >= max_messages_per_topic:
                    break
    except Exception as exc:  # noqa: BLE001
        errors.append({"mcap_file": relative_mcap, "topic": topic, "error_type": type(exc).__name__, "message": str(exc)})

    length_stats = summarize_numeric(lengths)
    stats["binary_length_min"] = length_stats["min"]
    stats["binary_length_max"] = length_stats["max"]
    stats["binary_length_mean"] = length_stats["mean"]
    stats["magic_counts"] = dict(sorted(magic_counter.items()))
    return stats


def build_raw_lidar_outputs(
    project_root: Path,
    mcap_dir: Path,
    output_dir: Path,
    topics: set[str],
    max_messages_per_topic: int,
) -> dict[str, Any]:
    project_root = project_root.resolve()
    mcap_dir = mcap_dir.resolve()
    output_dir = output_dir.resolve()

    candidates = load_candidate_rows(mcap_dir, topics)
    topic_rows = [base_topic_row(row) for row in candidates]
    message_rows: list[dict[str, Any]] = []
    jsonl_rows: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    stats_rows: list[dict[str, Any]] = []

    for candidate in candidates:
        stats = sample_topic_messages(project_root, candidate, max_messages_per_topic, message_rows, jsonl_rows, errors)
        row = base_topic_row(candidate)
        row.update(stats)
        row["first_log_time_sec"] = ns_to_seconds(stats.get("first_log_time_ns"))
        row["last_log_time_sec"] = ns_to_seconds(stats.get("last_log_time_ns"))
        row["first_publish_time_sec"] = ns_to_seconds(stats.get("first_publish_time_ns"))
        row["last_publish_time_sec"] = ns_to_seconds(stats.get("last_publish_time_ns"))
        stats_rows.append(row)

    topic_fieldnames = [
        "mcap_file",
        "linked_flight_id",
        "linked_flight_identifier",
        "channel_id",
        "topic",
        "lidar_kind",
        "topic_role",
        "message_encoding",
        "schema_id",
        "schema_name",
        "schema_encoding",
        "schema_size_bytes",
        "message_count",
        "has_messages",
    ]
    write_csv(output_dir / "raw_lidar_topics.csv", topic_rows, topic_fieldnames)
    write_csv(
        output_dir / "raw_lidar_topic_stats.csv",
        stats_rows,
        [
            *topic_fieldnames,
            "scanned_messages",
            "first_log_time_ns",
            "last_log_time_ns",
            "first_log_time_sec",
            "last_log_time_sec",
            "first_publish_time_ns",
            "last_publish_time_ns",
            "first_publish_time_sec",
            "last_publish_time_sec",
            "binary_length_min",
            "binary_length_max",
            "binary_length_mean",
            "magic_counts",
        ],
    )
    write_csv(
        output_dir / "raw_lidar_messages.csv",
        message_rows,
        [
            "mcap_file",
            "linked_flight_id",
            "linked_flight_identifier",
            "channel_id",
            "topic",
            "lidar_kind",
            "schema_name",
            "message_index",
            "sequence",
            "log_time_ns",
            "publish_time_ns",
            "log_time_sec",
            "publish_time_sec",
            "decoded_type",
            "top_level_fields",
            "stamp_ns",
            "stamp_sec",
            "stamp_nanosec",
            "frame_id",
            "rows",
            "cols",
            "reserved",
            "height",
            "width",
            "point_step",
            "row_step",
            "is_bigendian",
            "is_dense",
            "point_fields",
            "binary_field",
            "binary_length",
            "magic",
            "ascii_prefix",
            "hex_prefix",
            "u32_le_prefix",
        ],
    )
    write_jsonl(output_dir / "raw_lidar_samples.jsonl", jsonl_rows)
    write_csv(output_dir / "raw_lidar_errors.csv", errors, ["mcap_file", "topic", "error_type", "message"])

    summary = {
        "project_root": str(project_root),
        "mcap_dir": str(mcap_dir),
        "output_dir": str(output_dir),
        "scanned_at_utc": datetime.now(timezone.utc).isoformat(),
        "topics_filter": sorted(topics),
        "max_messages_per_topic": max_messages_per_topic,
        "counts": {
            "candidate_topics": len(candidates),
            "topic_stats_rows": len(stats_rows),
            "message_rows": len(message_rows),
            "errors": len(errors),
        },
        "topics_by_kind": dict(sorted(Counter(row.get("lidar_kind", "") for row in topic_rows).items())),
        "messages_by_topic": dict(sorted(Counter(row.get("topic", "") for row in message_rows).items())),
        "magic_by_message": dict(sorted(Counter(row.get("magic", "") for row in message_rows).items())),
        "errors_by_type": dict(sorted(Counter(row.get("error_type", "") for row in errors).items())),
    }
    write_json(output_dir / "raw_lidar_summary.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Index raw LiDAR-like MCAP messages.")
    parser.add_argument("project_root", nargs="?", type=Path, default=default_project_root())
    parser.add_argument("--mcap-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--topics", default=",".join(DEFAULT_LIDAR_TOPICS))
    parser.add_argument("--max-messages-per-topic", type=int, default=0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if not project_root.exists():
        raise SystemExit(f"Project root does not exist: {project_root}")
    if args.max_messages_per_topic < 0:
        raise SystemExit("--max-messages-per-topic must be >= 0")

    mcap_dir = args.mcap_dir.resolve() if args.mcap_dir else default_mcap_dir(project_root).resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else default_output_dir(project_root).resolve()
    topics = parse_topics(args.topics)
    summary = build_raw_lidar_outputs(project_root, mcap_dir, output_dir, topics, args.max_messages_per_topic)
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))
    print(f"Output: {summary['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
