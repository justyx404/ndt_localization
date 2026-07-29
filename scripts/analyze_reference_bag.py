#!/usr/bin/env python3
"""
Validate reference-pose reconstruction directly from a ROS 2 bag.

This offline path is intentionally independent of DDS replay and the localizer
under test. It remains complete even when the baseline localizer blocks a live
replay or exceeds its wall timeout.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import shlex
import time
from datetime import datetime, timezone

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from localization_benchmark_evaluator import (
    compose_pose,
    distribution,
    interpolate_pose,
    pose_error,
    pose_from_ros,
    pose_from_transform,
    stamp_seconds,
)


REFERENCE_TOPICS = (
    "/tf",
    "/odometry_lio",
    "/odometry_map",
    "/initialpose",
)


def finite_or_none(value):
    """Convert non-finite floats recursively for standards-compliant JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_or_none(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_or_none(item) for item in value]
    return value


def write_csv(path, rows, fieldnames):
    with open(path, "w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(
            output, fieldnames=fieldnames, extrasaction="ignore"
        )
        writer.writeheader()
        writer.writerows(rows)


def read_metadata(bag_path):
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    with open(metadata_path, "r", encoding="utf-8") as metadata_file:
        metadata = yaml.safe_load(metadata_file)["rosbag2_bagfile_information"]
    return metadata


def read_reference_topics(
    bag_path,
    storage_id,
    map_frame,
    odom_frame,
    base_frame,
):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(REFERENCE_TOPICS)))
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in REFERENCE_TOPICS
        if topic in topic_types
    }

    map_to_odom = []
    odom_to_base = []
    recorded_map_odometry = []
    recorded_initial_poses = []
    topic_counts = {topic: 0 for topic in REFERENCE_TOPICS}

    while reader.has_next():
        topic, serialized, storage_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        topic_counts[topic] += 1
        message = deserialize_message(serialized, message_types[topic])
        if topic == "/tf":
            for transform in message.transforms:
                parent = transform.header.frame_id.strip("/")
                child = transform.child_frame_id.strip("/")
                if parent == map_frame and child == odom_frame:
                    map_to_odom.append(
                        (
                            stamp_seconds(transform.header.stamp),
                            pose_from_transform(transform.transform),
                        )
                    )
        elif topic == "/odometry_lio":
            parent = message.header.frame_id.strip("/")
            child = message.child_frame_id.strip("/")
            if parent == odom_frame and child == base_frame:
                odom_to_base.append(
                    (
                        stamp_seconds(message.header.stamp),
                        pose_from_ros(message.pose.pose),
                    )
                )
        elif topic == "/odometry_map":
            recorded_map_odometry.append(
                (
                    stamp_seconds(message.header.stamp),
                    pose_from_ros(message.pose.pose),
                )
            )
        elif topic == "/initialpose":
            recorded_initial_poses.append(
                {
                    "header_timestamp": stamp_seconds(message.header.stamp),
                    "storage_timestamp": storage_timestamp_ns * 1.0e-9,
                    "pose": pose_from_ros(message.pose.pose),
                }
            )

    map_to_odom.sort(key=lambda item: item[0])
    odom_to_base.sort(key=lambda item: item[0])
    recorded_map_odometry.sort(key=lambda item: item[0])
    recorded_initial_poses.sort(key=lambda item: item["storage_timestamp"])
    return {
        "map_to_odom": map_to_odom,
        "odom_to_base": odom_to_base,
        "recorded_map_odometry": recorded_map_odometry,
        "recorded_initial_poses": recorded_initial_poses,
        "topic_counts": topic_counts,
    }


def reconstruct_at(data, timestamp, max_gap):
    map_to_odom = interpolate_pose(
        data["map_to_odom"], timestamp, max_gap
    )
    odom_to_base = interpolate_pose(
        data["odom_to_base"], timestamp, max_gap
    )
    if map_to_odom is None or odom_to_base is None:
        return None
    return compose_pose(map_to_odom, odom_to_base)


def analyze_data(data, max_gap, pose_threshold_m, yaw_threshold_deg):
    reference_rows = []
    for timestamp, recorded_pose in data["recorded_map_odometry"]:
        reconstructed = reconstruct_at(data, timestamp, max_gap)
        error = (
            pose_error(recorded_pose, reconstructed)
            if reconstructed is not None
            else (float("nan"), float("nan"))
        )
        reference_rows.append(
            {
                "timestamp": timestamp,
                "reference_x": recorded_pose[0][0],
                "reference_y": recorded_pose[0][1],
                "reference_z": recorded_pose[0][2],
                "reconstructed_x": (
                    reconstructed[0][0] if reconstructed else float("nan")
                ),
                "reconstructed_y": (
                    reconstructed[0][1] if reconstructed else float("nan")
                ),
                "reconstructed_z": (
                    reconstructed[0][2] if reconstructed else float("nan")
                ),
                "translation_error_m": error[0],
                "rotation_error_deg": error[1],
                "reconstruction_available": reconstructed is not None,
            }
        )

    initial_pose_rows = []
    for sample in data["recorded_initial_poses"]:
        header_reference = reconstruct_at(
            data, sample["header_timestamp"], max_gap
        )
        storage_reference = reconstruct_at(
            data, sample["storage_timestamp"], max_gap
        )
        header_error = (
            pose_error(header_reference, sample["pose"])
            if header_reference is not None
            else (float("nan"), float("nan"))
        )
        storage_error = (
            pose_error(storage_reference, sample["pose"])
            if storage_reference is not None
            else (float("nan"), float("nan"))
        )
        candidates = []
        if all(math.isfinite(value) for value in header_error):
            candidates.append(("header", header_error))
        if all(math.isfinite(value) for value in storage_error):
            candidates.append(("storage", storage_error))
        if candidates:
            selected_basis, selected_error = min(
                candidates,
                key=lambda item: item[1][0]
                + math.radians(item[1][1]),
            )
        else:
            selected_basis = "unavailable"
            selected_error = (float("nan"), float("nan"))
        matches_reference = (
            math.isfinite(selected_error[0])
            and math.isfinite(selected_error[1])
            and selected_error[0] <= pose_threshold_m
            and selected_error[1] <= yaw_threshold_deg
        )
        initial_pose_rows.append(
            {
                "header_timestamp": sample["header_timestamp"],
                "storage_timestamp": sample["storage_timestamp"],
                "header_storage_delta_ms": (
                    sample["storage_timestamp"]
                    - sample["header_timestamp"]
                )
                * 1000.0,
                "header_translation_error_m": header_error[0],
                "header_rotation_error_deg": header_error[1],
                "storage_translation_error_m": storage_error[0],
                "storage_rotation_error_deg": storage_error[1],
                "selected_timestamp_basis": selected_basis,
                "selected_translation_error_m": selected_error[0],
                "selected_rotation_error_deg": selected_error[1],
                "matches_reference_threshold": matches_reference,
            }
        )
    return reference_rows, initial_pose_rows


def analysis_digest(reference_rows, initial_pose_rows):
    canonical = json.dumps(
        finite_or_none(
            {
                "reference_rows": reference_rows,
                "initial_pose_rows": initial_pose_rows,
            }
        ),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def build_summary(
    args,
    metadata,
    data,
    reference_rows,
    initial_pose_rows,
    elapsed_seconds,
    repeatability_digests,
):
    reference_translation = [
        row["translation_error_m"] for row in reference_rows
    ]
    reference_rotation = [
        row["rotation_error_deg"] for row in reference_rows
    ]
    selected_initial_translation = [
        row["selected_translation_error_m"] for row in initial_pose_rows
    ]
    selected_initial_rotation = [
        row["selected_rotation_error_deg"] for row in initial_pose_rows
    ]
    matching_initial_poses = sum(
        1
        for row in initial_pose_rows
        if row["matches_reference_threshold"]
    )
    timestamp_basis_counts = {}
    for row in initial_pose_rows:
        basis = row["selected_timestamp_basis"]
        timestamp_basis_counts[basis] = (
            timestamp_basis_counts.get(basis, 0) + 1
        )
    command_parts = [
        "ros2",
        "run",
        "ndt_localization",
        "analyze_reference_bag.py",
        "--bag-path",
        args.bag_path,
        "--output-directory",
        args.output_directory,
        "--repeatability-runs",
        str(args.repeatability_runs),
        "--max-interpolation-gap-seconds",
        str(args.max_interpolation_gap_seconds),
        "--pose-match-threshold-m",
        str(args.pose_match_threshold_m),
        "--rotation-match-threshold-deg",
        str(args.rotation_match_threshold_deg),
        "--map-frame-id",
        args.map_frame_id,
        "--odom-frame-id",
        args.odom_frame_id,
        "--base-frame-id",
        args.base_frame_id,
    ]
    command = " ".join(shlex.quote(part) for part in command_parts)
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "bag": {
            "path": args.bag_path,
            "storage_id": metadata["storage_identifier"],
            "duration_seconds": metadata["duration"]["nanoseconds"] * 1.0e-9,
            "message_count": metadata["message_count"],
            "command": command,
        },
        "frames": {
            "map": args.map_frame_id,
            "odom": args.odom_frame_id,
            "base": args.base_frame_id,
        },
        "topic_counts": data["topic_counts"],
        "usable_samples": {
            "map_to_odom": len(data["map_to_odom"]),
            "odom_to_base": len(data["odom_to_base"]),
            "recorded_map_odometry": len(
                data["recorded_map_odometry"]
            ),
            "recorded_initial_pose": len(
                data["recorded_initial_poses"]
            ),
        },
        "reference_reconstruction": {
            "translation_error_m": distribution(reference_translation),
            "rotation_error_deg": distribution(reference_rotation),
            "available": sum(
                1
                for row in reference_rows
                if row["reconstruction_available"]
            ),
            "unavailable": sum(
                1
                for row in reference_rows
                if not row["reconstruction_available"]
            ),
        },
        "recorded_initial_pose_check": {
            "interpretation": (
                "Recorded initial poses are search priors, not ground truth."
            ),
            "thresholds": {
                "translation_m": args.pose_match_threshold_m,
                "rotation_deg": args.rotation_match_threshold_deg,
            },
            "matching_reference": matching_initial_poses,
            "not_matching_reference": (
                len(initial_pose_rows) - matching_initial_poses
            ),
            "selected_timestamp_basis_counts": timestamp_basis_counts,
            "header_storage_delta_ms": distribution(
                [
                    row["header_storage_delta_ms"]
                    for row in initial_pose_rows
                ]
            ),
            "header_translation_error_m": distribution(
                [
                    row["header_translation_error_m"]
                    for row in initial_pose_rows
                ]
            ),
            "header_rotation_error_deg": distribution(
                [
                    row["header_rotation_error_deg"]
                    for row in initial_pose_rows
                ]
            ),
            "storage_translation_error_m": distribution(
                [
                    row["storage_translation_error_m"]
                    for row in initial_pose_rows
                ]
            ),
            "storage_rotation_error_deg": distribution(
                [
                    row["storage_rotation_error_deg"]
                    for row in initial_pose_rows
                ]
            ),
            "selected_translation_error_m": distribution(
                selected_initial_translation
            ),
            "selected_rotation_error_deg": distribution(
                selected_initial_rotation
            ),
        },
        "repeatability": {
            "runs": args.repeatability_runs,
            "identical": len(set(repeatability_digests)) == 1,
            "digests": repeatability_digests,
        },
        "analysis_elapsed_seconds": elapsed_seconds,
    }


def write_report(path, summary):
    reference_translation = summary["reference_reconstruction"][
        "translation_error_m"
    ]
    reference_rotation = summary["reference_reconstruction"][
        "rotation_error_deg"
    ]
    initial_pose = summary["recorded_initial_pose_check"]

    def formatted(metrics, key):
        value = metrics.get(key)
        return "n/a" if value is None else "%.9f" % value

    lines = [
        "# Offline reference validation",
        "",
        "Bag: `%s`" % summary["bag"]["path"],
        "",
        "| Metric | p50 | p95 | maximum |",
        "|---|---:|---:|---:|",
        "| Reconstruction translation error (m) | %s | %s | %s |"
        % tuple(
            formatted(reference_translation, key)
            for key in ("p50", "p95", "maximum")
        ),
        "| Reconstruction rotation error (deg) | %s | %s | %s |"
        % tuple(
            formatted(reference_rotation, key)
            for key in ("p50", "p95", "maximum")
        ),
        "",
        "- Available reconstructions: %d"
        % summary["reference_reconstruction"]["available"],
        "- Unavailable reconstructions: %d"
        % summary["reference_reconstruction"]["unavailable"],
        "- Repeatability runs byte-identical: %s"
        % ("yes" if summary["repeatability"]["identical"] else "no"),
        "- Recorded initial poses matching reference threshold: %d/%d"
        % (
            initial_pose["matching_reference"],
            initial_pose["matching_reference"]
            + initial_pose["not_matching_reference"],
        ),
        "- Selected timestamp bases: `%s`"
        % json.dumps(
            initial_pose["selected_timestamp_basis_counts"],
            sort_keys=True,
        ),
        "",
        "Recorded initial poses are evaluated as search priors, not as "
        "independent ground truth.",
        "",
        "Reproduction command:",
        "",
        "```bash",
        summary["bag"]["command"],
        "```",
        "",
    ]
    with open(path, "w", encoding="utf-8") as output:
        output.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-path", required=True)
    parser.add_argument("--output-directory", required=True)
    parser.add_argument("--repeatability-runs", type=int, default=2)
    parser.add_argument(
        "--max-interpolation-gap-seconds", type=float, default=0.25
    )
    parser.add_argument("--pose-match-threshold-m", type=float, default=0.5)
    parser.add_argument(
        "--rotation-match-threshold-deg", type=float, default=5.0
    )
    parser.add_argument("--map-frame-id", default="map")
    parser.add_argument("--odom-frame-id", default="odom_lidar")
    parser.add_argument("--base-frame-id", default="base_link")
    args = parser.parse_args()

    args.bag_path = os.path.abspath(args.bag_path)
    args.output_directory = os.path.abspath(args.output_directory)
    if args.repeatability_runs < 1:
        parser.error("--repeatability-runs must be at least 1")
    if args.max_interpolation_gap_seconds <= 0.0:
        parser.error("--max-interpolation-gap-seconds must be positive")
    if args.pose_match_threshold_m < 0.0:
        parser.error("--pose-match-threshold-m cannot be negative")
    if args.rotation_match_threshold_deg < 0.0:
        parser.error("--rotation-match-threshold-deg cannot be negative")
    metadata = read_metadata(args.bag_path)
    os.makedirs(args.output_directory, exist_ok=True)

    start = time.monotonic()
    repeatability_digests = []
    first_data = None
    first_reference_rows = None
    first_initial_pose_rows = None
    for _ in range(args.repeatability_runs):
        data = read_reference_topics(
            args.bag_path,
            metadata["storage_identifier"],
            args.map_frame_id.strip("/"),
            args.odom_frame_id.strip("/"),
            args.base_frame_id.strip("/"),
        )
        reference_rows, initial_pose_rows = analyze_data(
            data,
            args.max_interpolation_gap_seconds,
            args.pose_match_threshold_m,
            args.rotation_match_threshold_deg,
        )
        repeatability_digests.append(
            analysis_digest(reference_rows, initial_pose_rows)
        )
        if first_data is None:
            first_data = data
            first_reference_rows = reference_rows
            first_initial_pose_rows = initial_pose_rows

    elapsed_seconds = time.monotonic() - start
    write_csv(
        os.path.join(
            args.output_directory, "reference_pose_metrics.csv"
        ),
        first_reference_rows,
        (
            "timestamp",
            "reference_x",
            "reference_y",
            "reference_z",
            "reconstructed_x",
            "reconstructed_y",
            "reconstructed_z",
            "translation_error_m",
            "rotation_error_deg",
            "reconstruction_available",
        ),
    )
    write_csv(
        os.path.join(args.output_directory, "initial_pose_checks.csv"),
        first_initial_pose_rows,
        (
            "header_timestamp",
            "storage_timestamp",
            "header_storage_delta_ms",
            "header_translation_error_m",
            "header_rotation_error_deg",
            "storage_translation_error_m",
            "storage_rotation_error_deg",
            "selected_timestamp_basis",
            "selected_translation_error_m",
            "selected_rotation_error_deg",
            "matches_reference_threshold",
        ),
    )
    summary = build_summary(
        args,
        metadata,
        first_data,
        first_reference_rows,
        first_initial_pose_rows,
        elapsed_seconds,
        repeatability_digests,
    )
    with open(
        os.path.join(args.output_directory, "summary.json"),
        "w",
        encoding="utf-8",
    ) as output:
        json.dump(summary, output, indent=2, sort_keys=True, allow_nan=False)
        output.write("\n")
    write_report(
        os.path.join(args.output_directory, "report.md"), summary
    )
    print(
        "Reference validation %s; wrote %s"
        % (
            (
                "is repeatable"
                if summary["repeatability"]["identical"]
                else "is NOT repeatable"
            ),
            args.output_directory,
        )
    )
    return 0 if summary["repeatability"]["identical"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
