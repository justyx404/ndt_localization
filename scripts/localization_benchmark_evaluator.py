#!/usr/bin/env python3
"""
Evaluate an isolated ndt_localization bag replay.

Recorded map corrections are consumed only from the /reference namespace.  The
node reconstructs map->base_link from recorded map->odom_lidar and
odom_lidar->base_link samples, then compares that reconstruction with the
recorded map odometry and the localizer-under-test output.
"""

import bisect
import csv
import hashlib
import json
import math
import os
import shlex
from collections import Counter
from datetime import datetime, timezone

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in q)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def rotate_vector(q, vector):
    x, y, z, w = normalize_quaternion(q)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_slerp(left, right, fraction):
    left = normalize_quaternion(left)
    right = normalize_quaternion(right)
    dot = sum(a * b for a, b in zip(left, right))
    if dot < 0.0:
        right = tuple(-value for value in right)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(
            tuple(a + fraction * (b - a) for a, b in zip(left, right))
        )
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    left_scale = math.sin((1.0 - fraction) * theta) / sin_theta
    right_scale = math.sin(fraction * theta) / sin_theta
    return normalize_quaternion(
        tuple(
            left_scale * a + right_scale * b
            for a, b in zip(left, right)
        )
    )


def compose_pose(left, right):
    left_position, left_quaternion = left
    right_position, right_quaternion = right
    rotated = rotate_vector(left_quaternion, right_position)
    position = tuple(a + b for a, b in zip(left_position, rotated))
    quaternion = quaternion_multiply(left_quaternion, right_quaternion)
    return position, quaternion


def pose_error(reference, estimate):
    translation = math.sqrt(
        sum((a - b) ** 2 for a, b in zip(reference[0], estimate[0]))
    )
    dot = abs(
        sum(a * b for a, b in zip(
            normalize_quaternion(reference[1]),
            normalize_quaternion(estimate[1]),
        ))
    )
    dot = min(1.0, max(-1.0, dot))
    rotation_degrees = math.degrees(2.0 * math.acos(dot))
    return translation, rotation_degrees


def pose_from_ros(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        normalize_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ),
    )


def pose_from_transform(transform):
    return (
        (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        normalize_quaternion(
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
        ),
    )


def interpolate_pose(samples, timestamp, max_gap_seconds):
    if not samples:
        return None
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, timestamp)
    if index < len(samples) and abs(samples[index][0] - timestamp) <= 1.0e-9:
        return samples[index][1]
    if index == 0 or index == len(samples):
        return None
    before_time, before_pose = samples[index - 1]
    after_time, after_pose = samples[index]
    if timestamp - before_time > max_gap_seconds:
        return None
    if after_time - timestamp > max_gap_seconds:
        return None
    span = after_time - before_time
    if span <= 0.0:
        return before_pose
    fraction = (timestamp - before_time) / span
    position = tuple(
        a + fraction * (b - a)
        for a, b in zip(before_pose[0], after_pose[0])
    )
    quaternion = quaternion_slerp(
        before_pose[1], after_pose[1], fraction
    )
    return position, quaternion


def percentile(values, percentile_value):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    rank = (len(values) - 1) * percentile_value / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return values[lower]
    return values[lower] + (rank - lower) * (values[upper] - values[lower])


def distribution(values):
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return {"count": 0}
    return {
        "count": len(finite_values),
        "p50": percentile(finite_values, 50.0),
        "p90": percentile(finite_values, 90.0),
        "p95": percentile(finite_values, 95.0),
        "p99": percentile(finite_values, 99.0),
        "p99_9": percentile(finite_values, 99.9),
        "maximum": max(finite_values),
    }


class LocalizationBenchmarkEvaluator(Node):
    def __init__(self):
        super().__init__("localization_benchmark_evaluator")
        self.declare_parameter("output_directory", "/tmp/ndt_localization_benchmark")
        self.declare_parameter("run_name", "benchmark")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("replay_rate", 1.0)
        self.declare_parameter("start_offset", 0.0)
        self.declare_parameter("max_interpolation_gap_seconds", 0.25)
        self.declare_parameter("deadline_ms", 80.0)
        self.declare_parameter("config_file", "")
        self.declare_parameter("initial_pose_delay", 10.0)
        self.declare_parameter("translation_x", 0.0)
        self.declare_parameter("translation_y", 0.0)
        self.declare_parameter("translation_z", 0.0)
        self.declare_parameter("yaw_degrees", 0.0)
        self.declare_parameter("position_sigma", 0.25)
        self.declare_parameter("yaw_sigma_degrees", 5.0)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom_lidar")
        self.declare_parameter("base_frame_id", "base_link")

        self.output_directory = self.get_parameter(
            "output_directory"
        ).get_parameter_value().string_value
        self.run_name = self.get_parameter("run_name").value
        self.bag_path = self.get_parameter("bag_path").value
        self.replay_rate = float(self.get_parameter("replay_rate").value)
        self.start_offset = float(self.get_parameter("start_offset").value)
        self.max_gap = float(
            self.get_parameter("max_interpolation_gap_seconds").value
        )
        self.deadline_ms = float(self.get_parameter("deadline_ms").value)
        self.config_file = self.get_parameter("config_file").value
        self.initial_pose_parameters = {
            "delay_seconds": float(
                self.get_parameter("initial_pose_delay").value
            ),
            "translation_x": float(self.get_parameter("translation_x").value),
            "translation_y": float(self.get_parameter("translation_y").value),
            "translation_z": float(self.get_parameter("translation_z").value),
            "yaw_degrees": float(self.get_parameter("yaw_degrees").value),
            "position_sigma": float(
                self.get_parameter("position_sigma").value
            ),
            "yaw_sigma_degrees": float(
                self.get_parameter("yaw_sigma_degrees").value
            ),
        }
        self.map_frame = self.get_parameter("map_frame_id").value.strip("/")
        self.odom_frame = self.get_parameter("odom_frame_id").value.strip("/")
        self.base_frame = self.get_parameter("base_frame_id").value.strip("/")

        self.map_to_odom = []
        self.localization_map_to_odom = []
        self.odom_to_base = []
        self.recorded_map_odometry = []
        self.localization_odometry = []
        self.recorded_initial_poses = []
        self.synthetic_initial_poses = []
        self.scan_metrics = []
        self.state_events = []
        self.finalized = False

        qos = QoSProfile(depth=2000)
        self.create_subscription(
            TFMessage, "/reference/tf", self.reference_tf_callback, qos
        )
        self.create_subscription(
            TFMessage,
            "/benchmark/tf",
            self.localization_tf_callback,
            qos,
        )
        self.create_subscription(
            Odometry, "/odometry_lio", self.odometry_callback, qos
        )
        self.create_subscription(
            Odometry,
            "/reference/odometry_map",
            self.recorded_map_odometry_callback,
            qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/reference/initialpose",
            self.recorded_initial_pose_callback,
            qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/benchmark/initialpose",
            self.synthetic_initial_pose_callback,
            qos,
        )
        self.create_subscription(
            Odometry,
            "/benchmark/odometry_map",
            self.localization_odometry_callback,
            qos,
        )
        self.create_subscription(
            DiagnosticArray,
            "/benchmark/scan_diagnostics",
            self.scan_diagnostic_callback,
            qos,
        )
        self.get_logger().info(
            "Collecting isolated replay metrics for '%s'" % self.run_name
        )

    def reference_tf_callback(self, message):
        for transform in message.transforms:
            parent = transform.header.frame_id.strip("/")
            child = transform.child_frame_id.strip("/")
            if parent == self.map_frame and child == self.odom_frame:
                self.map_to_odom.append(
                    (
                        stamp_seconds(transform.header.stamp),
                        pose_from_transform(transform.transform),
                    )
                )

    def localization_tf_callback(self, message):
        for transform in message.transforms:
            parent = transform.header.frame_id.strip("/")
            child = transform.child_frame_id.strip("/")
            if parent == self.map_frame and child == self.odom_frame:
                self.localization_map_to_odom.append(
                    (
                        stamp_seconds(transform.header.stamp),
                        pose_from_transform(transform.transform),
                    )
                )

    def odometry_callback(self, message):
        parent = message.header.frame_id.strip("/")
        child = message.child_frame_id.strip("/")
        if parent == self.odom_frame and child == self.base_frame:
            self.odom_to_base.append(
                (
                    stamp_seconds(message.header.stamp),
                    pose_from_ros(message.pose.pose),
                )
            )

    def recorded_map_odometry_callback(self, message):
        self.recorded_map_odometry.append(
            (
                stamp_seconds(message.header.stamp),
                pose_from_ros(message.pose.pose),
            )
        )

    def localization_odometry_callback(self, message):
        self.localization_odometry.append(
            (
                stamp_seconds(message.header.stamp),
                pose_from_ros(message.pose.pose),
            )
        )

    def recorded_initial_pose_callback(self, message):
        self.recorded_initial_poses.append(
            (
                stamp_seconds(message.header.stamp),
                pose_from_ros(message.pose.pose),
            )
        )

    def synthetic_initial_pose_callback(self, message):
        self.synthetic_initial_poses.append(
            (
                stamp_seconds(message.header.stamp),
                pose_from_ros(message.pose.pose),
            )
        )

    def scan_diagnostic_callback(self, message):
        timestamp = stamp_seconds(message.header.stamp)
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if status.name == "ndt_localization/state":
                self.state_events.append(
                    {
                        "timestamp": timestamp,
                        "reason": values.get("reason", status.message),
                        "state": values.get("state", "UNKNOWN"),
                        "correction_valid": (
                            values.get(
                                "correction_valid", "false"
                            ).lower()
                            == "true"
                        ),
                        "confirmation_count": int(
                            values.get("confirmation_count", "0")
                        ),
                        "odometry_buffer_samples": int(
                            values.get("odometry_buffer_samples", "0")
                        ),
                    }
                )
                continue
            if status.name != "ndt_localization/scan":
                continue
            row = {
                "timestamp": timestamp,
                "decision": values.get("decision", status.message),
                "accepted": values.get("accepted", "false").lower() == "true",
                "converged": values.get("converged", "false").lower() == "true",
                "state": values.get("state", "UNKNOWN"),
                "correction_valid": (
                    values.get("correction_valid", "false").lower() == "true"
                ),
            }
            for key in (
                "conversion_ms",
                "local_map_ms",
                "matcher_ms",
                "validation_ms",
                "total_ms",
                "input_age_ms",
                "odometry_before_gap_ms",
                "odometry_after_gap_ms",
                "translation_delta_m",
                "rotation_delta_deg",
                "fitness_score",
            ):
                try:
                    row[key] = float(values.get(key, "nan"))
                except ValueError:
                    row[key] = float("nan")
            for key in (
                "scan_points_raw",
                "scan_points_filtered",
                "map_points_raw",
                "target_points",
                "iterations",
                "confirmation_count",
                "consecutive_rejections",
            ):
                try:
                    row[key] = int(values.get(key, "0"))
                except ValueError:
                    row[key] = 0
            self.scan_metrics.append(row)

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        os.makedirs(self.output_directory, exist_ok=True)
        self.map_to_odom.sort(key=lambda item: item[0])
        self.localization_map_to_odom.sort(key=lambda item: item[0])
        self.odom_to_base.sort(key=lambda item: item[0])
        self.recorded_map_odometry.sort(key=lambda item: item[0])
        self.localization_odometry.sort(key=lambda item: item[0])
        self.recorded_initial_poses.sort(key=lambda item: item[0])
        self.synthetic_initial_poses.sort(key=lambda item: item[0])
        self.scan_metrics.sort(key=lambda item: item["timestamp"])
        self.state_events.sort(key=lambda item: item["timestamp"])

        reference_rows = self._reference_rows()
        initial_pose_rows = self._initial_pose_rows()
        self._write_csv(
            "reference_pose_metrics.csv",
            reference_rows,
            (
                "timestamp",
                "reference_x",
                "reference_y",
                "reference_z",
                "reconstructed_x",
                "reconstructed_y",
                "reconstructed_z",
                "reconstruction_translation_error_m",
                "reconstruction_rotation_error_deg",
                "localization_x",
                "localization_y",
                "localization_z",
                "localization_translation_error_m",
                "localization_rotation_error_deg",
            ),
        )
        self._write_csv(
            "initial_pose_checks.csv",
            initial_pose_rows,
            (
                "timestamp",
                "translation_error_m",
                "rotation_error_deg",
                "reconstruction_available",
            ),
        )
        self._write_csv(
            "scan_metrics.csv",
            self.scan_metrics,
            (
                "timestamp",
                "decision",
                "accepted",
                "converged",
                "state",
                "correction_valid",
                "conversion_ms",
                "local_map_ms",
                "matcher_ms",
                "validation_ms",
                "total_ms",
                "input_age_ms",
                "odometry_before_gap_ms",
                "odometry_after_gap_ms",
                "translation_delta_m",
                "rotation_delta_deg",
                "scan_points_raw",
                "scan_points_filtered",
                "map_points_raw",
                "target_points",
                "iterations",
                "confirmation_count",
                "consecutive_rejections",
                "fitness_score",
            ),
        )
        self._write_csv(
            "state_events.csv",
            self.state_events,
            (
                "timestamp",
                "reason",
                "state",
                "correction_valid",
                "confirmation_count",
                "odometry_buffer_samples",
            ),
        )

        summary = self._build_summary(reference_rows, initial_pose_rows)
        summary_path = os.path.join(self.output_directory, "summary.json")
        with open(summary_path, "w", encoding="utf-8") as output:
            json.dump(summary, output, indent=2, sort_keys=True, allow_nan=False)
            output.write("\n")
        self._write_markdown_summary(summary)
        if rclpy.ok():
            self.get_logger().info(
                "Wrote benchmark results to %s" % self.output_directory
            )

    def _reference_rows(self):
        rows = []
        evaluation_start = (
            self.synthetic_initial_poses[0][0]
            if self.synthetic_initial_poses
            else None
        )
        for timestamp, recorded_pose in self.recorded_map_odometry:
            map_to_odom = interpolate_pose(
                self.map_to_odom, timestamp, self.max_gap
            )
            odom_to_base = interpolate_pose(
                self.odom_to_base, timestamp, self.max_gap
            )
            reconstructed = (
                compose_pose(map_to_odom, odom_to_base)
                if map_to_odom is not None and odom_to_base is not None
                else None
            )
            local_pose = None
            # The odometry sample stamped exactly at the prior time may have
            # been published before the reset callback processed that prior.
            if evaluation_start is not None and timestamp > evaluation_start:
                local_pose = interpolate_pose(
                    self.localization_odometry, timestamp, self.max_gap
                )
            reconstruction_error = (
                pose_error(recorded_pose, reconstructed)
                if reconstructed is not None
                else (float("nan"), float("nan"))
            )
            localization_error = (
                pose_error(recorded_pose, local_pose)
                if local_pose is not None
                else (float("nan"), float("nan"))
            )
            rows.append(
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
                    "reconstruction_translation_error_m": reconstruction_error[0],
                    "reconstruction_rotation_error_deg": reconstruction_error[1],
                    "localization_x": (
                        local_pose[0][0] if local_pose else float("nan")
                    ),
                    "localization_y": (
                        local_pose[0][1] if local_pose else float("nan")
                    ),
                    "localization_z": (
                        local_pose[0][2] if local_pose else float("nan")
                    ),
                    "localization_translation_error_m": localization_error[0],
                    "localization_rotation_error_deg": localization_error[1],
                }
            )
        return rows

    def _initial_pose_rows(self):
        rows = []
        for timestamp, initial_pose in self.recorded_initial_poses:
            map_to_odom = interpolate_pose(
                self.map_to_odom, timestamp, self.max_gap
            )
            odom_to_base = interpolate_pose(
                self.odom_to_base, timestamp, self.max_gap
            )
            reconstructed = (
                compose_pose(map_to_odom, odom_to_base)
                if map_to_odom is not None and odom_to_base is not None
                else None
            )
            error = (
                pose_error(initial_pose, reconstructed)
                if reconstructed is not None
                else (float("nan"), float("nan"))
            )
            rows.append(
                {
                    "timestamp": timestamp,
                    "translation_error_m": error[0],
                    "rotation_error_deg": error[1],
                    "reconstruction_available": reconstructed is not None,
                }
            )
        return rows

    def _build_summary(self, reference_rows, initial_pose_rows):
        reconstruction_translation = [
            row["reconstruction_translation_error_m"] for row in reference_rows
        ]
        reconstruction_rotation = [
            row["reconstruction_rotation_error_deg"] for row in reference_rows
        ]
        localization_translation = [
            row["localization_translation_error_m"] for row in reference_rows
        ]
        localization_rotation = [
            row["localization_rotation_error_deg"] for row in reference_rows
        ]
        initial_translation = [
            row["translation_error_m"] for row in initial_pose_rows
        ]
        initial_rotation = [
            row["rotation_error_deg"] for row in initial_pose_rows
        ]
        decisions = Counter(row["decision"] for row in self.scan_metrics)
        states = Counter(row["state"] for row in self.scan_metrics)
        accepted = sum(1 for row in self.scan_metrics if row["accepted"])
        total_latencies = [row["total_ms"] for row in self.scan_metrics]
        deadline_misses = sum(
            1
            for latency in total_latencies
            if math.isfinite(latency) and latency > self.deadline_ms
        )
        launch_values = {
            "bag_path": self.bag_path,
            "output_directory": self.output_directory,
            "run_name": self.run_name,
            "rate": self.replay_rate,
            "start_offset": self.start_offset,
            "config_file": self.config_file,
            "initial_pose_delay": self.initial_pose_parameters["delay_seconds"],
            "translation_x": self.initial_pose_parameters["translation_x"],
            "translation_y": self.initial_pose_parameters["translation_y"],
            "translation_z": self.initial_pose_parameters["translation_z"],
            "yaw_degrees": self.initial_pose_parameters["yaw_degrees"],
            "position_sigma": self.initial_pose_parameters["position_sigma"],
            "yaw_sigma_degrees": self.initial_pose_parameters[
                "yaw_sigma_degrees"
            ],
            "deadline_ms": self.deadline_ms,
        }
        command = "ros2 launch ndt_localization benchmark_replay.launch.py " + " ".join(
            "%s:=%s" % (key, shlex.quote(str(value)))
            for key, value in launch_values.items()
        )
        config_sha256 = None
        if self.config_file and os.path.isfile(self.config_file):
            with open(self.config_file, "rb") as config_input:
                config_sha256 = hashlib.sha256(config_input.read()).hexdigest()
        return {
            "schema_version": 2,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "run": {
                "name": self.run_name,
                "bag_path": self.bag_path,
                "replay_rate": self.replay_rate,
                "start_offset": self.start_offset,
                "command": command,
                "localizer_config": {
                    "path": self.config_file,
                    "sha256": config_sha256,
                },
                "synthetic_initial_pose": self.initial_pose_parameters,
                "frames": {
                    "map": self.map_frame,
                    "odom": self.odom_frame,
                    "base": self.base_frame,
                },
                "max_interpolation_gap_seconds": self.max_gap,
                "deadline_ms": self.deadline_ms,
            },
            "samples": {
                "map_to_odom_transforms": len(self.map_to_odom),
                "localization_map_to_odom_transforms": len(
                    self.localization_map_to_odom
                ),
                "odometry_lio": len(self.odom_to_base),
                "recorded_odometry_map": len(self.recorded_map_odometry),
                "localization_odometry_map": len(self.localization_odometry),
                "recorded_initial_pose": len(self.recorded_initial_poses),
                "synthetic_initial_pose": len(self.synthetic_initial_poses),
                "scan_diagnostics": len(self.scan_metrics),
                "state_events": len(self.state_events),
            },
            "state_events": self.state_events,
            "localization_evaluation_start_timestamp": (
                self.synthetic_initial_poses[0][0]
                if self.synthetic_initial_poses
                else None
            ),
            "reference_reconstruction": {
                "translation_error_m": distribution(reconstruction_translation),
                "rotation_error_deg": distribution(reconstruction_rotation),
            },
            "recorded_initial_pose_header_timestamp_check": {
                "translation_error_m": distribution(initial_translation),
                "rotation_error_deg": distribution(initial_rotation),
            },
            "localization_against_recorded_reference": {
                "translation_error_m": distribution(localization_translation),
                "rotation_error_deg": distribution(localization_rotation),
            },
            "scan_decisions": {
                "counts": dict(sorted(decisions.items())),
                "state_counts": dict(sorted(states.items())),
                "accepted": accepted,
                "rejected": len(self.scan_metrics) - accepted,
                "accepted_fraction": (
                    accepted / len(self.scan_metrics)
                    if self.scan_metrics
                    else None
                ),
                "deadline_misses": deadline_misses,
            },
            "latency_ms": {
                key: distribution([row[key] for row in self.scan_metrics])
                for key in (
                    "conversion_ms",
                    "local_map_ms",
                    "matcher_ms",
                    "validation_ms",
                    "total_ms",
                    "input_age_ms",
                )
            },
            "registration_validation": {
                "translation_delta_m": distribution(
                    [
                        row["translation_delta_m"]
                        for row in self.scan_metrics
                    ]
                ),
                "rotation_delta_deg": distribution(
                    [
                        row["rotation_delta_deg"]
                        for row in self.scan_metrics
                    ]
                ),
                "odometry_before_gap_ms": distribution(
                    [
                        row["odometry_before_gap_ms"]
                        for row in self.scan_metrics
                    ]
                ),
                "odometry_after_gap_ms": distribution(
                    [
                        row["odometry_after_gap_ms"]
                        for row in self.scan_metrics
                    ]
                ),
            },
        }

    def _write_csv(self, filename, rows, fieldnames):
        path = os.path.join(self.output_directory, filename)
        with open(path, "w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(
                output, fieldnames=fieldnames, extrasaction="ignore"
            )
            writer.writeheader()
            writer.writerows(rows)

    def _write_markdown_summary(self, summary):
        reference = summary["reference_reconstruction"]["translation_error_m"]
        localization = summary[
            "localization_against_recorded_reference"
        ]["translation_error_m"]
        latency = summary["latency_ms"]["total_ms"]
        decisions = summary["scan_decisions"]

        def metric(data, key):
            value = data.get(key)
            return "n/a" if value is None else "%.6f" % value

        lines = [
            "# Localization benchmark: %s" % self.run_name,
            "",
            "Generated: %s" % summary["generated_at_utc"],
            "",
            "This report compares against recorded localization output. It is "
            "regression/pseudo-ground-truth, not independent survey ground truth.",
            "",
            "| Metric | p50 | p95 | p99 | maximum |",
            "|---|---:|---:|---:|---:|",
            "| Reference reconstruction translation (m) | %s | %s | %s | %s |"
            % tuple(metric(reference, key) for key in ("p50", "p95", "p99", "maximum")),
            "| Localizer translation (m) | %s | %s | %s | %s |"
            % tuple(metric(localization, key) for key in ("p50", "p95", "p99", "maximum")),
            "| Scan decision latency (ms) | %s | %s | %s | %s |"
            % tuple(metric(latency, key) for key in ("p50", "p95", "p99", "maximum")),
            "",
            "- Scan decisions: %d" % summary["samples"]["scan_diagnostics"],
            "- Accepted: %d" % decisions["accepted"],
            "- Rejected/skipped: %d" % decisions["rejected"],
            "- Deadline misses (> %.3f ms): %d"
            % (summary["run"]["deadline_ms"], decisions["deadline_misses"]),
            "",
            "Reproduction command:",
            "",
            "```bash",
            summary["run"]["command"],
            "```",
            "",
        ]
        path = os.path.join(self.output_directory, "report.md")
        with open(path, "w", encoding="utf-8") as output:
            output.write("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBenchmarkEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
