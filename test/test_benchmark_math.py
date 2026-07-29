"""Deterministic tests for reference-pose reconstruction helpers."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest


SCRIPTS_PATH = Path(__file__).parents[1] / "scripts"
SCRIPT_PATH = SCRIPTS_PATH / "localization_benchmark_evaluator.py"
SPEC = importlib.util.spec_from_file_location(
    "benchmark_evaluator", SCRIPT_PATH
)
BENCHMARK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCHMARK)
sys.modules["localization_benchmark_evaluator"] = BENCHMARK
ANALYZER_SPEC = importlib.util.spec_from_file_location(
    "analyze_reference_bag",
    SCRIPTS_PATH / "analyze_reference_bag.py",
)
ANALYZER = importlib.util.module_from_spec(ANALYZER_SPEC)
ANALYZER_SPEC.loader.exec_module(ANALYZER)


def yaw_quaternion(degrees):
    half_angle = math.radians(degrees) * 0.5
    return 0.0, 0.0, math.sin(half_angle), math.cos(half_angle)


def test_compose_pose_rotates_child_translation():
    parent = ((1.0, 2.0, 3.0), yaw_quaternion(90.0))
    child = ((2.0, 0.0, 0.0), yaw_quaternion(0.0))

    composed = BENCHMARK.compose_pose(parent, child)

    assert composed[0] == pytest.approx((1.0, 4.0, 3.0))
    assert BENCHMARK.pose_error(
        composed, ((1.0, 4.0, 3.0), yaw_quaternion(90.0))
    ) == pytest.approx((0.0, 0.0), abs=1.0e-9)


def test_interpolate_pose_uses_linear_position_and_slerp_orientation():
    samples = [
        (10.0, ((0.0, 0.0, 0.0), yaw_quaternion(0.0))),
        (12.0, ((4.0, 2.0, 0.0), yaw_quaternion(90.0))),
    ]

    interpolated = BENCHMARK.interpolate_pose(samples, 11.0, 2.0)

    assert interpolated[0] == pytest.approx((2.0, 1.0, 0.0))
    _, rotation_error = BENCHMARK.pose_error(
        interpolated, ((2.0, 1.0, 0.0), yaw_quaternion(45.0))
    )
    assert rotation_error == pytest.approx(0.0, abs=1.0e-6)


def test_interpolate_pose_rejects_unbounded_gap():
    samples = [
        (10.0, ((0.0, 0.0, 0.0), yaw_quaternion(0.0))),
        (12.0, ((4.0, 2.0, 0.0), yaw_quaternion(90.0))),
    ]

    assert BENCHMARK.interpolate_pose(samples, 11.0, 0.5) is None
    assert BENCHMARK.interpolate_pose(samples, 9.0, 5.0) is None


def test_distribution_reports_required_percentiles_and_maximum():
    result = BENCHMARK.distribution([1.0, 2.0, 3.0, 4.0, float("nan")])

    assert result["count"] == 4
    assert result["p50"] == pytest.approx(2.5)
    assert result["p95"] == pytest.approx(3.85)
    assert result["p99_9"] == pytest.approx(3.997)
    assert result["maximum"] == pytest.approx(4.0)


def test_initial_pose_alignment_selects_storage_timestamp():
    identity = yaw_quaternion(0.0)
    data = {
        "map_to_odom": [
            (10.0, ((0.0, 0.0, 0.0), identity)),
            (11.0, ((0.0, 0.0, 0.0), identity)),
        ],
        "odom_to_base": [
            (10.0, ((0.0, 0.0, 0.0), identity)),
            (11.0, ((1.0, 0.0, 0.0), identity)),
        ],
        "recorded_map_odometry": [],
        "recorded_initial_poses": [
            {
                "header_timestamp": 10.0,
                "storage_timestamp": 11.0,
                "pose": ((1.0, 0.0, 0.0), identity),
            }
        ],
    }

    _, rows = ANALYZER.analyze_data(data, 1.0, 0.5, 5.0)

    assert rows[0]["header_translation_error_m"] == pytest.approx(1.0)
    assert rows[0]["storage_translation_error_m"] == pytest.approx(0.0)
    assert rows[0]["selected_timestamp_basis"] == "storage"
    assert rows[0]["matches_reference_threshold"] is True
