"""Deterministic tests for reference-pose reconstruction helpers."""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).parents[1]
    / "scripts"
    / "localization_benchmark_evaluator.py"
)
SPEC = importlib.util.spec_from_file_location("benchmark_evaluator", SCRIPT_PATH)
BENCHMARK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCHMARK)


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
