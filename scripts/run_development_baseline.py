#!/usr/bin/env python3
"""Run and aggregate the four development-bag localization baselines."""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

import yaml


DEVELOPMENT_BAGS = (
    "mine_nav1_r1",
    "mine_nav2_r1",
    "mine_nav3_r1",
    "mine_nav4_r1",
)


def load_json(path):
    with open(path, "r", encoding="utf-8") as source:
        return json.load(source)


def write_json(path, value):
    with open(path, "w", encoding="utf-8") as output:
        json.dump(value, output, indent=2, sort_keys=True)
        output.write("\n")


def nested_metric(value, path, key, precision=3):
    for part in path:
        value = value.get(part, {})
    value = value.get(key)
    return "n/a" if value is None else ("%.*f" % (precision, value))


def write_aggregate(output_directory, results, command):
    aggregate = {
        "schema_version": 2,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "runs": results,
    }
    write_json(
        os.path.join(output_directory, "development_baseline.json"),
        aggregate,
    )

    lines = [
        "# Development-bag NDT localization baseline",
        "",
        "Generated: %s" % aggregate["generated_at_utc"],
        "",
        "Recorded localization is regression/pseudo-ground-truth, not "
        "independent absolute ground truth. A timeout is a measured baseline "
        "outcome; offline reference validation remains complete.",
        "",
        "| Bag | Replay status | Scans | Accepted | Reference p95 (m) | "
        "Reference repeatable | Localizer p95 (m) | Decision p95 / max (ms) | "
        "Queue p95 / max (ms) | Max input age (ms) | Late discarded | "
        "Deadline overruns |",
        "|---|---|---:|---:|---:|:---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in DEVELOPMENT_BAGS:
        result = results.get(name, {})
        reference = result.get("reference", {})
        replay = result.get("replay") or {}
        samples = replay.get("samples", {})
        decisions = replay.get("scan_decisions", {})
        accepted_fraction = decisions.get("accepted_fraction")
        accepted = (
            "n/a"
            if accepted_fraction is None
            else "%.1f%%" % (100.0 * accepted_fraction)
        )
        repeatable = reference.get("repeatability", {}).get("identical")
        repeatable_text = (
            "yes"
            if repeatable is True
            else "no" if repeatable is False else "n/a"
        )
        decision_latency = replay.get("latency_ms", {}).get("total_ms", {})
        p95_latency = decision_latency.get("p95")
        max_latency = decision_latency.get("maximum")
        latency_text = (
            "n/a"
            if p95_latency is None or max_latency is None
            else "%.3f / %.3f" % (p95_latency, max_latency)
        )
        queue_wait = replay.get("latency_ms", {}).get("queue_wait_ms", {})
        p95_queue = queue_wait.get("p95")
        max_queue = queue_wait.get("maximum")
        queue_text = (
            "n/a"
            if p95_queue is None or max_queue is None
            else "%.3f / %.3f" % (p95_queue, max_queue)
        )
        lines.append(
            "| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | "
            "%s |"
            % (
                name,
                result.get("status", "not_run"),
                samples.get("scan_diagnostics", "n/a"),
                accepted,
                nested_metric(
                    reference,
                    ("reference_reconstruction", "translation_error_m"),
                    "p95",
                    precision=9,
                ),
                repeatable_text,
                nested_metric(
                    replay,
                    (
                        "localization_against_recorded_reference",
                        "translation_error_m",
                    ),
                    "p95",
                ),
                latency_text,
                queue_text,
                nested_metric(
                    replay, ("latency_ms", "input_age_ms"), "maximum"
                ),
                samples.get("late_results", "n/a"),
                decisions.get("deadline_misses", "n/a"),
            )
        )
    timed_out = [
        name
        for name, result in results.items()
        if result.get("status") == "timed_out"
    ]
    failed = [
        name
        for name, result in results.items()
        if result.get("status") in ("failed", "reference_failed")
    ]
    lines.extend(
        [
            "",
            "- Timed-out baseline runs: %s"
            % (", ".join(timed_out) if timed_out else "none"),
            "- Harness/reference failures: %s"
            % (", ".join(failed) if failed else "none"),
            "",
            "Reproduction command:",
            "",
            "```bash",
            command,
            "```",
            "",
        ]
    )
    with open(
        os.path.join(output_directory, "development_baseline.md"),
        "w",
        encoding="utf-8",
    ) as output:
        output.write("\n".join(lines))


def wait_for_process(process, wall_timeout, label):
    start = time.monotonic()
    timed_out = False
    try:
        return_code = process.wait(timeout=wall_timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        print(
            "%s exceeded its %.1fs wall timeout; requesting shutdown"
            % (label, wall_timeout),
            file=sys.stderr,
            flush=True,
        )
        os.killpg(process.pid, signal.SIGINT)
        try:
            return_code = process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                return_code = process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                return_code = process.wait()
    except KeyboardInterrupt:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
        raise
    return return_code, timed_out, time.monotonic() - start


def run_reference_analysis(args, bag_path, output_directory):
    summary_path = os.path.join(output_directory, "summary.json")
    command = [
        "ros2",
        "run",
        "ndt_localization",
        "analyze_reference_bag.py",
        "--bag-path",
        bag_path,
        "--output-directory",
        output_directory,
        "--repeatability-runs",
        str(args.reference_repeatability_runs),
    ]
    print("Validating reference:", " ".join(command), flush=True)
    try:
        completed = subprocess.run(
            command,
            check=False,
            timeout=args.reference_analysis_timeout_seconds,
        )
    except subprocess.TimeoutExpired:
        return None, (
            "offline reference analysis exceeded %.1f seconds"
            % args.reference_analysis_timeout_seconds
        )
    if completed.returncode != 0:
        return None, (
            "offline reference analysis exited with code %d"
            % completed.returncode
        )
    try:
        return load_json(summary_path), None
    except (OSError, json.JSONDecodeError) as error:
        return None, "cannot load reference summary: %s" % error


def run_live_replay(args, bag_name, bag_path, output_directory, duration_ns):
    summary_path = os.path.join(output_directory, "summary.json")
    replay_duration = duration_ns * 1.0e-9 / args.rate
    wall_timeout = (
        replay_duration * args.timeout_factor
        + args.timeout_overhead_seconds
    )
    command = [
        "ros2",
        "launch",
        "ndt_localization",
        "benchmark_replay.launch.py",
        "bag_path:=%s" % bag_path,
        "output_directory:=%s" % output_directory,
        "run_name:=%s" % bag_name,
        "rate:=%s" % args.rate,
        "start_offset:=%s" % args.start_offset,
        "initial_pose_delay:=%s" % args.initial_pose_delay,
    ]
    print(
        "Running (wall timeout %.1fs):" % wall_timeout,
        " ".join(command),
        flush=True,
    )
    process = subprocess.Popen(command, start_new_session=True)
    return_code, timed_out, elapsed_seconds = wait_for_process(
        process, wall_timeout, bag_name
    )
    replay_summary = None
    summary_error = None
    try:
        replay_summary = load_json(summary_path)
    except (OSError, json.JSONDecodeError) as error:
        summary_error = str(error)

    if timed_out:
        status = "timed_out"
    elif return_code != 0:
        status = "failed"
    elif replay_summary is None:
        status = "failed"
    else:
        status = "completed"
    return {
        "status": status,
        "wall_timeout_seconds": wall_timeout,
        "wall_elapsed_seconds": elapsed_seconds,
        "return_code": return_code,
        "summary_error": summary_error,
        "replay": replay_summary,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument(
        "--initial-pose-delay",
        type=float,
        default=10.0,
        help="Seconds from the first recorded map pose to inject the prior",
    )
    parser.add_argument(
        "--timeout-factor",
        type=float,
        default=2.0,
        help="Wall timeout as this multiple of replay duration",
    )
    parser.add_argument(
        "--timeout-overhead-seconds",
        type=float,
        default=30.0,
        help="Startup/finalization allowance added to each run timeout",
    )
    parser.add_argument(
        "--reference-repeatability-runs", type=int, default=2
    )
    parser.add_argument(
        "--reference-analysis-timeout-seconds",
        type=float,
        default=300.0,
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.start_offset < 0.0:
        parser.error("--start-offset cannot be negative")
    if args.initial_pose_delay < 0.0:
        parser.error("--initial-pose-delay cannot be negative")
    if args.timeout_factor < 0.0:
        parser.error("--timeout-factor cannot be negative")
    if args.timeout_overhead_seconds < 0.0:
        parser.error("--timeout-overhead-seconds cannot be negative")
    if args.reference_repeatability_runs < 1:
        parser.error("--reference-repeatability-runs must be at least 1")
    if args.reference_analysis_timeout_seconds <= 0.0:
        parser.error(
            "--reference-analysis-timeout-seconds must be positive"
        )

    bags_root = os.path.abspath(args.bags_root)
    output_directory = os.path.abspath(args.output_dir)
    os.makedirs(output_directory, exist_ok=True)
    reproduction_command = (
        "ros2 run ndt_localization run_development_baseline.py "
        "--bags-root %s --output-dir %s --rate %s --start-offset %s"
        " --initial-pose-delay %s"
        " --timeout-factor %s --timeout-overhead-seconds %s"
        " --reference-repeatability-runs %s"
        % (
            bags_root,
            output_directory,
            args.rate,
            args.start_offset,
            args.initial_pose_delay,
            args.timeout_factor,
            args.timeout_overhead_seconds,
            args.reference_repeatability_runs,
        )
    )
    results = {}
    for bag_name in DEVELOPMENT_BAGS:
        bag_path = os.path.join(bags_root, bag_name)
        metadata_path = os.path.join(bag_path, "metadata.yaml")
        run_output = os.path.join(output_directory, bag_name)
        reference_output = os.path.join(run_output, "reference")
        replay_output = os.path.join(run_output, "replay")
        status_path = os.path.join(run_output, "run_status.json")
        if not os.path.isfile(metadata_path):
            results[bag_name] = {
                "status": "reference_failed",
                "error": "bag not found: %s" % bag_path,
            }
            write_aggregate(
                output_directory, results, reproduction_command
            )
            continue
        if not args.force and os.path.isfile(status_path):
            results[bag_name] = load_json(status_path)
            write_aggregate(
                output_directory, results, reproduction_command
            )
            continue

        os.makedirs(reference_output, exist_ok=True)
        os.makedirs(replay_output, exist_ok=True)
        with open(metadata_path, "r", encoding="utf-8") as metadata_file:
            metadata = yaml.safe_load(metadata_file)[
                "rosbag2_bagfile_information"
            ]
        reference, reference_error = run_reference_analysis(
            args, bag_path, reference_output
        )
        if reference_error is not None:
            result = {
                "status": "reference_failed",
                "error": reference_error,
                "reference": reference,
                "replay": None,
            }
        else:
            result = run_live_replay(
                args,
                bag_name,
                bag_path,
                replay_output,
                metadata["duration"]["nanoseconds"],
            )
            result["reference"] = reference
        write_json(status_path, result)
        results[bag_name] = result
        write_aggregate(output_directory, results, reproduction_command)

    write_aggregate(output_directory, results, reproduction_command)
    harness_failures = [
        name
        for name, result in results.items()
        if result.get("status") in ("failed", "reference_failed")
    ]
    if harness_failures:
        print(
            "Harness failures: %s" % ", ".join(harness_failures),
            file=sys.stderr,
        )
        return 1
    print(
        "Wrote %s"
        % os.path.join(output_directory, "development_baseline.md")
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
