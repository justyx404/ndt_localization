#!/usr/bin/env python3
"""Run and aggregate the four development-bag localization baselines."""

import argparse
import json
import os
import signal
import subprocess
import sys
from datetime import datetime, timezone

import yaml


DEVELOPMENT_BAGS = (
    "mine_nav1_r1",
    "mine_nav2_r1",
    "mine_nav3_r1",
    "mine_nav4_r1",
)


def metric(summary, path, key):
    value = summary
    for part in path:
        value = value.get(part, {})
    value = value.get(key)
    return "n/a" if value is None else "%.3f" % value


def write_aggregate(output_directory, results, command):
    aggregate = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "runs": results,
    }
    with open(
        os.path.join(output_directory, "development_baseline.json"),
        "w",
        encoding="utf-8",
    ) as output:
        json.dump(aggregate, output, indent=2, sort_keys=True)
        output.write("\n")

    lines = [
        "# Development-bag NDT localization baseline",
        "",
        "Generated: %s" % aggregate["generated_at_utc"],
        "",
        "Recorded localization is used as regression/pseudo-ground-truth; "
        "these values are not independent absolute-accuracy measurements.",
        "",
        "| Bag | Scans | Accepted | Ref reconstruction p95 (m) | "
        "Localizer p95 (m) | Decision p95 (ms) | Decision max (ms) | "
        "Deadline misses |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in DEVELOPMENT_BAGS:
        result = results.get(name, {})
        if "error" in result:
            lines.append("| %s | failed: %s | | | | | | |" % (name, result["error"]))
            continue
        samples = result.get("samples", {})
        decisions = result.get("scan_decisions", {})
        accepted_fraction = decisions.get("accepted_fraction")
        accepted = (
            "n/a"
            if accepted_fraction is None
            else "%.1f%%" % (100.0 * accepted_fraction)
        )
        lines.append(
            "| %s | %s | %s | %s | %s | %s | %s | %s |"
            % (
                name,
                samples.get("scan_diagnostics", "n/a"),
                accepted,
                metric(
                    result,
                    ("reference_reconstruction", "translation_error_m"),
                    "p95",
                ),
                metric(
                    result,
                    (
                        "localization_against_recorded_reference",
                        "translation_error_m",
                    ),
                    "p95",
                ),
                metric(result, ("latency_ms", "total_ms"), "p95"),
                metric(result, ("latency_ms", "total_ms"), "maximum"),
                decisions.get("deadline_misses", "n/a"),
            )
        )
    lines.extend(
        [
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument(
        "--timeout-factor",
        type=float,
        default=3.0,
        help="Wall timeout as this multiple of replay duration",
    )
    parser.add_argument(
        "--timeout-overhead-seconds",
        type=float,
        default=30.0,
        help="Startup/finalization allowance added to each run timeout",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    bags_root = os.path.abspath(args.bags_root)
    output_directory = os.path.abspath(args.output_dir)
    os.makedirs(output_directory, exist_ok=True)
    reproduction_command = (
        "ros2 run ndt_localization run_development_baseline.py "
        "--bags-root %s --output-dir %s --rate %s --start-offset %s"
        " --timeout-factor %s --timeout-overhead-seconds %s"
        % (
            bags_root,
            output_directory,
            args.rate,
            args.start_offset,
            args.timeout_factor,
            args.timeout_overhead_seconds,
        )
    )
    results = {}
    for bag_name in DEVELOPMENT_BAGS:
        bag_path = os.path.join(bags_root, bag_name)
        metadata_path = os.path.join(bag_path, "metadata.yaml")
        run_output = os.path.join(output_directory, bag_name)
        summary_path = os.path.join(run_output, "summary.json")
        if not os.path.isfile(metadata_path):
            results[bag_name] = {"error": "bag not found: %s" % bag_path}
            continue
        if args.force or not os.path.isfile(summary_path):
            os.makedirs(run_output, exist_ok=True)
            with open(metadata_path, "r", encoding="utf-8") as metadata_file:
                metadata = yaml.safe_load(metadata_file)
            duration_ns = metadata["rosbag2_bagfile_information"]["duration"][
                "nanoseconds"
            ]
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
                "output_directory:=%s" % run_output,
                "run_name:=%s" % bag_name,
                "rate:=%s" % args.rate,
                "start_offset:=%s" % args.start_offset,
            ]
            print(
                "Running (wall timeout %.1fs):" % wall_timeout,
                " ".join(command),
                flush=True,
            )
            process = subprocess.Popen(command, start_new_session=True)
            timed_out = False
            try:
                return_code = process.wait(timeout=wall_timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                print(
                    "%s exceeded its %.1fs wall timeout; requesting shutdown"
                    % (bag_name, wall_timeout),
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
            if timed_out:
                results[bag_name] = {
                    "error": "wall timeout after %.1f seconds" % wall_timeout
                }
                write_aggregate(
                    output_directory, results, reproduction_command
                )
                continue
            if return_code != 0:
                results[bag_name] = {
                    "error": "launch exited with code %d" % return_code
                }
                continue
        try:
            with open(summary_path, "r", encoding="utf-8") as summary_file:
                results[bag_name] = json.load(summary_file)
        except (OSError, json.JSONDecodeError) as error:
            results[bag_name] = {"error": str(error)}
        write_aggregate(output_directory, results, reproduction_command)

    write_aggregate(output_directory, results, reproduction_command)
    failed = [name for name, result in results.items() if "error" in result]
    if failed:
        print("Failed runs: %s" % ", ".join(failed), file=sys.stderr)
        return 1
    print(
        "Wrote %s"
        % os.path.join(output_directory, "development_baseline.md")
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
