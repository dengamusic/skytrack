#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Distributed tracking benchmark: two sensor nodes and a fusion centre.

    cmake --build build
    python3 tools/two_sites.py             # writes runs/two_sites_benchmark/results.md

For each seed of scenarios/two_sites.json it runs
  * each node alone (its own tracker on its own sensors, configs/nodes/two_sites.json),
  * a fusion centre on the two nodes' published tracks, with three rules for
    combining two estimates of one target (covariance intersection, the
    information sum that assumes independent errors, and the single best node),
  * one central tracker on every measurement (the upper bound a fusion centre
    approaches without the raw data),
and scores all of them against ground truth; then again with site A's
radar out for a minute, which is when a second node earns its keep.
"""

from __future__ import annotations

import argparse
import json
import shutil
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMBINES = {"ci": "covariance intersection", "independent": "information sum (assumes independence)",
            "best": "best single node"}


def run(cmd: list[str | Path]) -> None:
    subprocess.run([str(c) for c in cmd], check=True, stdout=subprocess.DEVNULL)


def raw_bytes_per_s(run_dir: Path, sensors: set[str], duration_s: float) -> float:
    total = 0
    with open(run_dir / "measurements.jsonl", "rb") as f:
        for line in f:
            if json.loads(line)["sensor_id"] in sensors:
                total += len(line)
    return total / duration_s


def with_outage(run_dir: Path, out_dir: Path, sensor: str, t0: float, t1: float) -> Path:
    """A copy of the run in which one sensor sends nothing between t0 and t1."""
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    for name in ("scenario.json", "sensors.json", "summary.json"):
        shutil.copy(run_dir / name, out_dir / name)
    shutil.copytree(run_dir / "truth", out_dir / "truth")
    with open(run_dir / "measurements.jsonl") as src, open(out_dir / "measurements.jsonl", "w") as dst:
        for line in src:
            m = json.loads(line)
            if not (m["sensor_id"] == sensor and t0 <= m["t_arrival_s"] < t1):
                dst.write(line)
    return out_dir


def track_everything(build: Path, run_dir: Path, nodes: dict[str, list[str]]) -> dict[str, dict]:
    """Each node alone, the fusion centre with each combining rule, and a central tracker."""
    for node, sensors in nodes.items():
        run([build / "skytrack_track", run_dir, "--sensors", ",".join(sensors), "--out", run_dir / node])
    run([build / "skytrack_track", run_dir, "--out", run_dir / "central"])
    for combine in COMBINES:
        run([build / "skytrack_fuse", *(run_dir / n for n in nodes), "--out", run_dir / f"fused-{combine}",
             "--combine", combine])
    metrics = {}
    for v in list(nodes) + ["central"] + [f"fused-{c}" for c in COMBINES]:
        run([build / "skytrack_eval", run_dir, "--tracks", run_dir / v])
        metrics[v] = json.loads((run_dir / v / "eval" / "metrics.json").read_text())
    return metrics


def table(results: dict[str, list[dict]]) -> list[str]:
    names = [t["name"] for t in results["central"][0]["targets"]]
    lines = ["| | GOSPA (m) | breaks | false tracks | " + " | ".join(f"{n} NEES" for n in names) + " |",
             "|---" * (4 + len(names)) + "|"]
    for v, rs in results.items():
        gospa = statistics.mean(r["gospa"]["mean_m"] for r in rs)
        breaks = sum(t["id_switches"] + t["gaps"] for r in rs for t in r["targets"])
        false = sum(r["tracks"]["false"] for r in rs)
        nees = []
        for n in names:
            vals = [t["nees_mean"] for r in rs for t in r["targets"] if t["name"] == n and t["tracked_fraction"] > 0]
            nees.append(f"{statistics.mean(vals):.1f} (max {max(vals):.1f})" if vals else "-")
        lines.append(f"| {v} | {gospa:.1f} | {breaks} | {false} | " + " | ".join(nees) + " |")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", type=Path, default=ROOT / "build")
    parser.add_argument("--out", type=Path, default=ROOT / "runs" / "two_sites_benchmark")
    parser.add_argument("--seeds", type=int, nargs="+", default=[42, 1, 2, 3, 4, 5, 6, 7])
    parser.add_argument("--check", action="store_true", help="fail on a regression against tools/thresholds.json")
    args = parser.parse_args()
    nodes = json.loads((ROOT / "configs" / "nodes" / "two_sites.json").read_text())["nodes"]
    scenario = ROOT / "scenarios" / "two_sites.json"
    duration = json.loads(scenario.read_text())["duration_s"]
    outage = ("radar-1", 60.0, 120.0)

    nominal: dict[str, list[dict]] = {}
    degraded: dict[str, list[dict]] = {}
    uplink: dict[str, list[float]] = {}
    for seed in args.seeds:
        run_dir = args.out / f"seed{seed}"
        run([args.build / "skytrack_sim", scenario, run_dir, "--seed", seed])
        for v, m in track_everything(args.build, run_dir, nodes).items():
            nominal.setdefault(v, []).append(m)
        for node, sensors in nodes.items():
            uplink.setdefault(f"{node}: raw measurements", []).append(raw_bytes_per_s(run_dir, set(sensors), duration))
        for n in json.loads((run_dir / "fused-ci" / "summary.json").read_text())["nodes"]:
            uplink.setdefault(f"{n['node']}: confirmed tracks", []).append(n["uplink_bytes_per_s"])
        out_dir = with_outage(run_dir, args.out / f"seed{seed}-outage", *outage)
        for v, m in track_everything(args.build, out_dir, nodes).items():
            degraded.setdefault(v, []).append(m)

    seeds = ", ".join(map(str, args.seeds))
    lines = [f"# Two sites, {len(args.seeds)} seeds ({seeds})", "", "## Every sensor working", ""]
    lines += table(nominal)
    lines += ["", f"## {outage[0]} (site A's radar) sends nothing from {outage[1]:.0f} s to {outage[2]:.0f} s", ""]
    lines += table(degraded)
    lines += ["", "NEES is the mean position NEES over the run: 3 if the covariance is honest, above 3 overconfident.",
              "Breaks are track id switches plus gaps, summed over targets and seeds.",
              "", "## What each node sends", "", "| link | kB/s |", "|---|---|"]
    for k, vals in sorted(uplink.items()):
        lines.append(f"| {k} | {statistics.mean(vals) / 1000:.1f} |")
    text = "\n".join(lines) + "\n"
    (args.out / "results.md").write_text(text)
    print(text)
    if args.check:
        limits = json.loads((ROOT / "tools" / "thresholds.json").read_text())["two_sites"]
        fused, independent = nominal["fused-ci"], nominal["fused-independent"]
        failures = []
        gospa = statistics.mean(r["gospa"]["mean_m"] for r in fused)
        if gospa > limits["max_fused_gospa_m"]:
            failures.append(f"fused GOSPA {gospa:.1f} m")
        false = sum(r["tracks"]["false"] for r in fused)
        if false > limits["max_fused_false_tracks"]:
            failures.append(f"{false} fused false tracks")
        nees = max(t["nees_mean"] for r in fused for t in r["targets"])
        if nees > limits["max_fused_nees_max"]:
            failures.append(f"fused NEES up to {nees:.1f}")
        outage = statistics.mean(r["gospa"]["mean_m"] for r in degraded["fused-ci"])
        if outage > limits["max_outage_fused_gospa_m"]:
            failures.append(f"fused GOSPA {outage:.1f} m with site A's radar out")
        # The point of covariance intersection: more honest than the information sum.
        for name in ("intruder", "airliner"):
            ci = statistics.mean(t["nees_mean"] for r in fused for t in r["targets"] if t["name"] == name)
            naive = statistics.mean(t["nees_mean"] for r in independent for t in r["targets"] if t["name"] == name)
            if ci >= naive:
                failures.append(f"{name}: CI NEES {ci:.1f} not below the information sum's {naive:.1f}")
        for f in failures:
            print(f"REGRESSION: {f}", file=sys.stderr)
        sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
