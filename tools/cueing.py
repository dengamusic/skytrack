#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Closed-loop camera cueing benchmark on the C-UAS baseline scene.

    cmake --build build
    python3 tools/cueing.py                # writes runs/cueing_benchmark/results.md

Three ways to use the narrow-field PTZ camera, for each seed:
  * none:     no PTZ camera at all (the wide camera, radar, RF and broadcasts only);
  * scripted: the scenario's fixed pan/tilt schedule, written by hand knowing
              where the targets would be (an operator with perfect foresight);
  * cued:     pointed in closed loop by the tracker's own published tracks
              (skytrack_cue), with a 60 deg/s slew limit and the priority
              policy in include/skytrack/cueing/cueing.hpp.
The headline is time from first detection to the right class on the track,
for the targets that do not identify themselves.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def run(cmd: list[str | Path]) -> None:
    subprocess.run([str(c) for c in cmd], check=True, stdout=subprocess.DEVNULL)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", type=Path, default=ROOT / "build")
    parser.add_argument("--out", type=Path, default=ROOT / "runs" / "cueing_benchmark")
    parser.add_argument("--seeds", type=int, nargs="+", default=[42, 1, 2, 3, 4, 5, 6, 7])
    parser.add_argument("--check", action="store_true", help="fail on a regression against tools/thresholds.json")
    args = parser.parse_args()
    scenario = ROOT / "scenarios" / "cuas_baseline.json"
    sensors = [s["id"] for s in json.loads(scenario.read_text())["sensors"]]
    without_ptz = ",".join(s for s in sensors if s != "cam-ptz")

    results: dict[str, list[dict]] = {"none": [], "scripted": [], "cued": []}
    on_target = []
    for seed in args.seeds:
        sim = args.out / f"seed{seed}"
        run([args.build / "skytrack_sim", scenario, sim, "--seed", seed])
        run([args.build / "skytrack_track", sim, "--sensors", without_ptz, "--out", sim / "none"])
        run([args.build / "skytrack_track", sim, "--out", sim / "scripted"])
        for v in ("none", "scripted"):
            run([args.build / "skytrack_eval", sim, "--tracks", sim / v])
            results[v].append(json.loads((sim / v / "eval" / "metrics.json").read_text()))
        cued = args.out / f"seed{seed}-cued"
        run([args.build / "skytrack_cue", scenario, cued, "--seed", seed])
        run([args.build / "skytrack_eval", cued])
        results["cued"].append(json.loads((cued / "tracker" / "eval" / "metrics.json").read_text()))
        c = json.loads((cued / "tracker" / "cueing.json").read_text())
        on_target.append(c["frames_on_target"] / c["frames"])

    # Targets that do not say what they are (no transponder, no Remote ID).
    names = ["intruder", "gull", "crow"]
    lines = [f"# Camera cueing, {len(args.seeds)} seeds ({', '.join(map(str, args.seeds))})", "",
             "| PTZ camera | GOSPA (m) | breaks | " + " | ".join(f"{n}: time to class" for n in names)
             + " | " + " | ".join(f"{n}: right class" for n in names) + " |",
             "|---" * (3 + 2 * len(names)) + "|"]
    for v, rs in results.items():
        gospa = statistics.mean(r["gospa"]["mean_m"] for r in rs)
        breaks = sum(t["id_switches"] + t["gaps"] for r in rs for t in r["targets"])
        times, right = [], []
        for n in names:
            ts = [t for r in rs for t in r["targets"] if t["name"] == n]
            got = [t["time_to_classify_s"] for t in ts if t["time_to_classify_s"] is not None]
            times.append(f"{statistics.median(got):.1f} s ({len(got)}/{len(ts)})" if got else f"never (0/{len(ts)})")
            right.append(f"{100 * statistics.mean(t['class_correct'] for t in ts):.0f}%")
        lines.append(f"| {v} | {gospa:.1f} | {breaks} | " + " | ".join(times) + " | " + " | ".join(right) + " |")
    lines += ["", "Time to class: median over the seeds where it happened, from the first detection of the target "
              "to the first output with the right class on its track. Right class: share of tracked time.",
              f"Cued camera on a chosen track {100 * statistics.mean(on_target):.0f}% of frames."]
    text = "\n".join(lines) + "\n"
    (args.out / "results.md").write_text(text)
    print(text)
    if args.check:
        limits = json.loads((ROOT / "tools" / "thresholds.json").read_text())["cueing"]
        cued = results["cued"]
        birds = [t for r in cued for t in r["targets"] if t["name"] in ("gull", "crow")]
        failures = []
        gospa = statistics.mean(r["gospa"]["mean_m"] for r in cued)
        if gospa > limits["max_cued_gospa_m"]:
            failures.append(f"cued GOSPA {gospa:.1f} m")
        share = sum(t["time_to_classify_s"] is not None for t in birds) / len(birds)
        if share < limits["min_birds_classified_share"]:
            failures.append(f"birds classified in only {100 * share:.0f}% of runs")
        right = statistics.mean(t["class_correct"] for t in birds)
        if right < limits["min_birds_right_class"]:
            failures.append(f"birds have the right class {100 * right:.0f}% of the time")
        for f in failures:
            print(f"REGRESSION: {f}", file=sys.stderr)
        sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
