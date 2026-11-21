#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Runs the tracker over many simulated runs and prints the results tables.

    cmake --build build
    python3 tools/benchmark.py            # writes runs/benchmark/results.md

1. The C-UAS baseline scene with several seeds (each seed redraws every
   detection, false alarm, latency and GNSS error), tracked with the default
   configuration, the strict one, and with single features switched off.
2. The same scene with shorter and longer reorder windows, with and without
   late track initiation: what each costs in report age and accuracy.
3. The same site with no targets at all, only clutter, to measure how often
   false tracks get confirmed.

With --check, it also compares the default configuration against
tools/thresholds.json and exits non-zero on a regression (CI runs this).

Every number in the README's tracker results comes from this script.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# name -> (config file or None for the defaults, description)
CONFIGS = {
    "default": (None, "2-of-3 confirmation, every feature on"),
    "strict": (ROOT / "configs" / "strict.json", "3-of-4 confirmation"),
    "no_notch": (ROOT / "configs" / "ablations" / "no_notch_inference.json", "no Doppler-notch inference"),
    "no_registration": (ROOT / "configs" / "ablations" / "no_registration.json", "no online sensor registration"),
    "white_registration": (ROOT / "configs" / "ablations" / "white_registration.json",
                           "remaining registration error as extra white noise, not a consider parameter"),
}

# Reorder window and late initiation: name -> (settings, description).
LATENCY = {
    "window_0.3": ({"reorder_window_s": 0.3}, "0.3 s window, late initiation up to 3 s"),
    "window_0.6": ({}, "0.6 s window, late initiation up to 3 s (default)"),
    "window_1.3": ({"reorder_window_s": 1.3}, "1.3 s window, late initiation up to 3 s"),
    "window_0.3_no_late": ({"reorder_window_s": 0.3, "late_initiation_s": 0.0}, "0.3 s window, no late initiation"),
    "window_0.6_no_late": ({"late_initiation_s": 0.0}, "0.6 s window, no late initiation"),
}


def run(cmd: list[str | Path]) -> None:
    subprocess.run([str(c) for c in cmd], check=True, stdout=subprocess.DEVNULL)


def track_and_score(build: Path, run_dir: Path, name: str, config: Path | None = None) -> tuple[dict, dict]:
    if name in CONFIGS:
        config, _ = CONFIGS[name]
    out = run_dir / name
    cmd: list[str | Path] = [build / "skytrack_track", run_dir, "--out", out]
    if config is not None:
        cmd += ["--config", config]
    run(cmd)
    run([build / "skytrack_eval", run_dir, "--tracks", out])
    return json.loads((out / "summary.json").read_text()), json.loads((out / "eval" / "metrics.json").read_text())


def mean(xs: list[float]) -> float:
    return statistics.fmean(xs) if xs else float("nan")


def baseline_table(build: Path, out: Path, seeds: list[int]) -> tuple[str, dict]:
    rows = {name: [] for name in CONFIGS}
    for seed in seeds:
        run_dir = out / f"baseline_seed{seed}"
        run([build / "skytrack_sim", ROOT / "scenarios" / "cuas_baseline.json", run_dir, "--seed", str(seed)])
        for name in CONFIGS:
            rows[name].append(track_and_score(build, run_dir, name)[1])

    lines = [
        f"Baseline scene, {len(seeds)} seeds x 5 targets x 180 s (seeds {', '.join(map(str, seeds))}):",
        "",
        "| configuration | mean GOSPA | tracked | time to track | breaks | false tracks | RMSE drones | RMSE birds |"
        " NEES drones | NEES birds | NEES airliner |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    summary = {}
    for name, metrics in rows.items():
        targets = [t for m in metrics for t in m["targets"]]
        drones = [t["rmse_horizontal_m"] for t in targets if t["class"] == "drone"]
        birds = [t["rmse_horizontal_m"] for t in targets if t["class"] == "bird"]
        nees = {cls: [t["nees_mean"] for t in targets if t["class"] == cls and t["nees_mean"] > 0]
                for cls in ("drone", "bird", "aircraft")}
        summary[name] = {
            "gospa_m": mean([m["gospa"]["mean_m"] for m in metrics]),
            "breaks": sum(t["id_switches"] + t["gaps"] for t in targets),
            "false_tracks": sum(m["tracks"]["false"] for m in metrics),
            "tracked_fraction": mean([t["tracked_fraction"] for t in targets]),
            "nees_drone_max": max(nees["drone"]),
            "nees_bird_max": max(nees["bird"]),
            "nees_aircraft_max": max(nees["aircraft"]),
        }
        lines.append(
            "| {name} ({desc}) | {gospa:.1f} m | {tracked:.1f}% | {ttt:.1f} s | {breaks} | {false} | {drones:.1f} m |"
            " {birds:.1f} m | {nees_d} | {nees_b} | {nees_a} |".format(
                name=f"`{name}`",
                desc=CONFIGS[name][1],
                gospa=mean([m["gospa"]["mean_m"] for m in metrics]),
                tracked=100.0 * mean([t["tracked_fraction"] for t in targets]),
                ttt=mean([t["time_to_track_s"] for t in targets if t.get("time_to_track_s") is not None]),
                breaks=sum(t["id_switches"] + t["gaps"] for t in targets),
                false=sum(m["tracks"]["false"] for m in metrics),
                drones=mean(drones),
                birds=mean(birds),
                nees_d=f"{mean(nees['drone']):.1f} (max {max(nees['drone']):.1f})",
                nees_b=f"{mean(nees['bird']):.1f} (max {max(nees['bird']):.1f})",
                nees_a=f"{mean(nees['aircraft']):.1f} (max {max(nees['aircraft']):.1f})",
            )
        )
    lines += [
        "",
        "breaks = track switches + gaps, summed over all target runs; false tracks = confirmed tracks that spent",
        "less than half their life on a target; RMSE = horizontal; NEES = position NEES of each target run,",
        "mean (and worst) over the runs of that class: 3 is honest, above 3 overconfident.",
    ]
    return "\n".join(lines), summary


def latency_table(build: Path, out: Path, seeds: list[int]) -> str:
    rows: dict[str, list[tuple[dict, dict]]] = {name: [] for name in LATENCY}
    configs = out / "configs"
    configs.mkdir(parents=True, exist_ok=True)
    for name, (settings, _) in LATENCY.items():
        (configs / f"{name}.json").write_text(json.dumps(settings))
    for seed in seeds:
        run_dir = out / f"baseline_seed{seed}"  # simulated by baseline_table
        for name in LATENCY:
            rows[name].append(track_and_score(build, run_dir, name, configs / f"{name}.json"))
    lines = [
        f"Reorder window and late initiation, the baseline scene, {len(seeds)} seeds:",
        "",
        "| setting | mean GOSPA | report age p50 | time to track | worst time to track | breaks |",
        "|---|---|---|---|---|---|",
    ]
    for name, results in rows.items():
        targets = [t for _, m in results for t in m["targets"]]
        ttt = [t["time_to_track_s"] for t in targets if t.get("time_to_track_s") is not None]
        lines.append(
            f"| {LATENCY[name][1]} | {mean([m['gospa']['mean_m'] for _, m in results]):.1f} m |"
            f" {mean([s['report_latency_s']['p50'] for s, _ in results]):.2f} s | {mean(ttt):.1f} s |"
            f" {max(ttt):.1f} s | {sum(t['id_switches'] + t['gaps'] for t in targets)} |"
        )
    lines += [
        "",
        "report age = host clock at publication minus the report's time (what a display or effector sees);",
        "late initiation lets a measurement released after the window still start a track if it is under 3 s late.",
    ]
    return "\n".join(lines)


def check(summary: dict, thresholds: dict) -> list[str]:
    """Regressions of the default configuration against the thresholds."""
    failures = []
    got = summary["default"]
    for key, limit in thresholds.items():
        if key.startswith("min_"):
            if got[key[4:]] < limit:
                failures.append(f"{key[4:]} = {got[key[4:]]:.3g}, below {limit}")
        elif key.startswith("max_"):
            if got[key[4:]] > limit:
                failures.append(f"{key[4:]} = {got[key[4:]]:.3g}, above {limit}")
    return failures


def clutter_table(build: Path, out: Path, seeds: list[int]) -> str:
    scenario = json.loads((ROOT / "scenarios" / "cuas_baseline.json").read_text())
    scenario["name"] = "clutter_only"
    scenario["description"] = "The baseline site with no targets: only false alarms."
    scenario["targets"] = []
    out.mkdir(parents=True, exist_ok=True)
    scenario_path = out / "clutter_only.json"
    scenario_path.write_text(json.dumps(scenario, indent=2))

    stats = {name: {"confirmed": 0, "lifetimes": []} for name in ("default", "strict")}
    for seed in seeds:
        run_dir = out / f"clutter_seed{seed}"
        run([build / "skytrack_sim", scenario_path, run_dir, "--seed", str(seed)])
        for name in stats:
            config, _ = CONFIGS[name]
            tracks_dir = run_dir / name
            cmd: list[str | Path] = [build / "skytrack_track", run_dir, "--out", tracks_dir]
            if config is not None:
                cmd += ["--config", config]
            run(cmd)
            summary = json.loads((tracks_dir / "summary.json").read_text())
            stats[name]["confirmed"] += summary["tracks_confirmed"]
            times: dict[str, list[float]] = {}
            with (tracks_dir / "tracks.csv").open() as f:
                header = f.readline().strip().split(",")
                t_i, id_i, st_i = header.index("t_s"), header.index("track_id"), header.index("status")
                for line in f:
                    cols = line.rstrip("\n").split(",")
                    if cols[st_i] == "confirmed":
                        times.setdefault(cols[id_i], []).append(float(cols[t_i]))
            stats[name]["lifetimes"] += [max(v) - min(v) for v in times.values()]

    hours = len(seeds) * scenario["duration_s"] / 3600.0
    lines = [
        f"Clutter only (the same site and sensors, no targets), {len(seeds)} seeds x 180 s:",
        "",
        "| configuration | confirmed false tracks | per hour | longest lived |",
        "|---|---|---|---|",
    ]
    for name, s in stats.items():
        longest = f"{max(s['lifetimes']):.1f} s" if s["lifetimes"] else "-"
        lines.append(f"| `{name}` | {s['confirmed']} | {s['confirmed'] / hours:.1f} | {longest} |")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", type=Path, default=ROOT / "build", help="directory with the built tools")
    parser.add_argument("--out", type=Path, default=ROOT / "runs" / "benchmark")
    parser.add_argument("--seeds", type=int, nargs="+", default=[42, 1, 2, 3, 4, 5, 6, 7])
    parser.add_argument("--clutter-seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6, 7, 8])
    parser.add_argument("--check", action="store_true", help="fail on a regression against tools/thresholds.json")
    args = parser.parse_args()

    baseline, summary = baseline_table(args.build, args.out, args.seeds)
    report = "\n\n".join(
        [baseline, latency_table(args.build, args.out, args.seeds), clutter_table(args.build, args.out, args.clutter_seeds)]
    )
    (args.out / "results.md").write_text(report + "\n")
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(report)
    if args.check:
        failures = check(summary, json.loads((ROOT / "tools" / "thresholds.json").read_text())["baseline"])
        for f in failures:
            print(f"REGRESSION: {f}", file=sys.stderr)
        sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
