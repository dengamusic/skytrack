#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
# ///
"""Plots one tracker run against the simulator's ground truth.

    ./build/skytrack_track runs/cuas_baseline
    ./build/skytrack_eval  runs/cuas_baseline
    uv run tools/plot_tracks.py runs/cuas_baseline

Reads <run>/truth/targets.csv, <run>/tracker/tracks.csv and the scores in
<run>/tracker/eval/, and writes three images to <run>/tracker/plots/:

  tracks.png  top-down view: true paths and the confirmed tracks on them,
              coloured by the target each track followed (grey = false track)
  gospa.png   GOSPA over time, and how many targets were missed and how many
              false tracks were reported at each output time
  errors.png  horizontal position error of each target's track over time,
              with the track's own 2-sigma bound
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

# Categorical slots in fixed order, assigned by target id (never by rank).
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
FALSE_TRACK = "#8a8985"
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": GRID,
        "axes.labelcolor": TEXT_SECONDARY,
        "axes.titlecolor": TEXT,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": TEXT_SECONDARY,
        "ytick.color": TEXT_SECONDARY,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.frameon": False,
        "legend.fontsize": 8,
        "font.size": 9,
    }
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


class Run:
    def __init__(self, run_dir: Path, tracks_dir: Path):
        self.truth: dict[int, np.ndarray] = {}  # id -> rows of (t, e, n, u)
        self.name: dict[int, str] = {}
        rows = defaultdict(list)
        for r in read_csv(run_dir / "truth" / "targets.csv"):
            tid = int(r["target_id"])
            self.name[tid] = r["name"]
            rows[tid].append([float(r["t_s"]), float(r["east_m"]), float(r["north_m"]), float(r["up_m"])])
        self.truth = {tid: np.array(v) for tid, v in sorted(rows.items())}
        self.color = {tid: SERIES[i % len(SERIES)] for i, tid in enumerate(self.truth)}

        # Confirmed track positions: track id -> rows of (t, e, n, sigma_h).
        tracks = defaultdict(list)
        for r in read_csv(tracks_dir / "tracks.csv"):
            if r["status"] != "confirmed":
                continue
            sigma_h = np.sqrt(max(float(r["p_e_e"]) + float(r["p_n_n"]), 0.0) / 2.0)
            tracks[int(r["track_id"])].append([float(r["t_s"]), float(r["east_m"]), float(r["north_m"]), sigma_h])
        self.tracks = {k: np.array(v) for k, v in tracks.items()}

        eval_dir = tracks_dir / "eval"
        self.metrics = json.loads((eval_dir / "metrics.json").read_text())
        self.gospa = read_csv(eval_dir / "gospa.csv")
        self.matches = read_csv(eval_dir / "matches.csv")
        # Each track's main target: the one it was matched to most often.
        votes: dict[int, Counter] = defaultdict(Counter)
        for m in self.matches:
            votes[int(m["track_id"])][int(m["target_id"])] += 1
        false_ids = {t["id"] for t in self.metrics["tracks"]["list"] if t["false"]}
        self.track_target = {
            k: (votes[k].most_common(1)[0][0] if votes[k] and k not in false_ids else None) for k in self.tracks
        }


def plot_map(run: Run, out: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.4), gridspec_kw={"width_ratios": [1.0, 1.0]})
    near = 2500.0  # half-width of the close-up, metres
    for ax, close_up in zip(axes, (False, True)):
        for tid, a in run.truth.items():
            ax.plot(a[:, 1], a[:, 2], color=run.color[tid], lw=5, alpha=0.22, solid_capstyle="round", zorder=1)
        for k, a in run.tracks.items():
            target = run.track_target[k]
            color = run.color[target] if target is not None else FALSE_TRACK
            ax.plot(a[:, 1], a[:, 2], color=color, lw=1.4, zorder=3)
            ax.plot(a[0, 1], a[0, 2], "o", ms=4, color=color, mec=SURFACE, mew=1.0, zorder=4)
            inside = not close_up or (abs(a[0, 1]) < near and abs(a[0, 2]) < near)
            if inside:
                ax.annotate(f"T{k}", (a[0, 1], a[0, 2]), xytext=(4, 4), textcoords="offset points",
                            fontsize=7, color=TEXT_SECONDARY, zorder=5)
        for tid, a in run.truth.items():
            # Name each path where it is visible, so colour never carries identity alone.
            idx = 0
            if close_up:
                ok = np.where((np.abs(a[:, 1]) < 0.9 * near) & (np.abs(a[:, 2]) < 0.9 * near))[0]
                if ok.size == 0:
                    continue
                idx = int(ok[0])
            ax.annotate(run.name[tid], (a[idx, 1], a[idx, 2]), xytext=(6, -10), textcoords="offset points",
                        fontsize=8, color=TEXT, weight="bold", zorder=6)
        ax.plot(0, 0, "^", color=TEXT, ms=8, zorder=7)
        ax.set_aspect("equal", adjustable="box" if close_up else "datalim")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        if close_up:
            ax.set_xlim(-near, near)
            ax.set_ylim(-near, near)
            ax.set_title("Close to the site")
        else:
            ax.set_title("Whole scene")

    handles = [
        Line2D([], [], color=TEXT_SECONDARY, lw=5, alpha=0.3, label="true path"),
        Line2D([], [], color=TEXT_SECONDARY, lw=1.4, marker="o", ms=4, mec=SURFACE, label="confirmed track (dot = confirmed)"),
        Line2D([], [], color=FALSE_TRACK, lw=1.4, label="false track"),
        Line2D([], [], marker="^", ls="", color=TEXT, label="sensor site"),
    ] + [Line2D([], [], color=run.color[t], lw=3, label=run.name[t]) for t in run.truth]
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("Confirmed tracks over the true paths", color=TEXT, weight="bold")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out / "tracks.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_gospa(run: Run, out: Path) -> None:
    t = np.array([float(r["t_s"]) for r in run.gospa])
    g = np.array([float(r["gospa_m"]) for r in run.gospa])
    missed = np.array([int(r["missed"]) for r in run.gospa])
    false = np.array([int(r["false_tracks"]) for r in run.gospa])
    c = run.metrics["gospa"]["c_m"]

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(10, 5.2), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    ax.plot(t, g, color=SERIES[0], lw=1.6)
    ax.axhline(np.mean(g), color=TEXT_SECONDARY, lw=0.8, ls="--")
    ax.annotate(f"mean {np.mean(g):.1f} m", (t[-1], np.mean(g)), xytext=(-4, 4), textcoords="offset points",
                ha="right", fontsize=8, color=TEXT_SECONDARY)
    ax.set_ylabel("GOSPA (m)")
    ax.set_ylim(bottom=0)
    ax.set_title(f"GOSPA (c = {c:g} m, p = 2): localisation error plus c/sqrt(2) per missed target or false track")

    ax2.step(t, missed, where="post", color=SERIES[1], lw=1.4, label="targets missed")
    ax2.step(t, false, where="post", color=FALSE_TRACK, lw=1.4, label="false tracks")
    ax2.set_ylabel("count")
    ax2.set_xlabel("time (s)")
    ax2.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax2.legend(loc="upper right", ncol=2)
    fig.tight_layout()
    fig.savefig(out / "gospa.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_errors(run: Run, out: Path) -> None:
    by_target: dict[int, list[dict[str, str]]] = defaultdict(list)
    for m in run.matches:
        by_target[int(m["target_id"])].append(m)
    tids = list(run.truth)
    fig, axes = plt.subplots(len(tids), 1, figsize=(10, 1.6 * len(tids) + 0.6), sharex=True, squeeze=False)
    sigma_at = {k: dict(zip(a[:, 0], a[:, 3])) for k, a in run.tracks.items()}
    scores = {s["id"]: s for s in run.metrics["targets"]}
    for ax, tid in zip(axes[:, 0], tids):
        rows = by_target.get(tid, [])
        if rows:
            t = np.array([float(r["t_s"]) for r in rows])
            err = np.array([float(r["error_horizontal_m"]) for r in rows])
            bound = np.array([2.0 * np.sqrt(2.0) * sigma_at[int(r["track_id"])].get(float(r["t_s"]), np.nan)
                              for r in rows])
            # Break the line where the target had no track.
            gaps = np.where(np.diff(t) > 1.5 * np.median(np.diff(t)))[0] + 1 if t.size > 1 else []
            for seg_t, seg_e, seg_b in zip(np.split(t, gaps), np.split(err, gaps), np.split(bound, gaps)):
                ax.fill_between(seg_t, 0, seg_b, color=run.color[tid], alpha=0.13, lw=0)
                ax.plot(seg_t, seg_e, color=run.color[tid], lw=1.3)
        s = scores.get(tid, {})
        label = run.name[tid]
        if s:
            label += f"   RMSE {s['rmse_horizontal_m']:.1f} m, NEES {s['nees_mean']:.1f}"
        ax.text(0.005, 0.92, label, transform=ax.transAxes, va="top", fontsize=8.5, color=TEXT, weight="bold")
        ax.set_ylim(bottom=0)
        ax.set_ylabel("m")
    axes[-1, 0].set_xlabel("time (s)")
    fig.suptitle("Horizontal position error per target (shaded: the track's own ~2-sigma bound)",
                 color=TEXT, weight="bold")
    fig.tight_layout()
    fig.savefig(out / "errors.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path, help="run directory written by skytrack_sim")
    parser.add_argument("--tracks", type=Path, help="tracker output directory (default: <run>/tracker)")
    args = parser.parse_args()
    tracks_dir = args.tracks or args.run / "tracker"
    if not (tracks_dir / "eval" / "metrics.json").exists():
        raise SystemExit(f"{tracks_dir}/eval/metrics.json not found: run skytrack_track and skytrack_eval first")
    run = Run(args.run, tracks_dir)
    out = tracks_dir / "plots"
    out.mkdir(parents=True, exist_ok=True)
    plot_map(run, out)
    plot_gospa(run, out)
    plot_errors(run, out)
    print(f"wrote {out}/tracks.png, gospa.png, errors.png")


if __name__ == "__main__":
    main()
