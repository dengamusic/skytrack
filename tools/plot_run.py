#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
# ///
"""Plots one skytrack_sim run.

    uv run tools/plot_run.py runs/cuas_baseline

Writes three images to <run>/plots/:

  map.png       top-down view: true paths, radar plots, ADS-B / Remote ID
                positions and RF bearing lines
  timeline.png  when each sensor detected each target
  latency.png   how old each message is when it arrives

Detections are placed on the map with the *nominal* sensor setup from
sensors.json, which is all a tracker gets. Injected mounting errors therefore
show up as offsets between the plots and the true paths.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
FALSE_ALARM = -1


# ---- geometry (same conventions as include/skytrack/geo.hpp) ----------------


def geodetic_to_ecef(lat: float, lon: float, alt: float) -> np.ndarray:
    s, c = math.sin(lat), math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return np.array([(n + alt) * c * math.cos(lon), (n + alt) * c * math.sin(lon), (n * (1.0 - WGS84_E2) + alt) * s])


class LocalFrame:
    def __init__(self, lat_deg: float, lon_deg: float, alt_m: float = 0.0):
        lat, lon = math.radians(lat_deg), math.radians(lon_deg)
        self.origin = geodetic_to_ecef(lat, lon, alt_m)
        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        self.R = np.array([[-so, co, 0.0], [-sl * co, -sl * so, cl], [cl * co, cl * so, sl]])

    def geodetic_to_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
        ecef = geodetic_to_ecef(math.radians(lat_deg), math.radians(lon_deg), alt_m)
        return self.R @ (ecef - self.origin)


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def world_from_body(mount: dict) -> tuple[np.ndarray, np.ndarray]:
    """Rotation and position of a sensor body frame (x boresight, y left, z up) in ENU."""
    heading = math.radians(mount.get("heading_deg", 0.0))
    pitch = math.radians(mount.get("pitch_deg", 0.0))
    roll = math.radians(mount.get("roll_deg", 0.0))
    R = rot_z(math.pi / 2.0 - heading) @ rot_y(-pitch) @ rot_x(roll)
    return R, np.array(mount.get("position_enu_m", [0.0, 0.0, 0.0]), dtype=float)


# ---- loading ----------------------------------------------------------------


class Run:
    def __init__(self, path: Path):
        self.path = path
        doc = json.loads((path / "sensors.json").read_text())
        self.frame = LocalFrame(**doc["origin"])
        self.sensors = {s["id"]: s for s in doc["sensors"]}
        self.sensor_order = [s["id"] for s in doc["sensors"]]
        with (path / "measurements.jsonl").open() as f:
            self.messages = [json.loads(line) for line in f]

        self.origin: dict[int, tuple[int, float]] = {}
        with (path / "truth" / "measurement_origins.csv").open() as f:
            for row in csv.DictReader(f):
                self.origin[int(row["meas_id"])] = (int(row["target_id"]), float(row["t_true_s"]))

        self.truth: dict[int, list[tuple[float, float, float, float]]] = defaultdict(list)
        self.target_name: dict[int, str] = {}
        with (path / "truth" / "targets.csv").open() as f:
            for row in csv.DictReader(f):
                tid = int(row["target_id"])
                self.target_name[tid] = f'{row["name"]} ({row["class"]})'
                self.truth[tid].append(
                    (float(row["t_s"]), float(row["east_m"]), float(row["north_m"]), float(row["up_m"]))
                )

        with (path / "truth" / "messages.csv").open() as f:
            self.message_truth = list(csv.DictReader(f))

        cmap = plt.get_cmap("tab10")
        self.color = {tid: cmap(i % 10) for i, tid in enumerate(sorted(self.truth))}
        self.color[FALSE_ALARM] = (0.55, 0.55, 0.55, 1.0)


# ---- plots ------------------------------------------------------------------


def plot_map(run: Run, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 10))

    radar_pts: dict[int, list[np.ndarray]] = defaultdict(list)
    coop_pts: dict[int, list[np.ndarray]] = defaultdict(list)
    for msg in run.messages:
        sensor = run.sensors[msg["sensor_id"]]
        kind = msg["sensor_type"]
        R, pos = world_from_body(sensor.get("mount", {}))
        if kind == "radar":
            for d in msg["detections"]:
                r, az, el = d["range_m"], d["az_rad"], d["el_rad"]
                body = r * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
                radar_pts[run.origin[d["meas_id"]][0]].append(R @ body + pos)
        elif kind == "cooperative":
            for d in msg["detections"]:
                coop_pts[run.origin[d["meas_id"]][0]].append(
                    run.frame.geodetic_to_enu(d["lat_deg"], d["lon_deg"], d["alt_m"])
                )
        elif kind == "rf_bearing" and msg["seq"] % 6 == 0:  # thin out: one report in six
            length = sensor.get("rf_bearing", {}).get("max_range_m", 3000.0)
            for d in msg["detections"]:
                ray = R @ np.array([math.cos(d["az_rad"]), math.sin(d["az_rad"]), 0.0])
                end = pos + length * ray
                tid = run.origin[d["meas_id"]][0]
                ax.plot([pos[0], end[0]], [pos[1], end[1]], color=run.color[tid], lw=0.6, alpha=0.15, zorder=1)

    fa = np.array(radar_pts.pop(FALSE_ALARM, [])).reshape(-1, 3)
    ax.scatter(fa[:, 0], fa[:, 1], s=6, marker="x", color=run.color[FALSE_ALARM], alpha=0.35, lw=0.6, zorder=2)
    for tid, pts in radar_pts.items():
        p = np.array(pts)
        ax.scatter(p[:, 0], p[:, 1], s=7, color=run.color[tid], alpha=0.55, lw=0, zorder=3)
    for tid, pts in coop_pts.items():
        p = np.array(pts)
        ax.scatter(p[:, 0], p[:, 1], s=22, marker="s", facecolors="none", edgecolors=run.color[tid], lw=0.8, zorder=4)

    for tid, rows in run.truth.items():
        a = np.array(rows)
        ax.plot(a[:, 1], a[:, 2], "-", color=run.color[tid], lw=2.2, zorder=5)
        ax.plot(a[0, 1], a[0, 2], "o", color=run.color[tid], ms=6, zorder=6)
        ax.annotate(run.target_name[tid], (a[0, 1], a[0, 2]), textcoords="offset points", xytext=(6, 6), fontsize=9)

    placed: set[tuple[float, float]] = set()
    for sid in run.sensor_order:
        _, pos = world_from_body(run.sensors[sid].get("mount", {}))
        key = (round(pos[0] / 50.0), round(pos[1] / 50.0))
        ax.plot(pos[0], pos[1], "^", color="black", ms=8, zorder=7)
        if key not in placed:
            ax.annotate("sensors", (pos[0], pos[1]), textcoords="offset points", xytext=(-52, -14), fontsize=9)
            placed.add(key)

    # Frame the truth paths, not every far-away false alarm.
    all_truth = np.concatenate([np.array(r)[:, 1:3] for r in run.truth.values()])
    lo, hi = all_truth.min(axis=0), all_truth.max(axis=0)
    lo, hi = np.minimum(lo, 0.0), np.maximum(hi, 0.0)
    pad = 0.12 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(f"{run.path.name}: true paths and what the sensors report")
    ax.grid(alpha=0.3)
    ax.legend(
        handles=[
            Line2D([], [], color="black", lw=2.2, label="true path (dot = start)"),
            Line2D([], [], marker="o", ls="", color="black", ms=4, alpha=0.6, label="radar plots"),
            Line2D([], [], marker="x", ls="", color="grey", label="radar false alarms"),
            Line2D([], [], marker="s", ls="", mfc="none", color="black", label="ADS-B / Remote ID positions"),
            Line2D([], [], color="black", lw=0.8, alpha=0.4, label="RF bearings (1 in 6)"),
            Line2D([], [], marker="^", ls="", color="black", label="sensor site"),
        ],
        loc="lower left",
        fontsize=9,
    )
    fig.savefig(out / "map.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_timeline(run: Run, out: Path) -> None:
    ids = run.sensor_order
    fig, ax = plt.subplots(figsize=(13, 0.75 * len(ids) + 2))
    tids = sorted(run.truth)
    lane = {tid: i for i, tid in enumerate(tids)}
    step = 0.7 / max(len(tids), 1)
    for msg in run.messages:
        row = ids.index(msg["sensor_id"])
        for d in msg["detections"]:
            tid, t_true = run.origin[d["meas_id"]]
            y = row + 0.38 if tid == FALSE_ALARM else row - 0.35 + step * lane[tid]
            ax.plot(t_true, y, "|", color=run.color[tid], ms=4, alpha=0.7)
    ax.set_yticks(range(len(ids)), ids)
    ax.set_ylim(len(ids) - 0.5, -0.5)
    ax.set_xlabel("true time of measurement (s)")
    ax.set_title("detections per sensor (one lane per target; grey = false alarm)")
    ax.grid(axis="x", alpha=0.3)
    ax.legend(
        handles=[Line2D([], [], color=run.color[t], lw=3, label=run.target_name[t]) for t in tids]
        + [Line2D([], [], color=run.color[FALSE_ALARM], lw=3, label="false alarm")],
        loc="upper left",
        bbox_to_anchor=(1.01, 1.0),
        fontsize=9,
    )
    fig.savefig(out / "timeline.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_latency(run: Run, out: Path) -> None:
    age: dict[str, list[float]] = defaultdict(list)
    for row in run.message_truth:
        age[row["sensor_id"]].append(float(row["t_arrival_s"]) - float(row["t_true_s"]))
    fig, ax = plt.subplots(figsize=(10, 5))
    bins = np.logspace(-2.5, 1.5, 80)
    for sid in run.sensor_order:
        values = np.clip(age.get(sid, []), bins[0], bins[-1])
        ax.hist(values, bins=bins, histtype="step", lw=1.6, label=sid)
    ax.set_xscale("log")
    ax.set_xlabel("age of a message when it arrives: arrival time - true scan start (s)")
    ax.set_ylabel("messages")
    ax.set_title("message age at arrival (a rotating radar adds its scan time)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.savefig(out / "latency.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run", type=Path, help="output directory of skytrack_sim")
    args = parser.parse_args()

    run = Run(args.run)
    out = args.run / "plots"
    out.mkdir(exist_ok=True)
    plot_map(run, out)
    plot_timeline(run, out)
    plot_latency(run, out)
    for name in ("map.png", "timeline.png", "latency.png"):
        print(out / name)


if __name__ == "__main__":
    main()
