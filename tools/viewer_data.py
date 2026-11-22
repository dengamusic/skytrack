#!/usr/bin/env python3
"""Builds the replay viewer: runs three scenes and packs each into one JSON file.

    cmake --build build
    python3 tools/viewer_data.py                      # -> runs/viewer/site/
    python3 -m http.server -d runs/viewer/site 8000   # open http://localhost:8000

Scenes (all seed 42, the seed the README's single-run plots use):

  baseline   one site, the pan-tilt camera on its hand-written schedule
  two_sites  two nodes 3.2 km apart; their tracks and the fusion centre's
  cued       one site, the pan-tilt camera pointed by the tracker in closed loop

Each data file holds the sensors, the true paths, the tracks (with their
horizontal covariance), every detection placed on the map the way the
tracker sees it (radar and broadcasts as points, cameras and the RF direction
finder as bearing lines from their nominal mounts), the camera's pointing,
GOSPA over time and the summary scores. Standard library only, so CI needs no
extra packages.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Text shown under each scene's title in the viewer; times are checked against
# the scenario files (the intruder brakes to a hover at 90-95 s and leaves at 125 s).
SCENES = {
    "baseline": {
        "title": "One site",
        "blurb": "Rotating radar, a wide camera, a zoom camera on a scripted pan-tilt schedule, "
                 "an RF direction finder, ADS-B and Remote ID receivers and a delayed online flight feed, "
                 "all fused into one track picture.",
        "notes": [
            {"t0": 90, "t1": 95, "text": "The intruder brakes to a hover."},
            {"t0": 95, "t1": 125, "text": "The intruder hovers. A Doppler radar cannot see a target with no "
                                          "radial speed, so the tracker infers \"range rate about 0\" from the "
                                          "radar's silence and holds the track while the cameras and RF keep it updated."},
            {"t0": 125, "t1": 149, "text": "The intruder climbs away and turns."},
        ],
    },
    "two_sites": {
        "title": "Two sites + fusion centre",
        "blurb": "Two sensor nodes 3.2 km apart each track on their own sensors and send their tracks to a "
                 "fusion centre, which combines them by covariance intersection (it stays honest about the "
                 "errors the nodes share). Switch between the fused picture and each node's own.",
        "notes": [],
    },
    "cued": {
        "title": "Camera cued by the tracker",
        "blurb": "The zoom camera is no longer on a schedule: the tracker points it, slewing at most 60 deg/s, "
                 "at whatever most needs a look (new unclassified tracks first, then re-checks, then keeping "
                 "custody of drones). It is the only setup that gets both birds classified.",
        "notes": [
            {"t0": 95, "t1": 125, "text": "The intruder hovers inside the radar's Doppler notch."},
        ],
    },
}

# Track rows: [t, east, north, up, v_east, v_north, P_ee, P_en, P_nn, status, class, identity,
#              sensors, last_update_t]; class, identity and sensors index the set's string tables.
TRACK_STATUS = ["tentative", "confirmed"]


def run(cmd: list) -> None:
    subprocess.run([str(c) for c in cmd], check=True, stdout=subprocess.DEVNULL)


# ---- geometry (mirrors src/geo.cpp and the sensor models) -------------------

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return ((n + alt_m) * math.cos(lat) * math.cos(lon),
            (n + alt_m) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - WGS84_E2) + alt_m) * math.sin(lat))


class Enu:
    def __init__(self, origin: dict):
        self.lat = math.radians(origin["lat_deg"])
        self.lon = math.radians(origin["lon_deg"])
        self.o = geodetic_to_ecef(origin["lat_deg"], origin["lon_deg"], origin.get("alt_m", 0.0))

    def from_geodetic(self, lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
        x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        dx, dy, dz = x - self.o[0], y - self.o[1], z - self.o[2]
        sl, cl, so, co = math.sin(self.lat), math.cos(self.lat), math.sin(self.lon), math.cos(self.lon)
        e = -so * dx + co * dy
        n = -sl * co * dx - sl * so * dy + cl * dz
        u = cl * co * dx + cl * so * dy + sl * dz
        return e, n, u


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def world_from_body(heading_deg: float, pitch_deg: float, roll_deg: float):
    # Body x is the boresight; a compass heading h is a yaw of 90 deg - h.
    yaw = math.pi / 2 - math.radians(heading_deg)
    return matmul(matmul(rot_z(yaw), rot_y(-math.radians(pitch_deg))), rot_x(math.radians(roll_deg)))


def compass_deg(east: float, north: float) -> float:
    return math.degrees(math.atan2(east, north)) % 360.0


# ---- reading a run -------------------------------------------------------------

def read_csv(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


class SensorGeometry:
    """Turns one sensor's detections into map coordinates using its nominal mount."""

    def __init__(self, spec: dict):
        self.id = spec["id"]
        self.type = spec["type"]
        mount = spec.get("mount", {})
        self.pos = mount.get("position_enu_m")
        self.R = world_from_body(mount.get("heading_deg", 0.0), mount.get("pitch_deg", 0.0), mount.get("roll_deg", 0.0))
        cam = spec.get("camera", {})
        self.fx = cam.get("fx_px", 1400.0)
        self.fy = cam.get("fy_px", self.fx)
        self.width = cam.get("width_px", 1920)
        self.height = cam.get("height_px", 1080)
        self.cx = cam.get("cx_px", self.width / 2)
        self.cy = cam.get("cy_px", self.height / 2)
        self.k1, self.k2 = cam.get("k1", 0.0), cam.get("k2", 0.0)
        self.range_m = (spec.get("radar") or spec.get("camera") or spec.get("rf_bearing") or {}).get("max_range_m")

    def view_rotation(self, gimbal: dict | None):
        if not gimbal:
            return self.R
        # Pan turns clockwise about the mount's up axis, then tilt raises the boresight.
        return matmul(self.R, matmul(rot_z(-gimbal["pan_rad"]), rot_y(-gimbal["tilt_rad"])))

    def boresight_deg(self, gimbal: dict | None) -> float:
        d = matvec(self.view_rotation(gimbal), [1.0, 0.0, 0.0])
        return compass_deg(d[0], d[1])

    def hfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(0.5 * self.width / self.fx))

    def body_point(self, rng: float, az: float, el: float) -> tuple[float, float]:
        b = [rng * math.cos(el) * math.cos(az), rng * math.cos(el) * math.sin(az), rng * math.sin(el)]
        w = matvec(self.R, b)
        return self.pos[0] + w[0], self.pos[1] + w[1]

    def body_bearing_deg(self, az: float) -> float:
        d = matvec(self.R, [math.cos(az), math.sin(az), 0.0])
        return compass_deg(d[0], d[1])

    def pixel_bearing_deg(self, u: float, v: float, gimbal: dict | None) -> float:
        xn, yn = (u - self.cx) / self.fx, (v - self.cy) / self.fy
        if self.k1 or self.k2:  # undistort by fixed-point iteration
            xd, yd = xn, yn
            for _ in range(10):
                r2 = xn * xn + yn * yn
                f = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
                xn, yn = xd / f, yd / f
        d = matvec(self.view_rotation(gimbal), [1.0, -xn, -yn])  # (forward, left, up)
        return compass_deg(d[0], d[1])


def export_tracks(tracks_dir: Path) -> dict:
    """Tracks, their best-matched target, GOSPA and the summary scores of one track set."""
    rows = read_csv(tracks_dir / "tracks.csv")
    classes: list[str] = []
    idents: list[str] = []
    sensor_sets: list[str] = []

    def index(table: list[str], value: str) -> int:
        if value not in table:
            table.append(value)
        return table.index(value)

    tracks: dict[int, list] = defaultdict(list)
    for r in rows:
        status = TRACK_STATUS.index(r["status"]) if r["status"] in TRACK_STATUS else 0
        tracks[int(r["track_id"])].append([
            round(float(r["t_s"]), 3),
            round(float(r["east_m"]), 1), round(float(r["north_m"]), 1), round(float(r["up_m"]), 1),
            round(float(r["vel_east_mps"]), 2), round(float(r["vel_north_mps"]), 2),
            round(float(r["p_e_e"]), 2), round(float(r["p_e_n"]), 2), round(float(r["p_n_n"]), 2),
            status, index(classes, r["class"]), index(idents, r["identity"]),
            index(sensor_sets, r["sensors"]), round(float(r["last_update_s"]), 2),
        ])

    eval_dir = tracks_dir / "eval"
    matched: dict[int, Counter] = defaultdict(Counter)
    for m in read_csv(eval_dir / "matches.csv"):
        matched[int(m["track_id"])][int(m["target_id"])] += 1
    gospa = [[float(g["t_s"]), round(float(g["gospa_m"]), 2), int(g["missed"]), int(g["false_tracks"])]
             for g in read_csv(eval_dir / "gospa.csv")]
    metrics = json.loads((eval_dir / "metrics.json").read_text())
    return {
        "classes": classes,
        "identities": idents,
        "sensor_sets": sensor_sets,
        "tracks": [{"id": tid, "target": (matched[tid].most_common(1)[0][0] if matched[tid] else None),
                    "rows": samples} for tid, samples in sorted(tracks.items())],
        "gospa": gospa,
        "metrics": {
            "gospa_mean_m": round(metrics["gospa"]["mean_m"], 2),
            "false_tracks_mean": round(metrics["gospa"]["mean_false_tracks"], 3),
            "targets": [{
                "id": t["id"], "name": t["name"], "class": t["class"],
                "tracked_fraction": round(t["tracked_fraction"], 3),
                "rmse_horizontal_m": round(t["rmse_horizontal_m"], 1),
                "nees_mean": round(t["nees_mean"], 2),
                "breaks": t["id_switches"] + t["gaps"],
                "time_to_classify_s": t.get("time_to_classify_s"),
                "class_correct": round(t["class_correct"], 2),
            } for t in metrics["targets"]],
        },
    }


def export_scene(name: str, run_dir: Path, track_sets: dict[str, Path], sites: dict[str, list[str]] | None) -> dict:
    sensors_json = json.loads((run_dir / "sensors.json").read_text())
    enu = Enu(sensors_json["origin"])
    geometry = {s["id"]: SensorGeometry(s) for s in sensors_json["sensors"]}
    sensor_ids = [s["id"] for s in sensors_json["sensors"]]
    sensor_index = {sid: i for i, sid in enumerate(sensor_ids)}
    site_of = {sid: site for site, ids in (sites or {}).items() for sid in ids}

    sensors = []
    for s in sensors_json["sensors"]:
        g = geometry[s["id"]]
        entry = {"id": s["id"], "type": s["type"], "site": site_of.get(s["id"], "site-a" if g.pos else None)}
        if g.pos:
            entry["pos"] = [round(g.pos[0], 1), round(g.pos[1], 1)]
        if g.range_m:
            entry["range_m"] = g.range_m
        if s["type"] == "camera":
            entry["hfov_deg"] = round(g.hfov_deg(), 3)
            entry["boresight_deg"] = round(g.boresight_deg(None), 2)
            entry["gimballed"] = "gimbal" in s.get("camera", {})
        if s["type"] == "cooperative":
            entry["protocol"] = s["cooperative"]["protocol"]
        sensors.append(entry)

    # Truth: every 0.5 s is plenty for drawing, and halves the file.
    targets: dict[int, dict] = {}
    for r in read_csv(run_dir / "truth" / "targets.csv"):
        t = float(r["t_s"])
        if abs(t * 2 - round(t * 2)) > 1e-6:
            continue
        tid = int(r["target_id"])
        entry = targets.setdefault(tid, {"id": tid, "name": r["name"], "class": r["class"], "rows": []})
        entry["rows"].append([t, round(float(r["east_m"]), 1), round(float(r["north_m"]), 1), round(float(r["up_m"]), 1)])

    origin = {int(r["meas_id"]): int(r["target_id"]) for r in read_csv(run_dir / "truth" / "measurement_origins.csv")}

    # Detections: [t_sensor, sensor index, kind, a, b, truth target (-1 = false alarm), label index]
    # kind 0 = point (a, b = east, north); kind 1 = bearing line (a = compass deg, b = 0).
    labels: list[str] = []
    detections = []
    pointing: dict[str, list] = defaultdict(list)
    with open(run_dir / "measurements.jsonl") as f:
        for line in f:
            m = json.loads(line)
            sid = m["sensor_id"]
            g = geometry[sid]
            gimbal = m.get("gimbal")
            t_msg = m.get("t_sensor_s")
            if gimbal and t_msg is not None:
                pointing[sid].append([round(t_msg, 3), round(g.boresight_deg(gimbal), 2)])
            for d in m["detections"]:
                t = d.get("t_sensor_s")
                if t is None:
                    t = m["t_arrival_s"]  # the RF finder stamps nothing; the tracker uses arrival time too
                label = d.get("label") or d.get("signature") or d.get("identity") or ""
                if label not in labels:
                    labels.append(label)
                truth = origin.get(d["meas_id"], -1)
                if m["sensor_type"] == "radar":
                    e, n = g.body_point(d["range_m"], d["az_rad"], d["el_rad"])
                    det = [0, round(e, 1), round(n, 1)]
                elif m["sensor_type"] == "cooperative":
                    e, n, _ = enu.from_geodetic(d["lat_deg"], d["lon_deg"], d["alt_m"])
                    det = [0, round(e, 1), round(n, 1)]
                elif m["sensor_type"] == "camera":
                    det = [1, round(g.pixel_bearing_deg(d["u_px"], d["v_px"], gimbal), 3), 0]
                elif m["sensor_type"] == "rf_bearing":
                    det = [1, round(g.body_bearing_deg(d["az_rad"]), 3), 0]
                else:
                    continue
                detections.append([round(t, 3), sensor_index[sid], *det, truth, labels.index(label)])
    detections.sort(key=lambda d: d[0])

    summary = json.loads((run_dir / "summary.json").read_text())
    duration = float(summary.get("duration_s") or max(r[0] for t in targets.values() for r in t["rows"]))
    scene = SCENES[name]
    return {
        "name": name,
        "title": scene["title"],
        "blurb": scene["blurb"],
        "notes": scene["notes"],
        "duration_s": duration,
        "sensors": sensors,
        "targets": sorted(targets.values(), key=lambda t: t["id"]),
        "labels": labels,
        "detections": detections,
        "pointing": {sid: rows for sid, rows in pointing.items()},
        "track_sets": {label: export_tracks(path) for label, path in track_sets.items()},
    }


# ---- the three scenes ----------------------------------------------------------

def build(bin_dir: Path, work: Path, out: Path) -> None:
    scenarios = ROOT / "scenarios"
    work.mkdir(parents=True, exist_ok=True)
    (out / "data").mkdir(parents=True, exist_ok=True)
    scenes = {}

    base = work / "baseline"
    run([bin_dir / "skytrack_sim", scenarios / "cuas_baseline.json", base])
    run([bin_dir / "skytrack_track", base])
    run([bin_dir / "skytrack_eval", base])
    scenes["baseline"] = export_scene("baseline", base, {"tracker": base / "tracker"}, None)

    two = work / "two_sites"
    nodes = json.loads((ROOT / "configs" / "nodes" / "two_sites.json").read_text())["nodes"]
    run([bin_dir / "skytrack_sim", scenarios / "two_sites.json", two])
    for node, sensors in nodes.items():
        run([bin_dir / "skytrack_track", two, "--sensors", ",".join(sensors), "--out", two / node])
        run([bin_dir / "skytrack_eval", two, "--tracks", two / node])
    run([bin_dir / "skytrack_fuse", *(two / n for n in nodes), "--out", two / "fused"])
    run([bin_dir / "skytrack_eval", two, "--tracks", two / "fused"])
    # A sensor both nodes take (the online feed) belongs to neither site on the map.
    sites = {node: [s for s in sensors if sum(s in other for other in nodes.values()) == 1]
             for node, sensors in nodes.items()}
    scenes["two_sites"] = export_scene("two_sites", two, {"fused": two / "fused",
                                                          **{n: two / n for n in nodes}}, sites)

    cued = work / "cued"
    run([bin_dir / "skytrack_cue", scenarios / "cuas_baseline.json", cued])
    run([bin_dir / "skytrack_eval", cued])
    scenes["cued"] = export_scene("cued", cued, {"tracker": cued / "tracker"}, None)

    index = []
    for name, scene in scenes.items():
        path = out / "data" / f"{name}.json"
        path.write_text(json.dumps(scene, separators=(",", ":")))
        index.append({"name": name, "title": scene["title"], "file": f"data/{name}.json"})
        print(f"{path.relative_to(out)}: {path.stat().st_size / 1e3:.0f} kB")
    (out / "data" / "scenes.json").write_text(json.dumps(index, indent=1))
    for f in (ROOT / "viewer").iterdir():
        if f.is_file():
            shutil.copy2(f, out / f.name)
    print(f"site in {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", type=Path, default=ROOT / "build", help="directory with the skytrack binaries")
    parser.add_argument("--work", type=Path, default=ROOT / "runs" / "viewer", help="where the runs go")
    parser.add_argument("--out", type=Path, default=None, help="the site (default: <work>/site)")
    args = parser.parse_args()
    build(args.build, args.work, args.out or args.work / "site")


if __name__ == "__main__":
    main()
