"""
plot_trajectory.py — Plot planned waypoints vs. flown trajectory for a
flight_engine.py mission log.

Reads the paired nav_log_<ts>.txt / .csv (or any renamed copy of them, e.g.
Logs/5.txt + Logs/5.csv) and plots three things on one map:
  - Planned route: approx_start -> waypoint 1 -> waypoint 2 -> ...
    (parsed from the engine_start line in the .txt log)
  - EKF trajectory: the position the navigator was actually steering from
    (ekf_lat/ekf_lon columns in the .csv)
  - True GPS trajectory: the raw MAVLink GPS position (gps_lat/gps_lon)

Usage:
    python3 plot_trajectory.py <path/to/N.txt> [<path/to/N.csv>] [-o out.png]

    If the .csv path is omitted, it's assumed to sit next to the .txt file
    with the same stem (N.txt -> N.csv).

Example:
    python3 plot_trajectory.py Logs/5.txt -o Logs/5_trajectory.png
    python3 plot_trajectory.py Logs/6.txt -o Logs/6_trajectory.png
"""
import argparse
import ast
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def parse_mission_config(txt_path):
    """Extract the `config={...}` dict from the engine_start line."""
    with open(txt_path) as f:
        for line in f:
            if "engine_start" not in line or "config=" not in line:
                continue
            start = line.index("config=") + len("config=")
            depth = 0
            end = None
            for i, ch in enumerate(line[start:], start):
                if ch == "{":
                    depth += 1
                elif ch == "}":
                    depth -= 1
                    if depth == 0:
                        end = i + 1
                        break
            if end is None:
                raise ValueError(f"could not find matching '}}' for config= in {txt_path}")
            return ast.literal_eval(line[start:end])
    raise ValueError(f"no engine_start line found in {txt_path}")


def planned_route(config):
    """[(lat, lon, label), ...] — approx_start followed by each waypoint."""
    route = []
    approx_start = config.get("approx_start")
    if approx_start:
        route.append((approx_start[0], approx_start[1], "start (visual)"))
    for i, wp in enumerate(config.get("waypoints", []), start=1):
        route.append((wp["lat"], wp["lon"], f"waypoint {i}"))
    return route


def plot_one(txt_path, csv_path, out_path):
    config = parse_mission_config(txt_path)
    route = planned_route(config)
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots(figsize=(9, 9))

    # Planned route: dashed line through approx_start -> waypoints, numbered.
    import numpy as np
    route_lats = np.array([p[0] for p in route])
    route_lons = np.array([p[1] for p in route])
    ax.plot(route_lons, route_lats, "k--", lw=1.5, marker="o",
            markersize=8, markerfacecolor="white", markeredgecolor="black",
            zorder=5, label="planned route")
    for lat, lon, label in route:
        ax.annotate(label, (lon, lat), textcoords="offset points",
                    xytext=(8, 8), fontsize=9, zorder=6)

    # Flown trajectory: EKF (what the navigator steered from) vs raw GPS.
    ekf_lon = df["ekf_lon"].to_numpy()
    ekf_lat = df["ekf_lat"].to_numpy()
    gps_lon = df["gps_lon"].to_numpy()
    gps_lat = df["gps_lat"].to_numpy()
    ax.plot(ekf_lon, ekf_lat, "b-", lw=1.3, alpha=0.85,
            zorder=3, label="EKF trajectory (flown)")
    ax.plot(gps_lon, gps_lat, "g-", lw=1.3, alpha=0.85,
            zorder=2, label="true GPS trajectory")

    # Mark flight start/end on the EKF trace.
    ax.scatter(ekf_lon[0], ekf_lat[0],
               c="blue", marker="^", s=90, zorder=7, label="EKF start")
    ax.scatter(ekf_lon[-1], ekf_lat[-1],
               c="blue", marker="s", s=90, zorder=7, label="EKF end")

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    mode = config.get("mode", "?")
    ax.set_title(f"{os.path.basename(txt_path)} — mode={mode}\n"
                 f"planned route vs. flown trajectory")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", "datalim")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"[Saved] {out_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("txt", help="path to the nav_log_*.txt file")
    ap.add_argument("csv", nargs="?", default=None,
                    help="path to the matching .csv (default: same stem as txt)")
    ap.add_argument("-o", "--out", default=None,
                    help="output PNG path (default: <txt stem>_trajectory.png)")
    args = ap.parse_args()

    csv_path = args.csv or os.path.splitext(args.txt)[0] + ".csv"
    out_path = args.out or os.path.splitext(args.txt)[0] + "_trajectory.png"
    plot_one(args.txt, csv_path, out_path)


if __name__ == "__main__":
    main()
