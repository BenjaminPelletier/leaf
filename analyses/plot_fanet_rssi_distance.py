#!/usr/bin/env python3
"""Plot RSSI versus distance for each Fanet device from a bus log.

This script parses a bus log file and looks for comment lines that record
``fanet_rx`` messages.  For each message it extracts the distance and RSSI
for the transmitting Fanet device and then renders a scatter plot with one
color per FanetID.  Entries with missing position information or with
distance closer than 0.1 km are ignored.

Usage:
    python plot_fanet_rssi_distance.py path/to/BusLog.log [--out plot.png]

If ``--out`` is omitted, the script saves the figure alongside the log file
using the same base name but a ``.png`` extension.
"""

from __future__ import annotations

import argparse
import collections
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

MIN_DISTANCE_KM = 0.1


def parse_log(filepath: str) -> Dict[str, Dict[str, List[float]]]:
    """Parse the log file and return distance/RSSI pairs grouped by FanetID."""
    data: Dict[str, Dict[str, List[float]]] = collections.defaultdict(
        lambda: {"dist": [], "rssi": []}
    )
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line.startswith("#"):
                continue
            content = line[1:]
            first_comma = content.find(",")
            if first_comma == -1:
                continue
            rest = content[first_comma + 1 :]
            if not rest.startswith("fanet_rx,"):
                continue
            parts = rest.split(",")
            if len(parts) < 9:
                continue
            fanet_id = parts[1]
            try:
                distance = float(parts[2])
                rssi = float(parts[3])
                my_lat = float(parts[7])
                my_lon = float(parts[8])
            except ValueError:
                continue
            if my_lat == 0.0 or my_lon == 0.0:
                continue
            if distance < MIN_DISTANCE_KM:
                continue
            data[fanet_id]["dist"].append(distance)
            data[fanet_id]["rssi"].append(rssi)
    return data


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_data(data: Dict[str, Dict[str, List[float]]], out_path: str | None) -> None:
    """Render the scatter plot either interactively or to an image file."""
    for fanet_id, vals in data.items():
        plt.scatter(vals["dist"], vals["rssi"], s=15, label=fanet_id)
    plt.xlabel("Distance (km)")
    plt.ylabel("RSSI (dBm)")
    plt.title("RSSI vs Distance by FanetID")
    plt.legend(title="FanetID", fontsize="small", markerscale=0.8)
    if out_path:
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close()
    else:
        plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logfile", help="Path to bus log file")
    ap.add_argument(
        "--out",
        help=(
            "Write plot to this image file (defaults to replacing the log file "
            "extension with .png)"
        ),
    )
    args = ap.parse_args()
    data = parse_log(args.logfile)
    if not data:
        print("No fanet_rx entries found.")
        return
    if args.out:
        out_path = None if args.out == "-" else args.out
    else:
        out_path = str(Path(args.logfile).with_suffix(".png"))
    plot_data(data, out_path)


if __name__ == "__main__":
    main()
