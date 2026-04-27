#!/usr/bin/env python
#
# *********     Stream Log Plotter      *********
#
# Time plots for CSV logs produced by keyboard_stream.py.
#
# Expected CSV schema (header row required):
#   t,target,commanded,present_pos,present_load,in_contact
#     t            seconds since stream start (monotonic)
#     target       user-commanded goal tick
#     commanded    tick actually written to the servo (== freeze_pos in contact)
#     present_pos  PRESENT_POSITION read back from the servo
#     present_load PRESENT_LOAD magnitude (0..1023)
#     in_contact   0 or 1
#
# Usage:
#   python3 plot_stream.py --csv run.csv
#   python3 plot_stream.py --csv run.csv --out run.png --show
#

import csv
import os
import sys
from dataclasses import dataclass
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import tyro


@dataclass
class Args:
    csv: str
    """Path to the CSV log to plot."""
    out: Optional[str] = None
    """If set, save figure to this path (PNG/PDF/SVG by extension). Defaults to <csv>.png."""
    show: bool = False
    """Open an interactive window in addition to saving."""
    title: Optional[str] = None
    """Optional figure title. Defaults to the CSV filename."""


def load_csv(path: str) -> dict:
    cols: dict = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames or []:
            cols[name] = []
        for row in reader:
            for name, val in row.items():
                cols[name].append(val)
    return {name: np.array(vals, dtype=float) for name, vals in cols.items()}


def contact_spans(t: np.ndarray, in_contact: np.ndarray) -> list:
    spans = []
    n = len(t)
    i = 0
    while i < n:
        if in_contact[i] >= 0.5:
            j = i
            while j < n and in_contact[j] >= 0.5:
                j += 1
            spans.append((t[i], t[min(j, n - 1)]))
            i = j
        else:
            i += 1
    return spans


def main(args: Args) -> None:
    if not os.path.isfile(args.csv):
        print(f"CSV not found: {args.csv}")
        sys.exit(1)

    data = load_csv(args.csv)
    required = ["t", "target", "commanded", "present_pos", "present_load", "in_contact"]
    missing = [c for c in required if c not in data]
    if missing:
        print(f"CSV missing columns: {missing}")
        sys.exit(1)

    t = data["t"]
    spans = contact_spans(t, data["in_contact"])

    fig, (ax_pos, ax_load) = plt.subplots(
        2, 1, sharex=True, figsize=(10, 6), gridspec_kw={"height_ratios": [2, 1]}
    )

    ax_pos.plot(t, data["target"], label="target", linewidth=1.2)
    ax_pos.plot(t, data["commanded"], label="commanded", linewidth=1.2, linestyle="--")
    ax_pos.plot(t, data["present_pos"], label="present_pos", linewidth=1.2, alpha=0.8)
    ax_pos.set_ylabel("ticks")
    ax_pos.legend(loc="upper right")
    ax_pos.grid(True, alpha=0.3)

    ax_load.plot(t, data["present_load"], color="tab:red", linewidth=1.0, label="present_load")
    ax_load.set_ylabel("load (0..1023)")
    ax_load.set_xlabel("time (s)")
    ax_load.grid(True, alpha=0.3)

    for ax in (ax_pos, ax_load):
        for t0, t1 in spans:
            ax.axvspan(t0, t1, color="orange", alpha=0.15)

    if spans:
        ax_load.axvspan(spans[0][0], spans[0][1], color="orange", alpha=0.15, label="in_contact")
        ax_load.legend(loc="upper right")

    fig.suptitle(args.title or os.path.basename(args.csv))
    fig.tight_layout()

    out_path = args.out or os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main(tyro.cli(Args))
