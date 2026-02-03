#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path):
    with path.open("r", newline="") as handle:
        rows = []
        header = None
        for line in handle:
            if line.startswith("#"):
                continue
            if header is None:
                header = [h.strip() for h in line.strip().split(",")]
                continue
            row = line.strip().split(",")
            if not row or len(row) != len(header):
                continue
            rows.append(dict(zip(header, row)))
    return rows


def col(rows, name):
    if not rows or name not in rows[0]:
        return None
    return [float(r[name]) for r in rows]


def main():
    parser = argparse.ArgumentParser(description="Quick plots for IPMMotorSim CSV logs")
    parser.add_argument("csv", type=Path, help="Path to run_*.csv log")
    parser.add_argument("--out", type=Path, default=None, help="Save figure to file instead of showing")
    args = parser.parse_args()

    rows = load_csv(args.csv)
    if not rows:
        raise SystemExit("No data rows found in CSV.")

    t = col(rows, "time_s")
    duty_a = col(rows, "duty_a")
    duty_b = col(rows, "duty_b")
    duty_c = col(rows, "duty_c")
    ia = col(rows, "ia")
    ib = col(rows, "ib")
    ic = col(rows, "ic")
    id_m = col(rows, "id")
    iq_m = col(rows, "iq")
    id_c = col(rows, "id_ctrl")
    iq_c = col(rows, "iq_ctrl")

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    if duty_a and duty_b and duty_c:
        axes[0].plot(t, duty_a, label="duty_a", linewidth=0.8)
        axes[0].plot(t, duty_b, label="duty_b", linewidth=0.8)
        axes[0].plot(t, duty_c, label="duty_c", linewidth=0.8)
        axes[0].set_ylabel("Duty")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)

    if ia and ib and ic:
        axes[1].plot(t, ia, label="ia", linewidth=0.8)
        axes[1].plot(t, ib, label="ib", linewidth=0.8)
        axes[1].plot(t, ic, label="ic", linewidth=0.8)
        axes[1].set_ylabel("Phase Currents (A)")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.3)

    if id_m and iq_m:
        axes[2].plot(t, id_m, label="id (model)", linewidth=0.9)
        axes[2].plot(t, iq_m, label="iq (model)", linewidth=0.9)
        if id_c and iq_c:
            axes[2].plot(t, id_c, "--", label="id (ctrl)", linewidth=0.9)
            axes[2].plot(t, iq_c, "--", label="iq (ctrl)", linewidth=0.9)
        axes[2].set_ylabel("DQ Currents (A)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)

    fig.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
