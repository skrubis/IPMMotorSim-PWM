#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

CLAMP_WARN_PCT = 0.5
OOB_WARN_PCT = 5.0


def wrap_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def phi_bin(phi_deg, bins, min_deg, max_deg):
    if bins <= 0:
        return None
    span = max_deg - min_deg
    if span <= 0:
        return None
    clamped = min(max_deg, max(min_deg, phi_deg))
    norm = (clamped - min_deg) / span
    idx = int(math.floor(norm * bins))
    if idx < 0:
        idx = 0
    if idx >= bins:
        idx = bins - 1
    return idx


def nearest_index(values, target):
    if not values:
        return None
    best_idx = 0
    best_dist = abs(values[0] - target)
    for i, v in enumerate(values):
        d = abs(v - target)
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx


def bracket_indices(values, target):
    if not values:
        return None
    if target <= values[0]:
        return 0, 0, 0.0
    if target >= values[-1]:
        return len(values) - 1, len(values) - 1, 0.0
    for i in range(len(values) - 1):
        v0 = values[i]
        v1 = values[i + 1]
        if v0 <= target <= v1:
            if v1 == v0:
                return i, i, 0.0
            t = (target - v0) / (v1 - v0)
            return i, i + 1, t
    return len(values) - 1, len(values) - 1, 0.0


def bilinear(values_x, values_y, target_x, target_y, get_value):
    ix0, ix1, tx = bracket_indices(values_x, target_x)
    iy0, iy1, ty = bracket_indices(values_y, target_y)
    if ix0 is None or iy0 is None:
        return math.nan, (ix0, ix1), (iy0, iy1)

    v00 = get_value(ix0, iy0)
    v10 = get_value(ix1, iy0)
    v01 = get_value(ix0, iy1)
    v11 = get_value(ix1, iy1)

    vals = [v00, v10, v01, v11]
    if all(not math.isfinite(v) for v in vals):
        return math.nan, (ix0, ix1), (iy0, iy1)

    def lerp(a, b, t):
        if not math.isfinite(a) and math.isfinite(b):
            return b
        if math.isfinite(a) and not math.isfinite(b):
            return a
        if not math.isfinite(a) and not math.isfinite(b):
            return math.nan
        return a + (b - a) * t

    v0 = lerp(v00, v10, tx)
    v1 = lerp(v01, v11, tx)
    v = lerp(v0, v1, ty)
    return v, (ix0, ix1), (iy0, iy1)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def read_summary(summary_path):
    rows = []
    with open(summary_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def build_loss_table(rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, warnings):
    table = {}
    best_table = {}

    def get_phi(row):
        if phi_source == "phi_pf_deg":
            val = row.get("phi_pf_deg")
            return float(val) if val not in (None, "") else math.nan
        val = row.get("phi_idiq_deg")
        return float(val) if val not in (None, "") else math.nan

    for row in rows:
        try:
            speed = float(row.get("speed_rpm", "nan"))
            iq = float(row.get("iq_A", "nan"))
            mode = row.get("mode", "")
            loss = float(row.get("avg_total_w", "nan"))
        except ValueError:
            continue

        if not mode:
            continue
        if str(mode).lower().startswith("auto"):
            continue
        invalid = row.get("invalid")
        if invalid not in (None, ""):
            try:
                if int(float(invalid)) != 0:
                    continue
            except Exception:
                pass
        if not math.isfinite(loss):
            continue

        phi_deg = get_phi(row)
        if not math.isfinite(phi_deg):
            continue
        phi_deg = wrap_deg(phi_deg)

        phi_idx = phi_bin(phi_deg, phi_bins, phi_min, phi_max)
        s_idx = nearest_index(speed_bins, speed)
        q_idx = nearest_index(iq_bins, iq)
        if phi_idx is None or s_idx is None or q_idx is None:
            continue

        ok = row.get("constraint_ok")
        if ok is not None and ok != "":
            if int(float(ok)) == 0:
                continue

        key = (phi_idx, s_idx, q_idx, mode)
        prev = table.get(key)
        if prev is None or loss < prev:
            table[key] = loss

    for (phi_idx, s_idx, q_idx, _mode), loss in table.items():
        key = (phi_idx, s_idx, q_idx)
        best = best_table.get(key)
        if best is None or loss < best:
            best_table[key] = loss

    if not table:
        warnings.append("loss table empty (no valid rows)")
    return table, best_table


def build_auto_table(rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, warnings, mode_name):
    table = {}

    def get_phi(row):
        if phi_source == "phi_pf_deg":
            val = row.get("phi_pf_deg")
            return float(val) if val not in (None, "") else math.nan
        val = row.get("phi_idiq_deg")
        return float(val) if val not in (None, "") else math.nan

    target = mode_name.lower()
    for row in rows:
        mode = str(row.get("mode", "")).lower()
        if mode != target:
            continue
        try:
            speed = float(row.get("speed_rpm", "nan"))
            iq = float(row.get("iq_A", "nan"))
            loss = float(row.get("avg_total_w", "nan"))
        except ValueError:
            continue
        invalid = row.get("invalid")
        if invalid not in (None, ""):
            try:
                if int(float(invalid)) != 0:
                    continue
            except Exception:
                pass
        if not math.isfinite(loss):
            continue

        phi_deg = get_phi(row)
        if not math.isfinite(phi_deg):
            continue
        phi_deg = wrap_deg(phi_deg)

        phi_idx = phi_bin(phi_deg, phi_bins, phi_min, phi_max)
        s_idx = nearest_index(speed_bins, speed)
        q_idx = nearest_index(iq_bins, iq)
        if phi_idx is None or s_idx is None or q_idx is None:
            continue

        ok = row.get("constraint_ok")
        if ok is not None and ok != "":
            if int(float(ok)) == 0:
                continue

        key = (phi_idx, s_idx, q_idx)
        prev = table.get(key)
        if prev is None or loss < prev:
            table[key] = loss

    if not table:
        warnings.append(f"{mode_name} table empty (no valid rows)")
    return table


def main():
    parser = argparse.ArgumentParser(description="Cycle evaluation from LUT and sweep summary")
    parser.add_argument("--cycle", required=True, help="cycle CSV with t_s,speed_rpm,iq_A[,id_A]")
    parser.add_argument("--in", dest="in_dir", required=True, help="input directory containing summary.csv and lut.json")
    parser.add_argument("--out", dest="out_dir", required=True, help="output directory")
    parser.add_argument("--interp", choices=["nearest", "bilinear"], default="nearest",
                        help="interpolation for speed/iq (phi remains binned)")
    args = parser.parse_args()

    summary_path = os.path.join(args.in_dir, "summary.csv")
    lut_path = os.path.join(args.in_dir, "lut.json")

    if not os.path.exists(summary_path):
        print(f"missing summary.csv at {summary_path}", file=sys.stderr)
        return 1
    if not os.path.exists(lut_path):
        print(f"missing lut.json at {lut_path}", file=sys.stderr)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)

    lut = load_json(lut_path)
    phi_source = lut.get("phi_source", "phi_idiq_deg")
    phi_min = float(lut.get("phi_min_deg", -180.0))
    phi_max = float(lut.get("phi_max_deg", 180.0))
    phi_bins = int(lut.get("phi_bins", len(lut.get("phi_bins_deg", [])) or 0))
    speed_bins = [float(v) for v in lut.get("speed_bins_rpm", lut.get("speed_rpm", []))]
    iq_bins = [float(v) for v in lut.get("iq_bins_a", lut.get("iq_A", []))]
    lut_mode = lut.get("lut_mode", [])
    lut_valid = lut.get("lut_valid", [])
    mode_id_map = lut.get("mode_id_map", {})
    phi_fill_ratio = float(lut.get("phi_bin_fill_ratio", float("nan")))
    avg_filled_phi = float(lut.get("avg_filled_phi_bins_per_speediq", float("nan")))

    warnings = []
    if phi_source == "phi_pf_deg":
        warnings.append("phi_source is phi_pf_deg; cycle.csv must include phi_pf_deg or LUT fallback uses 0")
    if math.isfinite(phi_fill_ratio) and phi_fill_ratio < 0.2:
        warnings.append("phi LUT fill ratio is low (<20%); results may be unreliable. Expand id_A sweep or reduce phi_bins.")
    if math.isfinite(avg_filled_phi) and avg_filled_phi <= 1.1 and phi_bins > 1:
        warnings.append("avg filled phi bins per (speed,iq) <= 1; LUT likely empty across phi.")

    summary_rows = read_summary(summary_path)
    loss_table, best_table = build_loss_table(
        summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, warnings
    )
    auto_pred_table = build_auto_table(
        summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, warnings, "AUTO_PRED"
    )

    cycle_rows = []
    with open(args.cycle, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cycle_rows.append(row)

    if not cycle_rows:
        print("cycle.csv is empty", file=sys.stderr)
        return 1

    warned_phi_missing = False

    def get_phi_for_cycle(row):
        nonlocal warned_phi_missing
        if phi_source == "phi_idiq_deg":
            iq = float(row.get("iq_A", "0"))
            id_val = float(row.get("id_A", "0")) if row.get("id_A") not in (None, "") else 0.0
            return wrap_deg(math.degrees(math.atan2(id_val, iq)))
        val = row.get("phi_pf_deg")
        if val in (None, ""):
            if not warned_phi_missing:
                warnings.append("phi_pf_deg missing in cycle.csv; using 0 deg")
                warned_phi_missing = True
            return 0.0
        return wrap_deg(float(val))

    times = []
    loss_svpwm = []
    loss_lut = []
    loss_best = []
    loss_auto_pred = []
    mode_trace = []

    missing_svpwm = 0
    missing_lut = 0
    missing_best = 0
    missing_auto_pred = 0

    speed_min = min(speed_bins) if speed_bins else math.nan
    speed_max = max(speed_bins) if speed_bins else math.nan
    iq_min = min(iq_bins) if iq_bins else math.nan
    iq_max = max(iq_bins) if iq_bins else math.nan

    cycle_speed_min = math.inf
    cycle_speed_max = -math.inf
    cycle_iq_min = math.inf
    cycle_iq_max = -math.inf
    used_speed_min = math.inf
    used_speed_max = -math.inf
    used_iq_min = math.inf
    used_iq_max = -math.inf
    cycle_phi_min = math.inf
    cycle_phi_max = -math.inf

    clamp_speed = 0
    clamp_iq = 0
    clamp_phi = 0
    oob_speed = 0
    oob_iq = 0
    oob_phi = 0
    endpoint_hits = 0
    invalid_cell_hits = 0
    clamp_endpoint_hits = 0

    for row in cycle_rows:
        t = float(row.get("t_s", "0"))
        speed = float(row.get("speed_rpm", "0"))
        iq = float(row.get("iq_A", "0"))
        phi_deg = get_phi_for_cycle(row)

        cycle_speed_min = min(cycle_speed_min, speed)
        cycle_speed_max = max(cycle_speed_max, speed)
        cycle_iq_min = min(cycle_iq_min, iq)
        cycle_iq_max = max(cycle_iq_max, iq)
        cycle_phi_min = min(cycle_phi_min, phi_deg)
        cycle_phi_max = max(cycle_phi_max, phi_deg)

        if math.isfinite(speed_min) and (speed < speed_min or speed > speed_max):
            clamp_speed += 1
            oob_speed += 1
        if math.isfinite(iq_min) and (iq < iq_min or iq > iq_max):
            clamp_iq += 1
            oob_iq += 1
        if phi_deg < phi_min or phi_deg > phi_max:
            clamp_phi += 1
            oob_phi += 1

        phi_idx = phi_bin(phi_deg, phi_bins, phi_min, phi_max)
        s_idx = nearest_index(speed_bins, speed)
        q_idx = nearest_index(iq_bins, iq)

        if s_idx in (0, len(speed_bins) - 1) or q_idx in (0, len(iq_bins) - 1) or phi_idx in (0, phi_bins - 1):
            endpoint_hits += 1

        if math.isfinite(speed):
            clamped_speed = min(speed_max, max(speed_min, speed))
            used_speed_min = min(used_speed_min, clamped_speed)
            used_speed_max = max(used_speed_max, clamped_speed)
        if math.isfinite(iq):
            clamped_iq = min(iq_max, max(iq_min, iq))
            used_iq_min = min(used_iq_min, clamped_iq)
            used_iq_max = max(used_iq_max, clamped_iq)

        times.append(t)
        if phi_idx is None or s_idx is None or q_idx is None:
            loss_svpwm.append(math.nan)
            loss_lut.append(math.nan)
            loss_best.append(math.nan)
            loss_auto_pred.append(math.nan)
            mode_trace.append(None)
            continue

        mode = None
        try:
            mode = lut_mode[phi_idx][s_idx][q_idx]
        except Exception:
            mode = None

        cell_valid = True
        if lut_valid:
            try:
                cell_valid = bool(lut_valid[phi_idx][s_idx][q_idx])
            except Exception:
                cell_valid = True
        if not cell_valid:
            invalid_cell_hits += 1
            mode = "SVPWM"

        mode_trace.append(mode)

        key_svpwm = (phi_idx, s_idx, q_idx, "SVPWM")
        key_lut = (phi_idx, s_idx, q_idx, mode) if mode else None
        key_best = (phi_idx, s_idx, q_idx)

        sv = loss_table.get(key_svpwm)
        if sv is None:
            missing_svpwm += 1
            sv = math.nan
        lu = loss_table.get(key_lut) if key_lut else None
        if lu is None:
            missing_lut += 1
            lu = sv if math.isfinite(sv) else math.nan
        be = best_table.get(key_best)
        if be is None:
            missing_best += 1
            be = math.nan

        if args.interp == "bilinear" and phi_idx is not None:
            def loss_at(si, qi):
                return loss_table.get((phi_idx, si, qi, mode), math.nan) if mode else math.nan
            lu_i, (s0, s1), (q0, q1) = bilinear(speed_bins, iq_bins, speed, iq, loss_at)
            if math.isfinite(lu_i):
                lu = lu_i
            if s0 in (0, len(speed_bins) - 1) or s1 in (0, len(speed_bins) - 1) or q0 in (0, len(iq_bins) - 1) or q1 in (0, len(iq_bins) - 1):
                clamp_endpoint_hits += 1

        loss_svpwm.append(sv)
        loss_lut.append(lu)
        loss_best.append(be)

        ap = auto_pred_table.get((phi_idx, s_idx, q_idx))
        if ap is None:
            missing_auto_pred += 1
            ap = math.nan
        loss_auto_pred.append(ap)

    if missing_svpwm:
        warnings.append(f"missing SVPWM loss for {missing_svpwm} samples")
    if missing_lut:
        warnings.append(f"missing LUT loss for {missing_lut} samples")
    if missing_best:
        warnings.append(f"missing best-per-cell loss for {missing_best} samples")
    if missing_auto_pred and auto_pred_table:
        warnings.append(f"missing AUTO_PRED loss for {missing_auto_pred} samples")

    dts = []
    for i in range(len(times)):
        if i + 1 < len(times):
            dt = times[i + 1] - times[i]
        elif len(times) > 1:
            dt = times[i] - times[i - 1]
        else:
            dt = 0.0
        if dt < 0:
            warnings.append("non-monotonic time values detected")
            dt = 0.0
        dts.append(dt)

    def integrate_energy(losses):
        total_wh = 0.0
        for loss, dt in zip(losses, dts):
            if not math.isfinite(loss):
                continue
            total_wh += loss * dt / 3600.0
        return total_wh

    e_svpwm = integrate_energy(loss_svpwm)
    e_lut = integrate_energy(loss_lut)
    e_best = integrate_energy(loss_best)
    auto_pred_samples = sum(1 for v in loss_auto_pred if math.isfinite(v))
    e_auto_pred = integrate_energy(loss_auto_pred) if auto_pred_samples > 0 else math.nan

    improvement_pct = 0.0
    if e_svpwm > 0:
        improvement_pct = 100.0 * (e_svpwm - e_lut) / e_svpwm
    auto_pred_improvement_pct = 0.0
    if e_svpwm > 0 and math.isfinite(e_auto_pred):
        auto_pred_improvement_pct = 100.0 * (e_svpwm - e_auto_pred) / e_svpwm

    total_samples = len(cycle_rows)
    def clamp_pct(count):
        return 100.0 * count / total_samples if total_samples else 0.0

    oob_stats = {
        "speed_rpm": {
            "min": cycle_speed_min if math.isfinite(cycle_speed_min) else math.nan,
            "max": cycle_speed_max if math.isfinite(cycle_speed_max) else math.nan,
            "map_min": speed_min,
            "map_max": speed_max,
            "clamp_pct": clamp_pct(clamp_speed),
            "oob_pct": clamp_pct(oob_speed),
        },
        "iq_A": {
            "min": cycle_iq_min if math.isfinite(cycle_iq_min) else math.nan,
            "max": cycle_iq_max if math.isfinite(cycle_iq_max) else math.nan,
            "map_min": iq_min,
            "map_max": iq_max,
            "clamp_pct": clamp_pct(clamp_iq),
            "oob_pct": clamp_pct(oob_iq),
        },
        "phi_deg": {
            "min": cycle_phi_min if math.isfinite(cycle_phi_min) else math.nan,
            "max": cycle_phi_max if math.isfinite(cycle_phi_max) else math.nan,
            "map_min": phi_min,
            "map_max": phi_max,
            "clamp_pct": clamp_pct(clamp_phi),
            "oob_pct": clamp_pct(oob_phi),
        },
    }

    clamp_warnings = []
    if oob_stats["speed_rpm"]["clamp_pct"] > CLAMP_WARN_PCT:
        clamp_warnings.append(f"speed_rpm clamped for {oob_stats['speed_rpm']['clamp_pct']:.2f}% of samples")
    if oob_stats["iq_A"]["clamp_pct"] > CLAMP_WARN_PCT:
        clamp_warnings.append(f"iq_A clamped for {oob_stats['iq_A']['clamp_pct']:.2f}% of samples")
    if oob_stats["phi_deg"]["clamp_pct"] > CLAMP_WARN_PCT:
        clamp_warnings.append(f"phi_deg clamped for {oob_stats['phi_deg']['clamp_pct']:.2f}% of samples")
    if oob_stats["speed_rpm"]["oob_pct"] > OOB_WARN_PCT:
        clamp_warnings.append(f"speed_rpm out-of-bounds for {oob_stats['speed_rpm']['oob_pct']:.2f}% of samples")
    if oob_stats["iq_A"]["oob_pct"] > OOB_WARN_PCT:
        clamp_warnings.append(f"iq_A out-of-bounds for {oob_stats['iq_A']['oob_pct']:.2f}% of samples")
    if oob_stats["phi_deg"]["oob_pct"] > OOB_WARN_PCT:
        clamp_warnings.append(f"phi_deg out-of-bounds for {oob_stats['phi_deg']['oob_pct']:.2f}% of samples")
    for w in clamp_warnings:
        warnings.append(w)
        print(f"WARNING: {w}", file=sys.stderr)

    dt_min = min(dts) if dts else math.nan
    dt_max = max(dts) if dts else math.nan
    dt_mean = sum(dts) / len(dts) if dts else math.nan

    def finite_or_nan(val):
        return val if math.isfinite(val) else math.nan

    result = {
        "totals": {
            "E_svpwm_Wh": e_svpwm,
            "E_lut_Wh": e_lut,
            "E_best_Wh": e_best,
            "E_auto_pred_Wh": e_auto_pred if math.isfinite(e_auto_pred) else math.nan,
        },
        "improvement_pct": improvement_pct,
        "auto_pred_improvement_pct": auto_pred_improvement_pct,
        "phi_source": phi_source,
        "warnings": warnings,
        "oob_stats": oob_stats,
        "interp": args.interp,
        "invalid_cell_hits": invalid_cell_hits,
        "endpoint_hit_pct": clamp_pct(endpoint_hits),
        "endpoint_hit_pct_interp": clamp_pct(clamp_endpoint_hits),
        "samples": {
            "count": total_samples,
            "time_s": sum(dts),
            "dt_min_s": dt_min,
            "dt_mean_s": dt_mean,
            "dt_max_s": dt_max,
        },
        "used_ranges": {
            "speed_rpm": {"min": finite_or_nan(used_speed_min), "max": finite_or_nan(used_speed_max)},
            "iq_A": {"min": finite_or_nan(used_iq_min), "max": finite_or_nan(used_iq_max)},
        },
    }

    out_json = os.path.join(args.out_dir, "cycle_eval.json")

    # plots
    try:
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(10, 5))
        plt.plot(times, loss_svpwm, label="SVPWM")
        plt.plot(times, loss_lut, label="LUT")
        if any(math.isfinite(x) for x in loss_auto_pred):
            plt.plot(times, loss_auto_pred, label="AUTO_PRED", alpha=0.6)
        if any(math.isfinite(x) for x in loss_best):
            plt.plot(times, loss_best, label="Best", alpha=0.6)
        plt.xlabel("Time (s)")
        plt.ylabel("Loss (W)")
        plt.title("Cycle Loss Trace")
        plt.legend()
        plt.tight_layout()
        fig.savefig(os.path.join(args.out_dir, "cycle_loss_trace.png"), dpi=150)
        plt.close(fig)

        if mode_trace and mode_id_map:
            mode_ids = []
            for m in mode_trace:
                mode_ids.append(mode_id_map.get(m, math.nan))
            fig = plt.figure(figsize=(10, 3))
            plt.plot(times, mode_ids, drawstyle="steps-post")
            plt.xlabel("Time (s)")
            plt.ylabel("Mode ID")
            plt.title("Cycle Mode Trace")
            plt.tight_layout()
            fig.savefig(os.path.join(args.out_dir, "cycle_mode_trace.png"), dpi=150)
            plt.close(fig)
    except Exception as ex:
        warnings.append(f"plotting failed: {ex}")

    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    return 0


if __name__ == "__main__":
    sys.exit(main())
