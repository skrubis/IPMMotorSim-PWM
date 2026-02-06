#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict


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


def read_summary(summary_path):
    rows = []
    with open(summary_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def safe_float(val, default=math.nan):
    try:
        if val in (None, ""):
            return default
        return float(val)
    except Exception:
        return default


def fmt_energy(wh):
    if not math.isfinite(wh):
        return "nan"
    if abs(wh) >= 1000.0:
        return f"{wh/1000.0:.3f} kWh"
    if abs(wh) < 0.01:
        joules = wh * 3600.0
        return f"{wh:.6f} Wh ({joules:.2f} J)"
    return f"{wh:.3f} Wh"


def build_loss_tables(rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins):
    table = {}
    thd_table = {}

    def get_phi(row):
        if phi_source == "phi_pf_deg":
            val = row.get("phi_pf_deg")
            return float(val) if val not in (None, "") else math.nan
        val = row.get("phi_idiq_deg")
        return float(val) if val not in (None, "") else math.nan

    for row in rows:
        mode = row.get("mode", "")
        if not mode or str(mode).lower().startswith("auto"):
            continue
        try:
            speed = float(row.get("speed_rpm", "nan"))
            iq = float(row.get("iq_A", "nan"))
            loss = float(row.get("avg_total_w", "nan"))
            thd = float(row.get("thd_a_pct", "nan")) if row.get("thd_a_pct") not in (None, "") else math.nan
        except ValueError:
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
        thd_table[(phi_idx, s_idx, q_idx, mode)] = thd

    best_table = {}
    for (phi_idx, s_idx, q_idx, _mode), loss in table.items():
        key = (phi_idx, s_idx, q_idx)
        best = best_table.get(key)
        if best is None or loss < best:
            best_table[key] = loss
    return table, best_table, thd_table


def build_value_table(rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, mode_name, key):
    table = {}

    def get_phi(row):
        if phi_source == "phi_pf_deg":
            val = row.get("phi_pf_deg")
            return float(val) if val not in (None, "") else math.nan
        val = row.get("phi_idiq_deg")
        return float(val) if val not in (None, "") else math.nan

    for row in rows:
        mode = row.get("mode", "")
        if mode != mode_name:
            continue
        try:
            speed = float(row.get("speed_rpm", "nan"))
            iq = float(row.get("iq_A", "nan"))
            value = float(row.get(key, "nan"))
        except ValueError:
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

        key_idx = (phi_idx, s_idx, q_idx)
        prev = table.get(key_idx)
        if prev is None or value < prev:
            table[key_idx] = value

    return table


def build_metric_table(rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, key):
    table = {}

    def get_phi(row):
        if phi_source == "phi_pf_deg":
            val = row.get("phi_pf_deg")
            return float(val) if val not in (None, "") else math.nan
        val = row.get("phi_idiq_deg")
        return float(val) if val not in (None, "") else math.nan

    for row in rows:
        mode = row.get("mode", "")
        if not mode or str(mode).lower().startswith("auto"):
            continue
        try:
            speed = float(row.get("speed_rpm", "nan"))
            iq = float(row.get("iq_A", "nan"))
            value = float(row.get(key, "nan"))
        except ValueError:
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

        key_idx = (phi_idx, s_idx, q_idx, mode)
        prev = table.get(key_idx)
        if prev is None or value < prev:
            table[key_idx] = value

    return table


def build_map(speed_bins, iq_bins, phi_idx, lut_mode, loss_table, mode_name):
    speed_count = len(speed_bins)
    iq_count = len(iq_bins)
    grid = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]
    for s in range(speed_count):
        for q in range(iq_count):
            key = (phi_idx, s, q, mode_name)
            val = loss_table.get(key)
            if val is not None:
                grid[q][s] = val
    return grid


def main():
    parser = argparse.ArgumentParser(description="Generate PDF report from sweep outputs")
    parser.add_argument("--in", dest="in_dir", required=True, help="input directory containing summary.csv (and optionally lut.json / cycle_eval.json)")
    parser.add_argument("--out", dest="out_pdf", required=True, help="output PDF path")
    args = parser.parse_args()

    summary_path = os.path.join(args.in_dir, "summary.csv")
    lut_path = os.path.join(args.in_dir, "lut.json")
    cycle_eval_path = os.path.join(args.in_dir, "cycle_eval.json")

    if not os.path.exists(summary_path):
        print("summary.csv missing", file=sys.stderr)
        return 1

    summary_rows = read_summary(summary_path)
    have_lut = os.path.exists(lut_path)

    if have_lut:
        lut = load_json(lut_path)

        phi_source = lut.get("phi_source", "phi_idiq_deg")
        phi_min = float(lut.get("phi_min_deg", -180.0))
        phi_max = float(lut.get("phi_max_deg", 180.0))
        phi_bins = int(lut.get("phi_bins", len(lut.get("phi_bins_deg", [])) or 0))
        speed_bins = [float(v) for v in lut.get("speed_bins_rpm", lut.get("speed_rpm", []))]
        iq_bins = [float(v) for v in lut.get("iq_bins_a", lut.get("iq_A", []))]
        phi_bins_deg = lut.get("phi_bins_deg", [])

        loss_table, best_table, thd_table = build_loss_tables(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins
        )
        sw_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "avg_igbt_sw_w"
        )
        total_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "avg_total_w"
        )
        min_pulse_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "min_pulse_margin_s"
        )
        clamp_a_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "clamp_frac_a"
        )
        clamp_b_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "clamp_frac_b"
        )
        clamp_c_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "clamp_frac_c"
        )
        inv_eff_table = build_metric_table(
            summary_rows, phi_source, phi_min, phi_max, phi_bins, speed_bins, iq_bins, "avg_inv_eff_pct"
        )

        # Determine phi bin closest to 0 deg
        phi_idx = 0
        if phi_bins_deg:
            phi_idx = min(range(len(phi_bins_deg)), key=lambda i: abs(phi_bins_deg[i]))
        elif phi_bins > 0:
            phi_idx = int(phi_bins / 2)

        # Compute LUT vs SVPWM delta map
        lut_mode = lut.get("lut_mode", [])
        speed_count = len(speed_bins)
        iq_count = len(iq_bins)

        delta_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]
        mode_map = [[None for _ in range(speed_count)] for _ in range(iq_count)]
        thd_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]

        for s in range(speed_count):
            for q in range(iq_count):
                try:
                    mode = lut_mode[phi_idx][s][q]
                except Exception:
                    mode = None
                mode_map[q][s] = mode
                sv = loss_table.get((phi_idx, s, q, "SVPWM"))
                lu = loss_table.get((phi_idx, s, q, mode)) if mode else None
                if sv is not None and lu is not None:
                    delta_map[q][s] = lu - sv
                thd_val = thd_table.get((phi_idx, s, q, "SVPWM"))
                if thd_val is not None:
                    thd_map[q][s] = thd_val

        delta_sw_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]
        total_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]
        min_pulse_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]
        clamp_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]
        inv_eff_map = [[math.nan for _ in range(speed_count)] for _ in range(iq_count)]

        for s in range(speed_count):
            for q in range(iq_count):
                mode = mode_map[q][s]
                sw_sv = sw_table.get((phi_idx, s, q, "SVPWM"))
                sw_lu = sw_table.get((phi_idx, s, q, mode)) if mode else None
                if sw_sv is not None and sw_lu is not None:
                    delta_sw_map[q][s] = sw_lu - sw_sv

                total_sv = total_table.get((phi_idx, s, q, "SVPWM"))
                if total_sv is not None:
                    total_map[q][s] = total_sv

                min_pulse = min_pulse_table.get((phi_idx, s, q, "SVPWM"))
                if min_pulse is not None:
                    min_pulse_map[q][s] = min_pulse

                ca = clamp_a_table.get((phi_idx, s, q, "SVPWM"))
                cb = clamp_b_table.get((phi_idx, s, q, "SVPWM"))
                cc = clamp_c_table.get((phi_idx, s, q, "SVPWM"))
                clamp_vals = [v for v in (ca, cb, cc) if v is not None]
                if clamp_vals:
                    clamp_map[q][s] = max(clamp_vals)

                inv_eff = inv_eff_table.get((phi_idx, s, q, "SVPWM"))
                if inv_eff is not None:
                    inv_eff_map[q][s] = inv_eff

        # Best / worst points
        best_saving = None
        worst_penalty = None
        for s in range(speed_count):
            for q in range(iq_count):
                try:
                    mode = lut_mode[phi_idx][s][q]
                except Exception:
                    mode = None
                sv = loss_table.get((phi_idx, s, q, "SVPWM"))
                lu = loss_table.get((phi_idx, s, q, mode)) if mode else None
                if sv is None or lu is None:
                    continue
                delta = lu - sv
                if best_saving is None or delta < best_saving[0]:
                    best_saving = (delta, s, q)
                if worst_penalty is None or delta > worst_penalty[0]:
                    worst_penalty = (delta, s, q)

    cycle_eval = None
    if os.path.exists(cycle_eval_path):
        cycle_eval = load_json(cycle_eval_path)

    # Build report PDF
    try:
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages
    except Exception as ex:
        print(f"matplotlib missing: {ex}", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(args.out_pdf) or ".", exist_ok=True)

    with PdfPages(args.out_pdf) as pdf:
        if not have_lut:
            # Fixed-point frequency sweep report (summary.csv only).
            modes = sorted({r.get("mode", "") for r in summary_rows if r.get("mode", "")})
            f_sw_values = sorted({float(r.get("f_sw_Hz", "nan")) for r in summary_rows if r.get("f_sw_Hz", "")})

            fig = plt.figure(figsize=(8.5, 11))
            fig.clf()
            text = []
            text.append("IPM Motor Sim Report")
            text.append("")
            text.append("Report type: Fixed-point frequency sweep (no LUT)")
            text.append("")
            if summary_rows:
                r0 = summary_rows[0]
                try:
                    text.append(
                        f"Operating point: speed={float(r0.get('speed_rpm','nan')):.0f} rpm, "
                        f"iq={float(r0.get('iq_A','nan')):.1f} A, id={float(r0.get('id_A','nan')):.1f} A, "
                        f"Vdc={float(r0.get('vdc_V','nan')):.0f} V, temp={float(r0.get('temp_C','nan')):.0f} C"
                    )
                except Exception:
                    pass
            text.append("")
            text.append(f"f_sw points: {', '.join(str(int(x)) for x in f_sw_values if math.isfinite(x))}")
            text.append(f"Strategies: {', '.join(modes)}")
            text.append("")
            text.append("Notes:")
            text.append("- LUT maps require OperatingMapSweep + LUT build (lut.json).")
            text.append("- THD is control-step proxy; not physical per-PWM current THD.")
            fig.text(0.05, 0.95, "\n".join(text), ha="left", va="top", fontsize=10)
            pdf.savefig(fig)
            plt.close(fig)

            def series_for(mode_name, key):
                vals = []
                for f in f_sw_values:
                    v = math.nan
                    for r in summary_rows:
                        if r.get("mode") != mode_name:
                            continue
                        try:
                            if float(r.get("f_sw_Hz", "nan")) != f:
                                continue
                        except Exception:
                            continue
                        try:
                            v = float(r.get(key, "nan"))
                        except Exception:
                            v = math.nan
                        break
                    vals.append(v)
                return vals

            fig, ax = plt.subplots(figsize=(8, 5))
            for mode in modes:
                ax.plot(f_sw_values, series_for(mode, "avg_total_w"), label=mode)
            ax.set_xlabel("f_sw (Hz) - firmware-supported: 4400/8800/17600")
            ax.set_ylabel("W")
            ax.set_title("Total Inverter Loss vs f_sw")
            ax.set_xticks(sorted(set(f_sw_values)))
            ax.legend(fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)

            fig, ax = plt.subplots(figsize=(8, 5))
            for mode in modes:
                igbt_sw = series_for(mode, "avg_igbt_sw_w")
                diode_rr = series_for(mode, "avg_diode_rr_w")
                sw = [(a + b) if (math.isfinite(a) and math.isfinite(b)) else math.nan for a, b in zip(igbt_sw, diode_rr)]
                ax.plot(f_sw_values, sw, label=mode)
            ax.set_xlabel("f_sw (Hz) - firmware-supported: 4400/8800/17600")
            ax.set_ylabel("W")
            ax.set_title("Switching+RR Loss vs f_sw")
            ax.set_xticks(sorted(set(f_sw_values)))
            ax.legend(fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)

            fig, ax = plt.subplots(figsize=(8, 5))
            any_thd = False
            for mode in modes:
                thd = series_for(mode, "thd_a_pct")
                if any(math.isfinite(x) for x in thd):
                    any_thd = True
                ax.plot(f_sw_values, thd, label=mode)
            ax.set_xlabel("f_sw (Hz) - firmware-supported: 4400/8800/17600")
            ax.set_ylabel("%")
            ax.set_title("THD Proxy vs f_sw")
            ax.set_xticks(sorted(set(f_sw_values)))
            if any_thd:
                ax.legend(fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)

            return 0

        # Executive summary page
        fig = plt.figure(figsize=(8.5, 11))
        fig.clf()
        text = []
        text.append("IPM Motor Sim Report")
        text.append("")
        text.append("DPWM ROI Summary:")
        if cycle_eval:
            totals = cycle_eval.get("totals", {})
            e_svpwm = float(totals.get("E_svpwm_Wh", float("nan")))
            e_lut = float(totals.get("E_lut_Wh", float("nan")))
            e_auto = float(totals.get("E_auto_pred_Wh", float("nan")))
            text.append(f"- SVPWM cycle loss: {fmt_energy(e_svpwm)}")
            imp_pct = float(cycle_eval.get("improvement_pct", float("nan")))
            text.append(f"- LUT cycle loss: {fmt_energy(e_lut)} (improvement {imp_pct:.2f}%)")
            if math.isfinite(e_auto):
                auto_imp_pct = float(cycle_eval.get("auto_pred_improvement_pct", float("nan")))
                text.append(f"- AUTO_PRED cycle loss: {fmt_energy(e_auto)} (improvement {auto_imp_pct:.2f}%)")
            samples = cycle_eval.get("samples", {})
            text.append(f"- Cycle duration: {safe_float(samples.get('time_s')):.2f} s")
        else:
            text.append("- Cycle eval not available")
        text.append(f"- LUT regret mean: {lut.get('regret_mean_w', float('nan')):.3f} W")
        text.append(f"- LUT regret max: {lut.get('regret_max_w', float('nan')):.3f} W")
        text.append(f"- Invalid cells: {lut.get('invalid_cells', 0)}")
        text.append(f"- Constraint violations: {lut.get('constraint_violations', 0)}")
        text.append("- THD is control-step proxy; not physical per-PWM current THD.")
        text.append("")
        if best_saving:
            delta, s, q = best_saving
            text.append(f"Best savings point: {delta:.2f} W at speed={speed_bins[s]}, iq={iq_bins[q]}")
        if worst_penalty:
            delta, s, q = worst_penalty
            text.append(f"Worst penalty point: {delta:.2f} W at speed={speed_bins[s]}, iq={iq_bins[q]}")
        text.append("")
        text.append("Truth labels:")
        text.append("- THD: control-step proxy")
        text.append(f"- phi source: {phi_source}")
        text.append("- DC-link model: not implemented")

        fig.text(0.05, 0.95, "\n".join(text), ha="left", va="top", fontsize=10)
        pdf.savefig(fig)
        plt.close(fig)

        # What was simulated
        fig = plt.figure(figsize=(8.5, 11))
        fig.clf()
        text = []
        text.append("What was simulated")
        text.append("")
        if summary_rows:
            r0 = summary_rows[0]
            text.append("Motor / operating point context (from first sweep row):")
            text.append(
                f"- speed_rpm={safe_float(r0.get('speed_rpm')):.0f}, iq_A={safe_float(r0.get('iq_A')):.1f}, id_A={safe_float(r0.get('id_A')):.1f}"
            )
            text.append(
                f"- Vdc_V={safe_float(r0.get('vdc_V')):.0f}, temp_C={safe_float(r0.get('temp_C')):.0f}, f_sw_Hz={safe_float(r0.get('f_sw_Hz')):.0f}"
            )
        if have_lut:
            constraints = lut.get("constraints", {})
            text.append("")
            text.append("Constraints:")
            text.append(f"- thd_max_pct={safe_float(constraints.get('thd_max_pct'))}")
            text.append(f"- i_ripple_rms_max_a={safe_float(constraints.get('i_ripple_rms_max_a'))}")
            text.append(f"- min_pulse_margin_min_s={safe_float(constraints.get('min_pulse_margin_min_s'))}")
        text.append("")
        text.append("Truth labels:")
        text.append("- THD: control-step proxy")
        text.append(f"- phi source: {phi_source}")
        text.append("- DC-link model: not implemented")
        fig.text(0.05, 0.95, "\n".join(text), ha="left", va="top", fontsize=10)
        pdf.savefig(fig)
        plt.close(fig)

        # Coverage and clamping
        fig = plt.figure(figsize=(8.5, 11))
        fig.clf()
        text = []
        text.append("Coverage & clamping")
        text.append("")
        text.append(f"phi_bin_fill_ratio: {lut.get('phi_bin_fill_ratio', float('nan')):.3f}")
        text.append(f"avg_filled_phi_bins_per_speediq: {lut.get('avg_filled_phi_bins_per_speediq', float('nan')):.3f}")
        text.append(f"invalid_cells: {lut.get('invalid_cells', 0)}")
        text.append(f"constraint_violations: {lut.get('constraint_violations', 0)}")
        if cycle_eval:
            oob = cycle_eval.get("oob_stats", {})
            def fmt_axis(axis):
                if not axis:
                    return "n/a"
                return (f"min={safe_float(axis.get('min'))}, max={safe_float(axis.get('max'))}, "
                        f"map=[{safe_float(axis.get('map_min'))},{safe_float(axis.get('map_max'))}], "
                        f"clamp={safe_float(axis.get('clamp_pct')):.2f}%, oob={safe_float(axis.get('oob_pct')):.2f}%")
            text.append("")
            text.append("Cycle coverage:")
            text.append(f"- speed_rpm: {fmt_axis(oob.get('speed_rpm'))}")
            text.append(f"- iq_A: {fmt_axis(oob.get('iq_A'))}")
            text.append(f"- phi_deg: {fmt_axis(oob.get('phi_deg'))}")
            text.append(f"- invalid LUT cell hits: {cycle_eval.get('invalid_cell_hits', 0)}")
            text.append(f"- endpoint hit pct: {cycle_eval.get('endpoint_hit_pct', 0):.2f}%")
            text.append(f"- interp endpoint hit pct: {cycle_eval.get('endpoint_hit_pct_interp', 0):.2f}%")
            samples = cycle_eval.get("samples", {})
            text.append(f"- cycle duration: {safe_float(samples.get('time_s')):.2f} s")
            text.append(f"- dt mean/min/max: {safe_float(samples.get('dt_mean_s')):.6f} / "
                        f"{safe_float(samples.get('dt_min_s')):.6f} / {safe_float(samples.get('dt_max_s')):.6f} s")
        fig.text(0.05, 0.95, "\n".join(text), ha="left", va="top", fontsize=10)
        pdf.savefig(fig)
        plt.close(fig)

        # Delta loss map
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(delta_map, origin="lower", aspect="auto")
        ax.set_title("Delta Total Loss vs SVPWM (LUT - SVPWM)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="W")
        pdf.savefig(fig)
        plt.close(fig)

        # Delta switching loss map
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(delta_sw_map, origin="lower", aspect="auto")
        ax.set_title("Delta IGBT Switching Loss vs SVPWM (LUT - SVPWM)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="W")
        pdf.savefig(fig)
        plt.close(fig)

        # Total loss map (SVPWM)
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(total_map, origin="lower", aspect="auto")
        ax.set_title("Total Loss Map (SVPWM)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="W")
        pdf.savefig(fig)
        plt.close(fig)

        # Min pulse margin map (SVPWM)
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(min_pulse_map, origin="lower", aspect="auto")
        ax.set_title("Min Pulse Margin Map (SVPWM)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="s")
        pdf.savefig(fig)
        plt.close(fig)

        # Clamp fraction map (SVPWM)
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(clamp_map, origin="lower", aspect="auto")
        ax.set_title("Clamp Fraction Map (SVPWM, max phase)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="fraction")
        pdf.savefig(fig)
        plt.close(fig)

        # Inverter efficiency map (SVPWM)
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(inv_eff_map, origin="lower", aspect="auto")
        ax.set_title("Inverter Efficiency Map (SVPWM)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="%")
        pdf.savefig(fig)
        plt.close(fig)

        # Strategy map
        fig, ax = plt.subplots(figsize=(8, 6))
        mode_id_map = lut.get("mode_id_map", {})
        mode_grid = [[mode_id_map.get(mode, -1) for mode in row] for row in mode_map]
        im = ax.imshow(mode_grid, origin="lower", aspect="auto")
        ax.set_title("LUT Strategy Map")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="Mode ID")
        pdf.savefig(fig)
        plt.close(fig)

        # Switching activity verification (loss proxy)
        def pick_reference_key(rows):
            by_key = {}
            for r in rows:
                key = (
                    safe_float(r.get("speed_rpm")),
                    safe_float(r.get("iq_A")),
                    safe_float(r.get("id_A")),
                    safe_float(r.get("vdc_V")),
                    safe_float(r.get("temp_C")),
                    safe_float(r.get("f_sw_Hz")),
                )
                mode = r.get("mode", "")
                by_key.setdefault(key, set()).add(mode)
            for k, modes in by_key.items():
                if "SVPWM" in modes and "DPWM1" in modes:
                    return k
            return None

        ref_key = pick_reference_key(summary_rows)
        if ref_key:
            ref_modes = ["SVPWM", "DPWM1"]
            sw_vals = []
            total_vals = []
            for m in ref_modes:
                row_match = None
                for r in summary_rows:
                    key = (
                        safe_float(r.get("speed_rpm")),
                        safe_float(r.get("iq_A")),
                        safe_float(r.get("id_A")),
                        safe_float(r.get("vdc_V")),
                        safe_float(r.get("temp_C")),
                        safe_float(r.get("f_sw_Hz")),
                    )
                    if key == ref_key and r.get("mode") == m:
                        row_match = r
                        break
                if row_match:
                    sw_vals.append(safe_float(row_match.get("avg_igbt_sw_w")) + safe_float(row_match.get("avg_diode_rr_w")))
                    total_vals.append(safe_float(row_match.get("avg_total_w")))
                else:
                    sw_vals.append(math.nan)
                    total_vals.append(math.nan)

            fig, ax = plt.subplots(figsize=(7, 4))
            ax.bar(ref_modes, sw_vals, color=["#4c72b0", "#55a868"])
            ax.set_ylabel("W")
            ax.set_title("Switching+RR Loss at Reference Point")
            pdf.savefig(fig)
            plt.close(fig)

            fig = plt.figure(figsize=(8.5, 11))
            fig.clf()
            text = []
            text.append("Switching activity verification (loss proxy)")
            text.append(f"Reference point: speed={ref_key[0]}, iq={ref_key[1]}, id={ref_key[2]}, f_sw={ref_key[5]}")
            for m, sw, tot in zip(ref_modes, sw_vals, total_vals):
                text.append(f"- {m}: switching+RR={sw:.3f} W, total={tot:.3f} W")
            fig.text(0.05, 0.95, "\n".join(text), ha="left", va="top", fontsize=10)
            pdf.savefig(fig)
            plt.close(fig)

        # THD map
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(thd_map, origin="lower", aspect="auto")
        ax.set_title("THD Proxy Map (SVPWM)")
        ax.set_xlabel("Speed bin")
        ax.set_ylabel("Iq bin")
        fig.colorbar(im, ax=ax, label="%")
        pdf.savefig(fig)
        plt.close(fig)

        # Top 10 points table
        deltas = []
        for s in range(speed_count):
            for q in range(iq_count):
                d = delta_map[q][s]
                if math.isfinite(d):
                    deltas.append((d, s, q))
        deltas.sort(key=lambda x: x[0])
        top_improve = deltas[:10]
        top_regress = list(reversed(deltas[-10:])) if deltas else []

        fig = plt.figure(figsize=(8.5, 11))
        fig.clf()
        text = []
        text.append("Top 10 DPWM improvement points (delta total W)")
        for d, s, q in top_improve:
            text.append(f"- speed={speed_bins[s]}, iq={iq_bins[q]}: {d:.3f} W")
        text.append("")
        text.append("Top 10 DPWM regression points (delta total W)")
        for d, s, q in top_regress:
            text.append(f"- speed={speed_bins[s]}, iq={iq_bins[q]}: {d:.3f} W")
        fig.text(0.05, 0.95, "\n".join(text), ha="left", va="top", fontsize=10)
        pdf.savefig(fig)
        plt.close(fig)

        # Cycle traces
        if cycle_eval:
            loss_png = os.path.join(args.in_dir, "cycle_loss_trace.png")
            mode_png = os.path.join(args.in_dir, "cycle_mode_trace.png")
            if os.path.exists(loss_png) or os.path.exists(mode_png):
                fig = plt.figure(figsize=(8.5, 11))
                fig.clf()
                if os.path.exists(loss_png):
                    img = plt.imread(loss_png)
                    ax = fig.add_subplot(2, 1, 1)
                    ax.imshow(img)
                    ax.axis("off")
                    ax.set_title("Cycle Loss Trace")
                if os.path.exists(mode_png):
                    img = plt.imread(mode_png)
                    ax = fig.add_subplot(2, 1, 2)
                    ax.imshow(img)
                    ax.axis("off")
                    ax.set_title("Cycle Mode Trace")
                pdf.savefig(fig)
                plt.close(fig)

        # Frequency sweep curves (SVPWM)
        f_sw_values = sorted({float(r.get("f_sw_Hz", "nan")) for r in summary_rows if r.get("f_sw_Hz", "")})
        if len(f_sw_values) > 1:
            f_sw = []
            total_loss = []
            sw_loss = []
            thd = []
            for r in summary_rows:
                if r.get("mode") != "SVPWM":
                    continue
                f = float(r.get("f_sw_Hz", "nan"))
                if not math.isfinite(f):
                    continue
                f_sw.append(f)
                total_loss.append(float(r.get("avg_total_w", "nan")))
                igbt_sw = float(r.get("avg_igbt_sw_w", "nan"))
                diode_rr = float(r.get("avg_diode_rr_w", "nan"))
                sw_loss.append(igbt_sw + diode_rr)
                thd.append(float(r.get("thd_a_pct", "nan")))

            if f_sw:
                fig, ax = plt.subplots(figsize=(8, 5))
                ax.plot(f_sw, total_loss, label="Total Loss")
                ax.plot(f_sw, sw_loss, label="Switching Loss")
                ax.set_xlabel("f_sw (Hz) - firmware-supported: 4400/8800/17600")
                ax.set_ylabel("W")
                ax.set_title("FixedPointFreqSweep (SVPWM)")
                ax.legend()
                ax.set_xticks(sorted(set(f_sw)))
                pdf.savefig(fig)
                plt.close(fig)

                fig, ax = plt.subplots(figsize=(8, 5))
                ax.plot(f_sw, thd, label="THD proxy")
                ax.set_xlabel("f_sw (Hz) - firmware-supported: 4400/8800/17600")
                ax.set_ylabel("%")
                ax.set_title("THD Proxy vs f_sw (SVPWM)")
                ax.set_xticks(sorted(set(f_sw)))
                pdf.savefig(fig)
                plt.close(fig)

    return 0


if __name__ == "__main__":
    sys.exit(main())
