#!/usr/bin/env python3
"""
plot_comparison_v5.py

Additions vs v4:
  - When parsing TXT, also produces a *long* CSV with per-(object, n_safety, push#, phase)
    estimates + errors. This yields 4 objects * 3 nsafety * 2 pushes * 2 phases = 48 rows
    (assuming your data contains both phases for both pushes).
  - Main figure: titles are drawn *inside* each subplot (instead of above).
    (Default placement is top-left; top-left panel uses top-right to avoid the legend.)

Assumptions about TXT format:
  - Each "Batch Fit Results:" block corresponds to one push (in file order).
  - Inside each block you may have one or more of:
        PUSH:     m=... kg, zc=... m, th*=... deg
        RETRACT:  m=... kg, zc=... m, th*=... deg
        COMBINED: m=... kg, zc=... m, th*=... deg
    (case-insensitive, extra whitespace tolerated)
  - If PUSH/RETRACT lines are absent, phase rows will be NaN and will not be emitted.

Usage:
  Parse TXT dir (writes both CSVs + figures):
    python3 plot_comparison_v5.py --txt_dir /path/to/summaries --strict_two_pushes

  Plot from existing CSV (main figure only):
    python3 plot_comparison_v5.py --in_csv /path/to/summary_by_object_nsafety.csv
"""

import argparse
import os
import glob
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# -------------------------
# Ground truth placeholders
# -------------------------
# Units:
#   m_kg: kg
#   zc_m: meters
#   theta_star_deg: degrees
GT = {
    "box":        {"m_kg": 0.664, "zc_m": 0.14624, "theta_star_deg": np.rad2deg(np.arctan(0.04515/0.14624))},
    "heart":      {"m_kg": 0.236, "zc_m": 0.09800, "theta_star_deg": np.rad2deg(np.arctan(0.04354/0.09800))},
    "flashlight": {"m_kg": 0.387, "zc_m": 0.09656, "theta_star_deg": np.rad2deg(np.arctan(0.02300/0.09656))},
    "monitor":    {"m_kg": 5.008, "zc_m": 0.25160, "theta_star_deg": np.rad2deg(np.arctan(0.06207/0.25160))},
}

OBJECT_ORDER = ["box", "heart", "flashlight", "monitor"]
VALID_OBJECTS = set(OBJECT_ORDER)

# -------------------------
# Regex helpers
# -------------------------
RE_OBJ_NS = re.compile(
    r'(?P<object>box|heart|flashlight|monitor)[_\- ](?P<nsafety>\d+(?:\.\d+)?)',
    re.IGNORECASE
)
RE_BLOCK_SPLIT = re.compile(r'Batch Fit Results:\s*', re.IGNORECASE)

# COMBINED line
RE_COMBINED = re.compile(
    r'COMBINED:\s*m=(?P<m>[\d\.]+)\s*kg,\s*zc=(?P<zc>[\d\.]+)\s*m,\s*th\*=(?P<th>[\d\.]+)\s*deg',
    re.IGNORECASE
)

# Phase lines (PUSH / RETRACT)
RE_PHASE = re.compile(
    r'(?P<phase>PUSH|RETRACT)\s*:\s*m=(?P<m>[\d\.]+)\s*kg,\s*zc=(?P<zc>[\d\.]+)\s*m,\s*th\*=(?P<th>[\d\.]+)\s*deg',
    re.IGNORECASE
)

PHASES = ["push", "retract"]  # normalized

def infer_obj_ns_from_filename(path: str):
    base = os.path.basename(path)
    m = RE_OBJ_NS.search(base)
    if not m:
        return None, None
    return m.group("object").lower(), float(m.group("nsafety"))

def parse_blocks_from_text(text: str):
    """
    Returns list of dicts, one per 'Batch Fit Results' block, in file order:
      {
        "combined": (m, zc, th) or None,
        "push":     (m, zc, th) or None,
        "retract":  (m, zc, th) or None,
      }
    """
    chunks = RE_BLOCK_SPLIT.split(text)
    blocks = [c.strip() for c in chunks if c.strip()]
    out = []
    for b in blocks:
        d = {"combined": None, "push": None, "retract": None}

        mc = RE_COMBINED.search(b)
        if mc:
            d["combined"] = (float(mc.group("m")), float(mc.group("zc")), float(mc.group("th")))

        # phase matches (there may be 0, 1, or 2)
        for mp in RE_PHASE.finditer(b):
            ph = mp.group("phase").lower()
            d[ph] = (float(mp.group("m")), float(mp.group("zc")), float(mp.group("th")))

        out.append(d)
    return out

def rel_err_pct(est, gt):
    """Relative magnitude error in percent: 100 * |est-gt|/|gt|."""
    if not np.isfinite(est) or not np.isfinite(gt) or abs(gt) < 1e-12:
        return np.nan
    return 100.0 * abs(est - gt) / abs(gt)

def attach_ground_truth_and_errors(agg: pd.DataFrame) -> pd.DataFrame:
    # Attach GT
    agg["m_gt_kg"] = agg["object"].apply(lambda o: GT[o]["m_kg"])
    agg["zc_gt_m"] = agg["object"].apply(lambda o: GT[o]["zc_m"])
    agg["theta_star_gt_deg"] = agg["object"].apply(lambda o: GT[o]["theta_star_deg"])

    # Mean errors
    if "m_est_mean_kg" in agg.columns:
        agg["m_err_kg"] = agg["m_est_mean_kg"] - agg["m_gt_kg"]
        agg["m_err_pct"] = [rel_err_pct(e, g) for e, g in zip(agg["m_est_mean_kg"], agg["m_gt_kg"])]

    if "zc_est_mean_m" in agg.columns:
        agg["zc_err_m"] = agg["zc_est_mean_m"] - agg["zc_gt_m"]
        agg["zc_err_pct"] = [rel_err_pct(e, g) for e, g in zip(agg["zc_est_mean_m"], agg["zc_gt_m"])]

    if "theta_star_est_mean_deg" in agg.columns:
        agg["theta_star_err_deg"] = agg["theta_star_est_mean_deg"] - agg["theta_star_gt_deg"]
        agg["theta_star_err_pct"] = [rel_err_pct(e, g) for e, g in zip(agg["theta_star_est_mean_deg"], agg["theta_star_gt_deg"])]

    # Per-push errors (if present)
    for push in ("push1", "push2"):
        mcol = f"m_est_{push}_kg"
        zcol = f"zc_est_{push}_m"
        tcol = f"theta_star_est_{push}_deg"
        if mcol in agg.columns:
            agg[f"m_err_pct_{push}"] = [rel_err_pct(e, g) for e, g in zip(agg[mcol], agg["m_gt_kg"])]
        if zcol in agg.columns:
            agg[f"zc_err_pct_{push}"] = [rel_err_pct(e, g) for e, g in zip(agg[zcol], agg["zc_gt_m"])]
        if tcol in agg.columns:
            agg[f"theta_star_err_pct_{push}"] = [rel_err_pct(e, g) for e, g in zip(agg[tcol], agg["theta_star_gt_deg"])]

    return agg.sort_values(["object", "n_safety"]).reset_index(drop=True)

def build_summary_from_txt(txt_dir: str, strict_two_pushes: bool):
    """
    Returns:
      summary_wide: per-(object, n_safety) aggregate (means across files), with errors attached
      summary_long: per-(object, n_safety, push_idx, phase) aggregate (means across files), with errors attached
      warnings: list[str]
    """
    txt_paths = sorted(glob.glob(os.path.join(txt_dir, "*.txt")))
    if not txt_paths:
        raise SystemExit(f"No .txt files found in: {txt_dir}")

    per_file_rows = []
    per_file_phase_rows = []
    warnings = []

    for p in txt_paths:
        obj, ns = infer_obj_ns_from_filename(p)
        if obj is None or ns is None:
            warnings.append(f"[WARN] Could not infer (object, n_safety) from filename: {p}")
            continue
        if obj not in VALID_OBJECTS:
            continue

        with open(p, "r") as f:
            text = f.read()

        blocks = parse_blocks_from_text(text)

        if strict_two_pushes and len(blocks) != 2:
            warnings.append(f"[WARN] Expected exactly 2 pushes, got {len(blocks)} in {p} (skipping due to --strict_two_pushes)")
            continue
        if len(blocks) == 0:
            warnings.append(f"[WARN] No 'Batch Fit Results' blocks parsed in {p}")
            continue

        # --- combined per push (if missing combined, fill NaN) ---
        vals_combined = []
        for b in blocks:
            if b["combined"] is None:
                vals_combined.append((np.nan, np.nan, np.nan))
            else:
                vals_combined.append(b["combined"])

        m_arr = np.array([v[0] for v in vals_combined], float)
        z_arr = np.array([v[1] for v in vals_combined], float)
        th_arr = np.array([v[2] for v in vals_combined], float)

        # per-push combined (NaN if not present)
        (m1, z1, t1) = (vals_combined[0] if len(vals_combined) >= 1 else (np.nan, np.nan, np.nan))
        (m2, z2, t2) = (vals_combined[1] if len(vals_combined) >= 2 else (np.nan, np.nan, np.nan))

        per_file_rows.append({
            "object": obj,
            "n_safety": float(ns),
            "source_txt": os.path.basename(p),
            "n_pushes_parsed": int(len(blocks)),

            # combined mean across pushes within this file
            "m_est_mean_kg": float(np.nanmean(m_arr)),
            "zc_est_mean_m": float(np.nanmean(z_arr)),
            "theta_star_est_mean_deg": float(np.nanmean(th_arr)),

            # push-specific (combined)
            "m_est_push1_kg": float(m1),
            "zc_est_push1_m": float(z1),
            "theta_star_est_push1_deg": float(t1),

            "m_est_push2_kg": float(m2),
            "zc_est_push2_m": float(z2),
            "theta_star_est_push2_deg": float(t2),
        })

        # --- per-phase rows (push/retract) per push ---
        for push_idx, b in enumerate(blocks, start=1):
            for phase in PHASES:
                v = b.get(phase, None)
                if v is None:
                    continue  # do not emit missing phase
                per_file_phase_rows.append({
                    "object": obj,
                    "n_safety": float(ns),
                    "source_txt": os.path.basename(p),
                    "push_idx": int(push_idx),
                    "phase": phase,  # "push" or "retract"
                    "m_est_kg": float(v[0]),
                    "zc_est_m": float(v[1]),
                    "theta_star_est_deg": float(v[2]),
                })

    if not per_file_rows:
        raise SystemExit("No rows parsed. Check filename patterns and txt format.")

    df = pd.DataFrame(per_file_rows)

    # Aggregate across multiple txts (runs) for same (object, n_safety)
    agg = df.groupby(["object", "n_safety"]).agg(
        n_files=("source_txt", "count"),
        n_pushes_total=("n_pushes_parsed", "sum"),

        m_est_mean_kg=("m_est_mean_kg", "mean"),
        zc_est_mean_m=("zc_est_mean_m", "mean"),
        theta_star_est_mean_deg=("theta_star_est_mean_deg", "mean"),

        m_est_push1_kg=("m_est_push1_kg", "mean"),
        zc_est_push1_m=("zc_est_push1_m", "mean"),
        theta_star_est_push1_deg=("theta_star_est_push1_deg", "mean"),

        m_est_push2_kg=("m_est_push2_kg", "mean"),
        zc_est_push2_m=("zc_est_push2_m", "mean"),
        theta_star_est_push2_deg=("theta_star_est_push2_deg", "mean"),
    ).reset_index()

    summary_wide = attach_ground_truth_and_errors(agg)

    # Long (phase) aggregation
    if per_file_phase_rows:
        dph = pd.DataFrame(per_file_phase_rows)
        aggph = dph.groupby(["object", "n_safety", "push_idx", "phase"]).agg(
            n_files=("source_txt", "count"),
            m_est_kg=("m_est_kg", "mean"),
            zc_est_m=("zc_est_m", "mean"),
            theta_star_est_deg=("theta_star_est_deg", "mean"),
        ).reset_index()

        # attach GT and errors
        aggph["m_gt_kg"] = aggph["object"].apply(lambda o: GT[o]["m_kg"])
        aggph["zc_gt_m"] = aggph["object"].apply(lambda o: GT[o]["zc_m"])
        aggph["theta_star_gt_deg"] = aggph["object"].apply(lambda o: GT[o]["theta_star_deg"])

        aggph["m_err_pct"] = [rel_err_pct(e, g) for e, g in zip(aggph["m_est_kg"], aggph["m_gt_kg"])]
        aggph["zc_err_pct"] = [rel_err_pct(e, g) for e, g in zip(aggph["zc_est_m"], aggph["zc_gt_m"])]
        aggph["theta_star_err_pct"] = [rel_err_pct(e, g) for e, g in zip(aggph["theta_star_est_deg"], aggph["theta_star_gt_deg"])]

        # stable order for readability
        aggph["obj_order"] = aggph["object"].apply(lambda o: OBJECT_ORDER.index(o))
        aggph = aggph.sort_values(["obj_order", "n_safety", "push_idx", "phase"]).drop(columns=["obj_order"]).reset_index(drop=True)

        summary_long = aggph
    else:
        summary_long = pd.DataFrame(columns=[
            "object","n_safety","push_idx","phase","n_files",
            "m_est_kg","zc_est_m","theta_star_est_deg",
            "m_gt_kg","zc_gt_m","theta_star_gt_deg",
            "m_err_pct","zc_err_pct","theta_star_err_pct",
        ])
        warnings.append("[WARN] No PUSH/RETRACT phase lines were parsed from any TXT. Phase CSV will be empty.")

    return summary_wide, summary_long, warnings

def make_main_figure(summary: pd.DataFrame, out_png: str, title_suffix: str = ""):
    """
    Main figure:
      - 1x4 grid of per-object panels (BOX, HEART, FLASHLIGHT, MONITOR).
      - Each panel shows per-n_safety grouped bars of *absolute relative error* (% of GT)
        for Mass, Zc, and θ*.
      - Titles are drawn INSIDE each axes.
    """
    need = {"m_err_pct", "zc_err_pct", "theta_star_err_pct"}
    missing = [c for c in need if c not in summary.columns]
    if missing:
        raise SystemExit(f"Missing columns in summary for main figure: {missing}")

    # Determine a shared y-limit for comparability
    ymax = np.nanmax(
        np.concatenate([
            summary["m_err_pct"].to_numpy(float),
            summary["zc_err_pct"].to_numpy(float),
            summary["theta_star_err_pct"].to_numpy(float),
        ])
    )
    if not np.isfinite(ymax):
        ymax = 1.0
    ymax *= 1.15  # headroom

    fig, axes = plt.subplots(1, 4, figsize=(16, 6), sharey=True)
    axes = axes.flatten()

    width = 0.24
    panel_labels = ['(a)', '(b)', '(c)', '(d)']

    for i, obj in enumerate(OBJECT_ORDER):
        ax = axes[i]
        sub = summary[summary["object"] == obj].sort_values("n_safety")

        if sub.empty:
            ax.text(0.5, 0.5, "No data", ha="center", va="center")
            ax.axis("off")
            continue

        ns = sub["n_safety"].to_numpy(float)
        x = np.arange(len(ns), dtype=float)

        ax.bar(x - width, sub["m_err_pct"].to_numpy(float), width=width, label="Mass (%GT)", color='tab:blue')
        ax.bar(x,         sub["zc_err_pct"].to_numpy(float), width=width, label="$z_c$ (%GT)", color='tab:orange')
        ax.bar(x + width, sub["theta_star_err_pct"].to_numpy(float), width=width, label="θ* (%GT)", color='tab:purple')

        ax.set_xticks(x)
        ax.set_xticklabels([f"{v:.2f}" for v in ns], fontsize=16)
        ax.tick_params(axis="y", labelsize=16)

        # Only bottom row gets x-label
        if i >= 2:
            ax.set_xlabel("$\\eta_{safety}$", fontsize=18)

        ax.grid(True, axis="y", alpha=0.35)
        ax.set_ylim(0, ymax)

        # Only left column gets y-label
        if i == 0:
            ax.set_ylabel("Relative Error (% GT)", fontsize=18)

        # Title INSIDE the axes
        title = f"{panel_labels[i]} {obj.upper()}"
        # Avoid legend overlap in top-left panel: place title top-right there.
        # if i == 0:
        #     ax.text(0.98, 0.98, title, transform=ax.transAxes,
        #             ha="right", va="top", fontsize=18,
        #             bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85))
        # elif i == 2:
        ax.text(0.98, 0.98, title, transform=ax.transAxes,
                ha="right", va="top", fontsize=18,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85))
        # else:
        #     ax.text(0.02, 0.98, title, transform=ax.transAxes,
        #             ha="left", va="top", fontsize=18,
        #             bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85))

    # Legend ONLY in top-left subplot
    handles, labels = axes[0].get_legend_handles_labels()
    axes[0].legend(handles, labels, loc="upper left", fontsize=17, frameon=True, framealpha=0.9)

    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)

def make_push_compare_figure(summary: pd.DataFrame, out_png: str, title_suffix: str = ""):
    """
    Push1 vs Push2 comparison (BAR CHARTS):
      - Grid: rows=objects, cols=3 metrics (mass, zc, θ*)
      - x-axis: n_safety (categorical)
      - y-axis: abs relative error (%GT)
      - bars: Push1 vs Push2
    Only works if per-push error columns exist.
    """
    need = {
        "m_err_pct_push1","m_err_pct_push2",
        "zc_err_pct_push1","zc_err_pct_push2",
        "theta_star_err_pct_push1","theta_star_err_pct_push2"
    }
    missing = [c for c in need if c not in summary.columns]
    if missing:
        return False, missing

    fig = plt.figure(figsize=(18, 10))
    gs = GridSpec(len(OBJECT_ORDER), 3, figure=fig, hspace=0.28, wspace=0.16)

    metrics = [
        ("Mass", "m_err_pct_push1", "m_err_pct_push2"),
        ("$z_c$", "zc_err_pct_push1", "zc_err_pct_push2"),
        # ("θ*", "theta_star_err_pct_push1", "theta_star_err_pct_push2"),
    ]

    bar_w = 0.36

    for r, obj in enumerate(OBJECT_ORDER):
        sub = summary[summary["object"] == obj].sort_values("n_safety")
        ns = sub["n_safety"].to_numpy(float)
        x = np.arange(len(ns), dtype=float)

        for c, (title, p1col, p2col) in enumerate(metrics):
            ax = fig.add_subplot(gs[r, c])
            if sub.empty:
                ax.text(0.5, 0.5, "No data", ha="center", va="center")
                ax.axis("off")
                continue

            ax.bar(x - bar_w/2, sub[p1col].to_numpy(float), width=bar_w, label="Push1", color='tab:cyan')
            ax.bar(x + bar_w/2, sub[p2col].to_numpy(float), width=bar_w, label="Push2", color='tab:green')

            ax.set_xticks(x)
            ax.set_xticklabels([f"{v:.2f}" for v in ns], fontsize=14)
            ax.tick_params(axis="y", labelsize=14)

            if r == len(OBJECT_ORDER) - 1:
                ax.set_xlabel("$\\eta_{safety}$", fontsize=14)

            ax.grid(True, axis="y", alpha=0.35)

            if r == 0:
                ax.set_title(title, fontsize=16, pad=8)

            if c == 0:
                ax.set_ylabel(f"{obj.upper()}\nRel Err (% GT)", fontsize=14)

            if r == 0 and c == 0:
                ax.legend(loc="upper left", fontsize=12)

    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return True, None

def _default_out_paths(txt_dir: str):
    out_csv = os.path.join(txt_dir, "summary_by_object_nsafety.csv")
    out_csv_phases = os.path.join(txt_dir, "summary_by_object_nsafety_push_phase.csv")
    out_png = os.path.join(txt_dir, "estimation_performance_summary.png")
    out_png_push = os.path.join(txt_dir, "push1_vs_push2_summary.png")
    return out_csv, out_csv_phases, out_png, out_png_push

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--txt_dir", type=str, default=None, help="Directory of .txt fit summaries to parse")
    ap.add_argument("--in_csv", type=str, default=None, help="Use an existing CSV instead of parsing txt")
    ap.add_argument("--out_csv", type=str, default=None, help="Where to write wide CSV (only when parsing txt)")
    ap.add_argument("--out_csv_phases", type=str, default=None, help="Where to write phase/push long CSV ( profiler )")
    ap.add_argument("--out_png", type=str, default=None, help="Where to write the main PNG figure")
    ap.add_argument("--out_png_push_compare", type=str, default=None, help="Where to write Push1 vs Push2 PNG figure")
    ap.add_argument("--strict_two_pushes", action="store_true", help="Require exactly 2 pushes in each txt")
    args = ap.parse_args()

    if args.txt_dir is None and args.in_csv is None:
        raise SystemExit("Provide either --txt_dir or --in_csv")

    if args.txt_dir is not None:
        out_csv, out_csv_phases, out_png, out_png_push = _default_out_paths(args.txt_dir)
        out_csv = args.out_csv or out_csv
        out_csv_phases = args.out_csv_phases or out_csv_phases
        out_png = args.out_png or out_png
        out_png_push = args.out_png_push_compare or out_png_push

        summary_wide, summary_long, warnings = build_summary_from_txt(
            args.txt_dir, strict_two_pushes=args.strict_two_pushes
        )

        summary_wide.to_csv(out_csv, index=False)
        summary_long.to_csv(out_csv_phases, index=False)

        for w in warnings:
            print(w)
        print(f"[OK] Wrote CSV (wide):  {out_csv}")
        print(f"[OK] Wrote CSV (long):  {out_csv_phases}")

        make_main_figure(summary_wide, out_png)
        print(f"[OK] Wrote main figure: {out_png}")

        ok, missing = make_push_compare_figure(summary_wide, out_png_push)
        if ok:
            print(f"[OK] Wrote push-compare figure: {out_png_push}")
        else:
            print(f"[WARN] Skipping push-compare figure (missing columns): {missing}")

    else:
        # in_csv mode
        out_png = args.out_png or "estimation_performance_summary.png"
        out_png_push = args.out_png_push_compare or "push1_vs_push2_summary.png"

        summary = pd.read_csv(args.in_csv)
        if "m_err_pct" not in summary.columns and "m_est_mean_kg" in summary.columns:
            summary = attach_ground_truth_and_errors(summary)

        make_main_figure(summary, out_png)
        print(f"[OK] Wrote main figure: {out_png}")

        ok, missing = make_push_compare_figure(summary, out_png_push)
        if ok:
            print(f"[OK] Wrote push-compare figure: {out_png_push}")
        else:
            print(f"[WARN] Skipping push-compare figure (missing columns): {missing}")

if __name__ == "__main__":
    main()
