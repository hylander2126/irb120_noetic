#!/usr/bin/env python3
"""
plot_comparison_v3.py

Changes vs prior:
  - Top plot is now *per-object-per-n_safety* (no averaging across n_safety).
  - Top plot includes relative theta* error (% of GT) in addition to mass and zc.
  - Adds an optional second figure comparing Push1 vs Push2 errors for each object, broken out by n_safety.

Inputs:
  A) Parse TXT directory (preferred if you want Push1 vs Push2 plots):
       python3 plot_comparison_v3.py --txt_dir /path/to/summaries

  B) Or plot from an existing summary CSV (top + n_safety plots only, unless per-push cols exist):
       python3 plot_comparison_v3.py --in_csv /path/to/summary.csv

Outputs:
  --out_csv (always written when parsing txt)
  --out_png (main figure)
  --out_png_push_compare (push1 vs push2 figure; written only when per-push columns exist)

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
RE_COMBINED = re.compile(
    r'COMBINED:\s*m=(?P<m>[\d\.]+)\s*kg,\s*zc=(?P<zc>[\d\.]+)\s*m,\s*th\*=(?P<th>[\d\.]+)\s*deg',
    re.IGNORECASE
)

def infer_obj_ns_from_filename(path: str):
    base = os.path.basename(path)
    m = RE_OBJ_NS.search(base)
    if not m:
        return None, None
    return m.group("object").lower(), float(m.group("nsafety"))

def parse_combined_from_text(text: str):
    """Returns list of (m_kg, zc_m, th_deg) from each Batch Fit Results block, in file order."""
    chunks = RE_BLOCK_SPLIT.split(text)
    blocks = [c.strip() for c in chunks if c.strip()]
    vals = []
    for b in blocks:
        mc = RE_COMBINED.search(b)
        if mc:
            vals.append((float(mc.group("m")), float(mc.group("zc")), float(mc.group("th"))))
    return vals

def rel_err_pct(est, gt):
    """Relative magnitude error in percent: 100 * |est-gt|/|gt|."""
    if not np.isfinite(est) or not np.isfinite(gt) or abs(gt) < 1e-12:
        return np.nan
    return 100.0 * abs(est - gt) / abs(gt)

def _ordered_objects(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df = df[df["object"].isin(OBJECT_ORDER)]
    df["order"] = df["object"].apply(lambda o: OBJECT_ORDER.index(o))
    return df.sort_values(["order"]).drop(columns=["order"])

def attach_ground_truth_and_errors(agg: pd.DataFrame) -> pd.DataFrame:
    # Attach GT
    agg["m_gt_kg"] = agg["object"].apply(lambda o: GT[o]["m_kg"])
    agg["zc_gt_m"] = agg["object"].apply(lambda o: GT[o]["zc_m"])
    agg["theta_star_gt_deg"] = agg["object"].apply(lambda o: GT[o]["theta_star_deg"])

    # Signed absolute errors (native units) for mean estimate
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
    txt_paths = sorted(glob.glob(os.path.join(txt_dir, "*.txt")))
    if not txt_paths:
        raise SystemExit(f"No .txt files found in: {txt_dir}")

    per_file_rows = []
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

        vals = parse_combined_from_text(text)
        if strict_two_pushes and len(vals) != 2:
            warnings.append(f"[WARN] Expected exactly 2 pushes, got {len(vals)} in {p} (skipping due to --strict_two_pushes)")
            continue
        if len(vals) == 0:
            warnings.append(f"[WARN] No COMBINED lines parsed in {p}")
            continue

        m_arr = np.array([v[0] for v in vals], float)
        z_arr = np.array([v[1] for v in vals], float)
        th_arr = np.array([v[2] for v in vals], float)

        # per-push (NaN if not present)
        m1, z1, t1 = (vals[0] if len(vals) >= 1 else (np.nan, np.nan, np.nan))
        m2, z2, t2 = (vals[1] if len(vals) >= 2 else (np.nan, np.nan, np.nan))

        per_file_rows.append({
            "object": obj,
            "n_safety": float(ns),
            "source_txt": os.path.basename(p),
            "n_pushes_parsed": int(len(vals)),

            # combined mean across pushes within this file
            "m_est_mean_kg": float(np.nanmean(m_arr)),
            "zc_est_mean_m": float(np.nanmean(z_arr)),
            "theta_star_est_mean_deg": float(np.nanmean(th_arr)),

            # push-specific
            "m_est_push1_kg": float(m1),
            "zc_est_push1_m": float(z1),
            "theta_star_est_push1_deg": float(t1),

            "m_est_push2_kg": float(m2),
            "zc_est_push2_m": float(z2),
            "theta_star_est_push2_deg": float(t2),
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

    return attach_ground_truth_and_errors(agg), warnings

def _iter_object_ns_rows(summary: pd.DataFrame):
    """Yield (object, nsafety, row) in stable plotting order."""
    for obj in OBJECT_ORDER:
        sub = summary[summary["object"] == obj]
        if sub.empty:
            continue
        for ns in sorted(sub["n_safety"].unique().tolist()):
            r = sub[sub["n_safety"] == ns].iloc[0]
            yield obj, ns, r

def make_main_figure(summary: pd.DataFrame, out_png: str, title_suffix: str = ""):
    """
    Main figure (no redundant bottom plots):
      - 2x2 grid of per-object panels (BOX, HEART, FLASHLIGHT, MONITOR).
      - Each panel shows per-n_safety grouped bars of *absolute relative error* (% of GT)
        for Mass, Zc, and θ*.
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

    fig, axes = plt.subplots(2, 2, figsize=(12, 12), sharey=True)
    axes = axes.flatten()

    width = 0.24

    panel_labels = ['(a)', '(b)', '(c)', '(d)']

    for i, obj in enumerate(OBJECT_ORDER):
        ax = axes[i]
        sub = summary[summary["object"] == obj].sort_values("n_safety")
        ax.set_title(f"{panel_labels[i]} {obj.upper()}", fontsize=20, pad=8)

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
        ax.set_xticklabels([f"{v:.2f}" for v in ns], fontsize=18)
        ax.set_yticklabels(ax.get_yticks(), fontsize=18)
        if i > 1: # For top two rows, don't show x-labels
            ax.set_xlabel("$\\eta_{safety}$", fontsize=20)
        ax.grid(True, axis="y", alpha=0.35)
        ax.set_ylim(0, ymax)

        # Only left column gets y-label
        if i % 2 == 0:
            ax.set_ylabel("Relative Error (% GT)", fontsize=20)

    # Single legend (top-left panel)
    # handles, labels = axes[0].get_legend_handles_labels()
    # fig.legend(handles, labels, loc="upper center", ncols=3, frameon=True, bbox_to_anchor=(0.5, 0.98))
    
    # Legend ONLY in top-left subplot
    handles, labels = axes[0].get_legend_handles_labels()
    axes[0].legend(
        handles,
        labels,
        loc="upper left",
        fontsize=18,
        frameon=True
    )

    # fig.suptitle("Estimation Performance Summary" + (f" {title_suffix}" if title_suffix else ""), fontsize=18, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
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

    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(len(OBJECT_ORDER), 2, figure=fig, hspace=0.3, wspace=0.15) # 3

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
            ax.set_xticklabels([f"{v:.2f}" for v in ns], fontsize=18)
            ax.set_yticklabels(ax.get_yticks(), fontsize=18)
            if r == len(OBJECT_ORDER) - 1: # bottom row
                ax.set_xlabel("$\\eta_{safety}$", fontsize=18)
            ax.grid(True, axis="y", alpha=0.35)

            if r == 0:
                ax.set_title(title, fontsize=20, pad=8)

            if c == 0:
                ax.set_ylabel(f"{obj.upper()}\nRel Err (% GT)", fontsize=18)
            # else:
            #     ax.set_ylabel("Rel err (%GT)", fontsize=14)

            if r == 0 and c == 0:
                ax.legend(loc="upper left", fontsize=18)

    # fig.suptitle("Push1 vs Push2 Error by Object and n_safety" + (f" {title_suffix}" if title_suffix else ""), fontsize=18, y=0.99)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return True, None

def _default_out_paths(txt_dir: str):
    out_csv = os.path.join(txt_dir, "summary_by_object_nsafety.csv")
    out_png = os.path.join(txt_dir, "estimation_performance_summary.png")
    out_png_push = os.path.join(txt_dir, "push1_vs_push2_summary.png")
    return out_csv, out_png, out_png_push

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--txt_dir", type=str, default=None, help="Directory of .txt fit summaries to parse")
    ap.add_argument("--in_csv", type=str, default=None, help="Use an existing CSV instead of parsing txt")
    ap.add_argument("--out_csv", type=str, default=None, help="Where to write CSV (only when parsing txt)")
    ap.add_argument("--out_png", type=str, default=None, help="Where to write the main PNG figure")
    ap.add_argument("--out_png_push_compare", type=str, default=None, help="Where to write Push1 vs Push2 PNG figure")
    ap.add_argument("--strict_two_pushes", action="store_true", help="Require exactly 2 pushes in each txt")
    args = ap.parse_args()

    if args.txt_dir is None and args.in_csv is None:
        raise SystemExit("Provide either --txt_dir or --in_csv")

    if args.txt_dir is not None:
        out_csv, out_png, out_png_push = _default_out_paths(args.txt_dir)
        out_csv = args.out_csv or out_csv
        out_png = args.out_png or out_png
        out_png_push = args.out_png_push_compare or out_png_push

        summary, warnings = build_summary_from_txt(args.txt_dir, strict_two_pushes=args.strict_two_pushes)
        summary.to_csv(out_csv, index=False)

        for w in warnings:
            print(w)
        print(f"[OK] Wrote CSV: {out_csv}")

        make_main_figure(summary, out_png)
        print(f"[OK] Wrote main figure: {out_png}")

        ok, missing = make_push_compare_figure(summary, out_png_push)
        if ok:
            print(f"[OK] Wrote push-compare figure: {out_png_push}")
        else:
            print(f"[WARN] Skipping push-compare figure (missing columns): {missing}")

    else:
        # in_csv mode
        out_png = args.out_png or "estimation_performance_summary.png"
        out_png_push = args.out_png_push_compare or "push1_vs_push2_summary.png"

        summary = pd.read_csv(args.in_csv)
        # If user provided a raw CSV without GT/errors, attempt to attach if possible
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
