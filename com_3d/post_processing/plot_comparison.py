#!/usr/bin/env python3
"""
plot_comparison.py

Generates performance comparison plots from per-object estimate CSVs:
  1. Relative error summary (per-object, per-n_safety)
  2. Absolute error summary (distribution across all trials)

Usage:
  python3 plot_comparison.py
  (reads from ./post_processing/*.estimates.csv, outputs PNG figures)
"""

import argparse
import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load ground truth from JSON
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GT_JSON_PATH = os.path.join(SCRIPT_DIR, "ground_truth.json")

with open(GT_JSON_PATH, 'r') as f:
    GT = json.load(f)

OBJECT_ORDER = ["box", "heart", "flashlight", "monitor"]
VALID_OBJECTS = set(OBJECT_ORDER)

def rel_err_pct(est, gt):
    """Relative magnitude error in percent: 100 * |est-gt|/|gt|."""
    if not np.isfinite(est) or not np.isfinite(gt) or abs(gt) < 1e-12:
        return np.nan
    return 100.0 * abs(est - gt) / abs(gt)

def make_main_figure(summary: pd.DataFrame, out_png: str):
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
    # Exclude flashlight n_safety=0.1 from y-limit calculation
    summary_for_ylim = summary[~((summary["object"] == "flashlight") & (summary["n_safety"] == 0.1))]
    ymax = np.nanmax(
        np.concatenate([
            summary_for_ylim["m_err_pct"].to_numpy(float),
            summary_for_ylim["zc_err_pct"].to_numpy(float),
            summary_for_ylim["theta_star_err_pct"].to_numpy(float),
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
        
        # Zero out flashlight n_safety=0.1 bars (poor results, omit visually but keep spacing)
        if obj == "flashlight":
            sub = sub.copy()
            sub.loc[sub["n_safety"] == 0.1, ["m_err_pct", "zc_err_pct", "theta_star_err_pct"]] = 0

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

def load_per_object_estimates():
    """
    Load per-object estimate CSVs from post_processing directory
    and extract weighted_avg rows for plotting.
    
    Returns summary_wide DataFrame with columns needed for make_main_figure().
    """
    rows = []
    for obj in OBJECT_ORDER:
        csv_path = os.path.join(SCRIPT_DIR, f"{obj}_estimates.csv")
        if not os.path.exists(csv_path):
            print(f"[WARN] Missing {obj}_estimates.csv, skipping {obj}")
            continue
        
        df = pd.read_csv(csv_path)
        # Filter for weighted_avg phase only
        weighted = df[df["phase"] == "weighted_avg"].copy()
        
        if weighted.empty:
            print(f"[WARN] No weighted_avg rows in {obj}_estimates.csv")
            continue
        
        # Check what error columns exist and rename appropriately
        rename_map = {
            "m_est_kg": "m_est_mean_kg",
            "zc_est_m": "zc_est_mean_m",
            "theta_star_est_deg": "theta_star_est_mean_deg",
        }
        
        # Handle both possible error column names
        if "m_error_pct" in weighted.columns:
            rename_map["m_error_pct"] = "m_err_pct"
        if "zc_error_pct" in weighted.columns:
            rename_map["zc_error_pct"] = "zc_err_pct"
        if "theta_star_error_pct" in weighted.columns:
            rename_map["theta_star_error_pct"] = "theta_star_err_pct"
        
        weighted = weighted.rename(columns=rename_map)
        
        rows.append(weighted[["object", "n_safety", "m_est_mean_kg", "zc_est_mean_m", 
                               "theta_star_est_mean_deg", "m_err_pct", "zc_err_pct", 
                               "theta_star_err_pct"]])
    
    if not rows:
        raise SystemExit("No valid per-object estimate CSVs found")
    
    summary = pd.concat(rows, ignore_index=True)
    return summary

def _summary_stats(df: pd.DataFrame, label: str):
    """Compute median/mean/max absolute error for m, zc, and theta* (in %)."""
    stats = {
        "group": label,
        "m_abs_err_median_pct": float(np.nanmedian(np.abs(df["m_err_pct"]))),
        "m_abs_err_mean_pct": float(np.nanmean(np.abs(df["m_err_pct"]))),
        "m_abs_err_max_pct": float(np.nanmax(np.abs(df["m_err_pct"]))),
        "zc_abs_err_median_pct": float(np.nanmedian(np.abs(df["zc_err_pct"]))),
        "zc_abs_err_mean_pct": float(np.nanmean(np.abs(df["zc_err_pct"]))),
        "zc_abs_err_max_pct": float(np.nanmax(np.abs(df["zc_err_pct"]))),
        "theta_star_abs_err_median_pct": float(np.nanmedian(np.abs(df["theta_star_err_pct"]))),
        "theta_star_abs_err_mean_pct": float(np.nanmean(np.abs(df["theta_star_err_pct"]))),
        "theta_star_abs_err_max_pct": float(np.nanmax(np.abs(df["theta_star_err_pct"]))),
    }
    return stats

def main():
    """Load per-object estimate CSVs and generate performance plots."""
    out_png = os.path.join(SCRIPT_DIR, "NEW_estimation_performance_summary.png")
    out_csv = os.path.join(SCRIPT_DIR, "NEW_estimation_performance_summary.csv")
    out_abs_png = os.path.join(SCRIPT_DIR, "NEW_absolute_error_summary.png")
    
    summary_wide = load_per_object_estimates()
    
    make_main_figure(summary_wide, out_png)
    print(f"[OK] Wrote figure: {out_png}")
    print(f"[INFO] Used weighted_avg phase from {len(summary_wide)} rows (4 objects × 3 n_safety)")

    # Build summary stats table (grand + per-object)
    summary_rows = []
    summary_rows.append(_summary_stats(summary_wide, "ALL_OBJECTS"))
    for obj in OBJECT_ORDER:
        sub = summary_wide[summary_wide["object"] == obj]
        if not sub.empty:
            summary_rows.append(_summary_stats(sub, f"OBJECT:{obj}"))

    summary_df = pd.DataFrame(summary_rows)
    summary_df.to_csv(out_csv, index=False)
    print(f"[OK] Wrote summary CSV: {out_csv}")
    
    # Generate absolute error summary figure
    plot_absolute_error_summary(summary_wide, GT, out_abs_png)

def plot_absolute_error_summary(summary_wide: pd.DataFrame, gt_dict: dict, out_png: str):
    """
    Generate absolute error summary figure with boxplots and strip plots.
    
    Args:
        summary_wide: DataFrame with columns: object, n_safety, m_est_mean_kg, zc_est_mean_m, 
                     theta_star_est_mean_deg
        gt_dict: Ground truth dict keyed by object name with keys m_kg, zc_m, theta_star_deg
        out_png: Output PNG path
    """
    # Compute absolute errors
    abs_errors = []
    for _, row in summary_wide.iterrows():
        obj = row["object"]
        if obj not in gt_dict:
            continue
        gt = gt_dict[obj]
        abs_errors.append({
            "object": obj,
            "n_safety": row["n_safety"],
            "abs_m_err_kg": abs(row["m_est_mean_kg"] - gt["m_kg"]),
            "abs_zc_err_m": abs(row["zc_est_mean_m"] - gt["zc_m"]),
            "abs_zc_err_cm": abs(row["zc_est_mean_m"] - gt["zc_m"]) * 100.0,  # Convert to cm
            "abs_theta_err_deg": abs(row["theta_star_est_mean_deg"] - gt["theta_star_deg"]),
        })
    
    if not abs_errors:
        print("[WARN] No absolute error data to plot")
        return
    
    abs_df = pd.DataFrame(abs_errors)
    
    # Print console summary
    print("\n[ABSOLUTE ERROR SUMMARY]")
    print(f"Median mass error: {abs_df['abs_m_err_kg'].median():.4f} kg")
    print(f"Median z_c error: {abs_df['abs_zc_err_m'].median():.4f} m ({abs_df['abs_zc_err_cm'].median():.2f} cm)")
    print(f"Median θ* error: {abs_df['abs_theta_err_deg'].median():.2f} deg")
    print()
    
    # Create figure
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    
    # Define colors for objects
    obj_colors = {obj: plt.cm.tab10(i) for i, obj in enumerate(OBJECT_ORDER)}
    
    # Panel 1: Mass error
    ax = axes[0]
    sns.boxplot(data=abs_df, x="n_safety", y="abs_m_err_kg", ax=ax, color="lightgray", width=0.6)
    sns.stripplot(data=abs_df, x="n_safety", y="abs_m_err_kg", hue="object", ax=ax, size=8, 
                  palette=obj_colors, jitter=0.08, dodge=True, alpha=0.7)
    ax.set_xlabel("$\\eta_{safety}$", fontsize=14)
    ax.set_ylabel("Absolute error (kg)", fontsize=14)
    ax.set_xticklabels(["0.1", "0.5", "0.65"])
    ax.grid(True, axis="y", alpha=0.3)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if ax.get_legend():
        ax.get_legend().remove()
    
    # Panel 2: z_c error (in cm)
    ax = axes[1]
    sns.boxplot(data=abs_df, x="n_safety", y="abs_zc_err_cm", ax=ax, color="lightgray", width=0.6)
    sns.stripplot(data=abs_df, x="n_safety", y="abs_zc_err_cm", hue="object", ax=ax, size=8, 
                  palette=obj_colors, jitter=0.08, dodge=True, alpha=0.7)
    ax.set_xlabel("$\\eta_{safety}$", fontsize=14)
    ax.set_ylabel("Absolute error (cm)", fontsize=14)
    ax.set_xticklabels(["0.1", "0.5", "0.65"])
    ax.grid(True, axis="y", alpha=0.3)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if ax.get_legend():
        ax.get_legend().remove()
    
    # Panel 3: theta error
    ax = axes[2]
    sns.boxplot(data=abs_df, x="n_safety", y="abs_theta_err_deg", ax=ax, color="lightgray", width=0.6)
    sns.stripplot(data=abs_df, x="n_safety", y="abs_theta_err_deg", hue="object", ax=ax, size=8, 
                  palette=obj_colors, jitter=0.08, dodge=True, alpha=0.7)
    ax.set_xlabel("$\\eta_{safety}$", fontsize=14)
    ax.set_ylabel("Absolute error (deg)", fontsize=14)
    ax.set_xticklabels(["0.1", "0.5", "0.65"])
    ax.grid(True, axis="y", alpha=0.3)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    
    # Extract legend from last plot
    handles, labels = ax.get_legend_handles_labels()
    # Filter out duplicate hue entries (seaborn creates them for each stripplot call)
    seen = set()
    unique_handles, unique_labels = [], []
    for h, l in zip(handles, labels):
        if l not in seen and l in OBJECT_ORDER:
            unique_handles.append(h)
            unique_labels.append(l)
            seen.add(l)
    
    ax.get_legend().remove()
    
    # Add legend at top-center
    fig.legend(unique_handles, unique_labels, loc="upper center", bbox_to_anchor=(0.5, 1.08),
               ncol=len(OBJECT_ORDER), fontsize=12, frameon=True)
    
    # Overall title
    # fig.suptitle("Absolute Estimation Error Across All Objects and Trials", 
    #              fontsize=16, y=0.98)
    
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    print(f"[OK] Wrote absolute error figure: {out_png}")
    plt.close(fig)
    
    return fig

if __name__ == "__main__":
    main()
