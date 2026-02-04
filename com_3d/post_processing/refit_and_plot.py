#!/usr/bin/env python3
"""
refit_and_plot.py

Post-processing script to refit experimental data and regenerate fit plots.

Takes a raw experiment CSV (from sync_logger) and:
  1. Filters and masks push/retract phases
  2. Fits CoM parameters separately for push and retract
  3. Generates a publication-quality plot with scatter data and fitted curves

Features:
  - Customizable ground truth theta_star value
  - Data point decimation for plot readability
  - Supports both push and retract phase visualization
  - Batch processing by object: processes all n_safety trials (0.100, 0.500, 0.650)
    - Uses experiment estimates from summary_by_object_nsafety_phase.csv (no refitting)

===== CONFIGURATION =====
THETA_STAR_GT_DEG: Ground truth theta_star value in degrees (used for all trials)
DECIMATE: Data point decimation factor (1 = no decimation)
N_SAFETY_VALUES: List of n_safety values to process (default: [0.100, 0.500, 0.650])
=========================

Usage:
  python3 refit_and_plot.py --csv_base /path/to/experiments/20260131_box
  
  This will process:
    - 20260131_box_0.100.csv
    - 20260131_box_0.500.csv
    - 20260131_box_0.650.csv
  
  And generate three corresponding _fit.png plots.
"""

import argparse
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# ===== CONFIGURATION =====
THETA_STAR_GT_DEG = 17.2  # Ground truth theta_star (degrees) - change this for different objects/targets
DECIMATE = 40  # Data point decimation factor - adjust for readability
N_SAFETY_VALUES = [0.100, 0.500, 0.650]  # n_safety trials to process
THETA_MIN_DEG = 1.0  # Ignore data below this angle (degrees)
THETA_MAX_MARGIN_DEG = 1.0  # Ignore data within this margin of max angle (degrees)
SUMMARY_CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "summary_by_object_nsafety_phase.csv")
# =========================

# Import com_3d utilities
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from com_3d.com_estimation import tau_app_model, tau_model
from com_3d.helper_fns import quat_normalize, quat_conj, quat_mul, quat_to_rotvec

class RefitEstimator:
    """
    Refit estimator for post-processing experimental data.
    """
    def __init__(self, o_obj, rc0_known, e_hat, fs_hz=500.0):
        self.o_obj = np.array(o_obj, dtype=float)
        self.rc0 = np.array(rc0_known, dtype=float)
        self.e_hat = np.array(e_hat, dtype=float)
        self.e_hat /= (np.linalg.norm(self.e_hat) + 1e-12)
        self.d = float(np.linalg.norm(self.rc0 - np.dot(self.rc0, self.e_hat) * self.e_hat))
        self.fs_hz = fs_hz
        # Filter params (4th order, 5Hz)
        self.b, self.a = butter(4, 5.0, fs=self.fs_hz, btype='low')

    def process(self, csv_path, summary_df, object_name, n_safety, q_ref=None):
        """
        Load CSV, process masks, and use summary estimates for push/retract fits.
        
        Returns:
            dict with all processed data and fit results
        """
        df = pd.read_csv(csv_path)
        rospy_time = df['ros_time_sec'].values
        f_raw = df[['fx', 'fy', 'fz']].values
        ee_raw = df[['ee_x', 'ee_y', 'ee_z']].values
        q_raw = df[['tag_qx', 'tag_qy', 'tag_qz', 'tag_qw']].values
        c_raw = df['fw_contact'].values.astype(bool)
        r_raw = df['ft_trigger'].values.astype(bool)

        # Set reference quaternion if not provided
        if q_ref is None:
            q_ref = q_raw[0]
        q_ref = quat_normalize(np.array(q_ref, float))

        # Filter force
        f_filt = filtfilt(self.b, self.a, f_raw, axis=0)

        # Calculate rotation angles
        rot_vecs = []
        q_ref_conj = quat_conj(q_ref)
        for q in q_raw:
            q_norm = quat_normalize(q)
            q_rel = quat_mul(q_ref_conj, q_norm)
            rv = quat_to_rotvec(q_rel, normalize=False)
            rot_vecs.append(rv)

        rot_vecs = np.array(rot_vecs)
        theta_exp = filtfilt(self.b, self.a, np.linalg.norm(rot_vecs, axis=1))

        # Masking
        theta_min = np.deg2rad(THETA_MIN_DEG)
        theta_max = float(np.nanmax(theta_exp))
        theta_max_allowed = theta_max - np.deg2rad(THETA_MAX_MARGIN_DEG)
        valid_theta = (theta_exp >= theta_min) & (theta_exp <= theta_max_allowed)

        # Only first push: find where contact first becomes True
        contact_indices = np.where(c_raw)[0]
        if len(contact_indices) == 0:
            raise ValueError("No contact data found in CSV")
        
        contact_start = contact_indices[0]
        contact_end = contact_indices[-1]
        
        # Find where ft_trigger latches True within contact window (marks start of retract phase)
        retract_indices = np.where(r_raw[contact_start:contact_end+1])[0]
        if len(retract_indices) == 0:
            raise ValueError("ft_trigger never latched during contact window. Cannot determine retract phase.")
        
        retract_start = contact_start + retract_indices[0]
        
        print(f"[DEBUG] contact_start={contact_start}, retract_start={retract_start}, contact_end={contact_end}")

        # Mask for push phase (contact=True, ft_trigger=False) and retract phase (contact=True, ft_trigger=True)
        push_mask = np.zeros(len(c_raw), dtype=bool)
        retr_mask = np.zeros(len(c_raw), dtype=bool)
        
        push_mask[contact_start:retract_start] = c_raw[contact_start:retract_start] & (~r_raw[contact_start:retract_start]) & valid_theta[contact_start:retract_start]
        retr_mask[retract_start:contact_end+1] = c_raw[retract_start:contact_end+1] & (r_raw[retract_start:contact_end+1]) & valid_theta[retract_start:contact_end+1]

        n_push = int(np.sum(push_mask))
        n_retr = int(np.sum(retr_mask))

        print(f"[DEBUG] push samples={n_push}, retract samples={n_retr}")

        if n_push < 15 or n_retr < 15:
            raise ValueError(f"Not enough samples after masking: push={n_push}, retract={n_retr}")

        def _trim(m):
            return f_filt[m], theta_exp[m], ee_raw[m], rospy_time[m]

        f_push, th_push, ee_push, t_push = _trim(push_mask)
        f_retr, th_retr, ee_retr, t_retr = _trim(retr_mask)

        # Calculate torque
        def _get_raveled_tapp(ee_pos, force):
            return tau_app_model(-force, ee_pos - self.o_obj)

        tau_push = _get_raveled_tapp(ee_push, f_push)
        tau_retr = _get_raveled_tapp(ee_retr, f_retr)

        # Pull experiment estimates from summary CSV
        def _get_phase_est(phase: str):
            sel = summary_df[
                (summary_df["object"] == object_name)
                & (summary_df["n_safety"] == float(n_safety))
                & (summary_df["push_idx"] == 1)
                & (summary_df["phase"] == phase)
            ]
            if sel.empty:
                raise ValueError(f"No summary estimates for {object_name}, n_safety={n_safety:.3f}, phase={phase}")
            row = sel.iloc[0]
            return {
                "m": float(row["m_est_kg"]),
                "zc": float(row["zc_est_m"]),
                "ths_deg": float(row["theta_star_est_deg"]),
            }

        fit_push = _get_phase_est("push")
        fit_retr = _get_phase_est("retract")

        # Combined estimate for annotation (mean of phase estimates)
        m_est = 0.5 * (fit_push["m"] + fit_retr["m"])
        zc_est = 0.5 * (fit_push["zc"] + fit_retr["zc"])
        ths_est = float(np.arctan2(self.d, zc_est))

        return {
            "fit_push": fit_push,
            "fit_retr": fit_retr,
            "m_est": m_est,
            "zc_est": zc_est,
            "ths_est": ths_est,
            "th_push": th_push,
            "tau_push": tau_push,
            "th_retr": th_retr,
            "tau_retr": tau_retr,
            "rc0": self.rc0,
            "e_hat": self.e_hat,
        }


def make_fit_plot(process_result, theta_star_gt_deg, csv_path, decimate=1):
    """
    Generate fit plot with scatter data and fitted curves.
    
    Args:
        process_result: dict from RefitEstimator.process()
        theta_star_gt_deg: ground truth theta_star in degrees
        csv_path: path to source CSV (used to determine output filename)
        decimate: downsample factor for data points (1 = no decimation)
    """
    fit_push = process_result["fit_push"]
    fit_retr = process_result["fit_retr"]
    ths_est = process_result["ths_est"]
    m_est = process_result["m_est"]
    zc_est = process_result["zc_est"]

    # Extract data (use masked data; no refitting)
    th_p = process_result["th_push"]
    tau_p_vec = process_result["tau_push"].reshape(-1, 3)
    th_r = process_result["th_retr"]
    tau_r_vec = process_result["tau_retr"].reshape(-1, 3)

    e_hat = process_result["e_hat"]
    tau_p = tau_p_vec @ e_hat
    tau_r = tau_r_vec @ e_hat

    # Decimate if requested
    if decimate > 1:
        th_p = th_p[::decimate]
        tau_p = tau_p[::decimate]
        th_r = th_r[::decimate]
        tau_r = tau_r[::decimate]

    # Generate fit curves over full range
    th_full = np.linspace(0, max(np.concatenate([th_p, th_r])) * 1.05, 200)
    
    # Fit curves using estimated parameters
    # Note: tau_model returns raveled (N*3,), so we need to extract the projected component
    # For simplicity, we'll compute tau values directly
    rc0 = process_result["rc0"]
    
    def tau_scalar(th, m, zc):
        """Compute scalar torque projection onto e_hat."""
        tau_vec = tau_model(th, m, zc, rc0_known=rc0, e_hat=e_hat)
        tau_vec_reshaped = tau_vec.reshape(-1, 3)
        return tau_vec_reshaped @ e_hat

    fit_p_curve = tau_scalar(th_full, fit_push["m"], fit_push["zc"])
    fit_r_curve = tau_scalar(th_full, fit_retr["m"], fit_retr["zc"])

    # Create figure
    fig, ax = plt.subplots(figsize=(9, 6))

    # Plot ground truth line
    ax.axvline(theta_star_gt_deg, color='green', linestyle='--', linewidth=2.5, label=f'GT θ* ({theta_star_gt_deg:.1f}°)', zorder=2)

    # Plot estimated lines
    ax.axvline(np.rad2deg(ths_est), color='orange', linestyle='-', linewidth=2.0, label=f'Est θ*={np.rad2deg(ths_est):.1f}°, m={m_est:.2f}kg, z={zc_est:.3f}m', zorder=2)

    # Horizontal zero line cyan
    ax.axhline(0, color='cyan', linewidth=1.5, label='_', zorder=2)
               
    # Plot push phase data and fit
    ax.scatter(np.rad2deg(th_p), tau_p, alpha=0.5, s=30, color='tab:blue', label='Push Phase Data', zorder=3)
    ax.plot(np.rad2deg(th_full), fit_p_curve, color='black', linewidth=2.5, label='Push Fit', zorder=4)

    # Plot retract phase data and fit
    ax.scatter(np.rad2deg(th_r), tau_r, alpha=0.5, s=30, color='tab:orange', label='Retract Phase Data', zorder=3)
    ax.plot(np.rad2deg(th_full), fit_r_curve, color='gray', linewidth=2.5, label='Retract Fit', zorder=4)

    # Formatting
    ax.set_ylim(bottom=-0.01) # Set lower lim just below zero
    ax.set_xlabel('Object Angle (deg)', fontsize=16)
    ax.set_ylabel('Torque (N-m)', fontsize=16)
    ax.grid(True, alpha=0.35)
    ax.legend(loc='upper right', fontsize=12, framealpha=0.95)
    ax.tick_params(axis='both', labelsize=14)

    fig.tight_layout()

    # Determine output path (always save next to this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_name = os.path.splitext(os.path.basename(csv_path))[0]
    out_png = os.path.join(script_dir, f"{base_name}_fit.png")
    fig.savefig(out_png, dpi=300, bbox_inches='tight')
    print(f"[OK] Saved plot to: {out_png}")

    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Refit experimental data and regenerate fit plots for all n_safety trials")
    ap.add_argument("--csv_base", type=str, required=True, help="Base path for CSV files (without n_safety suffix). Example: /path/to/20260131_box")
    ap.add_argument("--o_obj", type=float, nargs=3, default=[0.66, 0, 0], help="Object frame origin (default: [0.66, 0, 0])")
    ap.add_argument("--rc0", type=float, nargs=3, default=[-0.05, 0, 0], help="Contact point offset (default: [-0.05, 0, 0])")
    ap.add_argument("--e_hat", type=float, nargs=3, default=[0.0, 1.0, 0.0], help="Rotation axis (default: [0, 1, 0])")
    args = ap.parse_args()

    if not os.path.exists(SUMMARY_CSV_PATH):
        raise SystemExit(f"Summary CSV not found: {SUMMARY_CSV_PATH}")

    summary_df = pd.read_csv(SUMMARY_CSV_PATH)
    estimator = RefitEstimator(o_obj=args.o_obj, rc0_known=args.rc0, e_hat=args.e_hat)

    object_name = os.path.basename(args.csv_base).split("_")[-1].lower()
    print(f"[INFO] Processing trials for object: {args.csv_base}")
    print(f"[INFO] Using θ*_GT={THETA_STAR_GT_DEG:.1f}° and decimate={DECIMATE}")
    print(f"[INFO] Looking for n_safety trials: {N_SAFETY_VALUES}")
    print()

    success_count = 0
    fail_count = 0
    results = []

    for n_safety in N_SAFETY_VALUES:
        csv_path = f"{args.csv_base}_{n_safety:.3f}.csv"
        
        if not os.path.exists(csv_path):
            print(f"[SKIP] n_safety={n_safety:.3f}: File not found ({os.path.basename(csv_path)})")
            fail_count += 1
            continue

        basename = os.path.basename(csv_path)
        try:
            print(f"[PROCESSING] n_safety={n_safety:.3f} ({basename})...", end=" ")
            result = estimator.process(csv_path, summary_df, object_name, n_safety)
            make_fit_plot(result, THETA_STAR_GT_DEG, csv_path, decimate=DECIMATE)
            print(f"✓ m={result['m_est']:.3f}kg, z={result['zc_est']:.3f}m, θ*={np.rad2deg(result['ths_est']):.1f}°")
            success_count += 1
            results.append((n_safety, result))
        except Exception as e:
            print(f"✗ FAILED: {e}")
            fail_count += 1

    print()
    print(f"[SUMMARY] Processed {success_count}/{len(N_SAFETY_VALUES)} trials successfully")
    if fail_count > 0:
        print(f"[WARNING] {fail_count} trials failed or were not found")
    
    if results:
        print()
        print("[RESULTS TABLE]")
        print(f"{'n_safety':<10} {'m (kg)':<10} {'zc (m)':<10} {'θ* (deg)':<10}")
        print("-" * 40)
        for ns, res in results:
            print(f"{ns:<10.3f} {res['m_est']:<10.3f} {res['zc_est']:<10.3f} {np.rad2deg(res['ths_est']):<10.1f}")


if __name__ == "__main__":
    main()
