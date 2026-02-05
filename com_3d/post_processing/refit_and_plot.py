#!/usr/bin/env python3
"""
refit_and_plot.py

Post-processing script to refit experimental data and regenerate fit plots.

Takes a raw experiment CSV (from sync_logger) and:
  1. Filters and masks push/retract phases
  2. Fits CoM parameters separately for push and retract
  3. Generates a publication-quality plot with scatter data and fitted curves

Features:
  - Ground truth theta_star loaded from OBJECT_GROUND_TRUTH definitions
  - Data point decimation for plot readability
  - Supports both push and retract phase visualization
  - Batch processing by object: processes all n_safety trials (0.100, 0.500, 0.650)
    - Uses experiment estimates from summary_by_object_nsafety_phase.csv (no refitting)

===== CONFIGURATION =====
DECIMATE: Data point decimation factor (1 = no decimation)
N_SAFETY_VALUES: List of n_safety values to process (default: [0.100, 0.500, 0.650])
OBJECT_GROUND_TRUTH: Dictionary defining m_kg, zc_m, rc0_m, and theta_star_deg for each object
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
from scipy.optimize import curve_fit
from scipy.stats import linregress

# ===== CONFIGURATION =====
DECIMATE = 40  # Data point decimation factor - adjust for readability
N_SAFETY_VALUES = [0.100, 0.500, 0.650]  # n_safety trials to process
SUMMARY_CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "summary_by_object_nsafety_phase.csv")

# ===== OBJECT GROUND TRUTH VALUES =====
# Define ground truth parameters for each object here (mass in kg, zc in m, theta_star in degrees)
OBJECT_GROUND_TRUTH = {
    "box": {
        "m_kg": 0.658, # 0.664
        "zc_m": 0.14624,
        "rc0_m": [-0.04515, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(0.04515, 0.14624)), # atan((x^2 + y^2)^0.5 / z)
    },
    "heart": {
        "m_kg": 0.236,
        "zc_m": 0.098,
        "rc0_m": [-0.04354, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(0.04354, 0.098)),
    },
    "monitor": {
        "m_kg": 5.040, # 5.008
        "zc_m": 0.2316, # 0.2516 # 0.2316 close...
        "rc0_m": [-0.06107, 0, 0],# [-0.06207, 0, 0], 0.06107 close
        "theta_star_deg": np.rad2deg(np.arctan2(0.06107, 0.2316)),
    },
    "flashlight": {
        "m_kg": 0.387,
        "zc_m": 0.09656,
        "rc0_m": [-0.0230, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(0.0230, 0.09656)),
    },
}
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

    def _fit_phase_params(self, th, tau_vec):
        """
        Fit (m, zc) parameters for one phase from theta and torque data.
        Returns dict with fitted parameters, inlier indices, and goodness of fit (MSE).
        """
        th_orig = np.asarray(th, float).reshape(-1)
        tau_vec_orig = np.asarray(tau_vec, float).reshape(-1, 3)
        
        # Exclude first ~2 degrees where transients and outliers cluster
        theta_min = np.deg2rad(2.0)
        valid_theta_mask = th_orig >= theta_min
        valid_theta_indices = np.where(valid_theta_mask)[0]
        
        th = th_orig[valid_theta_mask]
        tau_vec = tau_vec_orig[valid_theta_mask]
        tau_proj = tau_vec @ self.e_hat  # (N,)

        # Linear seed
        slope, intercept, *_ = linregress(th, tau_proj)
        ths0 = float(np.clip(-intercept / (slope + 1e-12), 1e-2, np.pi/2))
        z0 = float(np.clip(abs(self.d / np.tan(ths0)), 0.02, 1.0))
        m0 = float(np.clip(abs(slope) / (9.81 * max(z0, 1e-3)), 0.05, 20.0))

        # Outlier rejection (tighter threshold for better rejection)
        outlier_k = 2.0
        resid = tau_proj - (slope * th + intercept)
        med = np.median(resid)
        mad = np.median(np.abs(resid - med))
        sigma = 1.4826 * mad + 1e-12
        inliers = np.abs(resid - med) <= (outlier_k * sigma)
        inlier_indices_relative = np.where(inliers)[0]
        
        # Map back to original indices (accounting for theta_min filtering)
        inlier_indices = valid_theta_indices[inlier_indices_relative]

        th_in = th[inliers]
        tau_in = tau_vec[inliers]

        # Nonlinear fit
        def _fit_target(th_in, m, zc):
            return tau_model(th_in, m, zc, rc0_known=self.rc0, e_hat=self.e_hat)

        popt, _ = curve_fit(
            _fit_target,
            th_in,
            tau_in.reshape(-1),
            p0=[m0, z0],
            bounds=([0.01, 0.01], [20.0, 1.0]),
            maxfev=4000
        )
        m_est, zc_est = map(float, popt)
        ths_est = float(np.arctan2(self.d, zc_est))
        
        # Calculate goodness of fit (MSE on inliers)
        tau_fit = tau_model(th_in, m_est, zc_est, rc0_known=self.rc0, e_hat=self.e_hat).reshape(-1, 3)
        tau_fit_proj = tau_fit @ self.e_hat
        mse = float(np.mean((tau_in @ self.e_hat - tau_fit_proj) ** 2))

        return {
            "m": m_est,
            "zc": zc_est,
            "ths": ths_est,
            "mse": mse,  # Goodness of fit metric
            "inlier_indices": inlier_indices,  # Return inlier indices for plotting
        }


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
        
        push_mask[contact_start:retract_start] = c_raw[contact_start:retract_start] & (~r_raw[contact_start:retract_start])
        retr_mask[retract_start:contact_end+1] = c_raw[retract_start:contact_end+1] & r_raw[retract_start:contact_end+1]

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
                "weight": float(row["weight"]),
            }

        fit_push = _get_phase_est("push")
        fit_retr = _get_phase_est("retract")

        # Combined estimate for annotation (weighted average using CSV weights)
        w_push = fit_push["weight"]
        w_retr = fit_retr["weight"]
        w_total = w_push + w_retr
        
        m_est = (w_push * fit_push["m"] + w_retr * fit_retr["m"]) / w_total
        zc_est = (w_push * fit_push["zc"] + w_retr * fit_retr["zc"]) / w_total
        ths_est_deg = (w_push * fit_push["ths_deg"] + w_retr * fit_retr["ths_deg"]) / w_total
        ths_est = float(np.deg2rad(ths_est_deg))

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
            "estimator": self,  # Include estimator so make_fit_plot can use _fit_phase_params
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
    estimator = process_result["estimator"]
    
    # Extract raw data (not decimated yet)
    th_p = process_result["th_push"]
    tau_p_vec = process_result["tau_push"].reshape(-1, 3)
    th_r = process_result["th_retr"]
    tau_r_vec = process_result["tau_retr"].reshape(-1, 3)

    e_hat = process_result["e_hat"]
    rc0 = process_result["rc0"]
    
    tau_p = tau_p_vec @ e_hat
    tau_r = tau_r_vec @ e_hat

    # FIT the data to get parameters (not use pre-computed summaries)
    fit_push = estimator._fit_phase_params(th_p, tau_p_vec)
    fit_retr = estimator._fit_phase_params(th_r, tau_r_vec)

    # Extract inlier indices for each phase
    inlier_idx_push = fit_push["inlier_indices"]
    inlier_idx_retr = fit_retr["inlier_indices"]
    
    # Extract only inliers for plotting
    th_p_inliers = th_p[inlier_idx_push]
    tau_p_inliers = tau_p[inlier_idx_push]
    th_r_inliers = th_r[inlier_idx_retr]
    tau_r_inliers = tau_r[inlier_idx_retr]

    # Weighted combined estimate
    w_push = 1.0  # Equal weighting for fitting
    w_retr = 1.0
    m_est = (w_push * fit_push["m"] + w_retr * fit_retr["m"]) / (w_push + w_retr)
    zc_est = (w_push * fit_push["zc"] + w_retr * fit_retr["zc"]) / (w_push + w_retr)
    ths_est = float(np.arctan2(estimator.d, zc_est))

    # Decimate for display (only inliers)
    if decimate > 1:
        th_p_plot = th_p_inliers[::decimate]
        tau_p_plot = tau_p_inliers[::decimate]
        th_r_plot = th_r_inliers[::decimate]
        tau_r_plot = tau_r_inliers[::decimate]
    else:
        th_p_plot = th_p_inliers
        tau_p_plot = tau_p_inliers
        th_r_plot = th_r_inliers
        tau_r_plot = tau_r_inliers

    # Generate smooth fit curves over full range
    th_full = np.linspace(0, max(np.concatenate([th_p_inliers, th_r_inliers])) * 1.05, 200)
    
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
    ax.scatter(np.rad2deg(th_p_plot), tau_p_plot, alpha=0.5, s=30, color='tab:blue', label='Push Phase Data', zorder=3)
    ax.plot(np.rad2deg(th_full), fit_p_curve, color='black', linewidth=2.5, label='Push Fit', zorder=4)

    # Plot retract phase data and fit
    ax.scatter(np.rad2deg(th_r_plot), tau_r_plot, alpha=0.5, s=30, color='tab:orange', label='Retract Phase Data', zorder=3)
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
    
    # Calculate weights based on inverse MSE (better fit = higher weight)
    eps = 1e-12
    w_push = 1.0 / (fit_push["mse"] + eps)
    w_retr = 1.0 / (fit_retr["mse"] + eps)
    w_total = w_push + w_retr
    
    # Compute weighted averages
    m_weighted = (w_push * fit_push["m"] + w_retr * fit_retr["m"]) / w_total
    zc_weighted = (w_push * fit_push["zc"] + w_retr * fit_retr["zc"]) / w_total
    ths_weighted = float(np.arctan2(estimator.d, zc_weighted))
    
    # Return fitted estimates with weighted average for CSV output
    return {
        "push": {
            "m": fit_push["m"],
            "zc": fit_push["zc"],
            "ths": fit_push["ths"],
            "mse": fit_push["mse"],
        },
        "retract": {
            "m": fit_retr["m"],
            "zc": fit_retr["zc"],
            "ths": fit_retr["ths"],
            "mse": fit_retr["mse"],
        },
        "weighted_avg": {
            "m": m_weighted,
            "zc": zc_weighted,
            "ths": ths_weighted,
        },
        "weights": {
            "push": w_push,
            "retract": w_retr,
            "total": w_total,
        },
    }


def main():
    ap = argparse.ArgumentParser(description="Refit experimental data and regenerate fit plots for all n_safety trials")
    ap.add_argument("--csv_base", type=str, required=True, help="Base path for CSV files (without n_safety suffix). Example: /path/to/20260131_box")
    ap.add_argument("--e_hat", type=float, nargs=3, default=[0.0, 1.0, 0.0], help="Rotation axis (default: [0, 1, 0])")
    args = ap.parse_args()

    if not os.path.exists(SUMMARY_CSV_PATH):
        raise SystemExit(f"Summary CSV not found: {SUMMARY_CSV_PATH}")

    summary_df = pd.read_csv(SUMMARY_CSV_PATH)
    
    # Extract object name and get rc0 from ground truth definitions
    object_name = os.path.basename(args.csv_base).split("_")[-1].lower()
    if object_name not in OBJECT_GROUND_TRUTH:
        raise SystemExit(f"Object '{object_name}' not found in OBJECT_GROUND_TRUTH. Available: {list(OBJECT_GROUND_TRUTH.keys())}")
    
    rc0 = OBJECT_GROUND_TRUTH[object_name]["rc0_m"]
    o_obj = [0.66, 0.0, 0.0]  # Fixed object frame origin (table location)
    
    estimator = RefitEstimator(o_obj=o_obj, rc0_known=rc0, e_hat=args.e_hat)
    
    print(f"[INFO] Processing trials for object: {args.csv_base}")
    print(f"[INFO] Using decimate={DECIMATE}")
    print(f"[INFO] Looking for n_safety trials: {N_SAFETY_VALUES}")
    print()

    success_count = 0
    fail_count = 0
    results = []
    estimates_data = []  # Collect estimates for CSV export

    # Get ground truth for this object
    gt = OBJECT_GROUND_TRUTH.get(object_name, None)
    if gt is None:
        print(f"[WARNING] No ground truth defined for object '{object_name}'. Skipping error calculations.")
        gt = {"m_kg": None, "zc_m": None, "theta_star_deg": None}

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
            phase_estimates = make_fit_plot(result, gt['theta_star_deg'], csv_path, decimate=DECIMATE)
            print(
                f"✓ m={phase_estimates['weighted_avg']['m']:.3f}kg, "
                f"z={phase_estimates['weighted_avg']['zc']:.3f}m, "
                f"θ*={np.rad2deg(phase_estimates['weighted_avg']['ths']):.1f}°"
            )
            success_count += 1
            results.append((n_safety, phase_estimates))  # Store phase_estimates for final table
            
            # Helper function to calculate percent error
            def calc_pct_error(est, gt_val):
                if gt_val is None:
                    return None
                return 100.0 * abs(est - gt_val) / abs(gt_val) if gt_val != 0 else 0.0
            
            # Normalize weights so all three rows (push, retract, weighted_avg) sum to 1.0
            w_push = phase_estimates["weights"]["push"]
            w_retr = phase_estimates["weights"]["retract"]
            w_total_all = w_push + w_retr + 1.0  # Include weighted_avg weight of 1.0
            w_push_norm = w_push / w_total_all
            w_retr_norm = w_retr / w_total_all
            w_avg_norm = 1.0 / w_total_all
            
            # Collect push phase estimate
            estimates_data.append({
                "object": object_name,
                "n_safety": round(n_safety, 3),
                "phase": "push",
                "m_est_kg": round(phase_estimates["push"]["m"], 3),
                "zc_est_m": round(phase_estimates["push"]["zc"], 3),
                "theta_star_est_deg": round(np.rad2deg(phase_estimates["push"]["ths"]), 3),
                "weight": round(w_push_norm, 3),
                "weight_push": round(phase_estimates["weights"]["push"], 3),
                "weight_retract": round(phase_estimates["weights"]["retract"], 3),
                "m_gt_kg": round(gt["m_kg"], 3) if gt["m_kg"] is not None else None,
                "zc_gt_m": round(gt["zc_m"], 3) if gt["zc_m"] is not None else None,
                "theta_star_gt_deg": round(gt["theta_star_deg"], 3) if gt["theta_star_deg"] is not None else None,
                "m_error_pct": round(calc_pct_error(phase_estimates["push"]["m"], gt["m_kg"]), 3) if gt["m_kg"] is not None else None,
                "zc_error_pct": round(calc_pct_error(phase_estimates["push"]["zc"], gt["zc_m"]), 3) if gt["zc_m"] is not None else None,
                "theta_star_error_pct": round(calc_pct_error(np.rad2deg(phase_estimates["push"]["ths"]), gt["theta_star_deg"]), 3) if gt["theta_star_deg"] is not None else None,
            })
            
            # Collect retract phase estimate
            estimates_data.append({
                "object": object_name,
                "n_safety": round(n_safety, 3),
                "phase": "retract",
                "m_est_kg": round(phase_estimates["retract"]["m"], 3),
                "zc_est_m": round(phase_estimates["retract"]["zc"], 3),
                "theta_star_est_deg": round(np.rad2deg(phase_estimates["retract"]["ths"]), 3),
                "weight": round(w_retr_norm, 3),
                "weight_push": round(phase_estimates["weights"]["push"], 3),
                "weight_retract": round(phase_estimates["weights"]["retract"], 3),
                "m_gt_kg": round(gt["m_kg"], 3) if gt["m_kg"] is not None else None,
                "zc_gt_m": round(gt["zc_m"], 3) if gt["zc_m"] is not None else None,
                "theta_star_gt_deg": round(gt["theta_star_deg"], 3) if gt["theta_star_deg"] is not None else None,
                "m_error_pct": round(calc_pct_error(phase_estimates["retract"]["m"], gt["m_kg"]), 3) if gt["m_kg"] is not None else None,
                "zc_error_pct": round(calc_pct_error(phase_estimates["retract"]["zc"], gt["zc_m"]), 3) if gt["zc_m"] is not None else None,
                "theta_star_error_pct": round(calc_pct_error(np.rad2deg(phase_estimates["retract"]["ths"]), gt["theta_star_deg"]), 3) if gt["theta_star_deg"] is not None else None,
            })
            
            # Collect weighted average estimate
            estimates_data.append({
                "object": object_name,
                "n_safety": round(n_safety, 3),
                "phase": "weighted_avg",
                "m_est_kg": round(phase_estimates["weighted_avg"]["m"], 3),
                "zc_est_m": round(phase_estimates["weighted_avg"]["zc"], 3),
                "theta_star_est_deg": round(np.rad2deg(phase_estimates["weighted_avg"]["ths"]), 3),
                "weight": round(w_avg_norm, 3),
                "weight_push": round(phase_estimates["weights"]["push"], 3),
                "weight_retract": round(phase_estimates["weights"]["retract"], 3),
                "m_gt_kg": round(gt["m_kg"], 3) if gt["m_kg"] is not None else None,
                "zc_gt_m": round(gt["zc_m"], 3) if gt["zc_m"] is not None else None,
                "theta_star_gt_deg": round(gt["theta_star_deg"], 3) if gt["theta_star_deg"] is not None else None,
                "m_error_pct": round(calc_pct_error(phase_estimates["weighted_avg"]["m"], gt["m_kg"]), 3) if gt["m_kg"] is not None else None,
                "zc_error_pct": round(calc_pct_error(phase_estimates["weighted_avg"]["zc"], gt["zc_m"]), 3) if gt["zc_m"] is not None else None,
                "theta_star_error_pct": round(calc_pct_error(np.rad2deg(phase_estimates["weighted_avg"]["ths"]), gt["theta_star_deg"]), 3) if gt["theta_star_deg"] is not None else None,
            })
        except Exception as e:
            print(f"✗ FAILED: {e}")
            fail_count += 1

    print()
    print(f"[SUMMARY] Processed {success_count}/{len(N_SAFETY_VALUES)} trials successfully")
    if fail_count > 0:
        print(f"[WARNING] {fail_count} trials failed or were not found")
    
    # Save estimates to CSV
    if estimates_data:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_csv = os.path.join(script_dir, f"{object_name}_estimates.csv")
        
        estimates_df = pd.DataFrame(estimates_data)
        estimates_df.to_csv(output_csv, index=False)
        print(f"[OK] Saved estimates to: {output_csv}")
    
    if results:
        print()
        print("[RESULTS TABLE]")
        print(f"{'n_safety':<10} {'m (kg)':<10} {'zc (m)':<10} {'θ* (deg)':<10}")
        print("-" * 40)
        for ns, phase_est in results:
            print(f"{ns:<10.3f} {phase_est['weighted_avg']['m']:<10.3f} {phase_est['weighted_avg']['zc']:<10.3f} {np.rad2deg(phase_est['weighted_avg']['ths']):<10.1f}")
        
        # Print gt in terminal too
        print(f"[GT]           {gt['m_kg']:<10.3f} {gt['zc_m']:<10.3f} {gt['theta_star_deg']:<10.1f}")


if __name__ == "__main__":
    main()
