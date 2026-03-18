#!/usr/bin/env python3
"""
plot_live_data_first_push_only.py

Recreates a live data stream plot showing only the FIRST push for box at n_safety=0.1.
Removes the second push entirely from visualization.

Plots:
  - Fx, Fy, Fz (left y-axis, in N)
  - Angle (right y-axis, in degrees)
  
Usage:
  python3 plot_live_data_first_push_only.py
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.signal import butter, filtfilt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EXPERIMENTS_DIR = os.path.join(SCRIPT_DIR, "..", "experiments")

# Helper functions (from helper_fns.py)
def quat_normalize(q):
    """Normalize quaternion to unit length."""
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        return q
    return q / norm

def quat_conj(q):
    """Conjugate of quaternion (xyzw convention)."""
    return np.array([-q[0], -q[1], -q[2], q[3]])

def quat_mul(q1, q2):
    """Quaternion multiplication q1 * q2 (xyzw convention)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def quat_to_rotvec(q, normalize=False):
    """Convert quaternion (xyzw) to rotation vector."""
    r = Rotation.from_quat(q)
    return r.as_rotvec()

def find_box_01_csv():
    """Find the first box 0.1 CSV file in experiments directory."""
    pattern = os.path.join(EXPERIMENTS_DIR, "*", "*_box_0.1.csv")
    csvs = sorted(glob.glob(pattern))
    if not csvs:
        raise FileNotFoundError(f"No box 0.1 CSV files found in {EXPERIMENTS_DIR}")
    return csvs[0]

def load_and_filter_data(csv_path):
    """
    Load CSV and identify the first push region.
    Apply butterworth filtering (4th order, 5Hz) like in refit_and_plot.
    Compute angle from relative quaternions (same as estimation).
    Remove the second push entirely.
    """
    df = pd.read_csv(csv_path)
    
    # Required columns
    required_forces = ["fx", "fy", "fz"]
    required_quat = ["tag_qx", "tag_qy", "tag_qz", "tag_qw"]
    
    if not all(c in df.columns for c in required_forces):
        raise ValueError(f"Missing force columns. Got {df.columns.tolist()}")
    
    if not all(c in df.columns for c in required_quat):
        raise ValueError(f"Missing quaternion columns. Got {df.columns.tolist()}")
    
    # Setup butterworth filter (4th order, 5Hz cutoff)
    fs_hz = 500.0
    b, a = butter(4, 5.0, fs=fs_hz, btype='low')
    
    # Extract raw data
    f_raw = df[['fx', 'fy', 'fz']].values
    q_raw = df[['tag_qx', 'tag_qy', 'tag_qz', 'tag_qw']].values
    c_raw = df['fw_contact'].values.astype(bool)
    
    # Calculate rotation angles from relative quaternions
    q_ref = quat_normalize(q_raw[0])  # Reference is first quaternion
    rot_vecs = []
    q_ref_conj = quat_conj(q_ref)
    
    for q in q_raw:
        q_norm = quat_normalize(q)
        q_rel = quat_mul(q_ref_conj, q_norm)
        rv = quat_to_rotvec(q_rel, normalize=False)
        rot_vecs.append(rv)
    
    rot_vecs = np.array(rot_vecs)
    theta_raw = np.linalg.norm(rot_vecs, axis=1)
    
    # Apply butterworth filter to angle
    theta_filt = filtfilt(b, a, theta_raw)
    theta_filt_deg = np.degrees(theta_filt)
    
    # Find first push and retract region
    contact_indices = np.where(c_raw)[0]
    if len(contact_indices) == 0:
        raise ValueError("No contact data found in CSV")
    
    # Find first contact and retract
    contact_starts = np.where(np.diff(c_raw.astype(int)) == 1)[0] + 1
    contact_ends = np.where(np.diff(c_raw.astype(int)) == -1)[0]
    
    if len(contact_starts) >= 1 and len(contact_ends) >= 1:
        # Keep data from first contact start to end of first retract
        first_retract_end_idx = contact_ends[0]
        # Add margin for retract motion to complete
        cutoff_idx = min(first_retract_end_idx + 300, len(df))
    else:
        # Fallback: use first 60% of data
        cutoff_idx = int(0.6 * len(df))
    
    # Filter dataframe
    df_filtered = df.iloc[:cutoff_idx].copy()
    df_filtered["time_s"] = np.arange(len(df_filtered)) / fs_hz
    df_filtered["angle_filt"] = theta_filt_deg[:cutoff_idx]
    
    return df_filtered

def plot_live_data(df, out_png):
    """Create live data stream plot with raw forces and filtered angle."""
    fig, ax1 = plt.subplots(figsize=(8, 4.5))
    
    time = df["time_s"].to_numpy()
    fx = df["fx"].to_numpy()  # Raw forces (keep noise)
    fy = df["fy"].to_numpy()
    fz = df["fz"].to_numpy()
    angle = df["angle_filt"].to_numpy()  # Filtered angle
    
    # Left axis: Forces (raw, with noise)
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Force (N)", fontsize=16)
    ax1.plot(time, fx, color="tab:red", linewidth=2.0, label="Fx", alpha=0.8)
    ax1.plot(time, fy, color="tab:green", linewidth=2.0, label="Fy", alpha=0.8)
    ax1.plot(time, fz, color="tab:blue", linewidth=2.0, label="Fz", alpha=0.8)
    ax1.tick_params(axis="y", labelsize=14)
    ax1.tick_params(axis="x", labelsize=14)
    ax1.grid(True, alpha=0.35)
    
    # Right axis: Angle (filtered)
    ax2 = ax1.twinx()
    ax2.set_ylabel("Angle (deg)", fontsize=16)
    ax2.plot(time, angle, color="black", linewidth=3.0, label="Angle")
    ax2.tick_params(axis="y", labelsize=14)
    
    # Legend combining both axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", fontsize=14, framealpha=0.9, frameon=True)
    
    # fig.suptitle("Live Data Stream (n_safety=0.1) — First Push Only", fontsize=20, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    print(f"[OK] Wrote figure: {out_png}")
    plt.close(fig)

def main():
    """Main entry point."""
    csv_path = find_box_01_csv()
    print(f"[INFO] Found CSV: {csv_path}")
    
    df = load_and_filter_data(csv_path)
    print(f"[INFO] Loaded {len(df)} samples (filtered to first push only)")
    print(f"[INFO] Time range: 0.0 - {df['time_s'].max():.1f} s")
    
    out_png = os.path.join(SCRIPT_DIR, "live_data_stream_box_01_first_push.png")
    plot_live_data(df, out_png)

if __name__ == "__main__":
    main()
