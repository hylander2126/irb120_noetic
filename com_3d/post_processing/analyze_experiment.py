#!/usr/bin/env python3
"""
fit_and_summary.py

Offline (no live plot):

1) Fit plot recreation (torque vs object angle)
   - scatter push/retract
   - black push/retract fit lines
   - vertical theta* lines (from your saved estimates dict; NOT redoing NLS)
   - optional GT theta* vertical line

2) Summary comparing all objects accuracy (m, z_c, theta*) across n_safety
   - aggregates across pushes (mean/std) per (object, n_safety)
   - compares to ground truth placeholders (fill them in)
   - writes CSV: summary_by_object_nsafety.csv

You can run in:
A) MANIFEST mode (batch plots + summary)
B) SINGLE-RUN mode (one fit plot)

See docstring inside for examples.
"""

import argparse
import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

try:
    from scipy.spatial.transform import Rotation as R
except Exception as e:
    raise ImportError("scipy is required (scipy.spatial.transform.Rotation).") from e

# =========================
# Ground truth placeholders
# =========================
GT_PLACEHOLDER = {
    "box":        {"m_kg": 0.664, "z_m": 0.14624, "theta_star_deg": np.rad2deg(np.arctan(0.04515/0.14624))},
    "heart":      {"m_kg": 0.236, "z_m": 0.09800, "theta_star_deg": np.rad2deg(np.arctan(0.04354/0.09800))},
    "flashlight": {"m_kg": 0.387, "z_m": 0.09656, "theta_star_deg": np.rad2deg(np.arctan(0.02300/0.09656))},
    "monitor":    {"m_kg": 5.008, "z_m": 0.25160, "theta_star_deg": np.rad2deg(np.arctan(0.06207/0.25160))},
}

# --------------- CSV parsing ---------------

def read_sync_csv(csv_path: str):
    with open(csv_path, 'r') as f:
        header = f.readline().strip().split(',')
    raw = np.genfromtxt(csv_path, delimiter=',', skip_header=1, dtype=float)
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    if len(header) != raw.shape[1]:
        raise ValueError(f"Header has {len(header)} cols but data has {raw.shape[1]} cols: {csv_path}")
    return {name: raw[:, i] for i, name in enumerate(header)}

# --------------- Quaternion helpers ---------------

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return q if n < 1e-12 else q / n

def quat_conj(q):
    q = np.asarray(q, dtype=float)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)

def quat_to_angle_deg(q_ref, q_now):
    q_ref = quat_normalize(q_ref)
    q_now = quat_normalize(q_now)
    q_rel = quat_mul(quat_conj(q_ref), q_now)
    r = R.from_quat(q_rel)
    return float(np.rad2deg(np.linalg.norm(r.as_rotvec())))

# --------------- Segmentation + angle ---------------

def seg_ids_from_time(t, gap_thresh=0.05):
    t = np.asarray(t, float)
    if t.size == 0:
        return np.array([], dtype=int)
    seg_ids = np.zeros_like(t, dtype=int)
    seg = 0
    for i in range(1, len(t)):
        if (t[i] - t[i-1]) > gap_thresh:
            seg += 1
        seg_ids[i] = seg
    return seg_ids

def angle_from_tags(tag_visible, qx,qy,qz,qw, seg_ids):
    tag_visible = np.asarray(tag_visible, float)
    q = np.vstack([qx,qy,qz,qw]).T.astype(float)
    th = np.full(len(tag_visible), np.nan, dtype=float)
    for seg in np.unique(seg_ids):
        idxs = np.where(seg_ids == seg)[0]
        if idxs.size == 0:
            continue
        vis = idxs[tag_visible[idxs] > 0.5]
        if vis.size == 0:
            continue
        q_ref = q[vis[0]]
        for i in idxs:
            if tag_visible[i] > 0.5 and np.all(np.isfinite(q[i])):
                th[i] = quat_to_angle_deg(q_ref, q[i])
    return th

def interp_nans(y):
    y = np.asarray(y, float)
    x = np.arange(len(y), dtype=float)
    good = np.isfinite(y)
    if good.sum() < 2:
        return y
    y2 = y.copy()
    y2[~good] = np.interp(x[~good], x[good], y[good])
    return y2

# --------------- Torque computation ---------------

def projected_tau(fx,fy,fz, ee_x,ee_y,ee_z, o_obj, e_hat):
    F = np.vstack([fx,fy,fz]).T.astype(float)
    rf = np.vstack([ee_x,ee_y,ee_z]).T.astype(float) - np.asarray(o_obj, float).reshape(1,3)
    tau_vec = np.cross(rf, -F)
    e = np.asarray(e_hat, float)
    e = e / (np.linalg.norm(e) + 1e-12)
    return tau_vec @ e

# --------------- Windowing: push vs retract ---------------

def compute_windows(seg_ids, fw_contact, ft_trigger, max_pushes=2):
    fw_contact = np.asarray(fw_contact).astype(int)
    ft_trigger = np.asarray(ft_trigger).astype(int)
    out = []
    for seg in np.unique(seg_ids):
        idxs = np.where(seg_ids == seg)[0]
        if idxs.size < 10:
            continue
        cands_c = idxs[fw_contact[idxs] == 1]
        if cands_c.size == 0:
            continue
        contact_idx = int(cands_c[0])

        cands_t = idxs[(ft_trigger[idxs] == 1) & (idxs >= contact_idx)]
        trigger_idx = int(cands_t[0]) if cands_t.size else int(idxs[len(idxs)//2])

        cands_end = idxs[(fw_contact[idxs] == 1) & (idxs >= trigger_idx)]
        end_idx = int(cands_end[-1]) if cands_end.size else int(idxs[-1])

        push_mask = np.zeros_like(seg_ids, dtype=bool)
        retr_mask = np.zeros_like(seg_ids, dtype=bool)
        push_mask[contact_idx:trigger_idx+1] = True
        retr_mask[trigger_idx:end_idx+1] = True
        push_mask &= (seg_ids == seg)
        retr_mask &= (seg_ids == seg)

        out.append((push_mask, retr_mask))
        if len(out) >= max_pushes:
            break
    return out

# --------------- Line fits (visual only) ---------------

def robust_line_fit(x, y, max_iters=5, zscore=2.5, min_keep=50):
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 2:
        return np.nan, np.nan
    keep = m.copy()
    for _ in range(max_iters):
        if keep.sum() < 2:
            break
        A = np.vstack([x[keep], np.ones(keep.sum())]).T
        a, b = np.linalg.lstsq(A, y[keep], rcond=None)[0]
        r = y - (a*x + b)
        rk = r[keep]
        med = np.median(rk)
        mad = np.median(np.abs(rk - med)) + 1e-12
        z = np.abs((r - med) / (1.4826*mad))
        new_keep = keep & (z < zscore)
        if new_keep.sum() == keep.sum():
            keep = new_keep
            break
        keep = new_keep
        if keep.sum() < min_keep:
            break
    A = np.vstack([x[keep], np.ones(keep.sum())]).T
    a, b = np.linalg.lstsq(A, y[keep], rcond=None)[0]
    return float(a), float(b)

def ls_line_fit(x, y):
    A = np.vstack([x, np.ones_like(x)]).T
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return float(a), float(b)

# --------------- Params dict loading ---------------

def _maybe_float(x):
    try:
        return float(x)
    except Exception:
        return None

def _get_by_key_variants(d, key):
    if d is None:
        return None
    if key in d:
        return d[key]
    sk = str(key)
    if sk in d:
        return d[sk]
    fk = _maybe_float(key)
    if fk is not None:
        for fmt in ["{:.3f}", "{:.2f}", "{:.1f}", "{:g}"]:
            kk = fmt.format(fk)
            if kk in d:
                return d[kk]
    return None

def load_estimates(params_json, object_key, n_safety):
    with open(params_json, "r") as f:
        P = json.load(f)
    node = _get_by_key_variants(P, object_key)
    if node is None:
        return []
    node2 = _get_by_key_variants(node, n_safety)
    if node2 is not None:
        node = node2

    if isinstance(node, dict) and "estimates" in node and isinstance(node["estimates"], list):
        return node["estimates"]
    if isinstance(node, list):
        return node
    if isinstance(node, dict) and any(k in node for k in ["theta_star_deg","m_kg","z_m","push_line","retr_line"]):
        return [node]
    return []

# --------------- Fit plot per run ---------------

def make_fit_plot(csv_path, object_key, n_safety, params_json,
                  out_dir, stem=None,
                  theta_min_deg=3.0,
                  theta_star_gt_deg=np.nan,
                  o_obj=(0.47065 + 0.081 + 0.114, 0.0, 0.0),
                  e_hat=(0.0, 1.0, 0.0),
                  gap_thresh=0.05,
                  robust=False, zscore=2.5, dpi=300):

    D = read_sync_csv(csv_path)
    seg_ids = seg_ids_from_time(D["ros_time_sec"], gap_thresh=gap_thresh)

    th = angle_from_tags(D["tag_visible"], D["tag_qx"], D["tag_qy"], D["tag_qz"], D["tag_qw"], seg_ids)
    th = interp_nans(th)

    tau = projected_tau(D["fx"], D["fy"], D["fz"], D["ee_x"], D["ee_y"], D["ee_z"], o_obj=o_obj, e_hat=e_hat)

    windows = compute_windows(seg_ids, D["fw_contact"], D["ft_trigger"], max_pushes=2)
    if not windows:
        raise RuntimeError(f"No push windows found for {csv_path}")

    ests = load_estimates(params_json, object_key, n_safety)
    if ests:
        ests = ests[:len(windows)]

    fig, ax = plt.subplots(1, 1, figsize=(10, 5.5))
    ax.set_xlabel("Object Angle (deg)", fontsize=16)
    ax.set_ylabel("Torque (N-m)", fontsize=16)
    ax.tick_params(axis='both', labelsize=14)
    ax.grid(True)
    ax.axhline(0.0, linewidth=1.0)

    if np.isfinite(theta_star_gt_deg):
        ax.axvline(theta_star_gt_deg, color="green", linestyle="--", linewidth=2,
                   label=f"GT θ* ({theta_star_gt_deg:.1f}°)")

    colors = ["tab:blue", "tab:orange"]
    markers = [("o","d"), ("^","s")]

    for i, (pm, rm) in enumerate(windows):
        pm = pm & np.isfinite(th) & (th >= theta_min_deg)
        rm = rm & np.isfinite(th) & (th >= theta_min_deg)

        th_p, tau_p = th[pm], tau[pm]
        th_r, tau_r = th[rm], tau[rm]
        if th_p.size < 10 or th_r.size < 10:
            continue

        est = ests[i] if i < len(ests) else {}
        theta_star_est = est.get("theta_star_deg", np.nan)
        m_kg = est.get("m_kg", None)
        z_m = est.get("z_m", None)

        push_line = est.get("push_line", None)
        retr_line = est.get("retr_line", None)

        if isinstance(push_line, dict) and "a" in push_line and "b" in push_line:
            a_p, b_p = float(push_line["a"]), float(push_line["b"])
        else:
            a_p, b_p = robust_line_fit(th_p, tau_p, zscore=zscore) if robust else ls_line_fit(th_p, tau_p)

        if isinstance(retr_line, dict) and "a" in retr_line and "b" in retr_line:
            a_r, b_r = float(retr_line["a"]), float(retr_line["b"])
        else:
            a_r, b_r = robust_line_fit(th_r, tau_r, zscore=zscore) if robust else ls_line_fit(th_r, tau_r)

        if not np.isfinite(theta_star_est):
            if np.isfinite(a_p) and abs(a_p) > 1e-12:
                theta_star_est = -b_p / a_p

        c = colors[i % len(colors)]
        mk_push, mk_retr = markers[i % len(markers)]

        ax.scatter(th_p, tau_p, s=30, alpha=0.70, marker=mk_push, color=c, label="_")
        ax.scatter(th_r, tau_r, s=30, alpha=0.70, marker=mk_retr, color=c, label="_")

        th_p_s = np.sort(th_p)
        th_r_s = np.sort(th_r)
        ax.plot(th_p_s, a_p*th_p_s + b_p, color="black", linewidth=2, alpha=0.95,
                label="Push Fit" if i == 0 else "_")
        ax.plot(th_r_s, a_r*th_r_s + b_r, color="black", linewidth=2, alpha=0.70,
                label="Retract Fit" if i == 0 else "_")

        label = f"Est {i+1} θ*={theta_star_est:.1f}°"
        if m_kg is not None:
            label += f", m={float(m_kg):.2f}kg"
        if z_m is not None:
            label += f", z={float(z_m):.3f}m"
        ax.axvline(theta_star_est, color=c, linewidth=2, alpha=0.9, label=label)

    ax.legend(loc="upper right", fontsize=10)
    fig.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    stem = stem or os.path.splitext(os.path.basename(csv_path))[0]
    out_png = os.path.join(out_dir, f"{stem}_fit_plot.png")
    fig.savefig(out_png, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return out_png

# --------------- Summary across objects/n_safety ---------------

def summarize(params_json, runs, gt=None, out_dir="."):
    gt = gt or GT_PLACEHOLDER
    rows = []
    for run in runs:
        obj = run["object"]
        ns = run["n_safety"]
        csv_path = run.get("csv","")
        ests = load_estimates(params_json, obj, ns)

        m = np.array([e.get("m_kg", np.nan) for e in ests], float) if ests else np.array([np.nan])
        z = np.array([e.get("z_m", np.nan) for e in ests], float) if ests else np.array([np.nan])
        ths = np.array([e.get("theta_star_deg", np.nan) for e in ests], float) if ests else np.array([np.nan])

        row = {
            "object": obj,
            "n_safety": float(ns),
            "csv": csv_path,
            "n_estimates": int(len(ests)) if ests else 0,

            "m_mean": float(np.nanmean(m)),
            "m_std":  float(np.nanstd(m)),
            "z_mean": float(np.nanmean(z)),
            "z_std":  float(np.nanstd(z)),
            "theta_star_mean": float(np.nanmean(ths)),
            "theta_star_std":  float(np.nanstd(ths)),

            "m_gt": gt.get(obj, {}).get("m_kg", np.nan),
            "z_gt": gt.get(obj, {}).get("z_m", np.nan),
            "theta_star_gt": gt.get(obj, {}).get("theta_star_deg", np.nan),
        }
        rows.append(row)

    df = pd.DataFrame(rows).sort_values(["object","n_safety"]).reset_index(drop=True)

    df["m_abs_err"] = np.abs(df["m_mean"] - df["m_gt"])
    df["z_abs_err"] = np.abs(df["z_mean"] - df["z_gt"])
    df["theta_star_abs_err"] = np.abs(df["theta_star_mean"] - df["theta_star_gt"])

    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, "summary_by_object_nsafety.csv")
    df.to_csv(out_csv, index=False)
    return df, out_csv

# --------------- CLI ---------------

def main():
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--manifest", help="Manifest JSON with key 'runs' for batch plots + summary")
    mode.add_argument("--csv", help="Single-run CSV (fit plot only)")

    ap.add_argument("--params_json", required=True)
    ap.add_argument("--out_dir", default=None)

    # single-run only
    ap.add_argument("--object", default=None)
    ap.add_argument("--n_safety", default=None)
    ap.add_argument("--theta_star_gt_deg", type=float, default=float("nan"))

    # plotting controls
    ap.add_argument("--theta_min_deg", type=float, default=3.0)
    ap.add_argument("--gap_thresh", type=float, default=0.05)
    ap.add_argument("--o_obj", nargs=3, type=float, default=[0.47065 + 0.081 + 0.114, 0.0, 0.0])
    ap.add_argument("--e_hat", nargs=3, type=float, default=[0.0, 1.0, 0.0])
    ap.add_argument("--robust", action="store_true")
    ap.add_argument("--zscore", type=float, default=2.5)
    ap.add_argument("--dpi", type=int, default=300)

    args = ap.parse_args()

    if args.manifest:
        with open(args.manifest, "r") as f:
            M = json.load(f)
        runs = M.get("runs", [])
        if not isinstance(runs, list) or not runs:
            raise ValueError("Manifest must contain a non-empty list under key 'runs'.")

        base_dir = os.path.dirname(os.path.abspath(args.manifest))
        out_dir = args.out_dir or os.path.join(base_dir, "out")
        os.makedirs(out_dir, exist_ok=True)

        # generate fit plots
        for run in runs:
            obj = run["object"]
            ns = run["n_safety"]
            csv_path = run["csv"]
            theta_gt = GT_PLACEHOLDER.get(obj, {}).get("theta_star_deg", np.nan)

            make_fit_plot(
                csv_path=csv_path,
                object_key=obj,
                n_safety=ns,
                params_json=args.params_json,
                out_dir=out_dir,
                stem=os.path.splitext(os.path.basename(csv_path))[0],
                theta_min_deg=args.theta_min_deg,
                theta_star_gt_deg=theta_gt,
                o_obj=args.o_obj,
                e_hat=args.e_hat,
                gap_thresh=args.gap_thresh,
                robust=args.robust,
                zscore=args.zscore,
                dpi=args.dpi
            )

        # summary
        df, out_csv = summarize(args.params_json, runs, gt=GT_PLACEHOLDER, out_dir=out_dir)
        print("\n=== Summary by (object, n_safety) ===")
        with pd.option_context("display.max_rows", 200, "display.max_columns", 200, "display.width", 160):
            print(df)
        print(f"\nWrote summary CSV: {out_csv}")
        print(f"Fit plots written to: {out_dir}")

    else:
        if args.object is None or args.n_safety is None:
            raise ValueError("Single-run mode requires --object and --n_safety.")
        csv_path = args.csv
        base_dir = os.path.dirname(os.path.abspath(csv_path))
        out_dir = args.out_dir or base_dir

        out_png = make_fit_plot(
            csv_path=csv_path,
            object_key=args.object,
            n_safety=args.n_safety,
            params_json=args.params_json,
            out_dir=out_dir,
            stem=os.path.splitext(os.path.basename(csv_path))[0],
            theta_min_deg=args.theta_min_deg,
            theta_star_gt_deg=args.theta_star_gt_deg,
            o_obj=args.o_obj,
            e_hat=args.e_hat,
            gap_thresh=args.gap_thresh,
            robust=args.robust,
            zscore=args.zscore,
            dpi=args.dpi
        )
        print(f"Wrote fit plot: {out_png}")

if __name__ == "__main__":
    main()
