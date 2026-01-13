#!/usr/bin/env python3
"""
live_fit_plotter.py

Live plot like analyze_experiment torque-vs-theta figure.

Subscribes:
  - /com_3d/online_fit_debug (Float64MultiArray) from sync_logger_streaming_v2.py
Optionally:
  - /com_3d/online_estimate (Float64MultiArray) for printing

The debug message is packed as:
  data = [N,
          theta_deg[0..N-1],
          tau_y[0..N-1],
          tau_fit_y[0..N-1],
          theta_star_deg,
          m_est, zc_est]
"""
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

import matplotlib
import matplotlib.pyplot as plt

class LiveFitPlotter:
    def __init__(self):
        rospy.init_node("live_fit_plotter", anonymous=True)
        self.last = None
        self.last_est = None

        rospy.Subscriber("/com_3d/online_fit_debug", Float64MultiArray, self._on_dbg, queue_size=2)
        rospy.Subscriber("/com_3d/online_estimate", Float64MultiArray, self._on_est, queue_size=2)

        self.update_hz = float(rospy.get_param("~update_hz", 5.0))

        # IMPORTANT: TkAgg/QtAgg drawing must happen in the main thread.
        # So: no rospy.Timer. We update the figure from our own while-loop.
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 3.8))
        self.ax.set_xlabel("Object Angle (degrees)")
        self.ax.set_ylabel("Applied Torque (N-m)")
        self.ax.grid(True)

        # Lines
        (self.l_tau,) = self.ax.plot([], [], linewidth=3, label="Y Applied Torque")
        (self.l_fit,) = self.ax.plot([], [], linewidth=3, linestyle="--", label="Y Full T Fit (est)")
        # Ground-truth theta* (set via ROS param by vel_move) + online estimate marker.
        self.v_gt = self.ax.axvline(0.0, linestyle="--", linewidth=4, label=r"Ground truth $\theta^*$")
        self.zero = self.ax.axhline(0.0, linewidth=4, color='cyan', label="_")
        self.star = self.ax.scatter([0.0], [0.0], s=300, marker="*", label=r"Estimated $\theta^*$")

        self.txt = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes, va="top")

        self.ax.legend(loc="upper right")

        # No rospy.Timer; we tick in the main loop.

    def _on_est(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= 3:
            self.last_est = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _on_dbg(self, msg: Float64MultiArray):
        d = np.asarray(msg.data, dtype=float).reshape(-1)
        if d.size < 10:
            return
        N = int(round(d[0]))
        if N <= 0:
            return
        need = 1 + 3*N + 3
        if d.size < need:
            return
        th = d[1:1+N]
        tau = d[1+N:1+2*N]
        tau_fit = d[1+2*N:1+3*N]
        ths = float(d[1+3*N])
        m = float(d[1+3*N+1])
        zc = float(d[1+3*N+2])
        self.last = {"th": th, "tau": tau, "tau_fit": tau_fit, "ths": ths, "m": m, "zc": zc}

    def tick(self):
        if self.last is None:
            return

        th = self.last["th"]
        tau = self.last["tau"]
        tau_fit = self.last["tau_fit"]
        ths = self.last["ths"]
        m = self.last["m"]
        zc = self.last["zc"]

        self.l_tau.set_data(th, tau)
        self.l_fit.set_data(th, tau_fit)

        # Ground truth theta* from params (prefer deg, fall back to rad)
        ths_gt = rospy.get_param("/com_3d/theta_star_gt_deg", None)
        if ths_gt is None:
            ths_gt_r = rospy.get_param("/com_3d/theta_star_gt_rad", None)
            if ths_gt_r is not None:
                ths_gt = float(np.rad2deg(float(ths_gt_r)))

        if ths_gt is not None:
            self.v_gt.set_visible(True)
            self.v_gt.set_xdata([float(ths_gt), float(ths_gt)])
        else:
            self.v_gt.set_visible(False)

        # Estimated theta* marker at y=0 like your figure (update existing artist)
        self.star.set_offsets(np.c_[[ths], [0.0]])

        # Autoscale nicely
        xmin = float(np.nanmin(th)) if th.size else 0.0
        xmax = float(np.nanmax(th)) if th.size else 1.0
        self.ax.set_xlim(xmin - 1.0, xmax + 1.0)

        ymin = float(np.nanmin(np.r_[tau, tau_fit])) if tau.size else -1.0
        ymax = float(np.nanmax(np.r_[tau, tau_fit])) if tau.size else 1.0
        pad = 0.1 * (ymax - ymin + 1e-6)
        self.ax.set_ylim(ymin - pad, ymax + pad)

        ths_rad = np.deg2rad(ths)
        if ths_gt is not None:
            self.txt.set_text(
                f"m={m:.3f} kg, zc={zc:.3f} m, theta* (est)={ths:.2f} deg ({ths_rad:.3f} rad)\n"
                f"theta* (gt) ={float(ths_gt):.2f} deg\nN={len(th)}"
            )
        else:
            self.txt.set_text(f"m={m:.3f} kg, zc={zc:.3f} m, theta* (est)={ths:.2f} deg ({ths_rad:.3f} rad)\nN={len(th)}")

        # draw_idle is safer than draw() with TkAgg
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

if __name__ == "__main__":
    p = LiveFitPlotter()
    r = rospy.Rate(max(p.update_hz, 1e-3))
    while not rospy.is_shutdown():
        p.tick()
        plt.pause(0.001)
        r.sleep()