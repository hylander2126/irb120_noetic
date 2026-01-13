import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.collections import PathCollection

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
        self.fig = plt.figure(figsize=(12, 8))
        self.gs = self.fig.add_gridspec(2, 1)
        
        # Subplot 1: Torque vs Angle (The Physics Fit)
        self.ax_fit = self.fig.add_subplot(self.gs[0, 0])
        self.sc = self.ax_fit.scatter([], [], c=[], cmap='viridis', s=10, label="Observed")
        self.l_fit, = self.ax_fit.plot([], [], 'r--', linewidth=2, label="Fit")
        self.ax_fit.set_xlabel("Angle (deg)")
        self.ax_fit.set_ylabel("Torque (N-m)")
        
        # Subplot 2: Time Series Debug (The Raw Signals)
        self.ax_time = self.fig.add_subplot(self.gs[1, 0])
        self.l_raw_th, = self.ax_time.plot([], [], 'g-', label="Angle (deg)")
        self.l_raw_tau, = self.ax_time.plot([], [], 'b-', label="Torque (N-m)")
        self.ax_time.set_xlabel("Sample Index")
        self.ax_time.legend(loc="upper right")

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
        if self.last is None: return
        th = self.last["th"]
        tau = self.last["tau"]
        
        # A2 Fix: Zero the angle if it's offset by ~180
        if np.median(th) > 90: th -= 180.0 

        # A1: Scatter with color-over-time (viridis)
        self.sc.set_offsets(np.c_[th, tau])
        self.sc.set_array(np.linspace(0, 1, len(th)))
        
        # A3: Plot against index for debugging
        self.l_raw_th.set_data(range(len(th)), th)
        self.l_raw_tau.set_data(range(len(tau)), tau)
        
        self.ax_fit.relim(); self.ax_fit.autoscale_view()
        self.ax_time.relim(); self.ax_time.autoscale_view()
        self.fig.canvas.draw_idle(); self.fig.canvas.flush_events()

if __name__ == "__main__":
    p = LiveFitPlotter()
    r = rospy.Rate(max(p.update_hz, 1e-3))
    while not rospy.is_shutdown():
        p.tick()
        plt.pause(0.001)
        r.sleep()