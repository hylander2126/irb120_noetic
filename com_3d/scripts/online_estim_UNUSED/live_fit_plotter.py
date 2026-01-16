import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import time

class LiveFitPlotter:
    def __init__(self):
        
        rospy.init_node("live_fit_plotter", anonymous=True)
        
        # Buffers for the "Instantaneous" fit (Top Plot)
        self.last = None 

        # Buffers for the "History" (Accumulated data)
        self.all_th = []
        self.all_tau = []
        self.all_time = []

        # State tracking
        self.th0 = None # ref angle to zero top plot
        self.last_msg_time = time.time() # track last update time
        self.last_t_processed = -1.0 # To prevent duplicate points

        rospy.Subscriber("/com_3d/online_fit_debug", Float64MultiArray, self._on_dbg, queue_size=2)

        self.update_hz = float(rospy.get_param("~update_hz", 5.0))
        self._dt_wall = 1.0 / max(self.update_hz, 1e-6)

        plt.ion()
        self.fig = plt.figure(figsize=(12, 8))
        self.gs = self.fig.add_gridspec(2, 1)
        
        # --- TOP PLOT: Torque vs Angle (ENTIRE HISTORY) ---
        self.ax_fit = self.fig.add_subplot(self.gs[0, 0])
        self.sc_fit = self.ax_fit.scatter([], [], c='b', s=15, label="History")
        # Fit line still shows the *current* local model
        # self.lin_fit, = self.ax_fit.plot([], [], 'r--', linewidth=2, label="Current Fit")
        self.zero_fit = self.ax_fit.axhline(0.0, c='c', linewidth=1)
        self.ax_fit.set_xlabel("Tip Angle (deg)")
        self.ax_fit.set_ylabel("Torque (N-m)")
        self.ax_fit.legend(loc="upper left")
        self.ax_fit.grid(True)
        
        # --- BOTTOM PLOT: Time Series (Entire History) ---
        self.ax_time = self.fig.add_subplot(self.gs[1, 0])
        self.tau_time, = self.ax_time.plot([], [], 'b-', label="Torque (N-m)")
        self.zero_time = self.ax_time.axhline(0.0, c='c', linewidth=1)

        self.ax_time2 = self.ax_time.twinx()
        self.th_time, = self.ax_time2.plot([], [], 'g-', label="Angle (deg)")
        
        self.ax_time.set_xlabel("Time (s)")

        # Combine Legends
        lines, labels = self.ax_time.get_legend_handles_labels()
        lines2, labels2 = self.ax_time2.get_legend_handles_labels()
        self.ax_time2.legend(lines + lines2, labels + labels2, loc="upper right")
        self.ax_time.grid(True)

        try:
            self.fig.canvas.manager.set_window_title("Live CoM Fit")
        except Exception:
            pass
        plt.show(block=False)


    def _on_dbg(self, msg: Float64MultiArray):
        current_time = time.time()
        
        # RESET Logic: If we haven't seen a message for >3 seconds, assume a NEW run is starting.
        # This clears old history so the new plot starts fresh at angle=0.
        if (current_time - self.last_msg_time) > 3.0 and self.last_msg_time > 0:
            rospy.loginfo("Detected new run (gap > 3s). Resetting plotter history.")
            self.all_th = []
            self.all_tau = []
            self.all_time = []
            self.th0 = None
            self.last_t_processed = -1.0
            # For debugging, print out the entire history of theta and tau
            rospy.loginfo(f"Previous run had {len(self.all_th)}theta samples and {len(self.all_tau)} tau samples.")

        # Reset the watchdog timer    
        self.last_msg_time = current_time

        d = np.asarray(msg.data, dtype=float).reshape(-1) # convert to np array
        if d.size < 10: return
        N = int(round(d[0]))
        if N <= 0 or d.size < (1+ 4*N + 3): return
            
        # Extract data from message using slicing
        idx = 1
        time_arr = d[idx : idx+N]; idx += N
        th = d[idx : idx+N]; idx += N
        tau = d[idx : idx+N]; idx += N
        # tau_fit = d[idx : idx+N]; idx += N
        
        ths = float(d[idx]); idx += 1
        # m = float(d[idx]); idx += 1
        # zc = float(d[idx]); idx += 1

        # Extract NEWEST sample from window
        t_new = time_arr[-1]

        if t_new > self.last_t_processed:
            self.last_t_processed = t_new

            # Convert to degrees FIRST
            th_deg = np.rad2deg(th[-1])
            # ths_deg = np.rad2deg(ths)
            if np.median(np.rad2deg(th)) > 90: 
                th_deg -= 180.0

            # Set Zero reference if first sample
            if self.th0 is None:
                self.th0 = th_deg

            # Update the "Window" data for the top plot
            # self.last = {"th": th_deg, "tau_fit": tau_fit}

            # Accumulate history for the both plots
            self.all_time.append(t_new)
            self.all_th.append(th_deg)
            self.all_tau.append(tau[-1])

    def tick(self):
        # SAFETY: Trim all buffers to the smallest length to prevent shape mismatch
        n = min(len(self.all_time), len(self.all_th), len(self.all_tau))        # Need reference zero to plot relative angles
        if self.th0 is None or len(self.all_th) == 0: return

        # Check if we are actively receiving data (within last 3 seconds)
        is_active = (time.time() - self.last_msg_time) < 3.0

        # --- TOP PLOT (Full History + Fit) ---
        th_hist = np.array(self.all_th) - self.th0
        tau_hist = np.array(self.all_tau)
        time_hist = np.array(self.all_time)
        
        # Scatter of FULL history
        self.sc_fit.set_offsets(np.c_[th_hist, tau_hist])

        # Color by time (to see evolution)
        # c_array = np.linspace(0, 1, len(th_hist))
        # self.sc_fit.set_array(c_array)

        # Update Fit line (current window only) - must shift window X-data by th0 to aligns with history
        # if self.last is not None:
        #     th_win = self.last["th"] - self.th0
        #     tau_fit = self.last["tau_fit"]
        #     self.lin_fit.set_data(th_win, tau_fit)

        # --- BOTTOM PLOT (Full History) ---
        # We plot the persistent lists, not the message buffes
        self.th_time.set_data(time_hist, self.all_th)
        self.tau_time.set_data(time_hist, tau_hist)

        if is_active:
            # Top plot
            self.ax_fit.set_xlim(0, max(max(th_hist)+5, 15))
            self.ax_fit.relim()
            self.ax_fit.autoscale_view()
            # Bottom plot
            max_t = time_hist[-1]
            self.ax_time.set_xlim(0, max(max_t + 0.5, 10))
            self.ax_time.relim()
            self.ax_time.autoscale_view(scalex=False, scaley=True)

        # Draw
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

if __name__ == "__main__":
    p = LiveFitPlotter()
    while not rospy.is_shutdown():
        p.tick()
        # Slightly faster pause to keep UI responsive
        plt.pause(0.001) 
        time.sleep(p._dt_wall)