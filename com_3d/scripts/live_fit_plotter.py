#!/usr/bin/env python3
import numpy as np
from std_msgs.msg import Float64MultiArray, Empty
from geometry_msgs.msg import WrenchStamped
from apriltag_ros.msg import AprilTagDetectionArray
import matplotlib.pyplot as plt
import rospy, threading, os, rospkg
from datetime import datetime

# Keep your imports
from com_3d.helper_fns import quat_normalize, quat_conj, quat_mul, quat_to_rotvec

class LiveFitPlotter:
    def __init__(self):
        rospy.init_node("live_fit_plotter", anonymous=True)
        
        # --- Thread Safety ---
        self.lock = threading.Lock()

        # --- Naming + output (match FTClockLogger) ---
        self.out_dir = None
        self.log_stem = None

        # --- Live Data Buffers (Separated for different rates) ---
        self.t_start = None
        self.q_ref = None
        
        # Buffer 1: High Rate (FT Sensor)
        self.live_t_ft = []
        self.live_f    = [] # [fx, fy, fz]

        # Buffer 2: Low Rate (Camera/Tag)
        self.live_t_tag = []
        self.live_th    = []

        self.last_tag_q = None

        # --- Async Data Transfer ---
        # Stores data from ROS callback to plot by main loop
        self.pending_result = None
        self.is_live = False
        self.save_pending = False

        # --- Track Multiple Estimates ---
        self.estimate_count = 0 # Counter of estimates in current trajectory
        self.fit_artists = []  # Store fit lines for multiple estimates
        self.clear_artists_pending = False # Flag to clear old artists
        
        # --- Config ---
        self.update_hz = 10.0
        self.dt_wall = 1.0 / self.update_hz

        # --- Timing for live plot ---
        self.t_offset = 0.0             # total accum 'active' time
        self.t_seg_start_wall = None    # wall start time for current active segment
        self.has_started_once = False

        # --- Matplotlib Setup ---
        plt.ion()
        
        # Create live plot
        self.fig_live, self.ax_live = plt.subplots(1, 1, figsize=(8, 4.5))
        self.fig_live.canvas.manager.set_window_title("CoM Estimator - Live")
        
        # --- LIVE SENSORS PLOT ---
        self.ln_fx, = self.ax_live.plot([], [], 'r-', label='Fx', alpha=0.6)
        self.ln_fy, = self.ax_live.plot([], [], 'g-', label='Fy', alpha=0.6)
        self.ln_fz, = self.ax_live.plot([], [], 'b-', label='Fz', alpha=0.6)
        self.ax_live.axhline(0.0, color='c', linewidth=1.0)
        self.ax_live.set_ylabel("Force (N)")
        self.ax_live.set_xlabel("Time (s) (gap-compressed)")
        self.ax_live.set_title("Live Data Stream")

        # Create Twin Axis for Angle (since deg != Newtons)
        self.ax_live_twin = self.ax_live.twinx()
        self.ln_th, = self.ax_live_twin.plot([], [], 'k-', linewidth=2, label='Angle')
        self.ax_live_twin.set_ylabel("Angle (deg)")

        # Combine Legends for Top Plot
        lines1, labels1 = self.ax_live.get_legend_handles_labels()
        lines2, labels2 = self.ax_live_twin.get_legend_handles_labels()
        self.ax_live.legend(lines1 + lines2, labels1 + labels2, loc="upper left")
        self.ax_live.grid(True)
        
        # --- Fit Plot ---
        self.fig_fit, self.ax_fit = plt.subplots(1, 1, figsize=(8, 4.5))
        self.fig_fit.canvas.manager.set_window_title("CoM Estimator - Fit Results")

        self.ax_fit.set_xlabel("Object Angle (deg)", fontsize=16)
        self.ax_fit.tick_params(axis='both', labelsize=14)
        self.ax_fit.set_ylabel("Torque (N-m)", fontsize=16)
        self.ax_fit.grid(True)
        # self.ax_res.set_title("Latest Fit Result (Waiting...)")

        # Ground truth line (persistent)
        self.ln_gt_th = self.ax_fit.axvline(0, color='g', linestyle='--', linewidth=2, label='Ground Truth')
        self.ax_fit.axhline(0.0, color='c', linewidth=1.0)

        # --- Subscribers ---
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_start)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_stop)
        rospy.Subscriber('/netft_data_transformed', WrenchStamped, self._on_ft)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._on_tag)
        rospy.Subscriber("/com_3d/online_fit_debug", Float64MultiArray, self._on_fit_result)

        print("[LiveFitPlotter] Ready. Waiting for data...")

    def _build_stem(self):
        """Match FTClockLogger naming convention exactly."""
        self.log_stem = rospy.get_param("/com_3d/current_log_stem", None)
        self.out_dir = rospy.get_param("/com_3d/current_log_dir", ".")

    def _on_start(self, _):
        """Reset live plots when a new log starts."""
        with self.lock:
            self._build_stem()

            now = rospy.get_time()

            # If brand new run, clear buffers, else accumulate time offset
            if not self.has_started_once:
                self.live_t_ft, self.live_f = [], []
                self.live_t_tag, self.live_th = [], []
                self.t_offset = 0.0
                self.estimate_count = 0
                self.clear_artists_pending = True
                self.has_started_once = True
                
                # Reset reference orientation
                if self.last_tag_q is not None:
                    self.q_ref = self.last_tag_q
                else:
                    self.q_ref = None 

                # Unused currently:
                # self.t_start = now
                rospy.loginfo("[Plotter] Log start detected - Cleared buffers and fit plots.")
            else:
                rospy.loginfo("[Plotter] Log restart detected - Resuming live buffers.")

            # else
            # Resume segmented timing
            self.t_seg_start_wall = now
            self.is_live = True
            self.pending_result = None
            self.save_pending = False

    def _clear_fit_artists(self):
        """Called from main thread only - safe to manipulate matplotlib."""
        for artist in self.fit_artists:
            artist.remove()
        self.fit_artists.clear()
        self.ax_fit.relim()
        self.ax_fit.autoscale_view()
    
    def _on_stop(self, _):
        with self.lock:
            if self.is_live and self.t_seg_start_wall is not None:
                self.t_offset += rospy.get_time() - self.t_seg_start_wall
            
            self.is_live = False
            self.t_seg_start_wall = None
        rospy.loginfo("[Plotter] Log stop detected - Freezing live buffers.")
    
    def _now_plot_time(self):
        now = rospy.get_time()
        if self.t_seg_start_wall is None:
            return self.t_offset
        return self.t_offset + (now - self.t_seg_start_wall)

    def _on_ft(self, msg):
        with self.lock:
            if not self.is_live: # or self.t_start is None:
                return
            # t = rospy.get_time() - self.t_start
            t = self._now_plot_time()
            f = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
            self.live_t_ft.append(t)
            self.live_f.append(f)

    def _on_tag(self, msg):
        if not msg.detections: return
        
        q = msg.detections[0].pose.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        self.last_tag_q = q_arr

        with self.lock:
            if not self.is_live: # or self.t_start is None:
                return

            if self.q_ref is None:
                self.q_ref = q_arr

            q_norm = quat_normalize(q_arr)
            q_ref_inv = quat_conj(self.q_ref)
            q_rel = quat_mul(q_ref_inv, q_norm)
            rv = quat_to_rotvec(q_rel, normalize=False)
            angle_rad = np.linalg.norm(rv)

            # t = rospy.get_time() - self.t_start
            t = self._now_plot_time()
            self.live_t_tag.append(t)
            self.live_th.append(np.rad2deg(angle_rad))

    def _on_fit_result(self, msg):
        """
        Triggered when Estimator publishes final batch result.
        Msg format:
         [Np, Nr,
         th_p[Np], tau_p[Np], fit_p[Np],
         th_r[Nr], tau_r[Nr], fit_r[Nr],
         th_star_est, m, z]
        All th_* are in RADIANS coming from estimator.
        tau_* and fit_* are scalar projected torque (N-m).
        """
        rospy.loginfo("[Plotter] Received Fit Result! Updating Plot...")
        
        d = np.array(msg.data, dtype=float)
        if d.size < 5: return
        
        # Slice Data safely
        try:
            idx = 0
            Np = int(d[idx]); idx += 1
            Nr = int(d[idx]); idx += 1

            th_p = d[idx:idx+Np]; idx += Np
            tau_p = d[idx:idx+Np]; idx += Np
            fit_p = d[idx:idx+Np]; idx += Np

            th_r = d[idx:idx+Nr]; idx += Nr
            tau_r = d[idx:idx+Nr]; idx += Nr
            fit_r = d[idx:idx+Nr]; idx += Nr

            th_star_est = float(d[idx]); idx += 1
            m_est = float(d[idx]); idx += 1
            z_est = float(d[idx]); idx += 1
        except Exception as e:
            rospy.logerr(f"[Plotter] Failed to parse fit payload: {e}. len={len(d)}")
            return
        
        # Save parsed data to shared variable
        payload = {
            "th_p": np.rad2deg(th_p),
            "tau_p": tau_p,
            "fit_p": fit_p,
            "th_r": np.rad2deg(th_r),
            "tau_r": tau_r,
            "fit_r": fit_r,
            "th_star": np.rad2deg(th_star_est),
            "m": m_est,
            "z": z_est,
        }
        with self.lock:
            self.pending_result = payload
            self.save_pending = True


    def _update_result_plot_and_save(self):
        """
        Main Thread Helper: Updates the bottom plot if data is pending.
        Adds new fit line for each estimate received.
        """
        with self.lock:
            if self.pending_result is None:
                return
            data = self.pending_result
            self.pending_result = None
            do_save = self.save_pending
            self.save_pending = False
            self.estimate_count += 1
            est_num = self.estimate_count

        gt_th_star_rad = rospy.get_param("/com_3d/theta_star", 0.0)
        gt_th_star_deg = np.rad2deg(gt_th_star_rad)

        # Update title to reflect n_safety value for this run
        n_safety = rospy.get_param("/com_3d/n_safety", 0.00)
        self.ax_live.set_title(f"Live Data Stream (n_safety={n_safety})")

        # Define colors and labels for different estimates
        cc = ['tab:blue', 'tab:orange']
        c_idx = (est_num - 1) % len(cc) # Alternate colors for each estimate
        colors = cc[c_idx]
        labels = ["Push Fit", "Retract Fit"] if c_idx==0 else ['_', '_']
        markers = ['o', 'd']
        alpha_push = [0.70, 0.95] # scatter, line
        alpha_retr = [0.70, 0.70] # scatter, line
        lp = ['-', '--'][c_idx] # Est 1 is Solid, Est 2 is Dashed
        # lr = [':', '-.'][c_idx] # Est 1 is Dotted, Est 2 is Dash-Dot
        lr = lp
        
        # Add a new scatter plot for this estimate
        # --- Push (solid dots + dashed line) ---
        scatter_push = self.ax_fit.scatter(data["th_p"], data["tau_p"],
            c=colors, alpha=alpha_push[0], s=30, label="_", marker=markers[0])
        
        fit_push, = self.ax_fit.plot(data["th_p"], data["fit_p"],
            color='black', linestyle=lp, linewidth=2, alpha=alpha_push[1],
            label=labels[0])

        # --- Retract (lighter dots + dotted line) ---
        scatter_retr = self.ax_fit.scatter(data["th_r"], data["tau_r"],
            c=colors, alpha=alpha_retr[0], s=30, label="_", marker=markers[1])

        fit_retr, = self.ax_fit.plot(data["th_r"], data["fit_r"],
            color='black', linestyle=lr, linewidth=2, alpha=alpha_retr[1], # max(0.5, alpha_line * 0.8),
            label=labels[1])

        # --- theta* line (combined estimate) ---
        vl = self.ax_fit.axvline(data["th_star"],
            color=colors, linestyle='-', linewidth=2, alpha=0.9,
            label=f'Est {est_num} θ*={data["th_star"]:.1f}°, m={data["m"]:.2f}kg, z={data["z"]:.3f}m')

        # Store artists for potential removal later
        self.fit_artists.extend([scatter_push, fit_push, scatter_retr, fit_retr, vl])

        # Update GT line
        self.ln_gt_th.set_xdata([gt_th_star_deg, gt_th_star_deg])
        self.ln_gt_th.set_label(f'GT $\\theta^*$ ({gt_th_star_deg:.1f}$^\\circ$)')

        # self.ax_res.set_title(f"Fit: m={data['m']:.2f}kg, zc={data['z']:.3f}m")
        self.ax_fit.legend(loc="upper right", fontsize=10)
        self.ax_fit.relim()
        self.ax_fit.autoscale_view()

        # Save (paper-ready)
        if do_save:
            if self.out_dir == '.' or self.log_stem is None:
                try:
                    self._build_stem() # Try again in case it wasn't set in sync_logger yet
                except:
                    pass
            fit_png_path = os.path.join(self.out_dir, self.log_stem + "_fit_plot.png")
            live_png_path = os.path.join(self.out_dir, self.log_stem + "_live_plot.png")

            try:
                # Render fit first
                self.fig_fit.canvas.draw_idle()
                self.fig_fit.canvas.flush_events()
                self.fig_fit.savefig(fit_png_path, dpi=300, bbox_inches="tight")
                rospy.loginfo("[Plotter] Saved: %s", fit_png_path)
                # Render live plot
                self.fig_live.canvas.draw_idle()
                self.fig_live.canvas.flush_events()
                self.fig_live.savefig(live_png_path, dpi=300, bbox_inches="tight")
                rospy.loginfo("[Plotter] Saved: %s", live_png_path)
            except Exception as e:
                rospy.logwarn("[Plotter] Save failed: %s", str(e))
        # We don't draw here; the main loop handles drawing to keep GUI responsive


    def run(self):
        while not rospy.is_shutdown():
            # 0. Handle any pending artist clearing (main thread safe)
            with self.lock:
                should_clear = self.clear_artists_pending
                self.clear_artists_pending = False

            if should_clear:
                self._clear_fit_artists()

            # 1. Update Live Plots
            with self.lock:
                t_ft = list(self.live_t_ft) # raw time with gaps (pure stream)
                f_data = np.array(self.live_f) if self.live_f else np.empty((0,3))
                t_tag = list(self.live_t_tag) # raw time with gaps
                th_data = list(self.live_th)

            if len(t_ft) > 1 and len(f_data) == len(t_ft):
                self.ln_fx.set_data(t_ft, f_data[:, 0])
                self.ln_fy.set_data(t_ft, f_data[:, 1])
                self.ln_fz.set_data(t_ft, f_data[:, 2])
                self.ax_live.relim()
                self.ax_live.autoscale_view()

            if len(t_tag) > 1 and len(th_data) == len(t_tag):
                self.ln_th.set_data(t_tag, th_data)
                self.ax_live_twin.relim()
                self.ax_live_twin.autoscale_view()
            
            # 2. Check for and Update Result Plot (Main Thread)
            self._update_result_plot_and_save()
            
            self.fig_live.canvas.draw_idle()
            self.fig_fit.canvas.draw_idle()
            self.fig_live.canvas.flush_events()
            self.fig_fit.canvas.flush_events()
            plt.pause(self.dt_wall)

if __name__ == "__main__":
    p = LiveFitPlotter()
    p.run()