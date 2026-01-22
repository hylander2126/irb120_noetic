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
        self.run_base = rospy.get_param("~run_base", None)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pkg = rospkg.RosPack().get_path("com_3d")
        self.out_dir = os.path.join(pkg, "experiments")
        os.makedirs(self.out_dir, exist_ok=True)
        self.traj_idx = 0
        self.stem = None  # current run stem for saving

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
        
        # --- Config ---
        self.update_hz = 10.0
        self.dt_wall = 1.0 / self.update_hz

        # --- Matplotlib Setup ---
        plt.ion()
        
        # Create 2 vertical subplots
        self.fig, self.axs = plt.subplots(2, 1, figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Center of Mass Estimator - Live")
        
        # --- SUBPLOT 1: LIVE SENSORS (Top) ---
        self.ax_live = self.axs[0]
        self.ln_fx, = self.ax_live.plot([], [], 'r-', label='Fx', alpha=0.6)
        self.ln_fy, = self.ax_live.plot([], [], 'g-', label='Fy', alpha=0.6)
        self.ln_fz, = self.ax_live.plot([], [], 'b-', label='Fz', alpha=0.6)
        self.ax_live.axhline(0.0, color='c', linewidth=1.0)
        self.ax_live.set_ylabel("Force (N)")
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
        

        # --- SUBPLOT 2: POST-MORTEM RESULT (Bottom) ---
        self.ax_res = self.axs[1]
        self.ax_res.set_xlabel("Object Angle (deg)")
        self.ax_res.set_ylabel("Torque (N-m)")
        self.ax_res.grid(True)
        self.ax_res.set_title("Latest Fit Result (Waiting...)")

        # Ground truth line (persistent)
        self.ln_gt_th = self.ax_res.axvline(0, color='g', linestyle='--', linewidth=2, label='Ground Truth')
        self.ax_res.axhline(0.0, color='c', linewidth=1.0)

        # Placeholders for result plot
        # self.sc_data = self.ax_res.scatter([], [], c='b', alpha=0.3, s=10, label='Measured Data')
        # self.ln_fit, = self.ax_res.plot([], [], 'r--', linewidth=2, label='Fit Model')
        # self.ln_est_th = self.ax_res.axvline(0, color='m', linestyle='-', linewidth=2, label='Estimate')
        # self.ax_res.legend(loc="upper right")

        # --- Subscribers ---
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_start)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_stop)
        rospy.Subscriber('/netft_data_transformed', WrenchStamped, self._on_ft)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._on_tag)
        rospy.Subscriber("/com_3d/online_fit_debug", Float64MultiArray, self._on_fit_result)

        print("[LiveFitPlotter] Ready. Waiting for data...")

    def _build_stem(self):
        """Match FTClockLogger naming convention exactly."""
        object_name = rospy.get_param("/com_3d/object_name", "unknown")
        base = self.run_base if self.run_base is not None else f"{self.timestamp}_{object_name}"
        return f"{base}_t{self.traj_idx:02d}"

    def _on_start(self, _):
        """Reset live plots when a new log starts."""
        with self.lock:
            self.traj_idx += 1
            self.stem = self._build_stem()

            self.t_start = rospy.get_time()
            self.live_t_ft = []
            self.live_f = []
            self.live_t_tag = []
            self.live_th = []
            
            # Reset reference orientation
            if self.last_tag_q is not None:
                self.q_ref = self.last_tag_q
            else:
                self.q_ref = None 

            # Clear any pending result from a previous run
            self.is_live = True
            self.pending_result = None
            self.save_pending = False

            # Reset estimate counter and clear old fit artists
            self.estimate_count = 0
            for artist in self.fit_artists:
                artist.remove()
            self.fit_artists.clear()

        rospy.loginfo("[Plotter] Log start detected - Cleared buffers and fit plots.")

    def _on_stop(self, _):
        with self.lock:
            self.is_live = False
        rospy.loginfo("[Plotter] Log stop detected - Freezing live buffers.")

    def _on_ft(self, msg):
        with self.lock:
            if not self.is_live or self.t_start is None:
                return
            t = rospy.get_time() - self.t_start
            f = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
            self.live_t_ft.append(t)
            self.live_f.append(f)

    def _on_tag(self, msg):
        if not msg.detections: return
        
        q = msg.detections[0].pose.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        self.last_tag_q = q_arr

        with self.lock:
            if not self.is_live or self.t_start is None:
                return

            if self.q_ref is None:
                self.q_ref = q_arr

            q_norm = quat_normalize(q_arr)
            q_ref_inv = quat_conj(self.q_ref)
            q_rel = quat_mul(q_ref_inv, q_norm)
            rv = quat_to_rotvec(q_rel, normalize=False)
            angle_rad = np.linalg.norm(rv)

            t = rospy.get_time() - self.t_start
            self.live_t_tag.append(t)
            self.live_th.append(np.rad2deg(angle_rad))

    def _on_fit_result(self, msg):
        """
        Triggered when Estimator publishes final batch result.
        Msg format assumed: [N, t[N], th[N], tau[N], fit[N], th_star_est, m, z]
        """
        rospy.loginfo("[Plotter] Received Fit Result! Updating Plot...")
        
        d = np.array(msg.data, dtype=float)
        if d.size < 4: return

        N = int(d[0])
        
        # Slice Data safely
        try:
            idx = 1
            # t_arr = d[idx:idx+N]; idx += N # Unused
            idx += N 
            th_arr = d[idx:idx+N]; idx += N
            tau_arr = d[idx:idx+N]; idx += N
            fit_arr = d[idx:idx+N]; idx += N
            
            th_star_est = float(d[idx]); idx += 1
            m_est = float(d[idx]); idx += 1
            z_est = float(d[idx]); idx += 1
        except IndexError:
            rospy.logerr(f"[Plotter] Data array too short! Got len={len(d)}, expected > {4*N}")
            return
        
        # Save parsed data to shared variable
        payload = {
            "th": np.rad2deg(th_arr),
            "tau": tau_arr,
            "fit": fit_arr,
            "th_star": np.rad2deg(th_star_est),
            "m": m_est,
            "z": z_est
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
            stem = self.stem or "run"
            self.estimate_count += 1
            est_num = self.estimate_count

        gt_th_star_rad = rospy.get_param("/com_3d/theta_star", 0.0)
        gt_th_star_deg = np.rad2deg(gt_th_star_rad)

        # Define colors for different estimates
        colors = ['blue', 'orange', 'purple']
        color_idx = (est_num - 1) % len(colors)
        color = colors[color_idx]
        # Create label with estimate number
        alpha_scatter = 0.5 if est_num == 1 else 0.3
        alpha_line = 1.0 if est_num == 1 else 0.7

        # Add a new scatter plot for this estimate
        sc = self.ax_res.scatter(
            data["th"], data["tau"], 
            c=color, alpha=alpha_scatter, s=10, 
            label='_' #f'Push {est_num} Data'
        )

        # Add a new fit line for this estimate
        ln, = self.ax_res.plot(
            data["th"], data["fit"], 
            color=color, linestyle='--', linewidth=2, alpha=alpha_line,
            label=f'Push {est_num} Fit (m={data["m"]:.2f}kg, zc={data["z"]:.3f}m)'
        )

        # Add estimate theta* line
        vl = self.ax_res.axvline(
            data["th_star"], 
            color=color, linestyle='-', linewidth=2, alpha=alpha_line,
            label=f'Push {est_num} θ*={data["th_star"]:.1f}°'
        )
        # Store artists for potential removal later
        self.fit_artists.extend([sc, ln, vl])

        # Update GT line
        self.ln_gt_th.set_xdata([gt_th_star_deg, gt_th_star_deg])
        self.ln_gt_th.set_label(f'GT $\\theta^*$ ({gt_th_star_deg:.1f}$^\\circ$)')

        self.ax_res.set_title(f"Fit: m={data['m']:.2f}kg, zc={data['z']:.3f}m")
        self.ax_res.legend(loc="upper right")
        self.ax_res.relim()
        self.ax_res.autoscale_view()

        # Save (paper-ready)
        if do_save:
            png_path = os.path.join(self.out_dir, f"{stem}_fit_est{est_num}.png")

            try:
                # Render first
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()

                self.fig.savefig(png_path, dpi=300, bbox_inches="tight")
                rospy.loginfo("[Plotter] Saved: %s", png_path)
            except Exception as e:
                rospy.logwarn("[Plotter] Save failed: %s", str(e))
        # We don't draw here; the main loop handles drawing to keep GUI responsive


    def run(self):
        while not rospy.is_shutdown():
            # 1. Update Live Plots
            with self.lock:
                t_ft = list(self.live_t_ft)
                f_data = np.array(self.live_f) if self.live_f else np.empty((0,3))
                t_tag = list(self.live_t_tag)
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
            
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            plt.pause(self.dt_wall)

if __name__ == "__main__":
    p = LiveFitPlotter()
    p.run()