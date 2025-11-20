#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

# ----------------------- force monitor -----------------------
class ForceWatcher:
    """
    Pseudo-state-machine to monitor force to stop robot motion at threshold during tipping.
        - 'baseline': estimate noise floor from rolling window
        - 'contact': once a sustained rise above baseline is detected
        - 'falling': track peak and trigger when force falls below
                        (1 - k_safe) * peak for several samples
    """
    def __init__(
            self,
            ft_topic,
            k_safe=0.9,
            baseline_window=50,
            median_window=5,
            debug=False,
            initial_state=None # (i.e. for starting baseline after motion)
        ):
        self.k_safe = k_safe
        self.debug = debug

        # Baseline / noise estimation
        self.baseline_buf = [0.0] * baseline_window # prepare empty buffer
        self.baseline_ready = False
        self.noise_floor = 0.0

        # Median smoothing
        self.median_buf = [0.0] * median_window

        # Peak bufffer
        self.peak_buf = [0.0] * 50 # 5 being the peak window

        # Contact detection parameters
        self.contact_delta = 0.07  # N above baseline to declare contact (this is our resolution!)
        # ^ noise floor (empirical) appears to be 0.138 N
        self.contact_slope = 0.1  # N per sample increase to declare contact
        self.contact_samples = 2  # 3 worked... consecutive samples needed for contact

        # Falling / trigger parameters
        self.fall_samples = 5     # consecutive samples needed for falling trigger

        # State
        self.STATE = initial_state
        self.trigger = False
        self.peak = 0.0

        self.prev_med = None
        self.contact_count = 0
        self.below_count = 0

        self.sub = rospy.Subscriber(ft_topic, WrenchStamped, self.ft_cb, queue_size=50)

        # Looks like lshape (lightest) has peaks F: 0.117 -> 0.128 -> 0.248 -> 0.331
        # This is lower than our 

    def ft_cb(self, msg):
        # If triggered (motion stopped), unregister the subscriber and callback
        if self.trigger:
            rospy.loginfo("[ForceWatcher] Triggered; unsubscribing from FT topic.")
            self.sub.unregister()
            return

        fx, fy, fz = msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z
        f = np.linalg.norm([fx, fy, fz])

        # Median smoothing (rolling window)
        self.median_buf.pop(0)
        self.median_buf.append(f)
        f_med = np.median(self.median_buf)

        prev_med = self.prev_med

        # Get peak always
        # self.peak = max(self.peak, f_med)
        # self.debug_msg(f"New Peak!: {self.peak:.3f} N", 0.05)

        # ------------ 1) BASELINE / SEARCHING STATE ------------
        if self.STATE == "BASELINE":
            # Build baseline buffer
            self.baseline_buf.pop(0)
            self.baseline_buf.append(f_med)

            if not self.baseline_ready:
                # Wait until enough samples collected
                if self.baseline_buf.count(0.0) == 0:
                    self.baseline_ready = True
                    # Estimate noise floor as median of baseline buffer/window
                    self.noise_floor = float(np.median(self.baseline_buf))
            else:
                self.debug_msg(f"Noise floor found: {self.noise_floor:.3f} N\n plus contact delta: {self.noise_floor + self.contact_delta:.3f}", 0)
                self.STATE = "MONITOR"
        
        # ------------ 2) MONITORING FOR CONTACT ------------
        if self.STATE == "MONITOR":
            # Check for sustained rise above baseline
            if prev_med is None:
                df = 0.0 # Catch the first sample
            else:
                df = f_med - prev_med # current (smoothed) minus previous
            
            cond1 = (f_med-self.noise_floor) > self.contact_delta
            cond2 = True # TEMP DISABLING     df > self.contact_slope:
            if cond1 and cond2:
                self.contact_count += 1
                self.debug_msg(f"""Contact magnitude met: f_med={f_med:.3f} N, (contact_delta={self.contact_delta:.3f})""") if cond1 else None
                self.debug_msg(f"Min slope met (TEMP check disabled)")
                if self.contact_count >= self.contact_samples:
                    self.STATE = "PEAK"
                    self.debug_msg(f"{self.contact_count} CONTACTS DETECTED!", 0)
            else:
                self.contact_count = 0
                self.debug_msg(f"""Contact magnitude NOT met: \nf_med={f_med:.3f} <= noise floor+delta={self.noise_floor+self.contact_delta:.3f} N \nContact count reset.""", 0.1)

        # ------------ 3) PEAK TRACKING ------------
        if self.STATE == "PEAK":
            # Record the peak and average over several samples
            self.peak_buf.pop(0)
            self.peak_buf.append(f_med)
            if self.peak_buf.count(0.0) == 0:
                # self.debug_msg(f"Peak buffer looks like: {np.round(self.peak_buf, 3)}")
                self.peak = float(np.max(self.peak_buf))
                self.debug_msg(f"Peak force recorded: {self.peak:.3f} N")
                self.STATE = "CONTACT"


        # ------------ 3) CONTACT / PEAK TRACING ------------
        if self.STATE == "CONTACT":
            # # Require valid peak before checking fall
            # if self.peak <= 0.0:
            #     self.prev_med = f_med
            #     # pass

            # thresh = (1.0 - self.k_safe) * self.peak
            f_safe = self.peak - (self.k_safe * self.peak)
            thresh = np.max([f_safe, self.noise_floor])
            if thresh != f_safe:
                self.debug_msg(f"Adjusted f_safe to noise floor: {thresh:.3f} N (original f_safe: {f_safe:.3f} N, noise floor: {self.noise_floor:.3f} N)", 0)

            self.debug_msg(f"Tracking current:{f_med:.3f}, peak: {self.peak:.3f} for f_safe: {thresh:.3f}", 0.1)
            
            if self.peak > 0.0:
                if f_med < thresh:
                    self.below_count += 1
                    self.debug_msg(f"Falling detected: f_med={f_med:.3f} N < thresh={thresh:.3f} N "
                                f"({self.below_count}/{self.fall_samples})", 1.0)
                    if self.below_count >= self.fall_samples:
                        self.STATE = "TRIGGERED"
                        self.trigger = True
                        self.debug_msg(f"Stop triggered(peak: {self.peak:.3f} N, threshold: {thresh:.3f} N, current: {f_med:.3f} N).")
                        self.sub.unregister()
                else:
                    self.below_count = 0
        
        # ------------ 4) UPDATE PREV_MED ONCE ------------
        self.prev_med = f_med


    def debug_msg(self, msg, throttle=None):
        if self.debug:
            if throttle is None:
                rospy.loginfo(f"[ForceWatcher] {msg}")
            elif throttle == 0:
                rospy.loginfo_once(f"[ForceWatcher] {msg}")
            else:
                rospy.loginfo_throttle(throttle, f"[ForceWatcher] {msg}")

    def exit_fw(self):
        self.sub.unregister()