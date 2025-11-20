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
            k_safe=0.5,
            baseline_window=50,
            median_window=5,
            contact_delta=0.150,  # N above baseline to declare contact (this is our resolution!)
            # ^ noise floor (empirical) appears to be 0.138 N
            contact_slope=0.1,  # N per sample increase to declare contact
            contact_samples=2,  # 3 worked... consecutive samples needed for contact
            fall_samples=5,     # consecutive samples needed for falling trigger
            debug=False,
        ):
        self.k_safe = k_safe
        self.debug = debug

        # Baseline / noise estimation
        self.baseline_window = baseline_window
        self.baseline_buf = [0.0] * baseline_window # prepare empty buffer
        self.baseline_ready = False
        self.noise_floor = 0.0

        # Median smoothing
        self.median_window = median_window
        self.median_buf = [0.0] * median_window

        # Contact detection parameters
        self.contact_delta = contact_delta
        self.contact_slope = contact_slope
        self.contact_samples = contact_samples

        # Falling / trigger parameters
        self.fall_samples = fall_samples

        # State
        self.state = "baseline"
        self.in_contact = False
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

        # Get peak always
        self.peak = max(self.peak, f_med)
        self.debug_msg(f"New Peak!: {self.peak:.3f} N", 1.0)

        # ------------ 1) BASELINE / SEARCHING STATE ------------
        if self.state == "baseline":
            # Build baseline buffer
            self.baseline_buf.pop(0)
            self.baseline_buf.append(f_med)

            if not self.baseline_ready:
                # Wait until enough samples collected
                if self.baseline_buf.count(0.0) == 0:
                    self.baseline_ready = True    
                # Still update prev_med for slope computation later
                self.prev_med = f_med
                return
            
            # Estimate noise floor as median of baseline buffer/window
            self.noise_floor = float(np.median(self.baseline_buf))

            # Check for sustained rise above baseline
            if self.prev_med is not None:
                df = 0.0
            else:
                df = f_med - self.prev_med # current (smoothed) minus previous
            

            if (f_med-self.noise_floor) > self.contact_delta:
                self.debug_msg(f"Contact magnitude met: df={df:.3f}, contact_slope={self.contact_slope}")
                
                if True: # TEMP DISABLING self.contact_slope:
                    self.debug_msg(f"Contact slope met: df={df:.3f} > {self.contact_slope:.3f}")
                    self.contact_count += 1
                    if self.contact_count >= self.contact_samples:
                        self.state = "contact"
                        self.in_contact = True
                        # self.peak = f_med
                        self.debug_msg(f"CONTACT DETECTED! Noise floor {self.noise_floor:.3f} N")
                else:
                    self.debug_msg(f"Contact slope NOT met: df={df:.3f} <= {self.contact_slope:.3f}")
                    self.contact_count = 0
            else:
                self.contact_count = 0

            self.prev_med = f_med
            return
        
        # ------------ 2) CONTACTE / PEAK TRACING ------------
        if self.state == "contact":
            rospy.loginfo
            # Require valid peak before checking fall
            if self.peak <= 0.0:
                self.prev_med = f_med
                return
            
            thresh = (1.0 - self.k_safe) * self.peak

            if f_med < thresh:
                self.below_count += 1
                self.debug_msg(f"Falling detected: f_med={f_med:.3f} N < thresh={thresh:.3f} N "
                               f"({self.below_count}/{self.fall_samples})", 1.0)
                if self.below_count >= self.fall_samples:
                    self.state = "triggered"
                    self.trigger = True
                    
                    self.debug_msg(f"Stop triggered(peak: {self.peak:.3f} N, threshold: {thresh:.3f} N, current: {f_med:.3f} N).")
            else:
                self.below_count = 0

            self.prev_med = f_med
            return
        

    def debug_msg(self, msg, throttle=None):
        if self.debug:
            if throttle is not None:
                rospy.loginfo_throttle(throttle, f"[ForceWatcher] {msg}")
            else:
                rospy.loginfo(f"[ForceWatcher] {msg}")