#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

# ----------------------- force monitor -----------------------
class ForceWatcher:
    """Minimal force peak tracker: detects contact → peak → triggers stop when F <= k_safe * F_peak."""
    def __init__(self, ft_topic, k_safe=0.5, contact_thresh=5.0, axis=(1,0,0)):
        self.k_safe = k_safe
        self.contact_thresh = contact_thresh # sensor noise (wavelength basically) SET THIS HIGH TO IGNORE STOP

        self.peak = 0.0
        self.in_contact = False
        self.armed = False
        self.trigger = False

        self.buf = [0.0]*5 # small rolling window
        self.below_countr = 0

        self.sub = rospy.Subscriber(ft_topic, WrenchStamped, self.cb, queue_size=50)

    def cb(self, msg):
        fx, fy, fz = msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z
        f = np.linalg.norm([fx, fy, fz])

        self.buf.pop(0)
        self.buf.append(f)
        f_med = np.median(self.buf)

        # detect contact
        if not self.in_contact:
            if f_med > self.contact_thresh:
                self.in_contact = True
            return

        # track peak once contact
        if f_med > self.peak:
            self.peak = f_med
            
        thresh = (1-self.k_safe) * self.peak

        if f_med <= thresh:
            self.below_countr += 1
            if self.below_countr >= 5:  # require 5 consecutive below-thresh readings
                self.trigger = True
                rospy.loginfo(f"\n[ForceWatcher] Triggered at force {f_med:.3f} N (peak was {self.peak:.3f} N).\n")
        else:
            self.below_countr = 0
        
        # once force is falling and we have a peak, arm
        # if self.peak > 0 and f < self.peak*0.98:  # loose drop threshold
        #     self.armed = True
        # # if armed, stop when below k_safe * peak
        # if self.armed and f <= 0.3:  # self.k_safe * self.peak:
        #     self.trigger = True
        #     rospy.loginfo(f"[ForceWatcher] Triggered at force {f:.3f} N (peak was {self.peak:.3f} N).")
