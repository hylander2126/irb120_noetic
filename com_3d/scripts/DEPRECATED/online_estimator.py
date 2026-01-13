import numpy as np
from scipy.optimize import curve_fit
from scipy.stats import linregress
from com_3d.com_estimation import tau_app_model, tau_model

class OnlineCoMEstimator:
    def __init__(self, o_obj):
        self.o_obj = o_obj  # Tool flange/object origin offset
        self.data_buffer = []
        self.is_collecting = False

    def start_collection(self):
        self.data_buffer = []
        self.is_collecting = True

    def add_datum(self, force, ee_pose, angle, tip_axis):
        if self.is_collecting:
            self.data_buffer.append({
                'f': force,        # [fx, fy, fz]
                'ee': ee_pose,     # [x, y, z]
                'th': angle,       # float (radians)
                'axis': tip_axis   # [ux, uy, uz]
            })

    def estimate(self, m_gt_guess, zc_gt_guess):
        if len(self.data_buffer) < 10:
            return None

        # Convert buffer to numpy arrays
        f_trim = np.array([d['f'] for d in self.data_buffer])
        th_trim = np.array([d['th'] for d in self.data_buffer])
        ee_trim = np.array([d['ee'] for d in self.data_buffer])
        tip_axis = self.data_buffer[-1]['axis'] # Use last known axis

        # 1. Linear Fit for Initial Guess
        # Projecting X-force vs Theta
        lin_slope, lin_b, _, _, _ = linregress(th_trim, f_trim[:, 0])
        th_star_calc = -lin_b / lin_slope
        zc_calc = abs(0.05 / np.tan(th_star_calc)) # Assuming 50mm CoM X-offset guess
        m_calc = abs(lin_slope * ee_trim[-1, 2] / (9.81 * zc_calc))

        # 2. Refined Fit using Tau Model
        rf = ee_trim - self.o_obj
        f_app = -f_trim 
        tau_app_trim = tau_app_model(f_app, rf)
        
        # We assume com_xy_offset is roughly known or part of the model
        rc0_known = np.array([-0.04, 0.0, 0.0]) 

        try:
            [m_est, zc_est], _ = curve_fit(
                lambda th, m, zc: tau_model(th, m, zc, rc0_known=rc0_known, e_hat=tip_axis),
                th_trim,
                tau_app_trim,
                p0=[m_calc, zc_calc],
                bounds=([0, 0], [np.inf, np.inf])
            )
            th_star_est = np.arctan2(0.04, zc_est)
            return m_est, zc_est, th_star_est
        except Exception as e:
            print(f"Online Fit Failed: {e}")
            return m_calc, zc_calc, th_star_calc