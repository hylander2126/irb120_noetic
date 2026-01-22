import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.signal import medfilt, butter, filtfilt, savgol_filter, find_peaks
from scipy.interpolate import interp1d
from com_3d.com_estimation import tau_app_model, tau_model, align_zeros, F_model
from scipy.stats import linregress
from scipy.spatial.transform import Rotation as R
from scipy.optimize import curve_fit
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
from com_3d.helper_fns import quat_to_axis_angle, enforce_quat_continuity

PATH_ROOT = "experiments/20251218_"
OBJECTS = {
        # X and Y is distance from tipping edge (object frame!) to projected CoM on table plane
        "box": {
            # "path": "experiments/20251208_155739_box_t02.csv",
            "path": PATH_ROOT + "163333_box_t01.csv",
            "com": [-0.04515, 0.0, 0.14624], # -0.0500, 0, 0.1500]
            "mass": 0.664, # 635
            "theta_star": 0.0, # placeholder
            "height": 0.3,
            "est": [0,0,0],
        },
        "heart": {
            # "path": "experiments/20251208_155739_heart_t03.csv",
            # "path": "experiments/20251215_182022_heart_t01.csv",
            "path": PATH_ROOT + "163508_heart_t01.csv",
            "com": [-0.04354, 0, 0.098],
            "mass": 0.236, # 0.269
            "theta_star": 0.0, # placeholder
            "height": 0.2,
            "est": [0,0,0],
        },
        "flashlight": {
            # "path": "experiments/20251208_143007_flashlight_t01.csv",  # older test
            # "path": "experiments/20251208_155739_flashlight_t04.csv",    # new force stop test
            # "path": "experiments/20251215_182909_flashlight_t03.csv",
            "path": PATH_ROOT + "163955_flashlight_t01.csv",
            "com": [-0.0230, 0.0, 0.09656], # [-0.0250, 0, 0.0950]
            "mass": 0.387,
            "theta_star": 0.0, # placeholder
            "height": 0.2,
            "est": [0,0,0],
        },
        "lshape": {
            # "path": "experiments/20251208_155739_lshape_t05.csv",
            # "path": "experiments/20251215_182315_lshape_t01.csv",
            "path": PATH_ROOT + "163653_lshape_t01.csv",
            "com": [-0.00881, 0.0, 0.05763], # [-0.0250, 0, 0.0887]
            "mass": 0.106, # 0.118
            "theta_star": 0.0, # placeholder
            "height": 0.1492,
            "est": [0,0,0],
        },
        "monitor": {
            # "path": "experiments/20251210_182429_monitor_t03.csv",
            # "path": "experiments/20251215_183134_monitor_t01.csv",
            # "path": "experiments/20251215_183134_monitor_t02.csv",
            "path": PATH_ROOT + "164236_monitor_t01.csv",
            "com": [-0.06207, 0.0, 0.2516], # [-0.0781, 0, 0.2362]
            "mass": 5.008,
            "theta_star": 0.0, # placeholder
            "height": 0.49828,
            "est": [0,0,0],
        },
        "soda": {
            "path": "experiments/20251215_184521_soda_t01.csv",
            "path": PATH_ROOT + "164541_soda_t01.csv",
            "com": [-0.0316, 0.0, 0.1151], # [-0.0431, 0, 0.133]
            "mass": 2.071,
            "theta_star": 0.0, # placeholder
            "height": 0.3022,
            "est": [0,0,0],
        },
    }

def read_csv(file_path, trim_rows=0):
    with open(file_path, mode ='r')as file:
        csvFile = csv.reader(file)
        csv_arr = []
        for lines in csvFile:
                csv_arr.append(lines)
        return np.array(csv_arr[trim_rows:], dtype=float)
    
def plot_3vec(ax, indep, vec, label=None, linewidth=3, linestyle='-', color_order=['b', 'r', 'm'], draw_axes=[0, 1, 2]):
    if label is None:
        label = ['_', '_', '_']
    else:
        label = [f'X {label}', f'Y {label}', f'Z {label}']

    for axis in draw_axes:
        ax.plot(indep, vec[:, axis], color_order[axis], linewidth=linewidth, linestyle=linestyle, label=label[axis])
    return ax

def set_fig_opts(ax, xlabel, ylabel, ax2=None, ylabel2=None):
    ax.tick_params(axis='x', labelsize=20, labelcolor='g')
    ax.tick_params(axis='y', labelsize=20, labelcolor='b')
    ax.set_xlabel(xlabel, fontsize=20, color='g')
    ax.set_ylabel(ylabel, fontsize=20, color='b')
    ax.grid(True)
    ax.axhline(0, color='c', linewidth=2) # Horizontal line at zero for reference
    if ax2 is not None:
        ax2.tick_params(axis='y', labelsize=20, labelcolor='k')
        ax2.set_ylabel(ylabel2, fontsize=20, color='k')
        align_zeros([ax, ax2])
        return ax, ax2
    return ax


def main(shape, csv_path, com_gt, m_gt, theta_star_gt, plot_raw=True, plot_ee=False, ax_raw=None, ax_tau=None):
    ## ================ Extract time series data ===================
    csv_data = read_csv(csv_path, trim_rows=1)  # Discard headers
    time         = csv_data[:, 0]
    f_exp_raw    = csv_data[:, 1:4]
    ee_exp       = csv_data[:, 7:10]
    rvec_exp_raw = csv_data[:, 16:20]
    cnt_exp      = csv_data[:, -2].astype(int)
    trig_exp     = csv_data[:, -1].astype(int)
    ## ================ Pre-process time, tag, and force data ===================
    time -= time[0]  # Normalize time to start at zero
    f_exp_raw = f_exp_raw[:, [2, 0, 1]] # X ← Z, Y ← X, Z ← Y (to match base frame)
    # Set object origin in the table/robot/base frame
    o_obj = np.array([0.47065 + 0.081 + 0.114, 0, 0]) # Tool flange pos (NEW from FlexPendent + FT + Finger) (0.66m measured, real close)
    print(f"\nUsing o_obj = {o_obj} for analysis")

    # Convert tag quaternions to rotation vector representation
    Q = enforce_quat_continuity(rvec_exp_raw)
    r_all = R.from_quat(Q)
    r_start = r_all[0:10].mean()
    # r_rel = r_start.inv() * r_all
    r_rel = r_all * r_start.inv()
    rot_vecs_cam = r_rel.as_rotvec() # rot vec magnitudes are angles in radians, unit direction is axis
    # Rotate camera to world frame
    R_cam_to_world = R.from_euler('x', -90, degrees=True)  # Rotate -90 deg about X axis
    rot_vecs_world = R_cam_to_world.apply(rot_vecs_cam)
    # Ignore out-of-plane (Z) rotations for tipping analysis
    rot_vecs_world[:, 2] = 0.0

    ## ================ Filter Force and Tag Data ===================
    # Butterworth filter
    b, a        = butter(4, 5, fs=500, btype='low') # 4,5,500 : order, cutoff freq (<0.5*fs), sampling freq
    f_exp_med   = medfilt(f_exp_raw, kernel_size=(5,1))
    k_sg        = 89
    f_exp_filt  = savgol_filter(f_exp_med, k_sg, polyorder=3, axis=0)

    f_exp_filt = filtfilt(b, a, f_exp_med, axis=0)
    # Now we can also filter the tag data
    rot_vecs_filt = filtfilt(b, a, rot_vecs_world, axis=0)
    # rot_vecs_filt = savgol_filter(rot_vecs_world, 31, polyorder=3, axis=0)
    theta_exp = np.linalg.norm(rot_vecs_filt, axis=1)

    theta_safe = theta_exp.copy()
    theta_safe[theta_safe < 1e-6] = 1.0 # prevent div by zero
    axes_exp_raw = rot_vecs_filt / theta_safe[:, None] # "Tipping Axis" unit vectors

    # noise_mask = theta_exp < np.deg2rad(0.5) # To prevent chatter in unit axis when angle is small
    # axes_exp_raw[noise_mask, :] = 0.0
    # theta_exp[noise_mask] = 0.0
    

    # ================ TRIM DATA TO ANALYSIS WINDOW ===================
    # Get contact and settle indexes from experiment
    try:
        contact_idx_og  = np.where(cnt_exp == 1)[0][0]  # first index where contact boolean is true
        settle_idx_og   = np.where(trig_exp == 1)[0][0]
    except IndexError as e:
        print(f"Error: Could not find contact or settle indexes in experiment data. Please check the CSV file tags: {e}")
    # TEMP: Fix the incorrect contact and settle indexes for box shape... TODO: fix this in the experiment...
    if shape == "box": # settle_idx_og <= contact_idx_og
        contact_idx_og = 2350
        settle_idx_og = contact_idx_og + 1750
    elif shape == "heart":
        contact_idx_og = int(4.1 * 500)  # 4.1 seconds
        settle_idx_og = int(6.4 * 500)  # 6.4 seconds
    elif shape == "lshape":
        contact_idx_og = int(1.2 * 500)  # 3.2 seconds
        settle_idx_og = int(3.5 * 500)  # 5.5 seconds
    elif shape == "monitor":
        contact_idx_og = int(3.5 * 500)  # 3.5 seconds
        settle_idx_og = contact_idx_og + int(4 * 500) # 4 seconds after contact
    
    # Get corresponding times
    contact_time_og = time[contact_idx_og]
    settle_time_og  = time[settle_idx_og]

    time_trim   = np.array(time[contact_idx_og:settle_idx_og])
    f_trim      = np.array(f_exp_filt[contact_idx_og:settle_idx_og, :])
    th_trim     = np.array(theta_exp[contact_idx_og:settle_idx_og])
    ee_trim     = np.array(ee_exp[contact_idx_og:settle_idx_og, :])
    TIP_AXIS    = np.array(axes_exp_raw[contact_idx_og:settle_idx_og, :]).mean(axis=0)

    print(f"\nContact at index {contact_idx_og} ({contact_time_og:.2f} s)")
    print(f"Settles at index {settle_idx_og} ({settle_time_og:.2f} s)")
    print(f"Primary tipping axis is {TIP_AXIS.round(3)} (avg over window)")

    print("\nUSING HARDCODED TIPPING AXIS INSTEAD: [0, 1, 0]\n")
    TIP_AXIS = np.array([0, 1, 0])

## ================ Plot raw data ===================
    if plot_raw:
        # fig, ax = plt.subplots(figsize=(8,4.5))
        ax = ax_raw
        ax = plot_3vec(ax, time, f_exp_raw, label='Force (raw)', linewidth=2, draw_axes=[0,2])
        ax = plot_3vec(ax, time, f_exp_filt, label=None, linewidth=2, color_order=['k', 'k', 'k'])
        ax.axvline(contact_time_og, color='k', linestyle='--', linewidth=2, label='Contact & Settle')
        ax.axvline(settle_time_og, color='k', linestyle='--', linewidth=2, label='_')
        tau_exp = tau_app_model(-f_exp_raw, ee_exp - o_obj).reshape(-1,3)
        ax = plot_3vec(ax, time, tau_exp, label='App Torque (raw)', linewidth=2, color_order=['gray', 'gray', 'gray'], draw_axes=[1])
        tau_grav = tau_model(theta_exp, m_gt, com_gt[2], rc0_known=np.array([com_gt[0], com_gt[1], 0.0]), e_hat=TIP_AXIS).reshape(-1,3)
        ax = plot_3vec(ax, time, tau_grav, label='Grav Torque (gt)', linewidth=2, linestyle='--', color_order=['orange', 'orange', 'orange'], draw_axes=[1])

        ax2 = ax.twinx()
        # ax2 = plot_3vec(ax2, time, axes_exp_raw, label='Tip Axis', linewidth=3)
        ax2.plot(time, np.rad2deg(theta_exp), color='g', linestyle='-', linewidth=3, label='Object angle (raw)')
        # ax.set_title(f"{shape.upper()}", fontsize=15)
        fig.legend(loc='upper left', fontsize=15)
        # FOR PAPER FIGURE, TRIM THE AXES LIMITS
        # ax.set_xlim(1.6, 6)
        f_max = abs(f_trim[0,0])
        ax.set_ylim(-f_max*1.1, f_max*1.1)
        ax, ax2 = set_fig_opts(ax, 'Time (s)', 'Force (N)', ax2, 'Angle (deg)')

    if plot_ee:
        # fig, ax = plt.subplots(figsize=(8,4.5))
        ax = ax_raw
        ax = plot_3vec(ax, time, ee_exp, label='EE Pos. (raw)', linewidth=3)
        ax.axvline(contact_time_og, color='k', linestyle='--', linewidth=2, label='_')
        ax.axvline(settle_time_og, color='k', linestyle='--', linewidth=2, label='_')
        ax2 = plt.twinx()
        ax2.plot(time, np.rad2deg(theta_exp), color='g', linestyle='-', linewidth=3, label='Object angle (raw)')
        ax, ax2 = set_fig_opts(ax, 'Time (s)', 'EE Position (m)', ax2, 'Angle (deg)')
        # fig.legend(loc='upper left', fontsize=15)
        # FOR PAPER FIGURE, TRIM THE AXES LIMITS
        # ax.set_xlim(1.6, 6)
        # ax2.set_ylim(0, 30)

    ## =================== HACK: Modify experiment parameters to make better fitting ===================
    # Amplify force a bit since we lose some due to friction in the FT sensor and robot
    # FRICTION_LOSS_FACTOR = 1.5 # 1.15
    # f_trim *= FRICTION_LOSS_FACTOR
    
    ## ================== CALCULATE THETA* AND ZC USING LINEAR FIT ==================
    # Fit straight line to force data to find zero crossing
    try:
        lin_slope, lin_b, _, _, _ = linregress(th_trim, f_trim[:, 0])
    except Exception as e:
        print(f"Linear regression failed: {e}")
        return 0, 0, 0
    theta_star_calc = -lin_b / lin_slope
    zc_calc = abs(abs(com_gt[0]) / np.tan(theta_star_calc))
    m_calc = abs(abs(lin_slope)*ee_trim[-1,2]/(9.81*zc_calc))

    print(f"\nRecall init guess from linear fit:")
    print(f"mass: {m_calc:.3f} kg (GT: {m_gt:.3f} kg)")
    print(f"zc: {zc_calc:.3f} m (GT: {com_gt[2]:.3f} m)")
    print(f"theta*: {np.rad2deg(theta_star_calc):.2f} deg (GT: {np.rad2deg(theta_star_gt):.2f} deg)")
    
    ## =============== Fit using TAU model ==================
    rc0_known       = np.array([com_gt[0], com_gt[1], 0.0])
    rf              = (ee_trim - o_obj)

    # TEMP HACK: Amplify rf a bit to account for friction losses in torque fitting
    # rf *= 2
    # print(f"Lever arms: {rf[:10]} (first 10 samples)")

    # Before fitting, must pre-compute corresponding PUSH torque
    f_app = -f_trim # NOTE: IMPORTANT NEGATE TO MATCH F OBJECT EXPERIENCES

    # TEMP HACK: FRICTION IN Z COMPONENT IS PARASITIC FOR TORQUE, SO ZERO IT OUT FOR NOW...
    # f_app[:, 1] = 0.0
    # f_app[:, 2] = 0.0
    tau_app_trim    = tau_app_model(f_app, rf)

    try:
        [m_est, zc_est], pcov = curve_fit(
            lambda th, m, zc: tau_model(th, m, zc, rc0_known=rc0_known, e_hat=TIP_AXIS),
            th_trim,
            tau_app_trim,
            p0=[m_calc, zc_calc],
            bounds=([0, 0],
                    [np.inf, np.inf])
            )
    except:
        print("Curve fitting failed, using calculated values from linear fit.")
        m_est = m_calc
        zc_est = zc_calc

    
    # TESTING: Fit only for zc, assume known mass
    # zc_est, _ = curve_fit(
    #     lambda th, zc: tau_model(th, m_gt, zc, rc0_known=rc0_known, e_hat=TIP_AXIS),
    #     th_trim,
    #     tau_app_trim,
    #     p0=zc_calc,
    #     bounds=(0, np.inf)
    #     )
    # zc_est = zc_est[0]
    # m_est = m_gt  # Use known mass for now

    # TESTING: Fit only for mass, assume known zc
    # m_est, _ = curve_fit(
    #     lambda th, m: tau_model(th, m, com_gt[2], rc0_known=rc0_known, e_hat=TIP_AXIS),
    #     th_trim,
    #     tau_app_trim,
    #     p0=m_calc,
    #     bounds=(0, np.inf)
    #     )
    # zc_est = com_gt[2] # Use known zc for now
    # m_est = m_est[0]

    # TEMP Print the lever or moment arms to see why our torque is off
    # rf_norms = np.linalg.norm(rf, axis=1)
    # print(f"Lever arm norms during experiment: {rf_norms[:10]}")

    # Now use fitted parameters to estimate theta_star
    theta_star_est = np.arctan2(np.linalg.norm(com_gt[0:1]), zc_est) # atan2( d (xy norm), z) 

    ## ================== PLOTTING THE TORQUE DIRECTLY ===================
    # th_extrap_est = np.linspace(0, theta_star_est, len(ee_trim))
    th_extrap_est = np.linspace(0, theta_star_gt*1.1, len(ee_trim))

    # fig, ax = plt.subplots(figsize=(8,4.5))
    ax = ax_tau
    # Plot the original data
    ax = plot_3vec(ax, np.rad2deg(th_trim), tau_app_trim.reshape(-1,3), label='Applied Torque', linewidth=3, color_order=['gray', 'k', 'gray'], draw_axes=[1])
    # Plot using the ground truth params for reference
    tau_model_full_gt = tau_model(th_extrap_est, m_gt, com_gt[2], rc0_known=rc0_known, e_hat=[0, 1, 0]).reshape(-1,3)
    ax = plot_3vec(ax, np.rad2deg(th_extrap_est), tau_model_full_gt, label='Full T Fit (gt)', linewidth=3, draw_axes=[1])
    # Plot our model to full extrapolated theta range
    tau_model_full_est = tau_model(th_extrap_est, m_est, zc_est, rc0_known=rc0_known, e_hat=TIP_AXIS).reshape(-1,3)
    ax = plot_3vec(ax, np.rad2deg(th_extrap_est), tau_model_full_est, label='Full T Fit (est)', linewidth=3, linestyle='--', draw_axes=[1])
    # Let's also plot the 'rf' to see the lever arm change as the object rotates/tips
    # ax2 = plt.twinx()
    # ax2.plot(np.rad2deg(th_trim), np.linalg.norm(rf, axis=1), color='orange', linestyle='-', linewidth=2, label='Lever arm norm')

    # Scatter the Estimated and Ground Truth theta*
    ax.axvline(np.rad2deg(theta_star_gt), color='g', linestyle='--', linewidth=5, label=r'Ground truth $\theta^*$')
    ax.scatter(np.rad2deg(theta_star_est), 0, s=500, marker='*', color='m', label=r'Estimated $\theta^*$', zorder=2)
    ax = set_fig_opts(ax, "Object Angle (degrees)", "Applied Torque (N-m)") #, ax2, "Lever Arm Norm (m)")
    ax.legend(loc='upper right', fontsize=15)


    ## ================== PLOTTING THE FIT RESULTS ===================
    PLOT_FORCE = False

    if PLOT_FORCE:
        # Plot our model using fitted params ONLY for the experienced data range
        f_app_model_est = F_model(th_trim, m_est, zc_est, rf, rc0_known=rc0_known, e_hat=TIP_AXIS)
        # Extrapolate experienced data to mimic a full toppling experiment for plotting
        f_app_full_est = F_model(th_extrap_est, m_est, zc_est, rf, rc0_known=rc0_known, e_hat=TIP_AXIS)
        f_app_full_gt  = F_model(th_extrap_est, m_gt, com_gt[2], rf, rc0_known=rc0_known, e_hat=TIP_AXIS)

        ## IMPORTANT: must negate the 'experienced force' back again!
        f_app_model_est *= -1
        f_app_full_est  *= -1
        f_app_full_gt   *= -1

        f_plot_exp = f_trim[:,0]
        f_plot_model_est = f_app_model_est[:,0]
        f_plot_full_est = f_app_full_est[:,0]
        f_plot_full_gt = f_app_full_gt[:,0]

        # Plot the linear fit using y = mx + b
        th_extrap_calc = np.linspace(0, theta_star_calc, len(ee_trim))
        f_plot_lin_calc = lin_slope * th_extrap_calc + lin_b


        ## ============================ PLOTTING ============================
        # fig, ax = plt.subplots(figsize=(8,4.5))
        ax = ax_tau
        # Plot the original data
        ax.plot(np.rad2deg(th_trim), f_plot_exp, 'k', linewidth=5, label='Original data')  # Plot F norm
        # Plot linear fit using theta_star_calc from prior linear fit
        ax.plot(np.rad2deg(th_extrap_calc), f_plot_lin_calc, color='r', linestyle='--', linewidth=3, label='Linear fit')
        # Plot our model using fitted params for the experienced data range
        ax.plot(np.rad2deg(th_trim), f_plot_model_est, color='b', linestyle='-', linewidth=3, label='Model fit (experienced)')
        ## FOR FUN, plot ALL theta and force
        ax.scatter(np.rad2deg(th_extrap_est), f_plot_full_est, color='m', label='Full fit (est)')
        ax.scatter(np.rad2deg(th_extrap_est), f_plot_full_gt, color='c', label='Full fit (gt)')
        # Scatter the Estimated, Linear (calc), and Ground Truth theta*
        ax.axvline(np.rad2deg(theta_star_gt), color='g', linestyle='--', linewidth=5, label=r'Ground truth $\theta^*$')
        ax.scatter(np.rad2deg(theta_star_calc), 0, s=500, marker='*', color='r', label=r'Linear Guess $\theta^*$', zorder=2)
        ax.scatter(np.rad2deg(theta_star_est), 0, s=500, marker='*', color='m', label=r'Estimated $\theta^*$', zorder=2)
        # TEMP HACK: Plot all 3 force axes for reference
        ax.plot(np.rad2deg(th_trim), f_app_full_est[:,1], 'r', linewidth=1, label='Y Force (trimmed)')
        ax.plot(np.rad2deg(th_trim), f_app_full_est[:,2], 'm', linewidth=1, label='Z Force (trimmed)')

        ax = set_fig_opts(ax, "Object Angle (degrees)", 'X-Force (N)')
        ax.legend(loc='lower right', fontsize=15)


    # ================ Final printout of results ==================
    print(f"\nFitted results:")
    print(f"mass: {m_est:.3f} kg (GT: {m_gt:.3f} kg)")
    print(f"zc: {zc_est:.3f} m (GT: {com_gt[2]:.3f} m)")
    print(f"theta*: {np.rad2deg(theta_star_est):.2f} deg (GT: {np.rad2deg(theta_star_gt):.2f} deg)")
    print(f"\n^^^^^^^^ END Analysis on Shape: {shape.upper()}^^^^^^^^\n")

    return m_est, zc_est, theta_star_est


if __name__ == "__main__":
    shapes_to_run = ["box", "heart", "flashlight", "monitor", "soda"]#, "lshape"]

    # --- Storage for Summary Plot ---
    summary_data = {
        "names": [],
        "mass_gt": [], "mass_est": [], "mass_err_pct": [],
        "zc_gt": [], "zc_est": [], "zc_err_pct": [],
        "th_gt": [], "th_est": [], "th_err_pct": []
    }
    
    for i, shape in enumerate(shapes_to_run):
        obj     = OBJECTS[shape]
        path    = obj["path"]
        com_gt  = obj["com"]
        m_gt    = obj["mass"]
        # Ground truth from geometry
        theta_star_gt = np.arctan2(abs(np.linalg.norm(com_gt[0:1])), com_gt[2])
        OBJECTS[shape]["theta_star_gt"] = theta_star_gt

        print(f"\n========= Analyzing object: {shape.upper()} =========")
        print(f"Ground truth from geometry:\ntheta*: {np.rad2deg(theta_star_gt):.2f} deg, zc = {com_gt[2]:.3f} m")

        fig, axs = plt.subplots(2, 1, figsize=(10, 12))
        fig.suptitle(f"Analysis Result: {shape.upper()}", fontsize=20)
        ax_raw, ax_tau = axs[0], axs[1]

        m_est, zc_est, theta_star_est = main(
            shape, path, com_gt, m_gt, theta_star_gt, 
            plot_raw=True, plot_ee=False, ax_raw=ax_raw, ax_tau=ax_tau
        )

        obj["est"] = [m_est, zc_est, theta_star_est]

        plt.tight_layout()

    print("\n\n========= FINAL ESTIMATES ========= ")
    for shape in shapes_to_run:
        obj             = OBJECTS[shape]
        com_gt          = obj["com"]
        m_gt            = obj["mass"]
        theta_star_gt   = obj["theta_star_gt"]
        height          = obj["height"]
        m_est, zc_est, theta_star_est = obj["est"]
        
        # Error metrics
        m_err = m_est - m_gt
        m_err_pct = abs(m_err)/m_gt * 100
        zc_err = zc_est - com_gt[2]
        # zc_err_pct = abs(zc_err)/height * 100
        zc_err_pct = abs(zc_err)/com_gt[2] * 100
        
        th_deg_est = np.rad2deg(theta_star_est)
        th_deg_gt  = np.rad2deg(theta_star_gt)
        theta_star_err = th_deg_est - th_deg_gt
        theta_star_pct = abs(theta_star_err)/th_deg_gt * 100

        print(f"{shape.upper()}:")
        print(f"Estimated: mass = {m_est:.3f} kg, zc = {zc_est:.3f} m, theta* = {th_deg_est:.2f} deg")
        print(f"Gnd Truth: mass = {m_gt:.3f} kg, zc = {com_gt[2]:.3f} m, theta* = {th_deg_gt:.2f} deg")
        print(f"   Errors: mass = {m_err:.3f} kg, zc = {zc_err:.3f} m, theta* = {theta_star_err:.2f} deg")
        print(f"  Percent: mass = {m_err_pct:.3f} %,  zc = {zc_err_pct:.3f} %,  theta* = {theta_star_pct:.3f} %\n")

        # --- Collect Data for Summary ---
        summary_data["names"].append(shape.upper())
        summary_data["mass_gt"].append(m_gt)
        summary_data["mass_est"].append(m_est)
        summary_data["mass_err_pct"].append(m_err_pct)
        summary_data["zc_gt"].append(com_gt[2])
        summary_data["zc_est"].append(zc_est)
        summary_data["zc_err_pct"].append(zc_err_pct)
        summary_data["th_gt"].append(th_deg_gt)
        summary_data["th_est"].append(th_deg_est)
        summary_data["th_err_pct"].append(theta_star_pct)

    # =========================================================================
    # ========================== SUMMARY DASHBOARD ============================
    # =========================================================================
    fig_sum = plt.figure(figsize=(14, 10))
    gs = fig_sum.add_gridspec(2, 2)
    fig_sum.suptitle("Estimation Performance Summary", fontsize=20, fontweight='bold')

    # --- 1. Grouped Bar Chart of Errors ---
    ax1 = fig_sum.add_subplot(gs[0, :])
    x = np.arange(len(summary_data["names"]))
    width = 0.25
    
    ax1.bar(x - width, summary_data["mass_err_pct"], width, label='Mass Error %', color='#1f77b4')
    ax1.bar(x, summary_data["zc_err_pct"], width, label='Zc Error %', color='#ff7f0e')
    # ax1.bar(x + width, summary_data["th_err_pct"], width, label='Theta* Error %', color='#2ca02c')

    ax1.set_ylabel('Absolute Error (%)', fontsize=14)
    ax1.set_title('Accuracy by Object', fontsize=16)
    ax1.set_xticks(x)
    ax1.set_xticklabels(summary_data["names"], fontsize=12, fontweight='bold')
    ax1.legend(fontsize=12)
    ax1.grid(axis='y', linestyle='--', alpha=0.7)
    # ax1.set_ylim(0, np.inf)  # Cap at 50% for readability

    # --- 2. Mass Correlation Scatter ---
    ax2 = fig_sum.add_subplot(gs[1, 0])
    sc1 = ax2.scatter(summary_data["mass_gt"], summary_data["mass_est"], s=200, c=x, cmap='viridis', zorder=3)
    # Identity line
    max_mass = max(max(summary_data["mass_gt"]), max(summary_data["mass_est"])) * 1.1
    ax2.plot([0, max_mass], [0, max_mass], 'k--', alpha=0.5, label='Perfect Fit')
    
    # Annotate points
    for i, txt in enumerate(summary_data["names"]):
        ax2.annotate(txt, (summary_data["mass_gt"][i], summary_data["mass_est"][i]), 
                     xytext=(5, 5), textcoords='offset points')
    
    ax2.set_xlabel('Ground Truth Mass (kg)', fontsize=14)
    ax2.set_ylabel('Estimated Mass (kg)', fontsize=14)
    ax2.set_title('Mass Estimation Consistency', fontsize=16)
    ax2.grid(True)
    ax2.legend()

    
    from matplotlib.gridspec import GridSpecFromSubplotSpec

    # --- 3. Results Tables (bottom-right) ---
    # Create a nested gridspec inside the bottom-right cell
    sub = GridSpecFromSubplotSpec(
        2, 1, subplot_spec=gs[1, 1],
        height_ratios=[1, 1], hspace=0.25
    )

    ax3a = fig_sum.add_subplot(sub[0, 0])
    ax3b = fig_sum.add_subplot(sub[1, 0])
    for ax in (ax3a, ax3b):
        ax.axis("off")

    ax3a.set_title("Mass Results", fontsize=14)
    ax3b.set_title("Zc Results", fontsize=14)

    # ---- compute absolute errors (signed and absolute if you want) ----
    mass_err_kg = [summary_data["mass_est"][i] - summary_data["mass_gt"][i] for i in range(len(summary_data["names"]))]
    zc_err_m    = [summary_data["zc_est"][i]   - summary_data["zc_gt"][i]   for i in range(len(summary_data["names"]))]

    # ===================== MASS TABLE =====================
    mass_col_labels = ["Object", "mass gt (g)", "mass est (g)", "error (g)", "error (%)"]
    mass_cell_text = []
    for i, name in enumerate(summary_data["names"]):
        mass_cell_text.append([
            name,
            f'{summary_data["mass_gt"][i]*1000:.2f}',
            f'{summary_data["mass_est"][i]*1000:.2f}',
            f'{mass_err_kg[i]*1000:.2f}',
            f'{summary_data["mass_err_pct"][i]:.1f}',
        ])

    tbl_m = ax3a.table(
        cellText=mass_cell_text,
        colLabels=mass_col_labels,
        loc="center",
        cellLoc="center",
        colLoc="center"
    )
    tbl_m.auto_set_font_size(False)
    tbl_m.set_fontsize(10.5)
    tbl_m.scale(1.15, 1.35)

    # ===================== ZC TABLE =====================
    zc_col_labels = ["Object", "CoM Z gt (mm)", "CoM Z est (mm)", "error (mm)", "error (%)"]
    zc_cell_text = []
    for i, name in enumerate(summary_data["names"]):
        zc_cell_text.append([
            name,
            f'{summary_data["zc_gt"][i]*1000:.2f}',
            f'{summary_data["zc_est"][i]*1000:.2f}',
            f'{zc_err_m[i]*1000:.2f}',
            f'{summary_data["zc_err_pct"][i]:.1f}',
        ])

    tbl_z = ax3b.table(
        cellText=zc_cell_text,
        colLabels=zc_col_labels,
        loc="center",
        cellLoc="center",
        colLoc="center"
    )
    tbl_z.auto_set_font_size(False)
    tbl_z.set_fontsize(10.5)
    tbl_z.scale(1.15, 1.35)

    # Bold header rows (both tables)
    for tbl in (tbl_m, tbl_z):
        for (r, c), cell in tbl.get_celld().items():
            if r == 0:
                cell.set_text_props(weight="bold")


    # --- 3. Angle Correlation Scatter ---
    # ax3 = fig_sum.add_subplot(gs[1, 1])
    # sc2 = ax3.scatter(summary_data["th_gt"], summary_data["th_est"], s=200, c=x, cmap='viridis', zorder=3)
    # max_th = max(max(summary_data["th_gt"]), max(summary_data["th_est"])) * 1.1
    # ax3.plot([0, max_th], [0, max_th], 'k--', alpha=0.5, label='Perfect Fit')

    # for i, txt in enumerate(summary_data["names"]):
    #     ax3.annotate(txt, (summary_data["th_gt"][i], summary_data["th_est"][i]), 
    #                  xytext=(5, -10), textcoords='offset points')

    # ax3.set_xlabel('Ground Truth Theta* (deg)', fontsize=14)
    # ax3.set_ylabel('Estimated Theta* (deg)', fontsize=14)
    # ax3.set_title('Tipping Angle Consistency', fontsize=16)
    # ax3.grid(True)

    plt.tight_layout(rect=[0, 0.0, 1, 0.96])
    plt.show()