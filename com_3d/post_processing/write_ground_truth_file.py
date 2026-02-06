import json
import numpy as np

# Define ground truth data as variables so we can calculate theta_star_deg
box_zc = 0.14624 # 0.14624 # 0.15 onshape
box_mass = 0.658 # 0.664 (was 0.686 onshape)
box_rc0 = 0.0462 # 0.04515 # 0.050 onshape

heart_zc = 0.098 # 0.1 # 0.098
heart_mass = 0.219 # 0.236 (was 0.269 onshape)
heart_rc0 = 0.0436 # 0.04582 # 0.04354

monitor_zc = 0.2320 # 0.236 # 0.2316 # 0.2516 (was 0.236 onshape)
monitor_mass = 5.04 # 5.008 # Empirical
monitor_rc0 = 0.06107 # 0.078 # 0.06107

flashlight_zc = 0.09656 # 0.09656
flashlight_mass = 0.336 # 0.387 # Empirical
flashlight_rc0 = 0.0261 # 0.0281 # 0.025 # 0.023

ground_truth = {
    "box": {
        "m_kg": box_mass,
        "zc_m": box_zc,
        "rc0_m": [-box_rc0, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(box_rc0, box_zc)) # 17.158
    },
    "heart": {
        "m_kg": heart_mass,
        "zc_m": heart_zc,
        "rc0_m": [-heart_rc0, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(heart_rc0, heart_zc)) # 23.987
    },
    "monitor": {
        "m_kg": monitor_mass,
        "zc_m": monitor_zc,
        "rc0_m": [-monitor_rc0, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(monitor_rc0, monitor_zc)) # 14.777
    },
    "flashlight": {
        "m_kg": flashlight_mass,
        "zc_m": flashlight_zc,
        "rc0_m": [-flashlight_rc0, 0, 0],
        "theta_star_deg": np.rad2deg(np.arctan2(flashlight_rc0, flashlight_zc)) # 13.407
    }
}

output_file = "ground_truth.json"
with open(output_file, "w") as f:
    json.dump(ground_truth, f, indent=2)

print(f"Ground truth data written to {output_file}")