import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

# =========================
# Directory containing all result files
# =========================
RESULTS_DIR = "/home/aa274/asl_tb3_aiet/results/"

# =========================
# cmd_wz vs Absolute Drift (from all files)
# =========================

# Collect data from all CSV files
csv_files = glob.glob(os.path.join(RESULTS_DIR, "drift_test_*.csv"))

cmd_wz_values = []
longitudinal_drift_values = []
lateral_drift_values = []

for csv_file in csv_files:
    file_df = pd.read_csv(csv_file)

    # All rows are now backward_end (drift measurements when robot returns)
    if file_df.empty:
        continue

    # Get the final iteration (represents drift after all round trips)
    max_iter = file_df["iteration"].max()
    final_row = file_df[file_df["iteration"] == max_iter]

    if final_row.empty:
        continue

    # Get cmd_wz (should be the same for all iterations in a file, but use absolute value)
    cmd_wz = abs(final_row["cmd_wz"].iloc[0])
    
    # Get drift values from final iteration (after all round trips complete)
    # This represents the total drift accumulated over all round trips
    longitudinal_drift = final_row["longitudinal_drift"].iloc[0]
    lateral_drift = final_row["lateral_drift"].iloc[0]

    cmd_wz_values.append(cmd_wz)
    longitudinal_drift_values.append(np.abs(longitudinal_drift))
    lateral_drift_values.append(np.abs(lateral_drift))

# Sort by cmd_wz for proper line plotting
sort_idx = np.argsort(cmd_wz_values)
cmd_wz_values = np.array(cmd_wz_values)[sort_idx]
longitudinal_drift_values = np.array(longitudinal_drift_values)[sort_idx]
lateral_drift_values = np.array(lateral_drift_values)[sort_idx]

# Plot: cmd_wz vs absolute drift components
plt.figure(figsize=(8, 6))
plt.plot(cmd_wz_values, longitudinal_drift_values, "-o", color="red", label="|Longitudinal Drift|", markersize=8)
plt.plot(cmd_wz_values, lateral_drift_values, "-s", color="green", label="|Lateral Drift|", markersize=8)
plt.xlabel("cmd_wz (rad/s)")
plt.ylabel("Absolute Drift (m)")
plt.title("Angular Velocity vs Absolute Drift")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
