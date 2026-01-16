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
drift_x_values = []
drift_y_values = []

for csv_file in csv_files:
    file_df = pd.read_csv(csv_file)

    # Get the final backward_end row (last iteration)
    max_iter = file_df["iteration"].max()
    final_row = file_df[(file_df["iteration"] == max_iter) & (file_df["phase"] == "backward_end")]

    if final_row.empty:
        continue

    # Get cmd_wz from any forward_end row (they should all be the same for a file)
    fwd_rows = file_df[file_df["phase"] == "forward_end"]
    if fwd_rows.empty:
        continue

    cmd_wz = fwd_rows["cmd_wz"].iloc[0]
    drift_x = final_row["drift_x"].iloc[0]
    drift_y = final_row["drift_y"].iloc[0]

    cmd_wz_values.append(cmd_wz)
    drift_x_values.append(np.abs(drift_x))
    drift_y_values.append(np.abs(drift_y))

# Sort by cmd_wz for proper line plotting
sort_idx = np.argsort(cmd_wz_values)
cmd_wz_values = np.array(cmd_wz_values)[sort_idx]
drift_x_values = np.array(drift_x_values)[sort_idx]
drift_y_values = np.array(drift_y_values)[sort_idx]

# Plot: cmd_wz vs absolute drift components
plt.figure(figsize=(8, 6))
plt.plot(cmd_wz_values, drift_x_values, "-o", color="red", label="|Drift X|", markersize=8)
plt.plot(cmd_wz_values, drift_y_values, "-s", color="green", label="|Drift Y|", markersize=8)
plt.xlabel("cmd_wz (rad/s)")
plt.ylabel("Absolute Drift (m)")
plt.title("Angular Velocity vs Absolute Drift")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
