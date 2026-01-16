import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

# =========================
# Path to CSV file (EDIT ME) - for single file plots
# =========================
CSV_PATH = "/home/aa274/asl_tb3_aiet/results/drift_test_vel_0.5_ang_0.0_t_3.0_iter_3.csv"

# Directory containing all result files - for multi-file analysis
RESULTS_DIR = "/home/aa274/asl_tb3_aiet/results/"

# =========================
# Load data
# =========================
df = pd.read_csv(CSV_PATH)

# Ensure expected ordering of phases
phase_order = ["initial", "forward_end", "backward_end"]
df["phase"] = pd.Categorical(df["phase"], categories=phase_order, ordered=True)

# Sort for consistent plotting
df = df.sort_values(["iteration", "phase"])

# Unique iterations
iterations = sorted(df["iteration"].unique())

# Colormap for iterations
cmap = plt.cm.get_cmap("tab10", len(iterations))

# =========================
# Figure 1: Trajectory Plot
# =========================
plt.figure(figsize=(8, 8))

# Build complete trajectory: initial -> forward_end(1) -> backward_end(1) -> forward_end(2) -> ...
trajectory_x = []
trajectory_y = []
segment_types = []  # 'forward' or 'backward' for coloring arrows

# Start with ground truth (iteration 0, initial)
init_row = df[(df["iteration"] == 0) & (df["phase"] == "initial")]
if not init_row.empty:
    trajectory_x.append(init_row["pos_x"].iloc[0])
    trajectory_y.append(init_row["pos_y"].iloc[0])

# Add forward_end and backward_end for each iteration (starting from iteration 1)
for it in sorted(df["iteration"].unique()):
    if it == 0:
        continue
    it_df = df[df["iteration"] == it]

    fwd = it_df[it_df["phase"] == "forward_end"]
    if not fwd.empty:
        trajectory_x.append(fwd["pos_x"].iloc[0])
        trajectory_y.append(fwd["pos_y"].iloc[0])
        segment_types.append("forward")

    bwd = it_df[it_df["phase"] == "backward_end"]
    if not bwd.empty:
        trajectory_x.append(bwd["pos_x"].iloc[0])
        trajectory_y.append(bwd["pos_y"].iloc[0])
        segment_types.append("backward")

trajectory_x = np.array(trajectory_x)
trajectory_y = np.array(trajectory_y)

# Plot the full trajectory line
plt.plot(trajectory_x, trajectory_y, "-o", color="blue", alpha=0.5, label="Trajectory")

# Add arrows with different colors for forward (green) and backward (orange)
for j in range(len(trajectory_x) - 1):
    dx = trajectory_x[j + 1] - trajectory_x[j]
    dy = trajectory_y[j + 1] - trajectory_y[j]
    color = "green" if segment_types[j] == "forward" else "orange"
    plt.arrow(
        trajectory_x[j],
        trajectory_y[j],
        dx,
        dy,
        length_includes_head=True,
        head_width=0.02,
        head_length=0.03,
        fc=color,
        ec=color,
        alpha=0.8,
    )

# Mark the initial ground-truth position
if not init_row.empty:
    plt.plot(
        init_row["pos_x"].iloc[0],
        init_row["pos_y"].iloc[0],
        marker="*",
        color="red",
        markersize=15,
        zorder=5,
        label="Ground Truth (Initial)",
    )

# Add legend entries for arrow colors
plt.plot([], [], color="green", label="Forward")
plt.plot([], [], color="orange", label="Backward")

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Trajectory Over Multiple Forward/Backward Cycles")
plt.legend()
plt.axis("equal")
plt.grid(True)

# =========================
# Figure 2: Start/End Points
# =========================
plt.figure(figsize=(8, 8))

# Plot initial position
if not init_row.empty:
    plt.plot(
        init_row["pos_x"].iloc[0],
        init_row["pos_y"].iloc[0],
        marker="*",
        color="red",
        markersize=15,
        label="Initial (Ground Truth)",
    )

for i, it in enumerate(iterations):
    it_df = df[df["iteration"] == it]
    color = cmap(i)

    # Forward end
    fwd = it_df[it_df["phase"] == "forward_end"]
    if not fwd.empty:
        plt.plot(
            fwd["pos_x"].values,
            fwd["pos_y"].values,
            marker="o",
            linestyle="None",
            color=color,
            label=f"Iter {it} Forward",
        )
        for _, row in fwd.iterrows():
            plt.text(
                row["pos_x"] + 0.01,
                row["pos_y"] + 0.01,
                str(it),
                fontsize=9,
            )

    # Backward end
    bwd = it_df[it_df["phase"] == "backward_end"]
    if not bwd.empty:
        plt.plot(
            bwd["pos_x"].values,
            bwd["pos_y"].values,
            marker="s",
            linestyle="None",
            color=color,
            label=f"Iter {it} Backward",
        )
        for _, row in bwd.iterrows():
            plt.text(
                row["pos_x"] + 0.01,
                row["pos_y"] + 0.01,
                str(it),
                fontsize=9,
            )

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Position at End of Each Phase")
plt.legend()
plt.axis("equal")
plt.grid(True)

# =========================
# Figure 3: cmd_wz vs Total Drift (from all files)
# =========================

# Collect data from all CSV files
csv_files = glob.glob(os.path.join(RESULTS_DIR, "drift_test_*.csv"))

cmd_wz_values = []
total_drift_xy = []
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
    total_drift_xy.append(np.sqrt(drift_x**2 + drift_y**2))

# Sort by cmd_wz for proper line plotting
sort_idx = np.argsort(cmd_wz_values)
cmd_wz_values = np.array(cmd_wz_values)[sort_idx]
total_drift_xy = np.array(total_drift_xy)[sort_idx]
drift_x_values = np.array(drift_x_values)[sort_idx]
drift_y_values = np.array(drift_y_values)[sort_idx]

# Create subplots
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Plot 1: cmd_wz vs total XY drift
axes[0].plot(cmd_wz_values, total_drift_xy, "-o", color="blue", markersize=8)
axes[0].set_xlabel("cmd_wz (rad/s)")
axes[0].set_ylabel("Total XY Drift (m)")
axes[0].set_title("Angular Velocity vs Total Position Drift")
axes[0].grid(True)

# Plot 2: cmd_wz vs drift components
axes[1].plot(cmd_wz_values, drift_x_values, "-o", color="red", label="|Drift X|", markersize=8)
axes[1].plot(cmd_wz_values, drift_y_values, "-s", color="green", label="|Drift Y|", markersize=8)
axes[1].set_xlabel("cmd_wz (rad/s)")
axes[1].set_ylabel("Absolute Drift (m)")
axes[1].set_title("Angular Velocity vs Absolute Drift Components")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()

plt.show()
