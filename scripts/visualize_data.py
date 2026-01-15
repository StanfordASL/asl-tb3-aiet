import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# =========================
# Path to CSV file (EDIT ME)
# =========================
CSV_PATH = "/home/aa274/asl_tb3_aiet/results/drift_test_vel_0.5_ang_-0.3_t_3.0_iter_3.csv"

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

plt.show()
