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

for i, it in enumerate(iterations):
    it_df = df[df["iteration"] == it]

    if len(it_df) < 2:
        continue

    x = it_df["pos_x"].values
    y = it_df["pos_y"].values

    color = cmap(i)

    # Plot trajectory line
    plt.plot(x, y, "-o", color=color, label=f"Iteration {it}")

    # Add arrows to show direction
    for j in range(len(x) - 1):
        dx = x[j + 1] - x[j]
        dy = y[j + 1] - y[j]
        plt.arrow(
            x[j],
            y[j],
            dx,
            dy,
            length_includes_head=True,
            head_width=0.02,
            head_length=0.03,
            fc=color,
            ec=color,
            alpha=0.8,
        )

# Mark the initial ground-truth position (iteration 0, phase initial)
init_row = df[(df["iteration"] == 0) & (df["phase"] == "initial")]
if not init_row.empty:
    plt.plot(
        init_row["pos_x"].iloc[0],
        init_row["pos_y"].iloc[0],
        marker="*",
        color="red",
        markersize=15,
        label="Initial Position",
    )

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
            fwd["pos_x"],
            fwd["pos_y"],
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
            bwd["pos_x"],
            bwd["pos_y"],
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
