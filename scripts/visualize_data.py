"""
High-quality visualization of robot drift vs angular velocity command.

Reads multiple drift test CSVs, extracts final accumulated drift,
and plots longitudinal, lateral, and total drift magnitude
as a function of angular velocity command.

Author: <your team name>
"""

import glob
import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def load_final_drift(csv_path):
    """
    Load a drift CSV and return the final iteration row.

    Parameters
    ----------
    csv_path : str
        Path to the CSV file.

    Returns
    -------
    pandas.Series
        Row corresponding to the maximum iteration.
    """
    df = pd.read_csv(csv_path)

    # Defensive programming: ensure expected columns exist
    required_cols = {
        "iteration", "cmd_wz",
        "longitudinal_drift", "lateral_drift"
    }
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path} missing columns: {missing}")

    final_iter = df["iteration"].max()
    return df.loc[df["iteration"] == final_iter].iloc[0]


def main():
    results_dir = "/home/aa274/asl_tb3_aiet/results"
    pattern = os.path.join(results_dir, "drift_test_*.csv")
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        raise FileNotFoundError(f"No CSV files found at {pattern}")

    records = []

    # Load data from each test run
    for csv_file in csv_files:
        final_row = load_final_drift(csv_file)

        long_drift = abs(final_row["longitudinal_drift"])
        lat_drift = abs(final_row["lateral_drift"])
        total_drift = np.hypot(long_drift, lat_drift)

        records.append({
            "cmd_wz": final_row["cmd_wz"],
            "longitudinal_drift": long_drift,
            "lateral_drift": lat_drift,
            "total_drift": total_drift,
        })

    # Build DataFrame and sort by angular velocity
    df = pd.DataFrame(records).sort_values("cmd_wz")

    # ---------------------- Plot Styling ----------------------
    plt.style.use("seaborn-v0_8-whitegrid")
    plt.rcParams.update({
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 14,
        "legend.fontsize": 10,
    })

    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(
        df["cmd_wz"],
        df["longitudinal_drift"],
        total_drift = np.hypot(long_drift, lat_drift)

        records.append({
            "cmd_wz": final_row["cmd_wz"],
            "longitudinal_drift": long_drift,
            "lateral_drift": lat_drift,
            "total_drift": total_drift,
        }))

    # Build DataFrame and sort by angular velocity
    df = pd.DataFrame(records).sort_values("cmd_wz")

    # ---------------------- Plot Styling ----------------------
    plt.style.use("seaborn-v0_8-whitegrid")
    plt.rcParams.update({
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 14,
        "legend.fontsize": 10,
    })

    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(
        df["cmd_wz"],
        df["longitudinal_drift"],
        marker="o",
        linewidth=2,
        label="Longitudinal Drift",
    )
    ax.plot(
        df["cmd_wz"],
        df["lateral_drift"],
        marker="s",
        linewidth=2,
        label="Lateral Drift",
    )
    ax.plot(
        df["cmd_wz"],
        df["total_drift"],
        marker="^",
        linestyle="--",
        linewidth=2,
        label="Total Drift Magnitude",
    )

    ax.set_xlabel("Angular Velocity Command $\\omega_z$ (rad/s)")
    ax.set_ylabel("Absolute Drift (m)")
    ax.set_title("Accumulated Robot Drift vs Angular Velocity Command")

    ax.legend()
    ax.grid(True, which="both", linestyle="--", alpha=0.6)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
        linewidth=2,
        label="Total Drift Magnitude",
    )

    ax.set_xlabel("Angular Velocity Command $\\omega_z$ (rad/s)")
    ax.set_ylabel("Absolute Drift (m)")
    ax.set_title("Accumulated Robot Drift vs Angular Velocity Command")

    ax.legend()
    ax.grid(True, which="both", linestyle="--", alpha=0.6)

    fig.tight_layout()
    plt.show()