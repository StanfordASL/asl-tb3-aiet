#!/usr/bin/env python3

# RUN with python3 analyze_velocity_battery.py "~/autonomy_ws/src/asl_tb3_aiet/results/*.csv"

import argparse
import glob
import os
import pandas as pd
import matplotlib.pyplot as plt


def read_csv_file(path):
    """Read a single CSV log file."""
    df = pd.read_csv(path)
    df['time_s'] = df['time'].astype(str).apply(lambda x: int(x.split(',')[0]))
    return df


def compute_statistics(df):
    """Compute tracking error and summary statistics."""
    df['tracking_error'] = abs(df['meas_vx'] - df['cmd_vx'])

    stats = {
        "mean_error": df['tracking_error'].mean(),
        "max_error": df['tracking_error'].max(),
        "mean_measured_vel": df['meas_vx'].mean(),
        "start_battery_pct": df['battery_percentage'].iloc[0] * 100,
        "end_battery_pct": df['battery_percentage'].iloc[-1] * 100,
    }

    return stats


def plot_single_trial(df, title):
    """Plot cmd vs measured velocity and tracking error."""
    t = df.index * 0.05  # assuming 20Hz logging → 0.05s step

    plt.figure(figsize=(10, 6))
    plt.plot(t, df['cmd_vx'], label="Commanded velocity")
    plt.plot(t, df['meas_vx'], label="Measured velocity")
    plt.title(f"Velocity Tracking: {title}")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.figure(figsize=(10, 4))
    plt.plot(t, df['tracking_error'])
    plt.title(f"Tracking Error: {title}")
    plt.xlabel("Time [s]")
    plt.ylabel("|cmd - measured| [m/s]")
    plt.grid(True)
    plt.tight_layout()


def plot_comparison(dfs, labels):
    """Overlay comparison of measured velocities for multiple trials."""
    plt.figure(figsize=(10, 6))

    for df, label in zip(dfs, labels):
        t = df.index * 0.05
        plt.plot(t, df['meas_vx'], label=label)

    plt.title("Comparison of Measured Velocity Across Trials")
    plt.xlabel("Time [s]")
    plt.ylabel("Measured Velocity [m/s]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(description="Analyze velocity/battery CSV logs.")
    parser.add_argument("pattern", help="CSV file or glob pattern (e.g. results/*.csv)")
    args = parser.parse_args()

    paths = sorted(glob.glob(args.pattern))
    if not paths:
        print("❌ No CSV files found. Check your pattern.")
        return

    print(f"Found {len(paths)} CSV file(s).")

    dfs = []
    labels = []

    for path in paths:
        print(f"\n📂 Processing: {path}")
        df = read_csv_file(path)
        df['tracking_error'] = abs(df['meas_vx'] - df['cmd_vx'])

        stats = compute_statistics(df)
        dfs.append(df)

        # Label includes battery + velocity from filename
        labels.append(os.path.basename(path))

        # Print stats
        print("   Mean tracking error: {:.4f} m/s".format(stats["mean_error"]))
        print("   Max tracking error:  {:.4f} m/s".format(stats["max_error"]))
        print("   Mean measured vel:   {:.4f} m/s".format(stats["mean_measured_vel"]))
        print("   Battery start/end:   {:.1f}% → {:.1f}%".format(
            stats["start_battery_pct"], stats["end_battery_pct"]
        ))

        # Plot per-trial results
        plot_single_trial(df, os.path.basename(path))

    # Plot comparison if multiple files
    if len(dfs) > 1:
        plot_comparison(dfs, labels)

    plt.show()


if __name__ == "__main__":
    main()
