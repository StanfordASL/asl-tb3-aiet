#!/usr/bin/env python3
"""
Script to clean demonstration data by removing initial timesteps with zero control input.
Processes all demonstration folders in the driving_data directory.
"""

import json
import os
import glob
from pathlib import Path
import numpy as np


def has_nonzero_control(action):
    """
    Check if an action has non-zero control input.

    Args:
        action: Dictionary containing linear_velocity and angular_velocity

    Returns:
        True if any velocity component is non-zero
    """
    linear_vel = action.get('linear_velocity', {})
    angular_vel = action.get('angular_velocity', {})

    linear_vel_vec = np.array([linear_vel.get(axis, 0.0) for axis in ['x', 'y', 'z']])
    lin_vel_norm = np.linalg.norm(linear_vel_vec)
    angular_vel_vec = np.array([angular_vel.get(axis, 0.0) for axis in ['x', 'y', 'z']])
    angular_vel_norm = np.linalg.norm(angular_vel_vec)

    if lin_vel_norm > 1e-4 and angular_vel_norm > 1e-4:
        # We only want one velocity
        return False
    elif lin_vel_norm > 0.005:
        return True
    elif angular_vel_norm > 0.005:
        return True
    else:
        return False


def find_first_nonzero_index(actions):
    """
    Find the index of the first action with non-zero control input.

    Args:
        actions: List of action dictionaries

    Returns:
        Index of first non-zero action, or None if all are zero
    """
    for i, action in enumerate(actions):
        if has_nonzero_control(action):
            return i
    return None


def clean_demonstration(demo_path):
    """
    Clean a single demonstration by removing initial zero-control timesteps.

    Args:
        demo_path: Path to demonstration folder
    """
    demo_path = Path(demo_path)
    actions_file = demo_path / 'actions.json'
    images_dir = demo_path / 'images'

    if not actions_file.exists():
        print(f"Skipping {demo_path.name}: No actions.json found")
        return

    if not images_dir.exists():
        print(f"Skipping {demo_path.name}: No images directory found")
        return

    # Load actions
    with open(actions_file, 'r') as f:
        actions = json.load(f)

    if not actions:
        print(f"Skipping {demo_path.name}: Empty actions list")
        return
    
    cleaned_actions = []
    clean_id = 0
    for i, action in enumerate(actions):
        if not has_nonzero_control(action):
            # Delete frame
            image_path = images_dir / f'frame_{i:06d}.jpg'
            if image_path.exists():
                image_path.unlink()
        else:
            action['frame_id'] = clean_id
            cleaned_actions.append(action)
            clean_id += 1

    # Save updated actions.json
    with open(actions_file, 'w') as f:
        json.dump(cleaned_actions, f, indent=2)

    print(f"Number of clean images = {clean_id}")
    print(f"Deleted {len(actions)-(clean_id)} images")


def main():
    """
    Main function to process all demonstration folders.
    """
    # Path to driving data
    driving_data_dir = Path.home() / 'section_assets' / 'driving_data'

    if not driving_data_dir.exists():
        print(f"Error: Directory not found: {driving_data_dir}")
        return

    # Find all demonstration folders
    demo_folders = sorted([d for d in driving_data_dir.iterdir() if d.is_dir()])

    if not demo_folders:
        print(f"No demonstration folders found in {driving_data_dir}")
        return

    print(f"Found {len(demo_folders)} demonstration folders")
    print("=" * 60)

    # Process each demonstration
    for demo_folder in demo_folders:
        clean_demonstration(demo_folder)
        print()

    print("=" * 60)
    print("Done!")


if __name__ == '__main__':
    main()
