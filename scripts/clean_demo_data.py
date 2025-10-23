#!/usr/bin/env python3
"""
Script to clean demonstration data by removing initial timesteps with zero control input.
Processes all demonstration folders in the driving_data directory.
"""

import json
import os
import glob
from pathlib import Path


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

    # Check if any component is non-zero
    for vel in [linear_vel, angular_vel]:
        if any(abs(vel.get(axis, 0.0)) > 1e-10 for axis in ['x', 'y', 'z']):
            return True
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

    # Find first non-zero control input
    first_nonzero_idx = find_first_nonzero_index(actions)

    if first_nonzero_idx is None:
        print(f"Warning: {demo_path.name} has no non-zero control inputs")
        return

    if first_nonzero_idx == 0:
        print(f"Skipping {demo_path.name}: Already starts with non-zero control")
        return

    print(f"Processing {demo_path.name}: Removing first {first_nonzero_idx} timesteps")

    # Get frame IDs to delete
    frames_to_delete = [action['frame_id'] for action in actions[:first_nonzero_idx]]

    # Delete corresponding images
    deleted_images = 0
    for frame_id in frames_to_delete:
        image_path = images_dir / f'frame_{frame_id:06d}.jpg'
        if image_path.exists():
            image_path.unlink()
            deleted_images += 1

    # Keep only actions from first non-zero onwards
    cleaned_actions = actions[first_nonzero_idx:]

    # Re-index frame_ids to start from 0
    for i, action in enumerate(cleaned_actions):
        action['frame_id'] = i

    # Save updated actions.json
    with open(actions_file, 'w') as f:
        json.dump(cleaned_actions, f, indent=2)

    print(f"  ✓ Removed {first_nonzero_idx} actions")
    print(f"  ✓ Deleted {deleted_images} images")
    print(f"  ✓ Remaining: {len(cleaned_actions)} timesteps")


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
