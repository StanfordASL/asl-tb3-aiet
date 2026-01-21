#############################
#############################
## TODO: Given the collect data from path /home/aa274/asl_tb3_aiet/results, think about what kind of plots do you need to prove your hypothesis.
## Please refer to the document for example prompts. 
## You can copy everthing generated from your coding agent here, if you got any errors, try to ask your coding agent to fix it for you :) 
## Have fun! 
#############################
#############################

import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

def visualize_robot_drift(directory_path):
    file_pattern = os.path.join(directory_path, 'drift_test_*.csv')
    csv_files = glob.glob(file_pattern)
    
    if not csv_files:
        print(f"No files found in {directory_path}")
        return

    results = []
    for file_path in csv_files:
        df = pd.read_csv(file_path)
        if df.empty:
            continue
        
        final_state = df.loc[df['iteration'].idxmax()]
        results.append({
            'cmd_wz': final_state['cmd_wz'],
            'longitudinal_drift': abs(final_state['longitudinal_drift']),
            'lateral_drift': abs(final_state['lateral_drift'])
        })

    summary_df = pd.DataFrame(results).sort_values(by='cmd_wz')

    # --- UPDATED PLOTTING SECTION ---
    plt.figure(figsize=(10, 6))
    
    # We use .values to convert Pandas Series to NumPy arrays
    # This bypasses the indexing error
    plt.plot(summary_df['cmd_wz'].values, 
             summary_df['longitudinal_drift'].values, 
             marker='o', label='Longitudinal Drift', linewidth=2)
    
    plt.plot(summary_df['cmd_wz'].values, 
             summary_df['lateral_drift'].values, 
             marker='s', label='Lateral Drift', linewidth=2)

    plt.title('Accumulated Robot Drift vs. Angular Velocity Command', fontsize=14)
    plt.xlabel('Angular Velocity Command (cmd_wz) [rad/s]', fontsize=12)
    plt.ylabel('Absolute Drift Magnitude [m]', fontsize=12)
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    results_path = '/home/aa274/asl_tb3_aiet/results'
    visualize_robot_drift(results_path)