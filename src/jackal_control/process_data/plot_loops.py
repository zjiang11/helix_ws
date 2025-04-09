import pandas as pd
import numpy as np
import os
import matplotlib
matplotlib.use('Agg')  # Add this to avoid display issues
import matplotlib.pyplot as plt

# Define the base path for the CSV files
base_path = '/root/helix_ws/data/square_path/final_improve'

# Define the number of cycles
num_cycles = 7

# Store average values
average_times = []
average_energies = []

# Process each cycle group
for cycle in range(1, num_cycles + 1):
    times = []
    energies = []
    
    for run in range(1, 11):  # cycle_X_1 to cycle_X_10
        file_name = f'cycle_{cycle}_{run}.csv'
        file_path = os.path.join(base_path, file_name)
        
        if os.path.exists(file_path):
            # Load CSV file
            data = pd.read_csv(file_path)
            
            # Extract last values
            last_timestamp = data['Timestamp'].iloc[-1]
            last_energy_usage = data['Energy Usage'].iloc[-1] / 1000
            
            times.append(last_timestamp)
            energies.append(last_energy_usage)
        else:
            print(f"File {file_name} not found.")
    
    # Compute averages
    if times and energies:
        avg_time = np.mean(times)
        avg_energy = np.mean(energies)
        
        average_times.append(avg_time)
        average_energies.append(avg_energy)

# Save the average results to CSV
output_file = os.path.join(base_path, 'average_results.csv')
results_df = pd.DataFrame({'Cycle': list(range(1, num_cycles + 1)),
                           'Average Time': average_times,
                           'Average Energy Usage': average_energies})
results_df.to_csv(output_file, index=False)

delta_u_ratio = np.arange(1, 8, 1)
# Plot Average Time
plt.figure(figsize=(10, 5))
plt.plot(delta_u_ratio, average_times, marker='o', linestyle='-')
plt.xticks(np.arange(0, 8, 1))
plt.xlabel(r'$\log_2 \left(\frac{u_{\max}}{\Delta u} \right)$', fontsize=16, labelpad=-1) 
plt.ylabel('Average Time (s)', fontsize=14)
# plt.title(r'Impact of $\Delta u / u_{\max}$ on Time Usage', fontsize=16)
plt.grid(False)
plt.savefig(os.path.join(base_path, 'average_time_plot.png'))
print("Average Time plot saved.")

# Plot Average Energy Usage
plt.figure(figsize=(10, 5))
plt.plot(delta_u_ratio, average_energies, marker='s', linestyle='-')
plt.xticks(np.arange(0, 8, 1))
plt.xlabel(r'$\log_2 \left(\frac{u_{\max}}{\Delta u} \right)$', fontsize=16, labelpad=-1)
plt.ylabel('Average Energy Usage (kw)', fontsize=14)
# plt.title(r'Impact of $\Delta u / u_{\max}$ on Energy Usage', fontsize=16)
plt.grid(False)
plt.savefig(os.path.join(base_path, 'average_energy_plot.png'))
print("Average Energy Usage plot saved.")
