import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import os
import numpy as np
import rospkg


# Optional: If running headless (server), keep 'Agg'; otherwise, comment it out.
matplotlib.use('Agg')

# Define data directory and file path
data_dir = '/root/helix_ws/data/square_path/final_improve'
save_dir = '/root/helix_ws/data/square_path/plot'
csv_file = os.path.join(data_dir, 'cycle_5_1.csv')

os.makedirs(save_dir, exist_ok=True)
# Check if the file exists before proceeding
if not os.path.exists(csv_file):
    raise FileNotFoundError(f"CSV file not found: {csv_file}")

# Load DataFrame
df = pd.read_csv(csv_file)

# Convert necessary columns to NumPy arrays
time = df['Timestamp'].to_numpy()

x_current = df['Current X'].to_numpy()
y_current = df['Current Y'].to_numpy()
theta_current = df['Current Theta'].to_numpy()
x_ref = df['Reference X'].to_numpy()
y_ref = df['Reference Y'].to_numpy()
theta_ref = df['Reference Theta'].to_numpy()
v_input = df['Linear Velocity'].to_numpy()
w_input = df['Angular Velocity'].to_numpy()
motor_current_1 = df['Current Motor1'].to_numpy()
motor_voltage_1 = df['Voltage Motor1'].to_numpy()




plt.figure()
plt.plot(x_current, y_current, label="real trajectory", linestyle='-', marker='.')
plt.plot(x_ref, y_ref, label="reference point", linestyle='', marker='o')
# plt.title("real trajectory")
plt.ylabel("Y (m)")
plt.xlabel("X (m)")
plt.legend(loc='upper right')
plt.xlim(-0.5, 2.8) 
plt.grid(False)

# plt.savefig(os.path.join(save_dir, 'trajectory_without_delta_u_limitation.png'))
plt.savefig(os.path.join(save_dir, 'trajectory_with_delta_u_limitation.png'))