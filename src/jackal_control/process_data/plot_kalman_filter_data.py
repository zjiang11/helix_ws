import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import os
import numpy as np


data_dir = os.path.join('/root/helix_ws/data/move2ref')
csv_file = os.path.join(data_dir, 'move2ref_without_theta_weight.csv')
save_dir = os.path.join(data_dir, 'fig')
os.makedirs(save_dir, exist_ok=True)

df = pd.read_csv(csv_file)

data={
    'x_kalman_filter':df['Current X'].to_numpy(),
    'y_kalman_filter':df['Current Y'].to_numpy(),
    'theta_kalman_filter':df['Current Theta'].to_numpy(),
    'x_speed_kalman_filter':df['Current X Speed'].to_numpy(),
    'y_speed_kalman_filter':df['Current Y Speed'].to_numpy(),
    'theta_speed_kalman_filter':df['Current Theta Speed'].to_numpy(),
    'x_lidar':df['Current X LiDAR'].to_numpy(),
    'y_lidar':df['Current Y LiDAR'].to_numpy(),
    'theta_lidar':df['Current Theta LiDAR'].to_numpy(),
    'x_speed_lidar':df['Current X Speed LiDAR'].to_numpy(),
    'y_speed_lidar':df['Current Y Speed LiDAR'].to_numpy(),
    'theta_speed_lidar':df['Current Theta Speed LiDAR'].to_numpy(),
}

time = df['Timestamp'].to_numpy()

def plot_comparison(time, data1, data2, label1, label2, title, ylabel, save_name, y_min, y_max):
    plt.figure(figsize=(10, 5))
    plt.plot(time, data1, label=label1, linestyle='-', marker='o')
    plt.plot(time, data2, label=label2, linestyle='--', marker='x')
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    # plt.title(title)
    plt.legend(loc='upper right')
    plt.ylim(y_min, y_max) 
    plt.grid(False)
    plt.savefig(os.path.join(save_dir, save_name))
    plt.close()


def plot_theta(time, data1, label1, title, ylabel, save_name, y_min, y_max):
    plt.figure(figsize=(10, 5))
    plt.plot(time, data1, label=label1, linestyle='-', marker='o')
    plt.xlabel('Times (s)')
    plt.ylabel(ylabel)
    # plt.title(title)
    plt.legend(loc='upper right')
    plt.ylim(y_min, y_max) 
    plt.grid(False)
    plt.savefig(os.path.join(save_dir, save_name))
    plt.close()


# Generate comparison plots
plot_comparison(time, data['x_kalman_filter'], data['x_lidar'], 'Processed by Kalman Filter', 'Measured by LiDAR', 'Comparison of X Position', 'X Position (m)', 'comparison_x.png',-0.25,2.5)
plot_comparison(time, data['y_kalman_filter'], data['y_lidar'], 'Processed by Kalman Filter', 'Measured by LiDAR', 'Comparison of Y Position', 'Y Position (m)', 'comparison_y.png',-0.25,2.5)
plot_theta(time, data['theta_lidar'], 'Measured by LiDAR', 'Theta Position', 'Theta (rad)', 'comparison_theta.png',-0.1,1.8)
plot_comparison(time, data['x_speed_kalman_filter'], data['x_speed_lidar'], 'Processed by Kalman Filter', 'Measured by LiDAR', 'Comparison of X Speed', 'X Speed (m/s)', 'comparison_x_speed.png',-0.2,0.6)
plot_comparison(time, data['y_speed_kalman_filter'], data['y_speed_lidar'], 'Processed by Kalman Filter', 'Measured by LiDAR', 'Comparison of Y Speed', 'Y Speed (m/s)', 'comparison_y_speed.png',-0.05,0.3)
plot_comparison(time, data['theta_speed_kalman_filter'], data['theta_speed_lidar'], 'Measured by IMU', 'Measured by LiDAR', 'Comparison of Theta Speed', 'Theta Speed (rad/s)', 'comparison_theta_speed.png',-0.1,0.7)

