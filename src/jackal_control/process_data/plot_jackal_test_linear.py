
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib
matplotlib.use('Agg')  # Use the Agg backend for non-GUI environments
import matplotlib.pyplot as plt

# Load CSV file
file_path = '/root/helix_ws/data/test_state_jackal_kalman_filter/csv_data/jackal_state_10.csv' 
data = pd.read_csv(file_path)

# Extract columns
timestamps = data['Timestamp'].values
velocities = data['Current X Speed'].values
inputs = data['Current Input V'].values

# Prepare training data
v_k = velocities[:-1]  # v(k)
v_k1 = velocities[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

# Prepare X (features) and y (target)
X = np.vstack((v_k, u_k)).T  # Features: [v(k), u(k)]
y = v_k1                     # Target: v(k+1)

# Train model to find a and b
model = LinearRegression()
model.fit(X, y)

# Extract a and b
a, b = model.coef_

params_file_path = '/root/helix_ws/data/test_state_jackal_kalman_filter/params_ab_linear.csv'


params_df = pd.DataFrame({'Parameter': ['a', 'b'], 'Value': [a, b]})
params_df.to_csv(params_file_path, index=False)
print(a,b)
print(f"Parameters saved as '{params_file_path}'.")

# Estimate all velocities using trained a and b
estimated_velocities = np.zeros_like(velocities)
estimated_velocities[0] = velocities[0]  # Initial velocity

# Calculate estimated velocities iteratively
for i in range(1, len(velocities)):
    estimated_velocities[i] = a * estimated_velocities[i - 1] + b * inputs[i - 1]

# Calculate Mean Squared Error (MSE)
mse = np.mean((velocities - estimated_velocities) ** 2)
print(f"Mean Squared Error (MSE): {mse:.4f}")

# Plot real vs estimated velocities
plt.figure(figsize=(10, 6))
plt.plot(timestamps, velocities, label='Measured by Sensors', linestyle='-', marker='o')
plt.plot(timestamps, estimated_velocities, label='Estimated by System State', linestyle='--', marker='x')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
# plt.title(f'Real vs Estimated Velocity\nMSE = {mse:.4f}')
plt.legend()
plt.grid(False)

# Save the plot as an image file
plt.savefig('/root/helix_ws/data/test_state_jackal_kalman_filter/state_test.png')
print("Plot saved as 'velocity_comparison.png' in the specified directory.")
