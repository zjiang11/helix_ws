import numpy as np
from filterpy.kalman import KalmanFilter

# Initialize Kalman Filter
kf = KalmanFilter(dim_x=6, dim_z=3)

dt = 1.0 / 50  # time step for odometry (50Hz)

# State transition matrix (F)
kf.F = np.array([[1, 0, 0, dt, 0, 0],
                 [0, 1, 0, 0, dt, 0],
                 [0, 0, 1, 0, 0, dt],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]])

# Measurement function (H)
kf.H = np.array([[1, 0, 0, 0, 0, 0],
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 1, 0, 0, 0]])

# Initial uncertainty
kf.P *= 1000

# Measurement noise covariance matrix (R)
kf.R = np.array([[0.1, 0, 0],
                 [0, 0.1, 0],
                 [0, 0, 0.1]])

# Process noise covariance matrix (Q)
kf.Q = np.eye(6) * 0.1

# Set initial state
kf.x = np.array([0, 0, 0, 0, 0, 0])  # Initial position (x, y, theta) and velocity (vx, vy, vtheta)

# Function to simulate the Kalman filter
def test_kalman_filter():
    # Simulate a few measurements (position data from LiDAR)
    measurements = [[1, 2, 0.1], [2, 3, 0.15], [3, 4, 0.2]]  # x, y, theta

    for z in measurements:
        print(f"Before update: {kf.x}")

        # Predict the next state
        kf.predict()

        # Update step with the new measurement
        kf.update(z)

        print(f"After update: {kf.x}\n")

# Run the test function
test_kalman_filter()
