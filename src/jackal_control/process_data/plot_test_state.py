import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import os
import numpy as np


data_dir = os.path.join('/root/helix_ws/data/test_state_jackal_kalman_filter/csv_data')
csv_file = os.path.join(data_dir, 'jackal_state_10.csv')
save_dir = os.path.join('/root/helix_ws/data/test_state_jackal_kalman_filter')

df = pd.read_csv(csv_file)

data={
    'input':df['Current Input V'].to_numpy(),
    'speed':df['Current X Speed'].to_numpy(),
    'acceleration':df['Current X Acceleration'].to_numpy(),
    'current_motor1':df['Current Motor1'].to_numpy(),
    'current_motor2':df['Current Motor2'].to_numpy(),
    'voltage_motor1':df['Voltage Motor1'].to_numpy(),
    'voltage_motor2':df['Voltage Motor2'].to_numpy(),
}

time = df['Timestamp'].to_numpy()

plt.figure()
plt.plot(time, data['input'], label='input', color='red')
plt.plot(time, data['speed'], label='speed(m/s)', color='blue')
plt.plot(time, data['acceleration'], label = 'acceleration(m/s^2)', color='purple')
plt.plot(time, data['current_motor1'], label = 'current(A)', color='green')
plt.plot(time, data['voltage_motor1'], label = 'voltage(V)', color='yellow')
plt.xlabel('Time (s)')
plt.ylabel('Values')
# plt.title('Input, Speed, Acceleration, Current, Voltage vs Time')
plt.legend()
plt.grid(False)

plt.savefig(os.path.join(save_dir, 'input_speed_acceleration_current_voltage.png'))