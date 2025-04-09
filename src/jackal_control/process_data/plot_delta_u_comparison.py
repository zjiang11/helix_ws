import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import os
import numpy as np

save_dir = os.path.join('/root/helix_ws/data/square_path/plot')
os.makedirs(save_dir, exist_ok= True)

data_dir = os.path.join('/root/helix_ws/data/square_path/final_improve')
csv_file = os.path.join(data_dir, 'cycle_1_1.csv')

df = pd.read_csv(csv_file)

data_without_delta_u={
    'input_linear':df['Linear Velocity'].to_numpy(),
    'input_angular':df['Angular Velocity'].to_numpy(),
    'current_motor1':df['Current Motor2'].to_numpy(),
    'voltage_motor1':df['Voltage Motor1'].to_numpy(),
}


data_dir = os.path.join('/root/helix_ws/data/square_path/final_improve')
csv_file = os.path.join(data_dir, 'cycle_5_5.csv')

df = pd.read_csv(csv_file)

data_with_delta_u={
    'input_linear':df['Linear Velocity'].to_numpy(),
    'input_angular':df['Angular Velocity'].to_numpy(),
    'current_motor1':df['Current Motor2'].to_numpy(),
    'voltage_motor1':df['Voltage Motor1'].to_numpy(),
}

time = df['Timestamp'].to_numpy()

max_length = min(len(data_without_delta_u['current_motor1']), len(data_with_delta_u['current_motor1']))


data_without_delta_u['input_linear'] = data_without_delta_u['input_linear'][:max_length]
data_with_delta_u['input_linear'] = data_with_delta_u['input_linear'][:max_length]
data_without_delta_u['current_motor1'] = data_without_delta_u['current_motor1'][:max_length]
data_with_delta_u['current_motor1'] = data_with_delta_u['current_motor1'][:max_length]

time = time[:max_length]


plt.figure()
plt.plot(time, data_without_delta_u['input_linear'], label=r'linear input without max △u limitation', color='red')
plt.plot(time, data_with_delta_u['input_linear'], label=r'linear input with max △u limitation', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Linear Input')
# plt.title('comparison for linear input')
plt.legend(loc='upper right')
plt.ylim(0, 0.72) 
plt.grid(False)
plt.savefig(os.path.join(save_dir, 'comparison_linear_input.png'))

plt.figure()
plt.plot(time, data_without_delta_u['current_motor1'], label=r'motor current without max △u limitation', color='red')
plt.plot(time, data_with_delta_u['current_motor1'], label=r'motor current with max △u limitation', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Motor Current (A)')
# plt.title('comparison for motor current')
plt.legend(loc='upper right')
plt.grid(False)
plt.savefig(os.path.join(save_dir, 'comparison_motor_current.png'))

