import numpy as np
import csv
import os
import matplotlib.pyplot as plt
import math
import pandas as pd
import matplotlib
matplotlib.use('Agg')


class TestState:
    def __init__(self):
        # Define data directory and CSV file path
        self.data_dir = os.path.join(os.path.expanduser("~"), 'helix_ws', 'data', 'test_state_jackal_kalman_filter')
        self.csv_file_path = os.path.join(self.data_dir, 'csv_data','jackal_state_5.csv')

        # Simulation parameters
        self.simulation_duration = 4.0
        self.freq = 25
        self.helix_width = 1.0

        self.a, self.b, self.c1, self.c2 = self.load_state_function()
        self.input = np.zeros(4)

        # States storage
        self.real_states = []
        self.time_axis = []
        self.simulated_states = []

        # Read real data and run simulation
        self.read_real_data()
        self.run_simulation()
        self.plot_states()
    
    def load_state_function(self):
        a_b_csv_file = os.path.join(self.data_dir, 'linearization_parameter.csv')
        with open(a_b_csv_file, mode="r") as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                parameter, value = row
                if parameter == "Slope (a)":
                    a = float(value)
                elif parameter == "Intercept (b)":
                    b = float(value)

        c1_c2_csv_file = os.path.join(self.data_dir, 'C1_C2.csv')
        df = pd.read_csv(c1_c2_csv_file)
        c1 = df['C1'].values[0]
        c2 = df['C2'].values[0]
        
        return a, b, c1, c2


    def read_real_data(self):
        # Check if the CSV file exists
        if not os.path.exists(self.csv_file_path):
            print(f"CSV file {self.csv_file_path} not found.")
            return

        # Read data from CSV file
        with open(self.csv_file_path, mode='r') as file:
            reader = csv.DictReader(file)
            initial_position = None
            for row in reader:
                time_stamp = float(row['Timestamp'])

                if time_stamp <= self.simulation_duration:
                    current_state = [
                        float(row['Current X']),
                        float(row['Current X Speed']),
                    ]

                    if initial_position is None:
                        initial_position = current_state[:1]  # Save initial position (x, y, theta)

                    # Normalize the state based on the initial position
                    normalized_state = [
                        current_state[0] - initial_position[0],
                        current_state[1],
                    ]

                    self.real_states.append(normalized_state)
                    self.time_axis.append(time_stamp)




    def run_simulation(self):
        # Define input sequence for simulation
        input_sequence = [0.9] * (3 * self.freq) + [0.0] * (1 * self.freq )

        simulated_states = self.simulate(input_sequence)
        self.simulated_states.extend(simulated_states)

    def simulate(self, input_sequence):
        # Simulate the system dynamics
        states = []
        states.append([0.0, 0.0])
        t = 1 / self.freq
        speed = 0.0
        position = 0.0

        for input_value in input_sequence:
            if input_value == 0.0 and abs(speed)<0.05:
                    acceleration = 0.0
                    speed = 0.0
                    position = position

            else:
                acceleration = self.c1 * (self.a * speed + self.b) + self.c2 * input_value
                speed = speed + acceleration * t
                position = position + speed * t

            states.append([position, speed])

        return np.array(states)

    def plot_states(self):
        # Plot real and simulated states for comparison
        fig, axs = plt.subplots(2, 1, figsize=(15, 20))
        fig.suptitle('Comparison of Simulated and Real States')

        labels = ['position', 'speed']
        real_states_np = np.array(self.real_states)
        simulated_states_np = np.array(self.simulated_states)

        if len(simulated_states_np) != len(self.time_axis):
            print("Warning: Mismatch between simulated states and time axis length")

        for i in range(2):
            axs[i].plot(self.time_axis, real_states_np[:, i], label='Real ' + labels[i], color='b', linestyle='-', marker='o')
            axs[i].plot(self.time_axis, simulated_states_np[:, i], label='Simulated ' + labels[i], color='r', linestyle='--')
            axs[i].set_xlabel('Time (s)')
            axs[i].set_ylabel(labels[i])
            axs[i].legend(loc='best')
            axs[i].grid(True)

        plt.tight_layout()
        plt.subplots_adjust(top=0.95)

        # Save the plot image to the data directory
        output_image_path = os.path.join(self.data_dir, 'simulation_real_comparison.png')
        plt.savefig(output_image_path)
        print(f"Plot saved to {output_image_path}")
        plt.show()


if __name__ == '__main__':
    # Run the TestState class
    test_state = TestState()
