#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import tf
import tf.transformations
import casadi as ca
import numpy as np
import threading
from helix_msgs.msg import JackalState
from jackal_msgs.msg import Feedback
from nav_msgs.msg import Odometry
import tf
import tf.transformations
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool
import math
import csv
import time
import os
from std_msgs.msg import Float32

class MPCController:
    def __init__(self):

        self.mode_subscriber = rospy.Subscriber('/manual_mode_value',Twist, self.get_manual_value_callback, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel_test_yousa', Twist, queue_size=1)
        self.cmd_vel_publisher_real = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.reference_state_publisher = rospy.Publisher('/ref_state', JackalState, queue_size=1)
        self.current_state_publisher = rospy.Publisher('/current_state', JackalState, queue_size=1)
        self.control_state_publisher = rospy.Publisher('/control_state', Bool, queue_size=1)
        self.do_mpc_time_publisher = rospy.Publisher('/do_mpc_time', Float32, queue_size=1)

        self.temporary_stop = False
        self.is_mpc_on = False
        self.is_manual_on = True
        self.loop_process = False
        self.is_position_initialize = False
        self.record_data_init_flag = False
        self.is_mpc_init_on = False
        self.is_reference_state_updated = False
        self.is_data_collect_on = False
        self.calculate_speed_flag_lidar = False

        self.current_time_lidar = None
        self.previous_time_lidar = None

        self.mpc_stage = 'off'

        self.lbx = None
        self.ubx = None
        self.current_time = None
        self.previous_time = None

        self.stored_position = Point()
        self.stored_orientation = Quaternion()

        self.cmd_vel_mpc = Twist()
        self.cmd_vel_manual = Twist()
        self.cmd_vel_pub = Twist()

        self.current_state = JackalState()
        self.correct_current_state = JackalState()
        self.reference_state = JackalState()
        self.motor_state = Feedback()

        self.current_state_lidar = JackalState()
        self.previous_state_lidar = JackalState()


        self.speed_limitation = int()
        self.speed_limitation = 0
        self.delta_speed_limitation = int()
        self.delta_speed_limitation = 0
        self.initialize_stage_flag = int()
        self.initialize_stage_flag = 0

        self.distance_square = 0.0

        self.time_stamp = 0.0
        self.energy_usage = 0.0
        self.voltage = 24.0

        self.nx = 5
        self.nu = 2

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.a_linear = 0.9674
        self.b_linear = 0.0342
        self.a_angular = 0.9144
        self.b_angular = 0.0795
        
        self.predictive_horizon = 20
        self.time_interval = 0.02
        self.previous_U_input = np.array([0.0, 0.0])

        self.data_dir = os.path.join(os.path.expanduser("~"), 'helix_ws', 'data', 'move2refhhh')
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file_path = None

        self.start_user_input_thread()

        self.stage_flag_update = threading.Thread(target=self.stage_flag_update_callback)
        self.stage_flag_update.daemon = True
        self.stage_flag_update.start()

        self.record_data = threading.Thread(target = self.record_data_callback)
        self.record_data.daemon = True
        self.record_data.start()

        self.get_current_state = threading.Thread(target=self.get_current_state_callback)
        self.get_current_state.daemon = True
        self.get_current_state.start()

        self.mpc_thread = threading.Thread(target=self.do_mpc_callback)
        self.mpc_thread.daemon = True
        self.mpc_thread.start()

        self.publish_vel_cmd_thread = threading.Thread(target = self.publish_vel_cmd_callback)
        self.publish_vel_cmd_thread.daemon = True
        self.publish_vel_cmd_thread.start()




    def run(self):
        rospy.spin()



    
    def start_user_input_thread(self):
        input_thread = threading.Thread(target=self.handle_user_input)
        input_thread.daemon = True
        input_thread.start()

    def handle_user_input(self):
        while not rospy.is_shutdown():
            try:
                user_input = input('Enter [x y theta]: ')
                data = [float(value) for value in user_input.split()]
                if len(data) == 3:
                    self.is_mpc_on = True
                    self.is_manual_on = False
                    self.record_data_init_callback()
                    self.publish_reference(*data)

                else:
                    print("Invalid input. Please enter 3 values.")
            except ValueError:
                print("Invalid input. Please enter numeric values.")

    def publish_reference(self, x, y, theta):
        self.reference_state.x = x
        self.reference_state.y = y
        self.reference_state.theta = theta
        self.reference_state_publisher.publish(self.reference_state)

        self.mpc_stage = 'xy'
        self.is_data_collect_on = True




    def record_data_init_callback(self):
        self.csv_file_path = os.path.join(self.data_dir, 'move2ref_without_theta_weight.csv')
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write CSV header
            writer.writerow(['Timestamp', 
                             'Reference X', 'Reference Y', 'Reference Theta', 
                             'Current X', 'Current Y', 'Current Theta', 
                             'Current X Speed', 'Current Y Speed', 'Current Theta Speed', 
                             'Current X LiDAR', 'Current Y LiDAR', 'Current Theta LiDAR', 
                             'Current X Speed LiDAR', 'Current Y Speed LiDAR', 'Current Theta Speed LiDAR', 
                             'Linear Velocity', 'Angular Velocity',
            ])
        self.time_stamp = 0.0


    def record_data_callback(self):

        while not rospy.is_shutdown():
            if self.is_data_collect_on == True:

                start_time = time.time()
    
                data = [round(self.time_stamp, 3),
                        round(self.reference_state.x, 3), round(self.reference_state.y, 3), round(self.reference_state.theta, 3), 
                        round(self.current_state.x, 3), round(self.current_state.y, 3), round(self.current_state.theta, 3),
                        round(self.current_state.v_x, 3), round(self.current_state.v_y, 3), round(self.current_state.v_theta, 3),
                        round(self.current_state_lidar.x, 3), round(self.current_state_lidar.y, 3), round(self.current_state_lidar.theta, 3),
                        round(self.current_state_lidar.v_x, 3), round(self.current_state_lidar.v_y, 3), round(self.current_state_lidar.v_theta, 3),
                        round(self.cmd_vel_mpc.linear.x, 3), round(self.cmd_vel_mpc.angular.z, 3)]
     

                with open(self.csv_file_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(data)
                # Record reference state, current state, and control values

                self.time_stamp += 0.1

                while time.time() - start_time < 0.1:
                    time.sleep(0.001) 

    
    


    def get_manual_value_callback(self, msg):
        self.is_manual_on = True
        self.is_mpc_on = False
        self.cmd_vel_manual = msg




    def get_pose_callback(self, msg):
        self.current_state.x = msg.x
        self.current_state.y = msg.y
        self.current_state.theta = msg.theta
        self.current_state.v_x = msg.v_x
        self.current_state.v_y = msg.v_y
        self.current_state.v_theta = msg.v_theta

        self.distance_square = (self.current_state.x - self.reference_state.x) ** 2 + (self.current_state.y - self.reference_state.y) ** 2

        self.current_state_publisher.publish(self.current_state)

    def get_motor_state_callback(self, msg):
        self.motor_state = msg

    def get_lidar_info_callback(self, msg):

        def calculate_current_speed_callback():
            # delta_time = self.current_time_lidar - self.previous_time_lidar
            delta_x = self.current_state_lidar.x - self.previous_state_lidar.x
            delta_y = self.current_state_lidar.y - self.previous_state_lidar.y
            if self.current_state_lidar.theta < -2.0 and self.previous_state_lidar.theta > 2.0:
                correct_current_theta = self.current_state_lidar.theta + 2 * math.pi
            elif self.current_state_lidar.theta > 2.0 and self.previous_state_lidar.theta < -2.0:
                correct_current_theta = self.current_state_lidar.theta - 2 * math.pi
            else:
                correct_current_theta = self.current_state_lidar.theta
            delta_theta = correct_current_theta - self.previous_state_lidar.theta

            delta_time = time.time() - self.previous_time_lidar
            delta_time = 0.1
            self.previous_time_lidar = time.time()
            x_speed = delta_x / delta_time
            y_speed = delta_y / delta_time
            theta_speed = delta_theta / delta_time

            return x_speed, y_speed, theta_speed

        # self.current_time_lidar = time.time()
        self.current_state_lidar.x = msg.pose.pose.position.x
        self.current_state_lidar.y = msg.pose.pose.position.y

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.current_state_lidar.theta = yaw

        if self.calculate_speed_flag_lidar == True:
            self.current_state_lidar.v_x, self.current_state_lidar.v_y, self.current_state_lidar.v_theta = calculate_current_speed_callback()

        else:
            self.previous_time_lidar = time.time()

        self.calculate_speed_flag_lidar = True
        self.previous_state_lidar.x = self.current_state_lidar.x
        self.previous_state_lidar.y = self.current_state_lidar.y
        self.previous_state_lidar.theta = self.current_state_lidar.theta
        # self.previous_time_lidar = self.current_time_lidar

    def get_current_state_callback(self):
        self.pos_subscriber = rospy.Subscriber('/motion_state_kalman_filter', JackalState, self.get_pose_callback, queue_size=1)
        self.motor_state_subscriber = rospy.Subscriber('/feedback',Feedback, self.get_motor_state_callback, queue_size=1)
        self.subscribe_lidar_info = rospy.Subscriber('/aft_mapped_to_init_high_frec',Odometry,self.get_lidar_info_callback,queue_size=10)
        rospy.spin()



    def stage_flag_update_callback(self):

        rate = rospy.Rate(100)  # Run this loop at 10 Hz
        while not rospy.is_shutdown():
                
            if self.mpc_stage == 'xy' or self.mpc_stage == 'theta':
                self.mpc_controller_init(self.mpc_stage)
                self.is_mpc_init_on = True

            if self.mpc_stage == 'xy':
                distance_to_ref_point = math.sqrt((self.current_state.x - self.reference_state.x)**2 + (self.current_state.y - self.reference_state.y)**2)
                if distance_to_ref_point < 0.05:
                    self.mpc_stage = 'theta'
            
            elif self.mpc_stage == 'theta':
                distance_to_ref_point = math.sqrt((self.current_state.theta - self.reference_state.theta)**2)
                if distance_to_ref_point < 0.05:
                    self.mpc_stage = 'off'
                    self.is_data_collect_on = False
                    self.is_mpc_on = False

            rate.sleep()



    def mpc_controller_init(self, mpc_stage):

        t = self.time_interval
        h = self.predictive_horizon

        a_linear = self.a_linear
        b_linear = self.b_linear
        a_angular = self.a_angular
        b_angular = self.b_angular

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        v_linear = ca.SX.sym('v_linear')
        v_angular = ca.SX.sym('v_angular')

        v_input = ca.SX.sym('v_input')  # Linear input
        w_input = ca.SX.sym('w_input')  # Angular input

        # Define state and control vectors
        states = ca.vertcat(x, y, theta, v_linear, v_angular)
        controls = ca.vertcat(v_input, w_input)


        # Define the system dynamics
        next_states = ca.vertcat(
            x + t * ca.cos(theta) * v_linear,
            y + t * ca.sin(theta) * v_linear,
            theta + t * v_angular,
            a_linear * v_linear + b_linear * v_input,
            a_angular * v_angular + b_angular * w_input
        )

                
        f = ca.Function('f', [states, controls], [next_states])

        # Optimization variables
        U = ca.SX.sym('U', 2, h)  # Control inputs over the horizon (v, w)
        X = ca.SX.sym('X', 5, h + 1)  # State variables over the horizon (x, y, theta)
        

        v_min = -0.3
        w_min = -0.6
        v_max = 0.3
        w_max = 0.6

        if mpc_stage == 'xy':
            Q = np.diag([1.0, 1.0, 0.0, 0.0, 0.0])
        elif mpc_stage == 'theta':
            Q = np.diag([0.0, 0.0, 1.0, 0.0, 0.0])
        
        R = np.diag([0.01, 0.01])  # Cost for v, w
        
        self.lbx = np.concatenate(
            [np.full(self.nx * (h + 1), -ca.inf), np.tile([v_min, w_min], h)]
        )
        self.ubx = np.concatenate(
            [np.full(self.nx * (h + 1), ca.inf), np.tile([v_max, w_max], h)]
        )    
        
        cost_fn = 0
        eq_g = []  # Equality constraints
        ineq_g = []  # Inequality constraints

        P = ca.SX.sym('P', 2 * self.nx)
        previous_U_input = ca.SX.sym('prev_U', 2)
        delta_U_max = ca.SX.sym('delta_U_max', 2)

        eq_g.append(X[:, 0] - P[:self.nx])

        for k in range(h):
            st = X[:, k]
            con = U[:, k]
            x_ref = P[self.nx:]

            if mpc_stage == 'xy':
                scaled_st = ca.vertcat(st[0], st[1])  # Apply scaling to y
                scaled_x_ref = ca.vertcat(x_ref[0],x_ref[1])
                scaled_cost = (scaled_st - scaled_x_ref).T @ np.diag([1.0, 1.0]) @ (scaled_st - scaled_x_ref)

            elif mpc_stage == 'theta':
                scaled_st = ca.vertcat(st[2])  # Apply scaling to y
                scaled_x_ref = ca.vertcat(x_ref[2])
                scaled_cost = (scaled_st - scaled_x_ref).T @ np.diag([1.0]) @ (scaled_st - scaled_x_ref)
            
            cost_fn += scaled_cost  # State cost
            cost_fn +=  con.T @ R @ con

            if k == 0:
                delta_U = U[:, k] - previous_U_input  # First U relative to previous control
            else:
                delta_U = U[:, k] - U[:, k-1]  # Regular delta_U

            ineq_g.append(delta_U - delta_U_max)  # delta_U <= delta_U_max
            ineq_g.append(-delta_U - delta_U_max)  # -delta_U <= delta_U_max
    
            st_next = X[:, k+1]
            f_value = f(st, con)
            eq_g.append(st_next - f_value)

        # Concatenate constraints and optimization variables
        eq_g = ca.vertcat(*eq_g)
        ineq_g = ca.vertcat(*ineq_g)
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

        nlp_prob = {
                'f': cost_fn,
                'x': OPT_variables,
                'g': ca.vertcat(eq_g, ineq_g),  # Concatenate equality and inequality constraints
                'p': ca.vertcat(P, previous_U_input, delta_U_max)
            }
        opts = {
            'ipopt.max_iter':1000,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 1e-6
        }

        # Create solver
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def do_mpc_callback(self):
        rate = rospy.Rate(100)  # Run this loop at 10 Hz
        while not rospy.is_shutdown():
            
            start_time = rospy.Time.now()

            if self.is_manual_on == True and self.is_mpc_on == False:
                rate.sleep()
                continue

   
            if self.is_mpc_init_on == False:
                rate.sleep()
                continue
    
            h = self.predictive_horizon
            n_states = self.nx
            n_controls = self.nu

            self.correct_current_state = self.current_state

            if self.current_state.theta - self.reference_state.theta > math.pi:
                self.correct_current_state.theta = self.current_state.theta - 2 * math.pi
            elif self.current_state.theta - self.reference_state.theta < -math.pi:
                self.correct_current_state.theta = self.current_state.theta + 2 * math.pi
            else:
                self.correct_current_state.theta = self.current_state.theta

            if abs(math.cos(self.current_state.theta) ** 2) > 0.5:
                v_linear = self.current_state.v_x / math.cos(self.current_state.theta)
            else: 
                v_linear = self.current_state.v_y / math.sin(self.current_state.theta)

            v_angular = self.current_state.v_theta
            
            x_current_state = np.array([self.correct_current_state.x, self.correct_current_state.y, self.correct_current_state.theta, v_linear, v_angular])
            x_ref = np.array([self.reference_state.x, self.reference_state.y, self.reference_state.theta, 0.0, 0.0])

            self.reference_state_publisher.publish(self.reference_state)

            u0 = np.zeros((n_controls * h, 1 ))
            u0 = u0.flatten()
            x_init = np.tile(x_current_state, (h + 1, 1)).T.flatten()
            P = np.concatenate((x_current_state, x_ref))


            delta_U_max_val = np.array([0.06, 1.2])
            #delta_U_max_val = np.array([0.06 * i, 1.2])
            

            args = {
                'x0': np.concatenate([x_init, u0]),  # Initial guess for states and controls
                'lbx': self.lbx,
                'ubx': self.ubx,
                'lbg': np.concatenate([
                np.zeros(n_states + n_states * h),  # Equality constraints: g(x) = 0
                np.full(2 * n_controls * h, -ca.inf)  # Inequality constraints: g(x) <= 0
                ]),
                'ubg': np.concatenate([
                np.zeros(n_states + n_states * h),  # Equality constraints: g(x) = 0
                np.zeros(2 * n_controls * h)  # Inequality constraints: g(x) <= 0
                ]),
                'p': np.hstack((P, self.previous_U_input, delta_U_max_val)) # ✅ Use CasADi's `vertcat` instead
            }

            sol = self.solver(**args)

            u_opt = sol['x'][n_states * (h + 1):].full().reshape((h, n_controls))

            self.cmd_vel_mpc.linear.x = u_opt[0, 0]
            self.cmd_vel_mpc.angular.z = u_opt[0, 1]

            end_time = rospy.Time.now()
            time_usage = (end_time - start_time).to_sec()

            self.previous_U_input = np.array([self.cmd_vel_mpc.linear.x, self.cmd_vel_mpc.angular.z])
            # rospy.loginfo(f"Execution Time: {time_usage:.6f} seconds")





    def publish_vel_cmd_callback(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            if self.is_mpc_on == True and self.is_manual_on == False:
                self.cmd_vel_pub = self.cmd_vel_mpc
            elif self.is_mpc_on == False and self.is_manual_on == True:
                self.cmd_vel_pub.angular.z = self.cmd_vel_manual.angular.z * 0.6
                self.cmd_vel_pub.linear.x = self.cmd_vel_manual.linear.x * 0.6

            else:
                self.cmd_vel_pub.angular.z = 0.0
                self.cmd_vel_pub.linear.x = 0.0

            self.control_state_publisher.publish(self.is_manual_on)

            self.cmd_vel_publisher.publish(self.cmd_vel_pub)
            self.cmd_vel_publisher_real.publish(self.cmd_vel_pub)

            rate.sleep()

        


if __name__ == '__main__':
    rospy.init_node('mpc_controller', anonymous=True)
    controller = MPCController()
    try:
        controller.run()
    except rospy.ROSInitException:
        pass