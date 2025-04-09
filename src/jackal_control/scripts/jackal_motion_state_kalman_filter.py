#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from helix_msgs.msg import JackalState
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import tf.transformations
import time
import math

class KalmanFilterNode:
    def __init__(self):
         
        self.motion_state_publisher = rospy.Publisher('/motion_state_kalman_filter', JackalState, queue_size=10)
        
        self.gravitational_force = np.array([[0.0], [0.0], [9.7303]])
        self.kf = KalmanFilter()
        
        self.jackal_current_motion_state = JackalState()
        self.jackal_previous_motion_state = JackalState()
        self.jackal_motion_state_kalman_filter = JackalState()

        self.subscribe_lidar_thread = threading.Thread(target=self.subscribe_lidar_thread_callback)
        self.subscribe_lidar_thread.daemon = True
        self.subscribe_lidar_thread.start()

        self.subscribe_imu_thread = threading.Thread(target=self.subscribe_imu_thread_callback)
        self.subscribe_imu_thread.daemon = True
        self.subscribe_imu_thread.start()

        self.calculate_speed_flag_lidar = False
        self.current_time_lidar = None
        self.previous_time_lidar = None
        self.do_lidar_calibration = False

        self.calculate_speed_flag_imu = False
        self.initilize_previous_time_imu_a_theta_flag = True
        self.current_time_imu = None
        self.previous_time_imu = None
        self.previous_time_imu_a_theta = None
        self.do_imu_prediction = False


        self.last_motion_state_update_time = None




    def run(self):
        rospy.spin()

    


    def get_lidar_info_callback(self, msg):

        def calculate_current_speed_callback():
            delta_time = self.current_time_lidar - self.previous_time_lidar
            delta_x = self.jackal_current_motion_state.x - self.jackal_previous_motion_state.x
            delta_y = self.jackal_current_motion_state.y - self.jackal_previous_motion_state.y
            if self.jackal_current_motion_state.theta < -2.0 and self.jackal_previous_motion_state.theta > 2.0:
                correct_current_theta = self.jackal_current_motion_state.theta + 2 * math.pi
            elif self.jackal_current_motion_state.theta > 2.0 and self.jackal_previous_motion_state.theta < -2.0:
                correct_current_theta = self.jackal_current_motion_state.theta - 2 * math.pi
            else:
                correct_current_theta = self.jackal_previous_motion_state.theta
                #correct_current_theta = self.jackal_current_motion_state.theta
            delta_theta = correct_current_theta - self.jackal_previous_motion_state.theta

            x_speed = delta_x / delta_time
            y_speed = delta_y / delta_time
            theta_speed = delta_theta / delta_time

            return x_speed, y_speed, theta_speed

        self.current_time_lidar = time.time()
        self.jackal_current_motion_state.x = msg.pose.pose.position.x
        self.jackal_current_motion_state.y = msg.pose.pose.position.y

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.jackal_current_motion_state.theta = yaw

        if self.calculate_speed_flag_lidar == True:
            self.jackal_current_motion_state.v_x, self.jackal_current_motion_state.v_y, self.jackal_current_motion_state.v_theta = calculate_current_speed_callback()

        if self.do_lidar_calibration == True:
            x = self.jackal_current_motion_state.x
            y = self.jackal_current_motion_state.y
            theta = self.jackal_current_motion_state.theta
            v_x = self.jackal_current_motion_state.v_x
            v_y = self.jackal_current_motion_state.v_y
            v_theta = self.jackal_current_motion_state.v_theta
            lidar_data = np.array([x, y, theta, v_x, v_y, v_theta])

            self.kf.calibrate(lidar_data)

        self.last_motion_state_update_time = time.time()

        self.previous_time_lidar = self.current_time_lidar
        self.jackal_previous_motion_state.x = self.jackal_current_motion_state.x
        self.jackal_previous_motion_state.y = self.jackal_current_motion_state.y
        self.jackal_previous_motion_state.theta = self.jackal_current_motion_state.theta
        self.calculate_speed_flag_lidar = True
        self.do_lidar_calibration = True


    def get_imu_info_callback(self,msg):

        def get_R_matrix(roll, pitch, yaw):
            R_x = np.array([
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)]
            ])

            R_y = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ])

            R_z = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]
            ])

            # Combined rotation matrix R = R_z * R_y * R_x
            R = R_z @ R_y @ R_x
            return R

        def remove_gravitational_effect(a_x_raw, a_y_raw, R):
            compensation_acceleration = np.linalg.inv(R) @ self.gravitational_force

            x_compensation = compensation_acceleration[0, 0]
            y_compensation = compensation_acceleration[1, 0]
            
            a_x_calibrate = a_x_raw - x_compensation
            a_y_calibrate = a_y_raw - y_compensation

            return a_x_calibrate, a_y_calibrate

        def calculate_angular_acceleration_callback(delta_time):
            delta_theta_velocity = self.jackal_current_motion_state.v_theta - self.jackal_previous_motion_state.v_theta
            theta_acceleration = delta_theta_velocity / delta_time

            return theta_acceleration
        
        a_x_raw = msg.linear_acceleration.x
        a_y_raw = msg.linear_acceleration.y
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        R_matrix = get_R_matrix(roll, pitch, yaw)
        self.jackal_current_motion_state.a_x, self.jackal_current_motion_state.a_y = remove_gravitational_effect(a_x_raw, a_y_raw, R_matrix)

        self.jackal_current_motion_state.v_theta = msg.angular_velocity.z
        self.current_time_imu = time.time()
        if self.calculate_speed_flag_imu == True:
            delta_time = self.current_time_imu - self.previous_time_imu_a_theta
            if delta_time > 0.1:
                self.jackal_current_motion_state.a_theta = calculate_angular_acceleration_callback(delta_time)
                self.previous_time_imu_a_theta = time.time()
        
        if self.do_imu_prediction == True:
            a_x = self.jackal_current_motion_state.a_x
            a_y = self.jackal_current_motion_state.a_y
            a_theta = self.jackal_current_motion_state.a_theta  
            imu_data = np.array([a_x, a_y, a_theta])
            
            if self.last_motion_state_update_time != None:
                dt = time.time() - self.last_motion_state_update_time
                x_predict, y_predict, theta_predict, v_x_predict, v_y_predict, v_theta_predict = self.kf.predict(imu_data, dt)
                
                self.jackal_motion_state_kalman_filter.x = x_predict
                self.jackal_motion_state_kalman_filter.y = y_predict
                #self.jackal_motion_state_kalman_filter.theta = theta_predict
                self.jackal_motion_state_kalman_filter.theta = self.jackal_current_motion_state.theta
                self.jackal_motion_state_kalman_filter.v_x = v_x_predict
                self.jackal_motion_state_kalman_filter.v_y = v_y_predict
                self.jackal_motion_state_kalman_filter.v_theta = self.jackal_current_motion_state.v_theta 
                #self.jackal_motion_state_kalman_filter.v_theta = v_theta_predict 
                self.jackal_motion_state_kalman_filter.a_x = a_x
                self.jackal_motion_state_kalman_filter.a_y = a_y
                self.jackal_motion_state_kalman_filter.a_theta = a_theta

                print(self.jackal_motion_state_kalman_filter)

                self.motion_state_publisher.publish(self.jackal_motion_state_kalman_filter)
 
        self.last_motion_state_update_time = time.time()
        
        if self.initilize_previous_time_imu_a_theta_flag == True:
            self.previous_time_imu_a_theta = self.current_time_imu
            self.initilize_previous_time_imu_a_theta_flag = False

        self.previous_time_imu = self.current_time_imu
        self.jackal_previous_motion_state.v_theta = self.jackal_current_motion_state.v_theta
        self.calculate_speed_flag_imu = True
        self.do_imu_prediction = True


    def subscribe_lidar_thread_callback(self):
        self.subscribe_lidar_info = rospy.Subscriber('/aft_mapped_to_init_high_frec',Odometry,self.get_lidar_info_callback,queue_size=10)
        rospy.spin()

    def subscribe_imu_thread_callback(self):
        self.subscribe_imu_info = rospy.Subscriber('/imu/data', Imu, self.get_imu_info_callback, queue_size=10)
        rospy.spin()




class KalmanFilter:
    def __init__(self):

        self.process_noise_var = 0.02
        self.measurement_noise_var = 1.0
        self.x = np.zeros((6,1))

        self.F = np.eye(6)

        self.B = np.zeros((6, 3))

        self.Q = self.process_noise_var * np.eye(6)

        self.H = np.zeros((6, 6))
        self.H[0, 0] = 1  # x
        self.H[1, 1] = 1  # y
        self.H[2, 2] = 1  # theta
        self.H[3, 3] = 1  # vx
        self.H[4, 4] = 1  # vy
        self.H[5, 5] = 1  # omega

        self.R = self.measurement_noise_var * np.eye(6)

        self.P = np.eye(6)

    def predict(self, imu_data, dt):
        u = imu_data.reshape(3,1)

        self.F[0, 3] = dt  # x += vx * dt
        self.F[1, 4] = dt  # y += vy * dt
        self.F[2, 5] = dt  # theta += v_theta * dt

        self.B[3, 0] = dt  # v_x += ax * dt
        self.B[4, 1] = dt  # v_y += ay * dt
        self.B[5, 2] = dt  # v_theta += a_theta * dt

        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

        x = self.x[0,0]
        y = self.x[1,0]
        theta = self.x[2,0]
        v_x = self.x[3,0]
        v_y = self.x[4,0]
        v_theta = self.x[5,0]

        return x, y, theta, v_x, v_y, v_theta
    
    def calibrate(self, lidar_data):
        measurement = lidar_data.reshape(6,1)
        y = measurement - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        



if __name__ == '__main__':
    rospy.init_node('kalman_filter', anonymous=True)
    controller = KalmanFilterNode()
    try:
        controller.run()
    except rospy.ROSInitException:
        pass