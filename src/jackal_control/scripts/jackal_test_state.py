import rospy
import numpy as np
from geometry_msgs.msg import Twist
import os
import csv
import threading
from jackal_msgs.msg import Feedback
from tf.transformations import euler_from_quaternion
from tf import TransformListener
from tf.msg import _tfMessage
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
import tf
import tf.transformations
from helix_msgs.msg import JackalState
from sensor_msgs.msg import Imu

class TestState:
    def __init__(self):
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_publisher_test = rospy.Publisher('/cmd_vel_test_hhhhhh', Twist, queue_size=1)
        self.mode_subscriber = rospy.Subscriber('/manual_mode_value',Twist, self.get_manual_value_callback, queue_size=1)

        self.data_dir = os.path.join(os.path.expanduser("~"), 'helix_ws','data','test_state_jackal_kalman_filter','csv_data')
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file_path = None

        self.is_test_on = False
        self.vel_cmd = Twist()
        self.vel_cmd_pub = Twist()
        self.vel_cmd_manual = Twist()
        self.real_vel_cmd_pub = Twist()
    

        self.previous_position_x = None
        self.previous_position_y = None
        self.previous_time = None
        self.current_time = None

        self.jackal_speed = Odometry()
        self.jackal_acceleration = Imu()
        self.motor_state = Feedback()
        self.jackal_motion_data = JackalState()

        self.time_stamp = 0.0

        user_input_thread = threading.Thread(target=self.user_input_thread_callback)
        user_input_thread.daemon = True
        user_input_thread.start()

        publish_cmd_thread = threading.Thread(target=self.publish_cmd_thread_callback)
        publish_cmd_thread.daemon = True
        publish_cmd_thread.start()
        
        subscribe_imu_thread = threading.Thread(target=self.subscribe_imu_thread_callback)
        subscribe_imu_thread.daemon = True
        subscribe_imu_thread.start()
        
        subscribe_motor_thread = threading.Thread(target=self.subscribe_motor_thread_callback)
        subscribe_motor_thread.daemon = True
        subscribe_motor_thread.start()






    def run(self):
        rospy.spin()




    def get_manual_value_callback(self, msg):
        self.is_test_on = False
        self.is_data_collect_on = False
        self.vel_cmd_manual = msg




    def do_test(self):
        rate = rospy.Rate(50)
        while self.is_test_on == True:
            if self.time_stamp < 2.0:
                # self.vel_cmd.linear.x = self.speed_level
                # self.vel_cmd.angular.z = 0.0

                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = self.speed_level
                 
            elif self.time_stamp >= 2.0 and self.time_stamp < 3.0:
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
               
            elif self.time_stamp >= 3.0:
                self.is_test_on = False
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
            
            self.vel_cmd_pub.linear.x = self.vel_cmd.linear.x * 0.1
            self.vel_cmd_pub.angular.z = self.vel_cmd.angular.z * 0.1
            
            # data = [round(self.time_stamp, 3),
            #         self.vel_cmd_pub.linear.x,
            #         round(self.jackal_motion_data.x, 3), 
            #         round(self.jackal_motion_data.v_x, 3), 
            #         round(self.jackal_motion_data.a_x, 3), 
            #         round(self.motor_state.drivers[0].current, 3), round(self.motor_state.drivers[1].current, 3),
            #         round(self.motor_state.drivers[0].duty_cycle, 3), round(self.motor_state.drivers[1].duty_cycle, 3)]
            

            data = [round(self.time_stamp, 3),
                    self.vel_cmd_pub.angular.z,
                    round(self.jackal_motion_data.theta, 3), 
                    round(self.jackal_motion_data.v_theta, 3), 
                    round(self.jackal_motion_data.a_theta, 3), 
                    round(self.motor_state.drivers[0].current, 3), round(self.motor_state.drivers[1].current, 3),
                    round(self.motor_state.drivers[0].duty_cycle, 3), round(self.motor_state.drivers[1].duty_cycle, 3)]

            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(data)
            # Record reference state, current state, and control values
            self.time_stamp += 0.02

            rate.sleep()




    def user_input_thread_callback(self):

        def record_state_init(speed_level):
            i = speed_level
            # self.csv_file_path = os.path.join(self.data_dir, f'jackal_state_{i}.csv')
            self.csv_file_path = os.path.join(self.data_dir, f'jackal_state_{i}_angular.csv')
            
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write CSV headers

                # writer.writerow(['Timestamp', 
                #                 'Current Input V',
                #                 'Current X',
                #                 'Current X Speed', 
                #                 'Current X Acceleration',
                #                 'Current Motor1', 'Current Motor2',
                #                 'Voltage Motor1', 'Voltage Motor2'])
                
                writer.writerow(['Timestamp', 
                                'Current Input W',
                                'Current Theta',
                                'Current Theta Speed', 
                                'Current Theta Acceleration',
                                'Current Motor1', 'Current Motor2',
                                'Voltage Motor1', 'Voltage Motor2'])
                
        while not rospy.is_shutdown():
            try:
                user_input = input('Enter "start" to begin the testing process.')
                if user_input.lower() == "start":
                    self.time_stamp = 0.0
                
                    while True:
                        speed_input = input('Enter a speed level between 1 and 10: ')
                        try:
                            speed_level = int(speed_input)
                            if 1 <= speed_level <= 10:
                                self.speed_level = speed_level
                                self.is_test_on = True
                                self.is_manual_on = False
                                self.time_stamp = 0.0
                                record_state_init(self.speed_level)
                                rospy.sleep(0.5)
                                self.do_test()
                                break
                            else:
                                print("Please enter a number between 1 and 10.")
                        except ValueError:
                            print("Invalid input. Please enter a valid integer between 1 and 10.")

            except ValueError:
                print("Invalid input.")




    def publish_cmd_thread_callback(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.is_test_on == True:
                self.real_vel_cmd_pub = self.vel_cmd_pub
                
            elif self.is_test_on == False:
                self.real_vel_cmd_pub = self.vel_cmd_manual

            else:
                self.real_vel_cmd_pub.linear.x = 0.0
                self.real_vel_cmd_pub.angular.z = 0.0
                
            self.cmd_vel_publisher_test.publish(self.real_vel_cmd_pub)
            self.cmd_vel_publisher.publish(self.real_vel_cmd_pub)

            rate.sleep()

               


    def get_imu_msg_callback(self, msg):
        self.jackal_motion_data = msg

    def subscribe_imu_thread_callback(self):
        self.imu_subscriber = rospy.Subscriber('/motion_state_kalman_filter', JackalState, self.get_imu_msg_callback, queue_size=1)
        rospy.spin()




    def get_motor_state_callback(self, msg):
        self.motor_state = msg

    def subscribe_motor_thread_callback(self):
        self.motor_state_subscriber = rospy.Subscriber('/feedback',Feedback, self.get_motor_state_callback, queue_size=1)
        rospy.spin()




if __name__ == "__main__":
    rospy.init_node('test_state',anonymous=True)
    controller = TestState()
    try:
        controller.run()
    except rospy.ROSInitException:
        pass