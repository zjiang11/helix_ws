#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import signal

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def signal_handler(sig, frame):
    # This function handles the signal for Ctrl+C (SIGINT)
    rospy.signal_shutdown("Exiting due to Ctrl+C")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('turtlebot_teleop')

    pub = rospy.Publisher('/manual_mode_value', Twist, queue_size=1)

    twist = Twist()
    
    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == 'w' or key == 'x' or key == 'a' or key == 'd' or key == 's':

                if key == 'w':
                    twist.linear.x = 0.25
                    twist.angular.z = 0.0
                elif key == 'x':
                    twist.linear.x = -0.25
                    twist.angular.z = 0.0
                elif key == 'a':
                    twist.angular.z = 0.5
                    twist.linear.x = 0.0
                elif key == 'd':
                    twist.angular.z = -0.5
                    twist.linear.x = 0.0
                elif key == 's':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)