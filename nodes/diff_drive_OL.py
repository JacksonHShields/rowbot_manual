#!/usr/bin/env python
# Author: Jackson Shields
#
import sys
import argparse
import rospy
import numpy as np
from std_msgs.msg import Float64, Bool, Float32
import math
import tf
from sensor_msgs.msg import Joy

class DiffDriveOL():
    """
    This class performs open loop tank driving with a joystick
    """

    def __init__(self):
        """ Initiates the tank drive class
        """
        robot_name = rospy.get_param("~robot_name", "")
        # # Map the namespace into the topics
        if robot_name is None or robot_name == "":
            joy_topic = '/joy'
            left_motor_topic = '/left_motor_cmd'
            right_motor_topic = '/right_motor_cmd'
        else:
            course_topic = '/' + robot_name + '/joy'
            left_motor_topic = '/' + robot_name + '/left_motor_cmd'
            right_motor_topic = '/' + robot_name + '/right_motor_cmd'
        # The publishers
        self.leftMotorPub_ = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        self.rightMotorPub_ = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        # Parameters
        self.max_fwd_cmd = rospy.get_param("~max_fwd_cmd", 1.0)  # If speed control is on, this is max speed
        self.max_rvs_cmd = rospy.get_param("~max_rvs_cmd", -1.0)
        max_axes_cmd = rospy.get_param("max_axes_cmd", 32767)
        self.axes_ratio = 1/math.fabs(max_axes_cmd)
        xbox_type = rospy.get_param("~xbox_controller_type", "wired")  # Options are wireless, wired
        if xbox_type == "wireless":
            self.controller_mode = 1
        else:
            self.controller_mode = 0
        self.motor_enable = False
        # Prefer adding the subscribers at the end of init
        rospy.Subscriber(joy_topic, Joy, self.joyCallback)
        print("End of Init")


    def joyCallback(self, data):
        """ Receives the localisation data and navigates the vessel
        Args:
            data (Joy): A sensor_msgs/Joy message
        Returns:
            None
        """
        if self.controller_mode == 1:
            if data.buttons[1]:
                self.motor_enable = False
            elif data.buttons[0]:
                self.motor_enable = True
            left_stick = data.axes[1]
            right_stick = data.axes[3]
        else:
            if data.buttons[1]:
                self.motor_enable = False
            elif data.buttons[0]:
                self.motor_enable = True
            left_stick = data.axes[1]
            right_stick = data.axes[4]

        # Left stick
        fwd_cmd = left_stick*self.axes_ratio
        # Right Stick
        yaw_cmd = right_stick*self.axes_ratio
        # yaw_cmd and fwd_cmd should be between -1, 1

        diff = yaw_cmd*self.max_axes_cmd
        left_cmd = fwd_cmd - yaw_cmd
        if left_cmd > 1.0:
            left_cmd = 1.0
        elif left_cmd < -1.0:
            left_cmd = -1.0
        right_cmd = fwd_cmd + yaw_cmd
        if right_cmd > 1.0:
            right_cmd = 1.0
        elif right_cmd < -1.0:
            right_cmd = -1.0
        rospy.loginfo("Left Cmd: %f, Right_Cmd: %f" %(left_cmd, right_cmd))
        self.leftMotorPub_.publish(left_cmd)
        self.rightMotorPub_.publish(right_cmd)

if __name__ == '__main__':
    """
    Diff Drive Open Loop
    """

    rospy.init_node('diff_drive_open_loop', anonymous=True)

    ddol = DiffDriveOL()

    rospy.spin()
