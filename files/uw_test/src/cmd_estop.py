#!/usr/bin/python

import rospy
from movo_msgs.msg import *

import time

def estop():
    send_cmd_none = False
    cfg_cmd = ConfigCmd()
    run_arm_ctl = False
    run_pan_tilt_ctl = False
    _init_pan_tilt = False
    arm_cmd = JacoCartesianVelocityCmd()
    arm_cmd.header.stamp = rospy.get_rostime()
    arm_cmd.header.frame_id = ''
    arm_pub_r.publish(arm_cmd)
    arm_pub_l.publish(arm_cmd)

    cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
    cfg_cmd.gp_param = 1 # value of DISABLE_REQUEST variable from system_defines.py
    cfg_cmd.header.stamp = rospy.get_rostime()
    cfg_pub.publish(cfg_cmd)

if __name__ == '__main__':
    rospy.init_node('cmd_estop')
    cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
    arm_pub_r = rospy.Publisher('/movo/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
    arm_pub_l = rospy.Publisher('/movo/left_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
    # while not rospy.is_shutdown():
    #     key = input("enter number")
    #     if key == 1:
    #             estop()
    #     else:
    #         print("wrong num")
