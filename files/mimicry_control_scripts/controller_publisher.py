#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, Wrench
from std_msgs.msg import Int8
from networking import *
from movo_msgs.msg import *


def parse(data):
    data_arr = data.split(';')

    two_controllers = int(data_arr[0])

    if two_controllers == 1:
        pos_goal_str_r = data_arr[1]
        quat_goal_str_r = data_arr[2]
        pos_goal_str_l = data_arr[3]
        quat_goal_str_l = data_arr[4]

        grab = int(data_arr[5])
        clutch = int(data_arr[6])

        pos_arr_r = pos_goal_str_r.split(',')
        quat_arr_r = quat_goal_str_r.split(',')
        pos_goal_r = [float(pos_arr_r[0]), float(pos_arr_r[1]), float(pos_arr_r[2])]
        quat_goal_r = [float(quat_arr_r[0]), float(quat_arr_r[1]), float(quat_arr_r[2]), float(quat_arr_r[3])]
       
        pos_arr_l = pos_goal_str_l.split(',')
        quat_arr_l = quat_goal_str_l.split(',')
        pos_goal_l = [float(pos_arr_l[0]), float(pos_arr_l[1]), float(pos_arr_l[2])]
        quat_goal_l = [float(quat_arr_l[0]), float(quat_arr_l[1]), float(quat_arr_l[2]), float(quat_arr_l[3])]

        return [two_controllers, pos_goal_r, quat_goal_r, pos_goal_l, quat_goal_l, grab, clutch]

    else:
        pos_goal_str_r = data_arr[1]
        quat_goal_str_r = data_arr[2]

        grab = int(data_arr[3])
        clutch = int(data_arr[4])

        pos_arr = pos_goal_str_r.split(',')
        quat_arr = quat_goal_str_r.split(',')
        pos_goal_r = [float(pos_arr[0]), float(pos_arr[1]), float(pos_arr[2])]
        quat_goal_r = [float(quat_arr[0]), float(quat_arr[1]), float(quat_arr[2]), float(quat_arr[3])]

        return [two_controllers, pos_goal_r, quat_goal_r, grab, clutch]

vibr = "0.0,0.0,0.0"
def wrench_cb(data):
    forces = data.force
    global vibr
    vibr = str(forces.x) + "," + str(forces.y) + "," + str(forces.z)

def estop():
    cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
    arm_pub_r = rospy.Publisher('/movo/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
    arm_pub_l = rospy.Publisher('/movo/left_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)

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

net = Networking()
sock = net.bind_socket(net.ip, net.UDP_PORT)
vibr_sock = net.create_udp_socket()
rospy.init_node('controller_publisher')

pos_pub_r =     rospy.Publisher('/vive_controller/position_r',Vector3Stamped, queue_size=5)
quat_pub_r =    rospy.Publisher('/vive_controller/quaternion_r', QuaternionStamped, queue_size=5)
pos_pub_l =     rospy.Publisher('/vive_controller/position_l', Vector3Stamped, queue_size=5)
quat_pub_l =    rospy.Publisher('/vive_controller/quaternion_l', QuaternionStamped, queue_size=5)
grab_pub =      rospy.Publisher('/vive_controller/grab', Int8, queue_size=5)
clutch_pub =    rospy.Publisher('/vive_controller/clutch', Int8, queue_size=5)

rospy.Subscriber("/panda/wrench", Wrench, wrench_cb)

rospy.sleep(0.3)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    data = net.get_socket_data(sock)
    parsed_data = parse(data)
    #print parsed_data

    net.send_socket_data(vibr_sock, vibr, net.vibr_ip, net.VIBR_PORT)

    two_controllers = parsed_data[0]

    pos_goal_r = parsed_data[1]
    quat_goal_r = parsed_data[2]

    if two_controllers == 1:
        grab = parsed_data[5]
        clutch = parsed_data[6]
    else:
        grab = parsed_data[3]
        clutch = parsed_data[4]
 
    pos_goal_ros = Vector3Stamped()
    quat_goal_ros = QuaternionStamped()

    pos_goal_ros.vector.x = pos_goal_r[0]
    pos_goal_ros.vector.y = pos_goal_r[1]
    pos_goal_ros.vector.z = pos_goal_r[2]
    pos_pub_r.publish(pos_goal_ros)

    quat_goal_ros.quaternion.w = quat_goal_r[0]
    quat_goal_ros.quaternion.x = quat_goal_r[1]
    quat_goal_ros.quaternion.y = quat_goal_r[2]
    quat_goal_ros.quaternion.z = quat_goal_r[3]
    quat_pub_r.publish(quat_goal_ros)

    grab_val_ros = Int8()
    grab_val_ros.data = grab
    grab_pub.publish(grab_val_ros)
    if grab == 1:
        estop()
        pass

    clutch_val_ros = Int8()
    clutch_val_ros.data = clutch
    clutch_pub.publish(clutch_val_ros)

    if two_controllers == 1:
        pos_goal_l = parsed_data[3]
        quat_goal_l = parsed_data[4]

        pos_goal_ros = Vector3Stamped()
        quat_goal_ros = QuaternionStamped()

        pos_goal_ros.vector.x = pos_goal_l[0]
        pos_goal_ros.vector.y = pos_goal_l[1]
        pos_goal_ros.vector.z = pos_goal_l[2]
        pos_pub_l.publish(pos_goal_ros)

        quat_goal_ros.quaternion.w = quat_goal_l[0]
        quat_goal_ros.quaternion.x = quat_goal_l[1]
        quat_goal_ros.quaternion.y = quat_goal_l[2]
        quat_goal_ros.quaternion.z = quat_goal_l[3]
        quat_pub_l.publish(quat_goal_ros)

    rate.sleep()




