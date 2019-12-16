#!/usr/bin/python

import rospy
import numpy as np
import math
import os
from movo_msgs.msg import JacoAngularVelocityCmd7DOF
from sensor_msgs.msg import JointState
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from relaxed_ik.msg import EEPoseGoals, JointAngles




r_vel_pub = None
r_arm_js = None
l_arm_js = None
relik_update_time = 0
rik = None
i = 0


def move_arms(right_sol, left_sol):
    global r_arm_js, l_arm_js, relik_update_time
    right_vel = np.array(right_sol) - np.array(r_arm_js)
    left_vel = np.array(left_sol) - np.array(l_arm_js)
    if (rospy.get_time() - relik_update_time <= 0.01):
        return
    if collision_detected(right_vel, left_vel) is True:
        relik_update_time = rospy.get_time()
        return
    move_right_arm(right_sol)
    move_left_arm(left_sol)
    relik_update_time = rospy.get_time()


def move_right_arm(sol):
    global r_vel_pub, relik_update_time
    s = 1.0 # 4.0 works well
    s = 2.0

    if r_vel_pub is None:
        r_vel_pub = rospy.Publisher('/movo/right_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)

    if r_arm_js is None:
        print("No joints")
        return
    vel = np.array(sol) - np.array(r_arm_js)

    vel = vel*s
    vel = (vel/math.pi)*180

    cmd = JacoAngularVelocityCmd7DOF()
    cmd.header.stamp = rospy.Time.now()
    cmd.theta_shoulder_pan_joint = vel[0]
    cmd.theta_shoulder_lift_joint = vel[1]
    cmd.theta_arm_half_joint = vel[2]
    cmd.theta_elbow_joint = vel[3]
    cmd.theta_wrist_spherical_1_joint = vel[4]
    cmd.theta_wrist_spherical_2_joint = vel[5]
    cmd.theta_wrist_3_joint = vel[6]
    # print(cmd)
    r_vel_pub.publish(cmd)

def move_left_arm(sol):
    global l_vel_pub
    s = 1.0 # 4.0 works well
    s = 2.0

    if l_vel_pub is None:
        l_vel_pub = rospy.Publisher('/movo/left_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)

    if l_arm_js is None:
        print("No joints")
        return
    vel = np.array(sol) - np.array(l_arm_js)
    #  print(vel)
    vel = vel*s
    vel = (vel/math.pi)*180

    cmd = JacoAngularVelocityCmd7DOF()
    cmd.header.stamp = rospy.Time.now()
    cmd.theta_shoulder_pan_joint = vel[0]
    cmd.theta_shoulder_lift_joint = vel[1]
    cmd.theta_arm_half_joint = vel[2]
    cmd.theta_elbow_joint = vel[3]
    cmd.theta_wrist_spherical_1_joint = vel[4]
    cmd.theta_wrist_spherical_2_joint = vel[5]
    cmd.theta_wrist_3_joint = vel[6]
    # print(cmd)
    l_vel_pub.publish(cmd)


def collision_detected(right_vel, left_vel):
    global r_arm_js, l_arm_js, rik, relik_update_time

    right_vel = np.array(right_vel)
    right_joints = np.array(r_arm_js)
    right_vel = right_vel * 2
    right_joints = right_joints + right_vel
    joints = [0.0]
    joints = np.concatenate((joints, right_joints))

    left_vel = np.array(left_vel)
    left_joints = np.array(l_arm_js)
    left_vel = left_vel * 2
    left_joints = left_joints + left_vel
    joints = np.concatenate((joints, left_joints))

    # joints = np.concatenate((joints, [0.0, 0.0]))

    col_val = rik.vars.collision_graph.get_collision_score_of_state(joints)
    b = rik.vars.collision_graph.b_value
    b = 5
    rik.vars.collision_graph.c.draw_all()

    if col_val >= b:
        print("%f, %f: Collision" % (col_val, b))
        return True
    else:
        print("%f, %f: No Collision" % (col_val, b))
        return False

def right_js_callback(data):
    global r_arm_js
    r_arm_js = data.position
    return

def left_js_callback(data):
    global l_arm_js
    l_arm_js = data.position
    return

def relik_sol_cb(data):
    '''
    Gets data from '/relaxed_ik/joint_angle_solutions' topic
    and sends right arm points to velocity controller
    '''
    global relik_update_time, r_arm_js
    global rik, i
    # print(data.angles.data[1:8])
    # rik.vars.collision_graph.c.draw_all()
    # if (rospy.get_time() - relik_update_time >= 0.01): #TODO: check diff rates
    #     joints = list(data.angles.data)
    #     joints[0] = 1.0
    #     col_val = rik.vars.collision_graph.get_collision_score_of_state(joints)
    #     b = rik.vars.collision_graph.b_value
    #     b = 11
    #     if col_val >= b:
    #         print("%f, %f: Collision" % (col_val, b))
    #     else:
    #         print("%f, %f: No Collision" % (col_val, b))
    #     i += 1
    #     relik_update_time = rospy.get_time()
    #
    #     return
    #     pass
    right_sol = data.angles.data[1:8]
    left_sol = data.angles.data[8:15]
    move_arms(right_sol, left_sol)
    # move_right_arm(data.angles.data[1:8], r_arm_js)

if __name__ == "__main__":
    rospy.init_node('movo_vel_controller')
    r_vel_pub = rospy.Publisher('/movo/right_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)
    r_js_sub = rospy.Subscriber('/movo/right_arm/joint_states', JointState, right_js_callback)
    l_vel_pub = rospy.Publisher('/movo/left_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)
    l_js_sub = rospy.Subscriber('/movo/left_arm/joint_states', JointState, left_js_callback)
    relik_sol_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, relik_sol_cb)
    # r_arm_js = [0]*7
    # l_arm_js = [0]*7
    path_to_src = os.path.join(os.path.dirname(__file__), '../../relaxed_ik/src')
    print(path_to_src)
    rik = get_relaxedIK_from_info_file(path_to_src, preconfig=True)
    relik_update_time = rospy.get_time()
    print("spinning...")
    rospy.spin()
