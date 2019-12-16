#!/usr/bin/python

import rospy
# from utils import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist, Point, Pose
from movo_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from relaxed_ik.msg import EEPoseGoals, JointAngles
import math
import time
import tf
import transforms3d as t3d
import numpy as np
import actionlib
from movo_vel_controller import move_r_arm
import RelaxedIK.Utils.transformations as T
import cv2 as cv


def print_point(point):
    '''
    Print a Point object in a single line.
    '''
    if point is not None:
        print("%f, %f, %f" % (point.x, point.y, point.z))

def aa_to_euler(axis, angle):
    '''
    Converts axis angle from to euler angles

    :param axis: Axis represented with Point type
    :param angle: Angle in radians
    :return: Point type with 3 euler angles
    '''
    p = ((angle/math.pi)*180)
    print("angle: %.2f" % p)

    euler = [0, 0, 0]
    x = axis[0]
    y = axis[1]
    z = axis[2]
    sin = math.sin(angle)
    cos = math.cos(angle)

    axis = np.array([axis[0], axis[1], axis[2]])
    mag = (axis ** 2).sum() ** 0.5
    if not mag == 1:
        axis = axis / mag
        x = axis[0]
        y = axis[1]
        z = axis[2]

    if (x * y * (1 - cos) + z * sin) > 0.998:
        euler[1] = 2 * math.atan2(x * math.sin(angle / 2), math.cos(angle / 2))
        euler[2] = math.pi / 2
        euler[0] = 0
        return euler

    if (x * y * (1 - cos) + z * sin) < -0.998:
        euler[1] = -2 * math.atan2(x * math.sin(angle / 2), math.cos(angle / 2))
        euler[2] = -math.pi / 2
        euler[0] = 0
        return euler

    a = y * sin - x * z * (1 - cos)
    b = 1 - (math.pow(y, 2) + math.pow(z, 2)) * (1 - cos)
    euler[1] = math.atan2(a, b)

    euler[2] = math.asin(x * y * (1 - cos) + z * sin)

    a = x * sin - y * z * (1 - cos)
    b = 1 - (math.pow(x, 2) + math.pow(z, 2)) * (1 - cos)
    euler[0] = math.atan2(a, b)

    # print(euler)

    return euler

def aa_to_quaternion(axis, angle):
    axis = np.array([axis[0], axis[1], axis[2]])
    mag = (axis ** 2).sum() ** 0.5
    if not mag == 1:
        axis = axis / mag

    s = math.sin(angle/2)
    quat = [1, 0, 0, 0]
    quat[0] = math.cos(angle/2) # w
    quat[1] = axis[0] * s       # x
    quat[2] = axis[1] * s       # y
    quat[3] = axis[2] * s       # z
    return quat

def multiply_quat(quat1, quat2):
    w1, x1, y1, z1 = quat1
    w2, x2, y2, z2 = quat2
    return np.array([-x2 * x1 - y2 * y1 - z2 * z1 + w2 * w1,
                     x2 * w1 + y2 * z1 - z2 * y1 + w2 * x1,
                     -x2 * z1 + y2 * w1 + z2 * x1 + w2 * y1,
                     x2 * y1 - y2 * x1 + z2 * w1 + w2 * z1], dtype=np.float64)

def get_average_length(centre, points):
    '''
    Returns avg length between centre and points and list of all lengths
    :param centre: 3 element array with x, y, z coordinates
    :param points: list of Point() type values
    :return: sum: average length
             lengths: list of all lengths
    '''
    n = len(points)
    sum = 0
    lengths = []
    for pt in points:
        l = math.sqrt((pt.x - centre[0])**2 + (pt.y - centre[1])**2 + (pt.z - centre[2])**2)
        sum += l
        lengths.append(l)
    sum /= n
    return sum, lengths

def get_std_dev(avg, values):
    '''
    Returns standard deviation of values from avg
    :param avg: mean value
    :param values: list of values
    :return: sum: standard deviation
    '''
    n = len(values)
    sum = 0
    for v in values:
        sum += (v - avg)**2
    sum /= n-1
    sum = math.sqrt(sum)
    return sum

class ARMover:
    def __init__(self):
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.base_pub = rospy.Publisher('/movo/teleop/cmd_vel', Twist, queue_size=10)
        self.head_pub = rospy.Publisher('/movo/head/cmd', PanTiltCmd, queue_size=100)
        self.gp_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
        self.arm_pub = rospy.Publisher('/movo/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
        self.js_sub = rospy.Subscriber('/movo/right_arm/joint_states', JointState, self.js_callback)
        self.r_arm_joint_ctl = actionlib.SimpleActionClient('movo/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.lin_pub = rospy.Publisher('/movo/linear_actuator_cmd/', LinearActuatorCmd, queue_size=10)
        self.r_arm_ang_pub = rospy.Publisher('/movo/right_arm/angular_vel_cmd', JacoAngularVelocityCmd7DOF, queue_size=10)
        self.gripper_pub_r = rospy.Publisher('/movo/right_gripper/cmd', GripperCmd, queue_size=10)
        self.relik_goal_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size = 10)
        self.relik_sol_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, self.relik_sol_cb)
        self.relik_rot_mat_sub = rospy.Subscriber('/relaxed_ik/ee_rot_mat', JointAngles, self.relik_rot_mat_cb)
        self.relik_ee_pos_sub = rospy.Subscriber('/relaxed_ik/ee_pos', JointAngles, self.relik_ee_pos_cb)

        self.relik_update_time = rospy.get_time()  # time since last RelaxedIK solution was sent
        self.relik_pose_r = None  # last right arm pose sent to relaxedIK
        self.relik_rot_mat = None
        self.relik_ee_pos = None  # contains [right ee position (relative),  left ee position (relative),
                                  #           right init ee position, left init ee position]
        self.relik_pos_mode = "relative"
        self.relik_rot_mode = "relative"

        self.listener = tf.TransformListener()

        self.head_pos = 0
        self.head_change = 0.1

        self.lin_cmd = LinearActuatorCmd()
        self.lin_update = None # time since last linear actuator update
        self.linact_vel_lim = None

        self.arm_update = rospy.get_time()

        self.arm_pos_orig = Point(0.4143988321679809, -0.07375608272348022, 0.9732553832520083)
        self.arm_pos = None  # right arm ee position from transformations
        self.arm_vel_lim = None
        self.arm_js = None  # right arm joint states
        self.r_arm_pos_ar = None  # right arm ee position from ar tag

        self.found = False
        self.ar4_pos = None
        self.ar4_pos_prev = None
        self.ar5_pos = None
        self.calc_diff_x = 0
        self.calc_diff_y = 0
        self.calc_diff_z = 0

        self.calib_matrix = None

        rospy.Subscriber("/movo/feedback/active_configuration", Configuration, self.update_config)

        movo_dynamics = rospy.wait_for_message('/movo/feedback/dynamics', Dynamics)
        self.lin_cmd.desired_position_m = movo_dynamics.linear_actuator_position_m

    def start(self):
        # print("start")
        conf_cmd = ConfigCmd()
        conf_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
        conf_cmd.gp_param = 5 # TRACTOR_REQUEST
        conf_cmd.header.stamp = rospy.get_rostime()
        # print("send config")
        self.gp_pub.publish(conf_cmd)
        # print("sent")


        # conf_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
        # conf_cmd.gp_param = 0
        # conf_cmd.header.stamp = rospy.get_rostime()
        # self.gp_pub.publish(conf_cmd)
        # self.move_arm()


        # self.move_arm_joint()
        # self.relik_position_control()
        # time.sleep(0.2)
        # self.relik_position_control()

        # self.get_right_matrix()
        # time.sleep(3)
        # self.point_at_relik_2()
        # self.calibrate()
        # self.test_repeatability()

    def relik_sol_cb(self, data):
        '''
        Gets data from '/relaxed_ik/joint_angle_solutions' topic
        and sends right arm points to velocity controller
        '''
        # print(data.angles.data[1:8])
        if (rospy.get_time() - self.relik_update_time >= 0.01): #TODO: check diff rates
            pass
        move_r_arm(data.angles.data[1:8], self.arm_js)
        self.relik_update_time = rospy.get_time()
        pass

    def relik_rot_mat_cb(self, data):
        # print(data.angles.data)
        self.relik_rot_mat = data.angles.data

    def relik_ee_pos_cb(self, data):
        '''
        Gets current and initial end effector positions from '/relaxed_ik/ee_pos'
        and stores it in relik_ee_pos class variable
        '''
        ret = data.angles.data
        if len(ret) != 12:
            print("NOT BIG ENOUGH")
        r_ee_pos = [ret[0], ret[1], ret[2]]
        l_ee_pos = [ret[3], ret[4], ret[5]]
        r_init_pos = [ret[6], ret[7], ret[8]]
        l_init_pos = [ret[9], ret[10], ret[11]]
        self.relik_ee_pos = [r_ee_pos, l_ee_pos, r_init_pos, l_init_pos]
        # matrix = self.get_transform_matrix("/pan_base_link", "/right_wrist_3_link")
        # pts = self.transform(Point(0, 0, 0), matrix)
        # print(pts)
        # print("relik_ee_pos: %f, %f, %f" % (r_ee_pos[0] + r_init_pos[0],r_ee_pos[1] +  r_init_pos[1],r_ee_pos[2] +  r_init_pos[2]))
        # print("\n")

    def point_at(self):
        '''
        Point end effector at AR marker 4 using MoveIt
        :return:
        '''
        while not rospy.is_shutdown():
            matrix = self.get_right_matrix()

            vec1 = self.transform(Point(1, 0, 0), matrix)
            vec1 = np.array([vec1.x, vec1.y, vec1.z])
            vec1 = vec1 / (vec1 ** 2).sum() ** 0.5

            self.arm_pos = self.transform(Point(0, 0, 0), matrix)

            if self.ar4_pos is None:
                return

            vec2 = np.array([self.ar4_pos.x - self.arm_pos.x, self.ar4_pos.y - self.arm_pos.y, self.ar4_pos.z - self.arm_pos.z])
            vec2 = vec2 / (vec2**2).sum()**0.5
            dot = np.dot(vec1, vec2)
            angle = math.acos(dot)

            axis = np.cross(vec1, vec2)
            axis_norm = axis / (axis**2).sum()**0.5

            arm_cmd = JacoCartesianVelocityCmd()
            arm_cmd.header.stamp = rospy.get_rostime()
            arm_cmd.header.frame_id = ''
            euler = aa_to_euler(axis_norm, angle)
            s = 5
            arm_cmd.theta_x = euler[0] * s
            arm_cmd.theta_y = euler[1] * s
            arm_cmd.theta_z = euler[2] * s
            # print(arm_cmd)
            self.send_arm_cmd(arm_cmd)

    def point_at_relik(self):
        '''
        Point end effector at AR marker 4 using RelaxedIK
        calculated using axis angles
        '''
        update_time = rospy.get_time()
        print("inside")
        while (not rospy.is_shutdown()):
            # print(rospy.get_time() - update_time)

            # print(quat2)
            # if (rospy.get_time() - update_time >= 0.05):
            #     pass
            # print("in while")
            matrix = self.get_right_matrix()

            p1 = self.transform(Point(1, 0, 0), matrix)
            p1 = np.array([p1.x, p1.y, p1.z])
            # print(p1)

            self.arm_pos = self.transform(Point(0, 0, 0), matrix)

            p2 = np.array([self.arm_pos.x, self.arm_pos.y, self.arm_pos.z])
            # print(p2)

            vec1 = p1 - p2
            vec1 = vec1 / (vec1 ** 2).sum() ** 0.5
            # print(vec1)

            # vec1 = self.transform(Point(0, 1, 0), matrix)
            # vec1 = np.array([vec1.x, vec1.y, vec1.z])
            # vec1 = vec1 / (vec1 ** 2).sum() ** 0.5
            # print(vec1)


            pos = self.relik_pose_r.position
            rot = self.relik_pose_r.orientation

            if self.ar4_pos is None:
                continue

            vec2 = np.array(
                [self.ar4_pos.x - self.arm_pos.x, self.ar4_pos.y - self.arm_pos.y, self.ar4_pos.z - self.arm_pos.z])
            vec2 = vec2 / (vec2 ** 2).sum() ** 0.5
            dot = np.dot(vec1, vec2)
            angle = math.acos(dot)

            axis = np.cross(vec1, vec2)
            axis_norm = axis / (axis ** 2).sum() ** 0.5


            quat1 = aa_to_quaternion(axis_norm, angle)
            rotmat = [[self.relik_rot_mat[0], self.relik_rot_mat[1], self.relik_rot_mat[2]],
                      [self.relik_rot_mat[3], self.relik_rot_mat[4], self.relik_rot_mat[5]],
                      [self.relik_rot_mat[6], self.relik_rot_mat[7], self.relik_rot_mat[8]]]
            quat2 = T.quaternion_from_matrix(rotmat)
            # print(quat2)

            ## TODO: get real quat2
            # quat2 = [rot.w, rot.x, rot.y, rot.z]



            ret_quat = multiply_quat(quat2, quat1)


            pose_r = Pose()
            pose_r.position.x = pos.x
            pose_r.position.y = pos.y
            pose_r.position.z = pos.z

            pose_r.orientation.w = ret_quat[0]
            pose_r.orientation.x = ret_quat[1]
            pose_r.orientation.y = ret_quat[2]
            pose_r.orientation.z = ret_quat[3]

            print(pose_r)
            # print()

            self.send_position_relik(pose_r, None)
            update_time = rospy.get_time()

    def point_at_relik_2(self):
        '''
        Point end effector at AR marker 4 using RelaxedIK
        calculated using a rotation matrix
        '''
        while (not rospy.is_shutdown()):
            odom_rotmat = self.get_odom_to_right_matrix()
            matrix = self.get_right_matrix()
            self.arm_pos = self.transform(Point(0, 0, 0), matrix)

            if self.ar4_pos is None:
                continue

            try:
                vec1 = np.array(
                [self.ar4_pos.x - self.arm_pos.x, self.ar4_pos.y - self.arm_pos.y, self.ar4_pos.z - self.arm_pos.z])
            except:
                continue
            vec1 = vec1 / (vec1 ** 2).sum() ** 0.5
            vec2 = np.array([vec1[1], -1*vec1[0], 0])
            vec3 = np.cross(vec1, vec2)
            vec3 = vec3 / (vec3 ** 2).sum() ** 0.5
            # return

            # vec1 = np.array([1, 0, 0])
            # vec2 = np.array([0, 1, 0])
            # vec3 = np.array([0, 0, 1])

            # vec1[0] vec2[0] vec3[0]
            # vec1[1] vec2[1] vec3[1]
            # vec1[2] vec2[2] vec3[2]
            #
            # -vec3[0] vec2[0] vec1[0]
            # -vec3[1] vec2[1] vec1[1]
            # -vec3[2] vec2[2] vec1[2]

            if (self.relik_rot_mode == "relative"):
                rotmat = np.matrix([[-vec3[0], vec2[0], vec1[0]],
                                    [-vec3[1], vec2[1], vec1[1]],
                                    [-vec3[2], vec2[2], vec1[2]]])
            else:
                rotmat = np.matrix([[vec1[0], vec2[0], vec3[0]],
                                    [vec1[1], vec2[1], vec3[1]],
                                    [vec1[2], vec2[2], vec3[2]]])
            init_rotmat = np.matrix([[vec1[0], vec2[0], vec3[0]], [vec1[1], vec2[1], vec3[1]], [vec1[2], vec2[2], vec3[2]]])
            # rotmat = np.matmul(init_rotmat, odom_rotmat)
            
            quat = T.quaternion_from_matrix(rotmat)

            # quat = [0.7, 0, 0.7, 0]

            print(rotmat)
            print(quat)

            pos = self.relik_pose_r.position

            pose_r = Pose()
            pose_r.position.x = pos.x
            pose_r.position.y = pos.y
            pose_r.position.z = pos.z

            pose_r.orientation.w = quat[0]
            pose_r.orientation.x = quat[1]
            pose_r.orientation.y = quat[2]
            pose_r.orientation.z = quat[3]

            # print(pose_r)

            self.send_position_relik(pose_r, None)

    def calibrate(self):
        input("Enter to calibrate")
        # move to points
        test_points = []
        test_points.append([0.4, 0, 0.2])
        test_points.append([0.4, -0.2, 0.2])
        test_points.append([0.5, -0.1, 0.2])
        test_points.append([0.5, -0.3, 0.2])
        test_points.append([0.56, 0.1, 0.2])
        test_points.append([0.4, -0.1, 0.3])
        test_points.append([0.58, 0.1, 0.1])
        test_points.append([0.57, 0.2, 0.1])
        test_points.append([0.58, 0, 0.1])
        test_points.append([0.57, 0, 0])
        test_points.append([0.5, -0.2, 0])
        test_points.append([0.4, -0.2, 0])
        test_points.append([0.2, -0.2, 0])
        test_points.append([0, -0.15, 0])


        robot_pts = np.array([[0, 0, 0]])
        cam_pts = np.array([[0, 0, 0]])

        for i, pt in enumerate(test_points):
            pose_r = Pose()
            pose_r.position.x = pt[0]
            pose_r.position.y = pt[1]
            pose_r.position.z = pt[2]

            pose_r.orientation.w = 1
            pose_r.orientation.x = 0
            pose_r.orientation.y = 0
            pose_r.orientation.z = 0

            print("sent %d: %.2f, %.2f, %.2f" % (i, pt[0], pt[1], pt[2]))
            self.send_position_relik(pose_r, None)

            t = rospy.get_time()
            while rospy.get_time() - t < 3:
                if rospy.is_shutdown():
                    return
            print("eeps %d: %.2f, %.2f, %.2f" % (i, self.relik_ee_pos[0][0], self.relik_ee_pos[0][1], self.relik_ee_pos[0][2]))

            # TODO: send same pos multiple times
            if self.r_arm_pos_ar is not None:
                robot_pts = np.append(robot_pts, [self.relik_ee_pos[0]], axis=0)
                cam_pts = np.append(cam_pts, [[self.r_arm_pos_ar.x, self.r_arm_pos_ar.y, self.r_arm_pos_ar.z]], axis=0)
                print(self.r_arm_pos_ar)

            print("\n")

        print(robot_pts)
        print(cam_pts)
        retval, self.calib_matrix, inliers = cv.estimateAffine3D(cam_pts, robot_pts)
        self.calib_matrix = np.append(self.calib_matrix, [[0, 0, 0, 1]], axis=0)
        print(self.calib_matrix)
        print("done")
        self.test_calibration()
        return

    def test_calibration(self):
        input("Enter to test calibration")

        test_points = []
        test_points.append([0.4, -0.1, 0.23])
        test_points.append([0.55, -0.1, 0.28])
        test_points.append([0.58, 0.09, 0.31])
        test_points.append([0.53, 0, 0.25])
        test_points.append([0.58, -0.12, 0.23])
        test_points.append([0.58, 0, 0.14])
        test_points.append([0.52, -0.28, 0.14])
        test_points.append([0.5, -0.23, 0.12])
        test_points.append([0.52, -0.11, 0])
        test_points.append([0.43, -0.14, 0])
        test_points.append([0.43, -0.14, 0.35])
        test_points.append([0.50, -0.14, 0.4])
        test_points.append([0.45, -0.14, 0.2])
        test_points.append([0.3, -0.14, 0])
        test_points.append([0.2, -0.15, 0])
        test_points.append([0, -0.15, 0])

        for i, pt in enumerate(test_points):
            pose_r = Pose()
            pose_r.position.x = pt[0]
            pose_r.position.y = pt[1]
            pose_r.position.z = pt[2]

            pose_r.orientation.w = 1
            pose_r.orientation.x = 0
            pose_r.orientation.y = 0
            pose_r.orientation.z = 0

            print("sent %d: %.2f, %.2f, %.2f" % (i, pt[0], pt[1], pt[2]))
            self.send_position_relik(pose_r, None)

            t = rospy.get_time()
            while rospy.get_time() - t < 3:
                if rospy.is_shutdown():
                    return
            print("eeps %d: %.2f, %.2f, %.2f" % (
            i, self.relik_ee_pos[0][0], self.relik_ee_pos[0][1], self.relik_ee_pos[0][2]))

            # TODO: send same pos multiple times
            if self.r_arm_pos_ar is not None:
                trans_pts = self.transform(self.r_arm_pos_ar, self.calib_matrix)
                print("cams %d: %.2f, %.2f, %.2f" % (i, trans_pts.x, trans_pts.y, trans_pts.z))
            print("\n")

    def test_repeatability(self):
        input("Enter to test repeatability")

        test_points = []
        test_points.append([0, -0.1, 0])
        test_points.append([0.4, -0.2, 0.2])
        n = 5
        sums = np.array([0.0, 0.0, 0.0])
        acc_sum = np.array([0.0, 0.0, 0.0])
        points = []
        for j in range(n):
            for i, pt in enumerate(test_points):
                pose_r = Pose()
                pose_r.position.x = pt[0]
                pose_r.position.y = pt[1]
                pose_r.position.z = pt[2]

                pose_r.orientation.w = 1
                pose_r.orientation.x = 0
                pose_r.orientation.y = 0
                pose_r.orientation.z = 0

                self.send_position_relik(pose_r, None)
                print("sent %d: %.5f, %.5f, %.5f" % (j, pt[0], pt[1], pt[2]))

                t = rospy.get_time()
                while rospy.get_time() - t < 4:
                    if rospy.is_shutdown():
                        return
                print("eeps %d: %.5f, %.5f, %.5f" % (
                    j, self.relik_ee_pos[0][0], self.relik_ee_pos[0][1], self.relik_ee_pos[0][2]))

            matrix = self.get_transform_matrix("/pan_base_link", "/right_wrist_3_link")
            pts = self.transform(Point(0, 0, 0), matrix)
            # print_point(pts)
            print("\n")
            points.append(pts)
            sums[0] = sums[0] + pts.x
            sums[1] = sums[1] + pts.y
            sums[2] = sums[2] + pts.z
            acc_sum[0] += self.relik_ee_pos[0][0]
            acc_sum[1] += self.relik_ee_pos[0][1]
            acc_sum[2] += self.relik_ee_pos[0][2]

        sums = sums / n
        acc_sum /= n
        print("center: %f, %f, %f" % (sums[0], sums[1], sums[2]))

        avg_length, lengths = get_average_length(sums, points)
        std_dev = get_std_dev(avg_length, lengths)
        repeatability = avg_length + 3*std_dev
        acc = math.sqrt((acc_sum[0] - test_points[1][0])**2 + (acc_sum[1] - test_points[1][1] + (acc_sum[2]) - test_points[1][2])**2)
        print("avg_length: %f" % avg_length)
        # print(lengths)
        print("std_dev: %f" % std_dev)
        print("rpt: %f" % repeatability)
        print("accuracy: %f" % acc)

    def test1(self):
        ## Changed for test
        axis_norm = np.array([1, 0, 0])
        angle = 3.14 / 2

        arm_cmd = JacoCartesianVelocityCmd()
        arm_cmd.header.stamp = rospy.get_rostime()
        arm_cmd.header.frame_id = ''
        euler = aa_to_euler(axis_norm, angle)
        s = 5
        arm_cmd.theta_x = euler[0] * s
        arm_cmd.theta_y = euler[1] * s
        arm_cmd.theta_z = euler[2] * s

        self.send_arm_cmd(arm_cmd)

        return

        ## End test

    def test2(self):
        axis_norm = np.array([1, 0, 0])
        angle = 3.14/2

        pos = self.relik_pose_r.position
        rot = self.relik_pose_r.orientation

        euler = aa_to_euler(axis_norm, angle)

        q_rot = T.quaternion_from_euler(euler[1], -1*euler[0], euler[2])

        quat1 = aa_to_quaternion(axis_norm, angle)
        quat2 = [rot.w, rot.x, rot.y, rot.z]
        ret_quat = multiply_quat(quat2, quat1)

        print(q_rot)
        # print(ret_quat)

        pose_r = Pose()
        pose_r.position.x = pos.x
        pose_r.position.y = pos.y
        pose_r.position.z = pos.z

        pose_r.orientation.w = q_rot[0]
        pose_r.orientation.x = q_rot[1]
        pose_r.orientation.y = q_rot[2]
        pose_r.orientation.z = q_rot[3]

        self.send_position_relik(pose_r, None)

    def js_callback(self, data):
        self.arm_js = data.position
        return

    def move_arm_ang_vel(self):
        cmd = JacoAngularVelocityCmd7DOF()
        cmd.header.stamp = rospy.Time.now()
        cmd.theta_wrist_3_joint = -35
        # print(cmd)
        self.r_arm_ang_pub.publish(cmd)

    # def move_arm_joint(self):
    #     goal = None
    #     while not rospy.is_shutdown():
    #         if self.arm_js is None:
    #             continue
    #         if goal is None:
    #             goal = FollowJointTrajectoryGoal()
    #             arm = 'right'
    #             goal.trajectory.joint_names = ['%s_shoulder_pan_joint' % arm,
    #                                            '%s_shoulder_lift_joint' % arm,
    #                                            '%s_arm_half_joint' % arm,
    #                                            '%s_elbow_joint' % arm,
    #                                            '%s_wrist_spherical_1_joint' % arm,
    #                                            '%s_wrist_spherical_2_joint' % arm,
    #                                            '%s_wrist_3_joint' % arm]
    #             goal.goal_time_tolerance = rospy.Time(0.1)
    #         point = JointTrajectoryPoint()
    #         point.positions = [0]*7
    #         for i in range(len(self.arm_js)):
    #             point.positions[i] = self.arm_js[i]
    #         # point.positions = self.arm_js
    #         point.positions[6] = point.positions[6] + 0.001
    #         # point.positions = list(self.arm_js)
    #         print(point.positions[6])
    #         point.velocities = [0]*7
    #         point.time_from_start = rospy.Duration((len(goal.trajectory.points) + 1.0) / 10)
    #         goal.trajectory.points.append(point)
    #         if len(goal.trajectory.points) == 10:
    #             goal.trajectory.header.stamp = rospy.Time.now()
    #             # self.r_arm_joint_ctl.send_goal(goal)
    #             # print(goal)
    #             goal = None

    def send_arm_cmd(self, cmd):
        '''
        Sends JacoCartesianVelocityCmd to
        '/movo/right_arm/cartesian_vel_cmd' topic
        twice every second (2hz)

        :param cmd:
        '''
        if rospy.get_time() - self.arm_update > 0.01:
            self.arm_pub.publish(cmd)
            print("send")
            self.arm_update = rospy.get_time()

    def move_gripper(self, pos):
        cmd = GripperCmd()
        cmd.position = pos  # 0 - 0.9
        cmd.speed = 0.5
        self.gripper_pub_r.publish(cmd)

    def send_position_relik(self, pose1, pose2):

        ee_pose_goals = EEPoseGoals()

        if (pose1 == None):
            pose1 = self.relik_pose_r

        if (pose2 == None):
            pose2 = Pose()
            pose2.position.x = 0
            pose2.position.y = 0
            pose2.position.z = 0

            pose2.orientation.w = 1
            pose2.orientation.x = 0
            pose2.orientation.y = 0
            pose2.orientation.z = 0

        pose_r = pose1
        pose_l = pose2

        self.relik_pose_r = pose_r

        ee_pose_goals.ee_poses.append(pose_r)
        ee_pose_goals.ee_poses.append(pose_l)

        self.relik_goal_pub.publish(ee_pose_goals)

    def follow_ar_relik(self):
        '''
        Follow AR tag 4 with the right arm using RelaxedIK
        '''
        while not rospy.is_shutdown():

            matrix = self.get_right_matrix()
            self.arm_pos = self.transform(Point(0, 0, 0), matrix)

            if self.found and matrix is not None:
                try:
                    dist = [self.ar4_pos.x - self.arm_pos_orig.x,
                            self.ar4_pos.y + 0.05 - self.arm_pos_orig.y,
                            self.ar4_pos.z - self.arm_pos_orig.z]
                except:
                    return

                # send_point = [dist[2], -1*dist[0], -1 *dist[1]]
                send_point = [dist[0], dist[1], dist[2]]

                # if (self.relik_pos_mode == "absolute"):
                #     init_pos = self.relik_ee_pos[2]
                #     send_point[0] += init_pos[0]
                #     send_point[1] += init_pos[1]
                #     send_point[2] += init_pos[2]

                pose_r = Pose()
                pose_r.position.x = send_point[0]
                pose_r.position.y = send_point[1]
                pose_r.position.z = send_point[2]

                pose_r.orientation.w = 1
                pose_r.orientation.x = 0
                pose_r.orientation.y = 0
                pose_r.orientation.z = 0

                self.send_position_relik(pose_r, None)
                self.arm_update = rospy.get_time()

    def relik_position_control(self):

        x = 0.5
        y = 0
        z = 0
        w = 1
        qx = 0
        qy = 0
        qz = 0
        print("Enter coords")
        x = input()
        if x == 10:
            print("quit")
            return
        y = input()
        z = input()
        # w = input()
        # qx = input()
        # qy = input()
        # qz = input()

        # if (self.relik_pos_mode == "absolute"):
        #     init_pos = self.relik_ee_pos[2]
        #     x += init_pos[0]
        #     y += init_pos[1]
        #     z += init_pos[2]

        pose_r = Pose()
        pose_r.position.x = x
        pose_r.position.y = y
        pose_r.position.z = z

        pose_r.orientation.w = w
        pose_r.orientation.x = qx
        pose_r.orientation.y = qy
        pose_r.orientation.z = qz


        self.send_position_relik(pose_r, None)

        time.sleep(2.5)

        print("eeps: %.4f, %.4f, %.4f" % (
            self.relik_ee_pos[0][0], self.relik_ee_pos[0][1], self.relik_ee_pos[0][2]))

        matrix = self.get_transform_matrix("/pan_base_link", "/right_wrist_3_link")
        pts = self.transform(Point(0, 0, 0), matrix)
        # print(pts)
        print("\n")

        # print(pose_r)
        # print("send_pose")

    def follow_ar(self):
        '''
        Follow the AR tag 4 with the right arm using MoveIt
        '''
        while not rospy.is_shutdown():

            matrix = self.get_right_matrix()
            p = Point()
            p.x, p.y, p.z = 0, 0, 0

            self.arm_pos = self.transform(p, matrix)
            # print(self.arm_pos)
            #
            # return

            if rospy.get_time() - self.arm_update > 0.05:
                if self.found and matrix is not None:
                    # print_point(self.ar4_pos)
                    # return
                    if self.ar4_pos is None or self.arm_pos is None:
                        return
                    diff = [self.ar4_pos.x - self.arm_pos.x, self.ar4_pos.y - self.arm_pos.y, self.ar4_pos.z - self.arm_pos.z]
                    # print_point(Point(diff[0], diff[1], diff[2]))
                    arm_vel = 0.1 # max = 0.1
                    arm_cmd = JacoCartesianVelocityCmd()
                    arm_cmd.header.stamp = rospy.get_rostime()
                    arm_cmd.header.frame_id = ''
                    if diff[0] > 0.02:
                        arm_cmd.z = arm_vel
                    elif diff[0] < -0.02:
                        arm_cmd.z = -arm_vel
                    else:
                        arm_cmd.z = 0
                    if diff[1] > 0.02:
                        arm_cmd.x = -arm_vel
                    elif diff[1] < -0.02:
                        arm_cmd.x = arm_vel
                    else:
                        arm_cmd.x = 0
                    if diff[2] > 0.2:
                        arm_cmd.y = -arm_vel
                    elif diff[2] < 0.1:
                        arm_cmd.y = arm_vel
                    else:
                        arm_cmd.y = 0
                    # print(arm_cmd)
                    self.arm_pub.publish(arm_cmd)
                    # print("send")
                    self.arm_update = rospy.get_time()

    def transform(self, pos, matrix):
        '''
        Transforms Position object with given matrix and return
        array of transformed point.

        :param pos: position of object in Point type with x,y,z coordinates
        :param matrix: used to apply transformation
        :return: Point object with transformed coordinates
        '''
        vec = np.array([pos.x, pos.y, pos.z, 1]).reshape((4, 1))
        tvec = np.dot(matrix, vec)
        t_point = Point()
        t_point.x = tvec[0][0] / tvec[3][0]
        t_point.y = tvec[1][0] / tvec[3][0]
        t_point.z = tvec[2][0] / tvec[3][0]
        return t_point

    def get_right_matrix(self):
        '''
        Get transformation matrix from /right_ee_link to /odom.

        :return: 4D affine transformation matrix
        '''
        try:
            self.listener.waitForTransform('/movo_camera_color_optical_frame', '/right_ee_link', rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.listener.lookupTransform('/movo_camera_color_optical_frame', '/right_ee_link', rospy.Time(0))
            q = [rot[3], rot[0], rot[1], rot[2]]
            rotmat = t3d.quaternions.quat2mat(q)
            matrix = t3d.affines.compose(trans, rotmat, [1, 1, 1])

            self.listener.waitForTransform('/odom', '/right_ee_link', rospy.Time(0),
                                           rospy.Duration(0.5))
            (trans2, rot2) = self.listener.lookupTransform('/odom', '/right_ee_link',
                                                         rospy.Time(0))
            q2 = [rot2[3], rot2[0], rot2[1], rot2[2]]
            rotmat2 = t3d.quaternions.quat2mat(q2)
            matrix2 = t3d.affines.compose(trans2, rotmat2, [1, 1, 1])

            # print(matrix)
            # print(matrix2)

            return matrix2
        except(tf.LookupException, tf.ConnectivityException):
            return None

    def get_odom_to_right_matrix(self):
        '''
        Get transformation matrix from /odom to /right_ee_link.

        :return: 4D affine transformation matrix
        '''
        try:
            self.listener.waitForTransform('/right_ee_link', '/odom', rospy.Time(0),
                                           rospy.Duration(0.5))
            (trans, rot) = self.listener.lookupTransform('/right_ee_link', '/odom',
                                                         rospy.Time(0))
            q = [rot[3], rot[0], rot[1], rot[2]]
            rotmat = t3d.quaternions.quat2mat(q)
            matrix = t3d.affines.compose(trans, rotmat, [1, 1, 1])
            # rotmat = T.quaternion_matrix(q)


            # print(rotmat)

            return rotmat
        except(tf.LookupException, tf.ConnectivityException):
            return None

    def get_transform_matrix(self, toFrame, fromFrame):
        try:
            self.listener.waitForTransform(toFrame, fromFrame, rospy.Time(0),
                                           rospy.Duration(0.5))
            (trans, rot) = self.listener.lookupTransform(toFrame, fromFrame,
                                                         rospy.Time(0))
            q = [rot[3], rot[0], rot[1], rot[2]]
            rotmat = t3d.quaternions.quat2mat(q)
            matrix = t3d.affines.compose(trans, rotmat, [1, 1, 1])
            # rotmat = T.quaternion_matrix(q)

            # print(rotmat)

            return matrix
        except(tf.LookupException, tf.ConnectivityException):
            return None

    def get_cam_matrix(self):
        '''
        Get transformation matrix from /movo_camera_color_optical_frame to /odom.

        :return: 4D affine transformation matrix
        '''
        try:
            self.listener.waitForTransform('/odom', '/movo_camera_color_optical_frame', rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.listener.lookupTransform('/odom', '/movo_camera_color_optical_frame', rospy.Time(0))
            q = [rot[3], rot[0], rot[1], rot[2]]
            rotmat = t3d.quaternions.quat2mat(q)
            matrix = t3d.affines.compose(trans, rotmat, [1, 1, 1])

            return matrix
        except(tf.LookupException, tf.ConnectivityException):
            return None

    def update_config(self, data):
        self.linact_vel_lim = data.teleop_linear_actuator_vel_limit
        self.arm_vel_lim = data.teleop_arm_vel_limit

    def find_tag(self):
        '''
        Pan head left to right to find ar_tag and pan camera to bring tag
        to center of field of view after it is found.
        '''
        time_diff = rospy.get_time()
        while not rospy.is_shutdown() and not self.found:
            if rospy.get_time() - time_diff >= 0.5:
                if self.head_pos >= 1.5 or self.head_pos <= -1.5:
                    self.head_change = -1*self.head_change
                self.head_pos += self.head_change
                self.move_head([self.head_pos, 0, 0.2])
                time_diff = rospy.get_time()

        print("found while")
        while not rospy.is_shutdown():
            if self.found and math.fabs(self.ar4_pos.x) > 0.1 and rospy.get_time() - time_diff >= 0.03:
                if self.ar4_pos.x < 0:
                    diff = -0.02
                else:
                    diff = 0.02
                self.head_pos += diff
                self.move_head([self.head_pos, 0, 0.5])
                time_diff = rospy.get_time()
            if math.fabs(self.ar4_pos.x) < 0.1 or rospy.get_time() - time_diff >= 2:
                break

        print(self.found)
        print("exit found while")
        # self.ar4_pose_prev = None

    def move_head(self, cmd):
        '''
        Move head using '/movo/head/cmd' topic and PanTiltCmd object

        :param cmd: Array of pan position, pan velocity, tilt position
        '''
        pt_cmd = PanTiltCmd()
        if cmd[0] > 1.5:
            cmd[0] = 1.5
        elif cmd[0] < -1.5:
            cmd[0] = -1.5

        self.head_pos = cmd[0]
        pt_cmd.pan_cmd.pos_rad = cmd[0]
        pt_cmd.pan_cmd.vel_rps = cmd[2]
        pt_cmd.tilt_cmd.pos_rad = cmd[1]
        self.head_pub.publish(pt_cmd)

    def move_linact(self, dist):
        '''
        Move linear actuator using '/movo/linear_actuator_cmd/' topic

        :param dist: distance to move from current position
        '''
        if not self.lin_update:
            self.lin_update = rospy.get_time()
        dt = rospy.get_time() - self.lin_update
        self.lin_cmd.desired_position_m += (dist * self.linact_vel_lim) * dt
        if self.lin_cmd.desired_position_m >= 0.4:
            self.lin_cmd.desired_position_m = 0.4
        elif self.lin_cmd.desired_position_m <= 0.08:
            self.lin_cmd.desired_position_m = 0.08

        self.lin_cmd.header.stamp = rospy.get_rostime()
        self.lin_cmd.header.frame_id = ''
        self.lin_pub.publish(self.lin_cmd)
        self.lin_cmd.header.seq += 1

        self.lin_update = rospy.get_time()

    def move_base(self, twist):
        '''
        Move base using Twist command and '/movo/teleop/cmd_vel' topic

        :param twist: Twist command to send
        '''
        base_cmd = Twist()
        base_cmd.linear.x = twist[0]
        base_cmd.linear.y = twist[1]
        base_cmd.angular.z = twist[2]
        self.base_pub.publish(base_cmd)

    def ar_cb(self, data):
        '''
        Saves position of AR marker 4 into a class variable
        and calls functions to move base and linear actuator.

        :param data: callback data from '/ar_pose_marker'
        '''
        self.found = False
        found_marker3 = False
        cam_matrix = self.get_cam_matrix()

        for marker in data.markers:
            if (marker.id == 4):
                self.found = True
                trans_points = self.transform(marker.pose.pose.position, cam_matrix)
                if self.ar4_pos_prev == None:
                    self.ar4_pos_prev = trans_points
                self.ar4_pos = trans_points
                self.calc_diff_x = (self.ar4_pos.z - self.ar4_pos_prev.z) / 1
                self.calc_diff_y = (self.ar4_pos.x - self.ar4_pos_prev.x) / 1
                self.calc_diff_z = (self.ar4_pos.y - self.ar4_pos_prev.y) / 1
                # print("%.2f" % self.ar4_pos.x)
            if (marker.id == 3):
                self.r_arm_pos_ar = marker.pose.pose.position
                found_marker3 = True
        if not self.found:
            self.calc_diff_x = None
            self.ar4_pos_prev = None
            self.ar4_pos = None
            self.calc_diff_z = 0
            # print("Not found")

        if not found_marker3:
            self.r_arm_pos_ar = None

        base_move = [0, 0, 0]
        if self.calc_diff_x:
            if self.calc_diff_x >= 0.02 or self.calc_diff_x <= -0.02:
                base_move[0] = self.calc_diff_x
            if self.calc_diff_y >= 0.02 or self.calc_diff_y <= -0.02:
                base_move[1] = -1*self.calc_diff_y

        # self.move_linact(-10*self.calc_diff_z)
        # self.move_base(base_move)

if __name__ == "__main__":
    rospy.init_node('ar_control')
    ar_mover = ARMover()
    ar_mover.start()