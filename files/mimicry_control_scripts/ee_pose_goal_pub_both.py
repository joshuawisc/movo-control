#!/usr/bin/python
import rospy
from relaxed_ik.msg import JointAngles
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, Pose
from relaxed_ik.msg import EEPoseGoals
from std_msgs.msg import Int8, String
import transformations as T
import copy
import numpy as np

glob_vector3_r = ''
def vector3_right_cb(data):
    global glob_vector3_r
    glob_vector3_r = []
    glob_vector3_r.append(data.vector.x)
    glob_vector3_r.append(data.vector.y)
    glob_vector3_r.append(data.vector.z)



glob_quaternion_r = ''
def quaternion_right_cb(data):
    global glob_quaternion_r
    glob_quaternion_r = []
    glob_quaternion_r.append(data.quaternion.w)
    glob_quaternion_r.append(data.quaternion.x)
    glob_quaternion_r.append(data.quaternion.y)
    glob_quaternion_r.append(data.quaternion.z)

glob_vector3_l = ''
def vector3_left_cb(data):
    global glob_vector3_l
    glob_vector3_l = []
    glob_vector3_l.append(data.vector.x)
    glob_vector3_l.append(data.vector.y)
    glob_vector3_l.append(data.vector.z)


glob_quaternion_l = ''
def quaternion_left_cb(data):
    global glob_quaternion_l
    glob_quaternion_l = []
    glob_quaternion_l.append(data.quaternion.w)
    glob_quaternion_l.append(data.quaternion.x)
    glob_quaternion_l.append(data.quaternion.y)
    glob_quaternion_l.append(data.quaternion.z)


glob_clutch_down = 0
def clutch_cb(data):
    global glob_clutch_down
    glob_clutch_down = data.data

glob_grab_down = 0
def grab_cb(data):
    global glob_grab_down
    if data.data == 1 and glob_grab_down == 0:
        commands_pub.publish("button_pressed")
    elif data.data == 0 and glob_grab_down == 1:
        commands_pub.publish("button_released")
    glob_grab_down = data.data

rospy.init_node('pose_goal_pub')

rospy.Subscriber('/vive_controller/quaternion_r', QuaternionStamped, quaternion_right_cb)
rospy.Subscriber('/vive_controller/position_r', Vector3Stamped, vector3_right_cb)
rospy.Subscriber('/vive_controller/quaternion_l', QuaternionStamped, quaternion_left_cb)
rospy.Subscriber('/vive_controller/position_l', Vector3Stamped, vector3_left_cb)
rospy.Subscriber('/vive_controller/clutch', Int8, clutch_cb)
rospy.Subscriber('/vive_controller/grab', Int8, grab_cb)
commands_pub = rospy.Publisher('/interaction/commands', String, queue_size=3)
eepg_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)
rospy.sleep(0.3)

while glob_vector3_r == '': continue


mode = 'clutch'

clutch_val = 0
prev_clutch_down = 0
prev_pos_goal_r = [0,0,0]
prev_quat_goal_r = [1,0,0,0]
prev_pos_raw_r = [0,0,0]
prev_quat_raw_r = [1,0,0,0]
position_on_clutch_r = [0,0,0]
rotation_on_clutch_r = [1,0,0,0]

prev_pos_goal_l = [0,0,0]
prev_quat_goal_l = [1,0,0,0]
prev_pos_raw_l = [0,0,0]
prev_quat_raw_l = [1,0,0,0]
position_on_clutch_l = [0,0,0]
rotation_on_clutch_l = [1,0,0,0]


rate = rospy.Rate(100)
while not rospy.is_shutdown():
    quaternion_r = copy.deepcopy(glob_quaternion_r)
    vector3_r = copy.deepcopy(glob_vector3_r)

    quaternion_l = copy.deepcopy(glob_quaternion_l)
    vector3_l = copy.deepcopy(glob_vector3_l)

    clutch_down = glob_clutch_down

    length_error = False
    if not len(vector3_r) == 3: length_error = True
    if not len(quaternion_r) == 4: length_error = True

    if length_error: 
        rate.sleep()
        continue
    
    eepg = EEPoseGoals()
    pose_r = Pose()
    pose_l = Pose()

    left = True
    if not len(vector3_l) == 3 or not len(quaternion_l) == 4:
        left = False

    if not left:
        pose_l = Pose()
        pose_l.position.x = 0
        pose_l.position.y = 0
        pose_l.position.z = 0

        pose_l.orientation.w = 1
        pose_l.orientation.x = 0
        pose_l.orientation.y = 0
        pose_l.orientation.z = 0

    if mode == 'standard':
        ## Right
        pose_r.position.x = vector3_r[0]
        pose_r.position.y = vector3_r[1]
        pose_r.position.z = vector3_r[2]

        pose_r.orientation.w = quaternion_r[0]
        pose_r.orientation.x = quaternion_r[1]
        pose_r.orientation.y = quaternion_r[2]
        pose_r.orientation.z = quaternion_r[3]

        ## Left
        if left:
            pose_l.position.x = vector3_l[0]
            pose_l.position.y = vector3_l[1]
            pose_l.position.z = vector3_l[2]

            pose_l.orientation.w = quaternion_l[0]
            pose_l.orientation.x = quaternion_l[1]
            pose_l.orientation.y = quaternion_l[2]
            pose_l.orientation.z = quaternion_l[3]

    elif mode == 'clutch':
        if (clutch_down != prev_clutch_down) and (clutch_down == 1):
            # It's fine not to deep-copy since values are not updated unless clutch == 0
            position_on_clutch_r = prev_pos_goal_r
            rotation_on_clutch_r = prev_quat_goal_r

            position_on_clutch_l = prev_pos_goal_l
            rotation_on_clutch_l = prev_quat_goal_l

        if clutch_down == 1:
            pose_r.position.x = position_on_clutch_r[0]
            pose_r.position.y = position_on_clutch_r[1]
            pose_r.position.z = position_on_clutch_r[2]

            pose_r.orientation.w = rotation_on_clutch_r[0]
            pose_r.orientation.x = rotation_on_clutch_r[1]
            pose_r.orientation.y = rotation_on_clutch_r[2]
            pose_r.orientation.z = rotation_on_clutch_r[3]

            if left:
                pose_l.position.x = position_on_clutch_l[0]
                pose_l.position.y = position_on_clutch_l[1]
                pose_l.position.z = position_on_clutch_l[2]

                pose_l.orientation.w = rotation_on_clutch_l[0]
                pose_l.orientation.x = rotation_on_clutch_l[1]
                pose_l.orientation.y = rotation_on_clutch_l[2]
                pose_l.orientation.z = rotation_on_clutch_l[3]

        elif clutch_down == 0:
            ## Right
            translation_velocity_r = np.array(vector3_r) - np.array(prev_pos_raw_r)
            curr_pos_goal_r = np.array(prev_pos_goal_r) + translation_velocity_r
            pose_r.position.x = curr_pos_goal_r[0]
            pose_r.position.y = curr_pos_goal_r[1]
            pose_r.position.z = curr_pos_goal_r[2]
            prev_pos_goal_r = curr_pos_goal_r

            qv1_r = T.rotate_quaternion_representation(prev_quat_raw_r, T.quaternion_matrix(quaternion_r)[:3,:3])
            qv_r = T.rotate_quaternion_representation(T.quaternion_dispQ(qv1_r, quaternion_r),
                                                    T.quaternion_matrix([1., 0., 0., 0.])[:3, :3])
            curr_quat_goal_r = T.quaternion_multiply(qv_r, prev_quat_goal_r)

            pose_r.orientation.w = curr_quat_goal_r[0]
            pose_r.orientation.x = curr_quat_goal_r[1]
            pose_r.orientation.y = curr_quat_goal_r[2]
            pose_r.orientation.z = curr_quat_goal_r[3]
            prev_quat_goal_r = curr_quat_goal_r

            ##Left
            if left:
                translation_velocity_l = np.array(vector3_l) - np.array(prev_pos_raw_l)
                curr_pos_goal_l = np.array(prev_pos_goal_l) + translation_velocity_l
                pose_l.position.x = curr_pos_goal_l[0]
                pose_l.position.y = curr_pos_goal_l[1]
                pose_l.position.z = curr_pos_goal_l[2]
                prev_pos_goal_l = curr_pos_goal_l

                qv1_l = T.rotate_quaternion_representation(prev_quat_raw_l, T.quaternion_matrix(quaternion_l)[:3, :3])
                qv_l = T.rotate_quaternion_representation(T.quaternion_dispQ(qv1_l, quaternion_l),
                                                          T.quaternion_matrix([1., 0., 0., 0.])[:3, :3])
                curr_quat_goal_l = T.quaternion_multiply(qv_l, prev_quat_goal_l)

                pose_l.orientation.w = curr_quat_goal_l[0]
                pose_l.orientation.x = curr_quat_goal_l[1]
                pose_l.orientation.y = curr_quat_goal_l[2]
                pose_l.orientation.z = curr_quat_goal_l[3]
                prev_quat_goal_l = curr_quat_goal_l

        prev_pos_raw_r = copy.deepcopy(vector3_r)
        prev_quat_raw_r = copy.deepcopy(quaternion_r)

        if left:
            prev_pos_raw_l = copy.deepcopy(vector3_l)
            prev_quat_raw_l = copy.deepcopy(quaternion_l)

        prev_clutch_down = clutch_down

    eepg.ee_poses.append(pose_r)
    eepg.ee_poses.append(pose_l)


    # ### JOSH
    # pose_l = Pose()
    # pose_l.position.x = 0
    # pose_l.position.y = 0
    # pose_l.position.z = 0
    #
    # pose_l.orientation.w = 1
    # pose_l.orientation.x = 0
    # pose_l.orientation.y = 0
    # pose_l.orientation.z = 0
    # eepg.ee_poses.append(pose_l)
    # ########

    eepg_pub.publish(eepg)

    rate.sleep()


