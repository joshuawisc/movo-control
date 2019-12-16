#!/usr/bin/python

from ar_control import aa_to_euler, aa_to_quaternion
from geometry_msgs.msg import Twist, Point


if __name__ == '__main__':
    rot = aa_to_quaternion([0.8, 0.7, 0.7], 2.06)
    print("x: %.3f, y: %.3f, z: %.3f, w: %.3f" % (rot[1], rot[2], rot[3], rot[0]))