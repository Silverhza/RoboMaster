#!/usr/bin/python


from readline import get_line_buffer
import rospy
import numpy as np
import math
from roborts_msgs.msg import GimbalAngle, RobotHeat, GameStatus
import actionlib
import sys
import roborts_msgs.srv as rtssrv
import cv2
from yolo.msg import ArmorPoints, Box
import tf
import os

from autofire.msg import enemy_id

from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import LaserScan

from tf.msg import tfMessage

from time import time

from std_msgs.msg import Bool

from obstacle_detector.msg import Obstacles

import time as T
import pynput
from termcolor import cprint
sys.path.append('src/autofire/src/CisW')
from calibrate_mhn import GimbalController

import matplotlib.pyplot as plt
'''
    This program is designed on one assumption:
        The message given by scan topic is a list which represents different value
        from the center of circle.
    Thus, this program is aimed to reduce these message information
'''

def restore_obstacle(msg):
    lidar_msg = msg
    lidar_detail_dis = msg.ranges

    points = []
    angle = -1.570
    angle_increment = 0.0043
    for dis in lidar_detail_dis:
        x = round(dis * math.sin(angle), 2)
        y = round(dis * math.cos(angle), 2)
        points.append([x, y])
        angle += angle_increment

    points = np.array(points)
    # Display
    plt.scatter(points[:, 0], points[:, 1], s=1)
    plt.show()
    return None

if __name__ == '__main__':
    pid =  os.getpid()
    rospy.init_node("Visualize", anonymous=True)
    CAR_ID="CAR2"

    control = GimbalController()
    scan_sub = rospy.Subscriber("/"+CAR_ID+"/scan", LaserScan, restore_obstacle)
    rospy.spin()