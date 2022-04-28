#!/usr/bin/python

from ossaudiodev import control_labels
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
from pynput import mouse, keyboard

class GimbalController(object):
    def __init__(self,P=0.4,I=0.0,D=0): ## 0.42, 0, 0

        self.P_value = P
        self.P=P
        self.I=I
        self.D=D
        self.last_error=0
        self.Ierror=0

        self.pitch_size = 8
        self.pitch_index = 0 
        self.pitch_buffer = np.zeros(self.pitch_size)

        self.distance=0
        self.last_dist = 0

        self.image_height = 480
        self.image_width = 640

        self.timer = 0

        self.current_gimbal_rotaion = 0

        self.current_time = 0

        self.heat_limit = 240
        self.current_heat = 0
        self.bullet_speed = 23
        self.lidar_distance = 0




        ##### referee system ####
        self.Game_Start = 4
        self.Game_End = 5
        self.shoot_enable = False


    def convert_error_space(self,x):
        x = float(x)

        pixel_error = self.image_width / 2 - x

        error = (self.image_width / 2 - x) / self.image_width * 3.14 * 0.36

        pencent = (self.image_width / 2 - x) / (self.image_width / 2) * 100

        return error


    def box_center(self, box):
        return (box.xmin+box.xmax)/2, (box.ymin+box.ymax)/2


    def get_lidar_dis(self, msg):
        dis_array = msg.ranges
        mid=int(len(msg.ranges)/2)

        scan_list = []

        for i in range(3):
            if dis_array[mid-i] != float("inf"):
                scan_list.append(dis_array[mid-i])
            if dis_array[mid+i] != float("inf"):
                scan_list.append(dis_array[mid+i])
                       
        sum = 0
        for i in range(len(scan_list)):
            sum+=scan_list[i]
        self.lidar_distance = sum/len(scan_list)

    def yawController(self,box):

        centerx,centery=self.box_center(box)

        errorx=self.convert_error_space(centerx)

        self.Ierror += errorx
        if self.Ierror>1:
            self.Ierror=1
        if self.Ierror<-1:
            self.Ierror=-1


        yaw_angle=(errorx)*self.P+self.D*(errorx-self.last_error)+self.Ierror*self.I


        self.last_error=errorx

        return yaw_angle

    def change_yaw_angle(self, box, yaw_mode=True, pitch_mode=False): # Depend on box
        cmd = GimbalAngle()
        self.distance = float(box.distance)

        cmd.yaw_mode = yaw_mode
        cmd.pitch_mode = pitch_mode
        cmd.yaw_angle = self.yawController(box)

        cmd_pub.publish(cmd)

    
    def change_pitch_angle(self, dis, yaw_mode=True, pitch_mode=False):
        cmd = GimbalAngle()

        cmd.yaw_mode = yaw_mode
        cmd.pitch_mode = pitch_mode
        target_pitch_angle = -0.0082*dis**3 + 0.0938*dis**2 - 0.3728*dis + 0.453
        if target_pitch_angle > 0.33:
            target_pitch_angle = 0.33
        elif target_pitch_angle < -0.33:
            target_pitch_angle = -0.33

        # Avoid violent shaking
        if abs(cmd.pitch_angle - target_pitch_angle) > 0.13:
            return
        else:
            cmd.pitch_angle = target_pitch_angle
        cmd_pub.publish(cmd)  


    def search_enemy(self, msg):
        
        self.num_detect = msg.numDetected

        if self.num_detect < 1 :
            # No detecting enemy
            return

        else:
            # Change yaw angle
            box = msg.bbox[0]
            self.change_yaw_angle(box)
            self.change_pitch_angle(self.lidar_distance)
            self.shoot_by_mouse()

    
    def shoot(self, x, y, button, pressed):
        if button == mouse.Button.left and pressed:
            print('---Shoot---')
        


    def shoot_by_mouse(self):
        mouse_listener = mouse.Listener(on_click=self.shoot)
        mouse_listener.start()

if __name__ == '__main__':
    pid = os.getpid()

    os.system("taskset -pc 1 %s"%pid)    
    rospy.init_node("Core", anonymous=True)

    CAR_ID = "CAR2"
    control = GimbalController()

    cmd_pub = rospy.Publisher("/"+CAR_ID+"/cmd_gimbal_angle",GimbalAngle,queue_size=10)
    rospy.Subscriber("/"+CAR_ID+"/scan", LaserScan, control.get_lidar_dis) # Update gimbal lidar distance
    # rospy.Subscriber("/"+CAR_ID+"/detector/armormsg", ArmorPoints, control.search_enemy)

    
    rospy.spin()