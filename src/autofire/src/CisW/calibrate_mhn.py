#!/usr/bin/python

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
class PoseEstimator(object):
    def __init__(self):
        self._armorWidth=140
        self._armorHeight=130
        self.objPoints=self.creatObjPoints()

        if CAR_ID == "CAR1":
            self._K=np.array([[229.93762, 0.0, 161.18925],[ 0.0, 230.03427, 117.33975],[0.0, 0.0, 1.0]])
            self._D=np.array([0.035273, -0.058764, 0.000471, 0.000958, 0.000000])

        elif CAR_ID == "CAR2":
            self._K=np.array([[228.85851, 0.0, 165.17667],[ 0.0, 229.02883, 119.46514],[0.0, 0.0, 1.0]])
            self._D=np.array([0.038931, -0.063678, 0.000372, -0.000037, 0.000000])

        else:
            rospy.loginfo("calibration wrong")

    def creatObjPoints(self):
        objpoints=[]#start at lefttop, clockwise 
        objpoints.append([-self._armorWidth/2,self._armorHeight/2,0])
        objpoints.append([self._armorWidth/2,self._armorHeight/2,0])
        objpoints.append([self._armorWidth/2,-self._armorHeight/2,0])
        objpoints.append([-self._armorWidth/2,-self._armorHeight/2,0])
        return np.array(objpoints,dtype=float)

    def PnP(self,imgPoints):
        retval,rvec,tvec=cv2.solvePnP(self.objPoints,imgPoints.reshape((4,1,2)),self._K, self._D)
        return retval,rvec,tvec


class GimbalController(object):
    # def __init__(self,P=0.4,I=0.0,D=0): ## 0.42, 0, 0
    def __init__(self,P=0.45,I=0.0125,D=1.0): ## 0.42, 0, 0     P=0.29,I=0.0,D=0.79

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


        ##### referee system ####
        self.Game_Start = 4
        self.Game_End = 5
        self.shoot_enable = False



        # self.current_chassis_rotaion = 0 

        ##### enemy look at #####
        self.my_x = 0
        self.my_y = 0
        self.chassis_orientation = 0 ## in radius ##
        self.gimbal_orientation = 0 ## in radius ##

        self.enemy_x = 0
        self.enemy_y = 0
        
        self.last_track_time = 0
        self.last_pitch = 0

        self.direction_flag = False
        self.start_search = False

        self.counter = 0

        self.rotate_angle = 0.06

        self.lidar_distance = 0


        ##### publish enemy id ####
        self.enemy_orientation = 0
        # self.box = 0


    def convert_error_space(self,x):
        x = float(x)

        pixel_error = self.image_width / 2 - x

        error = (self.image_width / 2 - x) / self.image_width * 3.14 * 0.36

        pencent = (self.image_width / 2 - x) / (self.image_width / 2) * 100

        return error
        
    def box_center(self, box):
        return (box.xmin+box.xmax)/2, (box.ymin+box.ymax)/2
        
        
    def yawController(self,box):

        centerx,centery=self.box_center(box)

        errorx=self.convert_error_space(centerx)

        # if abs(errorx)>0.3:
        #     self.Ierror=0

        self.Ierror += errorx
        if self.Ierror>1:
            self.Ierror=1
        if self.Ierror<-1:
            self.Ierror=-1


        yaw_angle=(errorx)*self.P+self.D*(errorx-self.last_error)+self.Ierror*self.I


        # if abs(errorx) > 0.2:
        #     yaw_angle=(errorx)*self.P*0.6+self.D*(errorx-self.last_error)+self.Ierror*self.I

        # else:
        #     yaw_angle=(errorx)*self.P+self.D*(errorx-self.last_error)+self.Ierror*self.I

        self.last_error=errorx

        return yaw_angle

        
    def get_robot_heat(self, msg):
        self.current_heat = msg.shooter_heat


    def publish_enemy_id(self, box):
        if abs(self.last_error)>0.08:
            return
        
        
        info = enemy_id()

        if self.chassis_orientation >= 0:

        
            self.enemy_orientation = self.chassis_orientation  + self.gimbal_orientation

            # if orientation < 0:
                
            #     orientation = 3.14 * 2 + orientation

        else:
            self.enemy_orientation = 3.14 * 2 + self.chassis_orientation  + self.gimbal_orientation

        info.x = self.my_x + self.distance * 1.2 * math.cos(self.enemy_orientation)
        info.y = self.my_y + self.distance * 1.2 * math.sin(self.enemy_orientation)
        info.id = box.obj


        info_pub.publish(info)


    def gimbal_cmd(self, box, yaw_mode=True, pitch_mode=False):
        cmd = GimbalAngle()
        self.distance = float(box.distance)
        print(self.lidar_distance)
        
        cmd.yaw_mode = yaw_mode
        cmd.pitch_mode = pitch_mode    
        cmd.yaw_angle = self.yawController(box)
        cmd.pitch_angle= pitchTest
        cmd_pub.publish(cmd)

        # self.shootController()

        # if self.shoot_enable:
        #     self.shootController()

        # self.publish_enemy_id(box)

        self.last_track_time = time()
        self.start_search = False

        self.P = self.P_value


    def callback(self,msg):

        self.num_detect = msg.numDetected

        if self.num_detect<1 :
            return

        else:
            box = msg.bbox[0]          
            self.gimbal_cmd(box)

        return 

    def shoot_one(self, cmd_fric, cmd_shoot):
        cmd_shoot(1, 1, 1500)

    def adjust_pitch_angle(self, tar_angle):
        start_time = T.time()
        while True:
            cmd = GimbalAngle()
            cmd.yaw_mode = False
            cmd.pitch_mode = False
            cmd.yaw_angle = 0
            cmd.pitch_angle = tar_angle
            cmd_pub.publish(cmd)
            if T.time()-start_time > 0.01:
                break

    def get_lidar_dis(self, msg):  
        global scan_sub, current_scan_distance
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
        current_scan_distance = round(self.lidar_distance, 2)


def scroll_change_pitch_angle(x, y, dx, dy):
    # dy = 1: Up, dy = -1:Down
    global current_pitch_angle, scan_sub
    d_angle = 0.01*dy
    current_pitch_angle = round(current_pitch_angle + 0.01*dy,2)

    if current_pitch_angle > 0.33:
        current_pitch_angle = 0.33
    elif current_pitch_angle < -0.33:
        current_pitch_angle = -0.33

    # Output current pitch angle and lidar distance
    control.adjust_pitch_angle(current_pitch_angle)
    scan_sub = rospy.Subscriber("/"+CAR_ID+"/scan", LaserScan, control.get_lidar_dis)
    cprint('Current Pitch Angle:  %s' % str(current_pitch_angle), color='green')


def click_to_shoot_(x, y, button, pressed):
    global order, scan_sub, current_pitch_angle, current_scan_distance
    if button == pynput.mouse.Button.left and pressed:
        print('')
        print('Shoot the enemy')
        control.shoot_one(cmd_fric, cmd_shoot)
        print('Current distance:%s and angle:%s' %(str(current_scan_distance),str(current_pitch_angle)))
    elif button == pynput.mouse.Button.right and pressed:
        if order == False:
            order = True
            print('fric wheel activated')
        elif order == True:
            order = False
            print('fric wheel stop')
        #preq = 1000 - 2500
        #speed  25      12m/s
        #wheel  1450    1307
        cmd_fric(order, 1450, 1450)

def convert_error_space(self,x):
    x = float(x)

    pixel_error = self.image_width / 2 - x

    error = (self.image_width / 2 - x) / self.image_width * 3.14 * 0.36

    pencent = (self.image_width / 2 - x) / (self.image_width / 2) * 100

    return error
    
def box_center(self, box):
    return (box.xmin+box.xmax)/2, (box.ymin+box.ymax)/2
    

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
    
if __name__ == '__main__':
    pid = os.getpid()
    pitchTest = 6

    os.system("taskset -pc 1 %s"%pid)    
    rospy.init_node("aimer", anonymous=True)
    # CAR_ID=rospy.get_param('~CAR_ID')
    CAR_ID="CAR2"
    flg=1
    
    # CAR1 soren CAR2 andre
    cmd_fric  = rospy.ServiceProxy("/"+CAR_ID+"/cmd_fric_wheel",rtssrv.FricWhl)
    cmd_shoot = rospy.ServiceProxy("/"+CAR_ID+"/cmd_shoot", rtssrv.ShootCmd)

    cmd_pub=rospy.Publisher("/"+CAR_ID+"/cmd_gimbal_angle",GimbalAngle,queue_size=10)
      
    control=GimbalController()
    rospy.Subscriber("/"+CAR_ID+"/detector/armormsg", ArmorPoints, control.callback)
    global current_pitch_angle, current_scan_distance, order
    order = False
    current_pitch_angle = 0
    current_scan_distance = 0
    control.adjust_pitch_angle(current_pitch_angle) # Move to starting pitch angle

    # Monitor mouse
    with pynput.mouse.Listener(
        on_click = click_to_shoot_,
        on_scroll = scroll_change_pitch_angle
    ) as listener:
        listener.join()

