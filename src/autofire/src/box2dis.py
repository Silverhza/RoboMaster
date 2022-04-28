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

        self.speed = 1450
        self.freq = 1000

        self.lastPnPdis = 0
        self.lastLidardis = 0
        self.errorcnt = 0
        self.distance = 0
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

        # 0 for low speed, 1 for high speed
        self.pitchMode = 0

    def pitch_model(self, dist, mode):


        # A =  -0.160590967086966

        # B = 0.323302217183758
        
        # pitch = A+B/dist
        
        if mode == 1:
            x = dist
            a = -0.0024
            b = 0.0306
            c = -0.1612
            d = 0.2744
            # pitch = a + b*x + c*x**2 + d*x**3 + e*x**4 + f*x**5
            pitch = a*x**3 + b*x**2 + c*x + d
        else:
            x = dist
            a = -0.0252
            b = 0.1963
            c = -0.5792
            d = 0.5815
            # pitch = a + b*x + c*x**2 + d*x**3 + e*x**4 + f*x**5
            pitch = a*x**3 + b*x**2 + c*x + d

        return pitch

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

    def pitchController(self, dist, box):

        if self.distance == 0:
            if self.pitch_index != 0:
                pitch = self.pitch_buffer[self.pitch_index-1]
            else:
                pitch = self.pitch_buffer[self.pitch_size-1]
        else:
            error1 = self.lastPnPdis - self.distance
            error2 = self.lastLidardis - dist

            if (abs(error1-error2)<0.08) | (self.errorcnt>=15):
                #high speed low freq
                if dist>3:
                    self.pitchMode = 1
                    self.speed = 1450
                    self.freq = 1000
                    pitch = self.pitch_model(dist, self.pitchMode)
                    
                    centerx,centery=self.box_center(box)
                    print(centery)
                    print(dist)
                    print("***************")
                else:
                    self.pitchMode = 0
                    self.speed = 1307
                    self.freq = 2500
                    pitch = self.pitch_model(dist, self.pitchMode)
                    
                    centerx,centery=self.box_center(box)
                    print(centery)
                    print(dist)
                    print("***************")
                self.lastLidardis = dist
                self.lastPnPdis = self.distance
                self.errorcnt = 0
            else:
                self.errorcnt+=1
                if self.pitch_index != 0:
                    pitch = self.pitch_buffer[self.pitch_index-1]
                else:
                    pitch = self.pitch_buffer[self.pitch_size-1]

        self.pitch_buffer[self.pitch_index] = pitch

        self.pitch_index += 1

        if self.pitch_index == self.pitch_size:
            
            self.pitch_index = 0

        # pitch_avg = sum(self.pitch_buffer)/len(self.pitch_buffer)
        pitch_avg = np.median(self.pitch_buffer)

        self.last_pitch = pitch_avg
        return pitch_avg
        
    def get_robot_heat(self, msg):
        self.current_heat = msg.shooter_heat

    def get_lidar_dis(self, msg):
        dis_array = msg.ranges
        mid=int(len(msg.ranges)/2)
        mid-=3

        scan_list = []

        # left = 15
        for i in range(5):
            if dis_array[mid-i] != float("inf"):
                scan_list.append(dis_array[mid-i])
                #print(-i,dis_array[mid-i])
            if dis_array[mid+i] != float("inf"):
                scan_list.append(dis_array[mid+i])
                #print(+i,dis_array[mid+i])

        self.lidar_distance=np.median(scan_list)

    def publish_enemy_id(self, box):
        if abs(self.last_error)>0.08:
            return
            
        info = enemy_id()

        if self.chassis_orientation >= 0:
   
            self.enemy_orientation = self.chassis_orientation  + self.gimbal_orientation

        else:
            self.enemy_orientation = 3.14 * 2 + self.chassis_orientation  + self.gimbal_orientation

        info.x = self.my_x + self.distance * 1.2 * math.cos(self.enemy_orientation)
        info.y = self.my_y + self.distance * 1.2 * math.sin(self.enemy_orientation)
        info.id = box.obj

        info_pub.publish(info)

    def get_gimbal_yaw_rotation(self, msg):

        tf_message = msg.transforms[0]

        frame_id = tf_message.child_frame_id

        gimbal_frame_id = CAR_ID + "/gimbal_fixed"

        if frame_id == gimbal_frame_id:
            rotation_x = tf_message.transform.rotation.x
            rotation_y = tf_message.transform.rotation.y
            rotation_z = tf_message.transform.rotation.z
            rotation_w = tf_message.transform.rotation.w

            quaternion = (
                rotation_x,
                rotation_y,
                rotation_z,
                rotation_w)

            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.gimbal_orientation = euler[2]

            yaw_rotaion = euler[2]

        else:
            yaw_rotaion = self.gimbal_orientation

        return yaw_rotaion

    def gimabl_rotate(self, angle):

        cmd = GimbalAngle()

        cmd.yaw_mode = True         # relative angle #
        cmd.pitch_mode = False      # absolute angle #
        cmd.yaw_angle = angle
        cmd.pitch_angle= self.last_pitch
        cmd_pub.publish(cmd)

    def search_enemy(self, msg):
        
        cur_time = time()

        if (cur_time - self.last_track_time) < 2:
            return

        if self.start_search == False:
            self.start_search = True
            cmd = GimbalAngle()
            cmd.yaw_mode = False        # relative angle #
            cmd.pitch_mode = False      # absolute angle    #
            cmd.yaw_angle = 0
            cmd.pitch_angle= self.last_pitch
            cmd_pub.publish(cmd)

    def get_chassis_pose(self, msg):
        position = msg.pose.position

        self.my_x = position.x
        self.my_y = position.y

        orientation = msg.pose.orientation

        rotation_x = orientation.x
        rotation_y = orientation.y
        rotation_z = orientation.z
        rotation_w = orientation.w

        quaternion = (
            rotation_x,
            rotation_y,
            rotation_z,
            rotation_w)

        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.chassis_orientation = euler[2]  ## in radius ##

    def get_game_status(self, msg):
        if msg.game_status == self.Game_Start:
            self.shoot_enable = True

        elif msg.game_status == self.Game_End:
            self.shoot_enable = False

        else:
            return

    def cal_search_angle_dis(self, enemy_center):

        x_s = math.cos(self.my_orientation)
        y_s = math.sin(self.my_orientation)

        x_e = enemy_center.x - self.my_x
        y_e = enemy_center.y - self.my_y

        angle = math.acos((x_s*x_e + y_s*y_e) / math.sqrt(x_e ** 2 + y_e ** 2))

        a = np.array([x_s,y_s,0])  
        b = np.array([x_e,y_e,0])  
        c = np.cross(a,b)

        if c[2] < 0:
            angle *= -1

        dis = math.sqrt((x_e-x_s) ** 2 + (y_e-y_s) ** 2)

        return angle, dis

    def get_enemy_position(self, msg):

        enemy_pos = msg.circles

        cur_time = time()

        if (cur_time - self.last_track_time) < 2:
            return

        self.last_track_time += 2

        angle_candit = []
        dis_candit = []

        enemy_num = len(enemy_pos)
    
        for pos in enemy_pos:           
            center = pos.center

            angle, dist = self.cal_search_angle_dis(center)

            if abs(angle) > 1.308:
                pass

            angle_candit.append(angle)
            dis_candit.append(dist)

        if angle_candit:
            if len(angle_candit) > 1:
                dis_1 = dis_candit[0]
                dis_2 = dis_candit[1]

                if dis_1 < dis_2:
                    cmd_angle = angle_candit[0]

                else:
                    cmd_angle = angle_candit[1]

            else:
                cmd_angle = angle_candit[0]


            cmd = GimbalAngle()

            cmd.yaw_mode = False
            cmd.pitch_mode = False
            cmd.yaw_angle = cmd_angle
            cmd.pitch_angle= self.last_pitch
            cmd_pub.publish(cmd)     

        else:
            return
       
    def get_camera_dist(self, box):
        imgpoints=np.array([[box.xmin,box.ymin],[box.xmax,box.ymin],[box.xmax,box.ymax],[box.xmin,box.ymax]])
        imgpoints = imgpoints.astype('float32')
        _,_,tvec=pe.PnP(imgpoints)

        dist = np.sqrt(sum((tvec/1000.0)**2))

        return dist

    def gimbal_cmd(self, box, yaw_mode=True, pitch_mode=False):
        cmd = GimbalAngle()
        self.distance = float(box.distance)
        cmd.yaw_mode = yaw_mode
        cmd.pitch_mode = pitch_mode    
        cmd.yaw_angle = self.yawController(box)
        cmd.pitch_angle= self.pitchController(self.lidar_distance, box)
        cmd_pub.publish(cmd)
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
    
if __name__ == '__main__':
    pid = os.getpid()
    os.system("taskset -pc 1 %s"%pid)    
    rospy.init_node("aimer", anonymous=True)
    CAR_ID="CAR2"
    cmd_fric=rospy.ServiceProxy("/"+CAR_ID+"/cmd_fric_wheel",rtssrv.FricWhl)
    cmd_pub=rospy.Publisher("/"+CAR_ID+"/cmd_gimbal_angle",GimbalAngle,queue_size=10)
    contorl=GimbalController()
    rospy.Subscriber("/"+CAR_ID+"/detector/armormsg", ArmorPoints, contorl.search_enemy)
    rospy.Subscriber("/"+CAR_ID+"/scan", LaserScan, contorl.get_lidar_dis)
    rospy.Subscriber("/"+CAR_ID+"/detector/armormsg", ArmorPoints, contorl.callback)
    rospy.spin()






