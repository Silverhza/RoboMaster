#!/usr/bin/python
import rospy
import numpy as np
from roborts_msgs.msg import GimbalAngle, RobotHeat
import actionlib
import sys
import roborts_msgs.srv as rtssrv
import cv2
from yolo.msg import ArmorPoints, Box
import tf

from sensor_msgs.msg import LaserScan

from tf.msg import tfMessage

from time import time

from std_msgs.msg import Bool


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
    def __init__(self,P=0.48,I=0.0,D=0): ## 0.42, 0, 0


        self.P=P
        self.I=I
        self.D=D
        self.last_error=0
        self.Ierror=0

        self.last_pitch=0
        self.pitch=0

        self.pitch_size = 20
        self.pitch_index = 0 
        self.pitch_buffer = np.zeros(self.pitch_size)

        self.distance=0
        self.dist_size = 20
        self.dist_index = 0 
        self.dist_buffer = np.zeros(self.dist_size)
        self.last_dist = 0

        self.image_height = 240
        self.image_width = 320

        self.timer = 0

        self.current_gimbal_rotaion = 0

        self.current_time = 0

        self.heat_limit = 240
        self.current_heat = 0
        self.bullet_speed = 23

        # self.CAR_ID=rospy.get_param('~CAR_ID')
        # self.act=actionlib.SimpleActionServer("/shoot_switch", autofire.msg.ShootSwitchAction, self.handleAct,auto_start = False)# change to "/"+color+"_"+shoot_switch" if needed!
        # self.act.start()
        # self._result=autofire.msg.ShootSwitchResult()

    def pitch_model(self, dist):

        a = 0.68137077765313

        b = -0.686953112648725

        c = 0.27527458202346

        d = -0.0506009710987291

        e = 0.00342920068200132

        pitch = a + b*dist + c*dist*dist + d*dist*dist*dist + e*dist*dist*dist*dist

        return pitch

    def convert_error_space(self,x):
        x = float(x)

        pixel_error = self.image_width / 2 - x

        error = (self.image_width / 2 - x) / self.image_width * 3.14 * 0.36

        pencent = (self.image_width / 2 - x) / (self.image_width / 2) * 100



        # if self.timer == 20:
        #     rospy.loginfo("x direction pixel error "+str(pixel_error))
        #     rospy.loginfo("percentage "+str(pencent))
        #     rospy.logwarn("x radians error "+str(error))
        #     # rospy.logwarn("Ierror "+str(self.Ierror))
        #     self.timer = 0


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

        self.last_error=errorx

        return yaw_angle


    def pitchController(self, dist, camera_dist):

        # rospy.loginfo("pitch-------"+str(pitch))

        threshod = 1.5

        ###   prevent head up within 1.5m   ####

        if (camera_dist < threshod) & (dist > (threshod + 0.5)):
            dist = self.last_dist


        else:
            self.last_dist = dist


        # self.dist_buffer[self.dist_index] = dist

        # self.dist_index += 1

        # if self.dist_index == self.dist_size:
            
        #     self.dist_index = 0

        # dist_avg = sum(self.dist_buffer)/len(self.dist_buffer)

        # dist_min = min(self.dist_buffer)


        if dist > 4.2:
            pitch = -0.0298504108022

        elif dist < 0.7:
            pitch = 0.318855361987

        else:
            pitch = self.pitch_model(dist)


        self.pitch_buffer[self.pitch_index] = pitch

        self.pitch_index += 1

        if self.pitch_index == self.pitch_size:
            
            self.pitch_index = 0

        pitch_avg = sum(self.pitch_buffer)/len(self.pitch_buffer)

        return pitch_avg
        

    # def shootController(self):
    #     flag = Bool()

    #     # if abs(self.last_error)>0.08 or self.distance > 4.2 or (self.current_heat + self.bullet_speed) >= self.heat_limit:
    #     if abs(self.last_error)>0.1 or self.distance > 4.2:

    #         flag.data = False
    #         shoot_enalbe.publish(flag)

    #     # cmd_shoot=rospy.ServiceProxy("/"+CAR_ID+"/cmd_shoot",rtssrv.ShootCmd)
    #     # res=cmd_shoot(1,1)# STOP, ONCE, CONTINUOUS, mode,number


    #     else:
    #         flag.data = True
    #         shoot_enalbe.publish(flag)



    def shootController(self):

        # if abs(self.last_error)>0.08 or self.distance > 4.2 or (self.current_heat + self.bullet_speed) >= self.heat_limit:
        if abs(self.last_error)>0.1 or self.distance > 4.2:
            return

        cmd_shoot=rospy.ServiceProxy("/"+CAR_ID+"/cmd_shoot",rtssrv.ShootCmd)
        res=cmd_shoot(1,1)# STOP, ONCE, CONTINUOUS, mode,number

        # cmd_shoot=rospy.ServiceProxy("/"+CAR_ID+"/cmd_shoot",rtssrv.ShootCmd)
        # res=cmd_shoot(1,1)# STOP, ONCE, CONTINUOUS, mode,number

        
    def get_robot_heat(self, msg):
        self.current_heat = msg.shooter_heat


    def get_lidar_dis(self, msg):
        dis_array = msg.ranges

        scan_list = []

        ## scan size = 1440, 4/degree ##
        ## current 7 points ##

        left = 25

        for i in range(left):
            if dis_array[1440 - left + i] != float("inf"):
                scan_list.append(dis_array[1440 - left + i])
                

        for i in range(left + 1):
            if dis_array[i] != float("inf"):
                scan_list.append(dis_array[i])


        if len(scan_list) != 0:          
            self.distance = min(scan_list)


    def get_gimbal_yaw_rotation(self, msg):

        tf_message = msg.transforms[0]

        frame_id = tf_message.child_frame_id

        gimbal_frame_id = CAR_ID + "/gimbal_fixed"
        # rospy.loginfo(str(self.distance))      

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

        # roll = euler[0]
        # pitch = euler[1]
        self.current_gimbal_rotaion = euler[2]



        
    def get_camera_dist(self, box):
        imgpoints=np.array([[box.xmin,box.ymin],[box.xmax,box.ymin],[box.xmax,box.ymax],[box.xmin,box.ymax]])
        imgpoints = imgpoints.astype('float32')
        _,_,tvec=pe.PnP(imgpoints)

        dist = np.sqrt(sum((tvec/1000.0)**2))

        # print(dist)

        return dist


    def gimbal_cmd(self, box, distance, yaw_mode=True, pitch_mode=False):
        cmd = GimbalAngle()

        camera_based_dist = self.get_camera_dist(box)

        cmd.yaw_mode = yaw_mode
        cmd.pitch_mode = pitch_mode
        cmd.yaw_angle = self.yawController(box)
        cmd.pitch_angle= self.pitchController(distance, camera_based_dist)
        cmd_pub.publish(cmd)

        # cur_time = time()

        self.shootController()

        # print(time() - cur_time)

    def callback(self,msg):

        self.num_detect = msg.numDetected

        if self.num_detect<1 :
            return

        else:
            box = msg.bbox[0]          
            self.gimbal_cmd(box, self.distance)


        #     for i in range(int(self.num_detect)):
        #         # box_list=msg.bounding_boxes[i]

        #         box_list = msg.bbox[i]

        #         if box_list.obj=="red_1" or box_list.obj=="red_2":
        #             self.target="enemy"
        #             box = box_list
        #             break


        #         # if box_list.Class=="red_1" or box_list.Class=="red_2":
        #         #     self.target="enemy"
        #         #     box = box_list
        #         #     break

        #         else:
        #             self.target="null"


        return 

    
if __name__ == '__main__':
    rospy.init_node("shooter", anonymous=True)
    CAR_ID=rospy.get_param('~CAR_ID')

    # CAR1 soren CAR2 andre

    cmd_fric=rospy.ServiceProxy("/"+CAR_ID+"/cmd_fric_wheel",rtssrv.FricWhl)
    res=cmd_fric(True)
    # res=cmd_fric(False)

    cmd_pub=rospy.Publisher("/"+CAR_ID+"/cmd_gimbal_angle",GimbalAngle,queue_size=10)

    shoot_enalbe = rospy.Publisher("/"+CAR_ID+"/shoot_enable",Bool,queue_size=10)

    contorl=GimbalController()
    pe=PoseEstimator()

    # rospy.Subscriber("tf", tfMessage, contorl.get_gimbal_yaw_rotation)

    rospy.Subscriber("/"+CAR_ID+"/robot_heat", RobotHeat, contorl.get_robot_heat)

    rospy.Subscriber("/"+CAR_ID+"/front_scan", LaserScan, contorl.get_lidar_dis)

    rospy.Subscriber("/"+CAR_ID+"/detector/armormsg", ArmorPoints, contorl.callback)
    rospy.spin()
