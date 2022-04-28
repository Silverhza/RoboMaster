#!/usr/bin/python

import os
import cv2
import numpy as np
import time
# from backbone import darknet
import darknet
import argparse

import rospy
import roslib
roslib.load_manifest('yolo')
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from yolo.msg import ArmorPoints,Box
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from time import time



class YOLO(object):
    def __init__(self):
        self.configPath = "./cfg/lima_320.cfg"
        self.weightPath = "./cfg/lima_320.weights"

        self.metaPath = "./cfg/obj.data"
        if not os.path.exists(self.configPath):
            raise ValueError("Invalid config path `" +
                             os.path.abspath(self.configPath)+"`")
        if not os.path.exists(self.weightPath):
            raise ValueError("Invalid weight path `" +
                             os.path.abspath(self.weightPath)+"`")
        if not os.path.exists(self.metaPath):
            raise ValueError("Invalid data file path `" +
                             os.path.abspath(self.metaPath)+"`")

        self.network, self.class_names, self.class_colors = darknet.load_network(self.configPath,  self.metaPath, self.weightPath, batch_size=1)

        self.darknet_image=darknet.make_image(darknet.network_width(self.network),
                                        darknet.network_height(self.network),3)

    def detect(self,data):
        darknet.copy_image_from_bytes(self.darknet_image,data.tobytes())
        detections = darknet.detect_image(self.network, self.class_names, self.darknet_image, thresh=0.60)
        return detections


class Track_on_Detect(object):
    def __init__(self):
        self.tracker = cv2.TrackerMedianFlow_create()
        self.detector=YOLO()
        self.mode=0 # 0 detect 1 track
        self.count=0
        self.box=()
        self.track_obj="track"

        self.enemy_1 = enemy_color + "_1"
        self.enemy_2 = enemy_color + "_2"

        self.time = 0

        self._armorWidth=135 ## mm ##
        self._armorHeight=120
        self.objPoints=self.creatObjPoints()

        self.image_width = 320
        self.image_height = 240

        self.distance = 0

        ## calibration parameter ##

        if CAR_ID == "CAR1":
            self._K=np.array([[229.93762, 0.0, 161.18925],[ 0.0, 230.03427, 117.33975],[0.0, 0.0, 1.0]])
            self._D=np.array([0.035273, -0.058764, 0.000471, 0.000958, 0.000000])

        elif CAR_ID == "CAR2":
            self._K=np.array([[228.85851, 0.0, 165.17667],[ 0.0, 229.02883, 119.46514],[0.0, 0.0, 1.0]])
            self._D=np.array([0.038931, -0.063678, 0.000372, -0.000037, 0.000000])


    def creatObjPoints(self):
        objpoints=[]#start at lefttop, clockwise 
        objpoints.append([-self._armorWidth/2,self._armorHeight/2,0])
        objpoints.append([self._armorWidth/2,self._armorHeight/2,0])
        objpoints.append([self._armorWidth/2,-self._armorHeight/2,0])
        objpoints.append([-self._armorWidth/2,-self._armorHeight/2,0])
        return np.array(objpoints,dtype=float)

    def PnP(self,imgPoints):
        retval,rvec,tvec=cv2.solvePnP(self.objPoints,imgPoints.reshape((4,1,2)),self._K, self._D)
        # ,flags=cv2.SOLVEPNP_P3P

        return retval,rvec,tvec


    def convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax


    def cal_object_dis(self,box):

        x, y, w, h = (box[0],
            box[1],
            box[2],
            box[3])

        xmin, ymin, xmax, ymax = self.convertBack(float(x), float(y), float(w), float(h))

        imgpoints=np.array([[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]])
        imgpoints = imgpoints.astype('float32')
        _,_,tvec=self.PnP(imgpoints)

        dist=np.sqrt(sum((tvec/1000.0)**2))

        return dist



    def callback(self,image):
        
        img=bridge.imgmsg_to_cv2(image,"rgb8")
        # img=cv2.resize(img,(320,240),interpolation=cv2.INTER_LINEAR)
        
        self.count+=1

        time = rospy.get_time()

        if self.mode==1:
            status,self.box=self.tracker.update(img)

            if status:
                detections=[(self.track_obj,1,self.box)]

                if self.count>=10:
                    self.mode=0
                    self.count=0
                    self.tracker=cv2.TrackerMedianFlow_create()                

            else:
                self.mode=0
                self.tracker=cv2.TrackerMedianFlow_create()
                    

        if self.mode==0:

            detections=self.detector.detect(img)

            # ******detections = [class, prob*100, box]******

            detect_candit = []

            if detections:

                for detection in detections:
                    if detection[0] == self.enemy_1 or detection[0] == self.enemy_2:
                        detect_candit.append(detection)
                        # rospy.loginfo(str(detection[0]) +" [" + str(detection[1]) + "]\n")
                    
                    else:
                        # rospy.loginfo(str(detection[0]) +" [" + str(detection[1]) + "]\n")
                        continue

            else:
                pass

            if detect_candit:


                if len(detect_candit) > 1:
                    
                    tmp=min(detect_candit,key=lambda x: self.cal_object_dis(x[2]))

                    detect_candit.remove(tmp)

                    tmp_second=min(detect_candit,key=lambda x: self.cal_object_dis(x[2]))

                    distance_1 = self.cal_object_dis(tmp[2])
                    distance_2 = self.cal_object_dis(tmp_second[2])

                    detect_send = []

                    if abs(distance_1 - distance_2) < 0.12:
                        detect_compare_error = []

                        detect_compare_error.append(tmp)
                        detect_compare_error.append(tmp_second)                                

                        tmp=min(detect_compare_error,key=lambda x: abs(x[2][0] - self.image_width/2))
                        detect_send.append(tmp)

                    else:
                        detect_send.append(tmp)

                    self.box=tmp[2]
                    self.track_obj=tmp[0]
                    self.mode=1
                    self.tracker.init(img,self.box)
                    detections = detect_send
                    self.distance = self.cal_object_dis(self.box)


                else:
                    tmp=max(detect_candit,key=lambda x:x[1])

                    self.box=tmp[2]
                    self.track_obj=tmp[0]
                    self.mode=1
                    self.tracker.init(img,self.box)

                    detectios = detect_candit

                    self.distance = self.cal_object_dis(self.box)

        self.pbmsg(detections,img)


       
    def pbmsg(self,detections,image):

        a=ArmorPoints()
        for label, confidence, bbox in detections:
            if label == self.enemy_1 or label == self.enemy_2:

                if float(confidence)<60 and confidence != 1:
                    continue

                b=Box()

                x, y, w, h = (bbox[0],
                            bbox[1],
                            bbox[2],
                            bbox[3])

                b.xmin, b.ymin, b.xmax, b.ymax = self.convertBack(float(x), float(y), float(w), float(h))
                b.confidence=round(float(confidence) / 100,2)
                b.obj=label

                b.distance = self.distance

                a.bbox[a.numDetected]=b
                a.numDetected+=1

                # rospy.loginfo(b.obj +" [" + str(b.confidence) + "]\n")
                # rospy.loginfo(b.obj +" [" + str(b.distance) + "]\n")

                pt1 = (b.xmin, b.ymin)
                pt2 = (b.xmax, b.ymax)
                cv2.rectangle(image, pt1, pt2, (0, 255, 0), 1)
                cv2.putText(image,b.obj +" [" + str(b.confidence) + "]",
                                (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                [0, 255, 0], 2)
                cv2.line(image,(160,0),(160,240),(0, 255, 0),1)
            

        draw_pub.publish(bridge.cv2_to_imgmsg(image,"rgb8")) 
        detect_res_pub.publish(a)
        
        
        
if __name__ == "__main__":

    pid = os.getpid()

    os.system("taskset -pc 2 %s"%pid)
  
    rospy.init_node("armor_detector", anonymous=True)
    bridge = CvBridge()

    CAR_ID=rospy.get_param('~CAR_ID')
    enemy_color = rospy.get_param('~enemy_color')

    parser = argparse.ArgumentParser()
    parser.add_argument('-show','--showdemo',dest='show',type=int,default=0,help='Display the detected object (Default=0)')
    args = parser.parse_args(rospy.myargv()[1:])
    draw_pub=rospy.Publisher("/"+CAR_ID+"/detector/draw",Image,queue_size=1)
        
    inferencer=Track_on_Detect()
    detect_res_pub = rospy.Publisher("/"+CAR_ID+"/detector/armormsg",ArmorPoints,queue_size=1)
    rospy.Subscriber("/"+CAR_ID+"/image_raw",Image, inferencer.callback)
    rospy.loginfo("Detection thread start!")
    rospy.spin()
