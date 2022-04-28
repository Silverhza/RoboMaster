#! /usr/bin/env python3


import roslib
import rospy
import sys
import os
import time
import cv2
import torch
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from ros_yolo.msg import ArmorPoints,Box
from geometry_msgs.msg import Point
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np
from models.experimental import attempt_load
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, plot_one_box, strip_optimizer, set_logging)
from utils.torch_utils import select_device, load_classifier, time_synchronized
from matplotlib import pyplot as plt
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
ros_image=0


class Track_on_Detect(object):
    def __init__(self):
        self.box=()
        self.tracker = cv2.TrackerMedianFlow_create()
        self.track_obj="track"
        self.trackerbox=[]

        # self.enemy_1 = enemy_color + "_1"
        self.enemy_1 = "blue_2"
        # self.enemy_2 = enemy_color + "_2"
        self.enemy_2 = "red_1"

        self.time1 = 0
        self.time2 = 0
        self.count=0
        self.mode=0 # 0 detect 1 track

        self._armorWidth=135 ## mm ##
        self._armorHeight=120
        # self._armorWidth=70 ## mm ##
        # self._armorHeight=100

        self.objPoints=self.creatObjPoints()

        self.image_width = 640
        self.image_height = 480

        self.distance = 0

        ## calibration parameter ##

        # if CAR_ID == "CAR1":
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

    def letterbox(self,img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
        # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, ratio, (dw, dh)
    def loadimg(self,img):  # 接受opencv图片
        img_size=640
        cap=None
        path=None
        img0 = img
        img,ratio,pad = self.letterbox(img0, new_shape=img_size)
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        return path, img, img0, cap

    def cal_object_dis(self,det):

        xmin, ymin, xmax, ymax = det[0],det[1],det[2],det[3]
        imgpoints=np.array([[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]])
        imgpoints = imgpoints.astype('float32')
        _,_,tvec=self.PnP(imgpoints)
        dist=np.sqrt(sum((tvec/1000.0)**2))
        # print(dist)
        # print("____________________________________")
        return dist

    def convertBack(self, x, y, w, h):
        xmin = ((x - (w / 2)))
        xmax = ((x + (w / 2)))
        ymin = ((y - (h / 2)))
        ymax = ((y + (h / 2)))
        return xmin, ymin, xmax, ymax

    def detect(self,img):
        if self.count==0:
            self.time1 = time.time()
        global ros_image
        cudnn.benchmark = True
        dataset = self.loadimg(img)
        names = model.module.names if hasattr(model, 'module') else model.names
        #colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
        augment = 'store_true'
        conf_thres = 0.3
        iou_thres = 0.45
        # classes = (0,1,2,3,5,7,15,27,28,33,44,46,51,55,64,73,74,76,77,84)
        classes = (0,1,2,3,4)
        agnostic_nms = 'store_true'
        img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
        _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
        path = dataset[0]
        img = dataset[1]
        im0s = dataset[2]
        im0 = im0s
        trackerimg = im0s
        self.count+=1

        # time = rospy.get_time()
        detections=[]

        if self.mode==1:
            status,box=self.tracker.update(trackerimg)
            self.box = self.convertBack(box[0],box[1],box[2],box[3])
            # print(self.box)


            if status:
                detections.append([float(self.box[0]),float(self.box[1]),float(self.box[2]),float(self.box[3]),1.0,self.track_obj])
                plot_one_box(detections[0], im0, label=names[int(self.track_obj)], color=[0,255,0], line_thickness=3)# print(detections)

                # print("______________________________")
                if self.count>=5:
                    self.mode=0
                    self.count=0
                    self.time2=time.time() 
                    timecost = self.time2 - self.time1
                    print(timecost/5)
                    self.tracker=cv2.TrackerKCF_create()               

            else:
                self.mode=0
                self.tracker=cv2.TrackerKCF_create()
                    

        if self.mode==0:


            # print(img.size)
            # im0s = dataset[2]
            # print(im0s.size)
            # vid_cap = dataset[3]
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0

            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            # Inference
            pred = model(img, augment=augment)[0]
            # Apply NMS
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)

        #########################################################
            #aim#
        #########################################################
            # view_img = 1
            # save_txt = 1
            save_conf = 'store_true'
            detect_candit = []
            if pred is not None: 
                for i, det in enumerate(pred):  # detections per image
                    gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                    if det is not None:
                        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                        det=det.tolist()
                        for dett in det:
                            c = dett[5]
                            if names[int(c)] == self.enemy_1 or names[int(c)] == self.enemy_2:
                                detect_candit.append(dett)
                            else:
                                continue
            else:
                pass
            # print(detect_candit) 
            # print("____________________________________")
            if detect_candit:           
                if len(detect_candit) > 1:
                    tmp=min(detect_candit,key=lambda x: self.cal_object_dis(x))
                    detect_candit.remove(tmp)
                    tmp_second=min(detect_candit,key=lambda x: self.cal_object_dis(x))

                    distance_1 = self.cal_object_dis(tmp)
                    distance_2 = self.cal_object_dis(tmp_second)

                    detect_send = []

                    if abs(distance_1 - distance_2) < 0.12:

                        detect_compare_error = []
                        detect_compare_error.append(tmp)
                        detect_compare_error.append(tmp_second)                                
                        tmp=min(detect_compare_error,key=lambda x: abs(x[0] - self.image_width/2))
                        detect_send.append(tmp)

                    else:
                        detect_send.append(tmp)

                    self.box=tmp

                    self.trackerbox=[(tmp[0]+tmp[2])/2,(tmp[1]+tmp[3])/2,tmp[2]-tmp[0],tmp[3]-tmp[1]]
                    self.track_obj=tmp[5]

                    self.mode=1
                    self.tracker.init(trackerimg,self.trackerbox)

                    detections = detect_send
                    plot_one_box(detections[0], im0, label=names[int(tmp[5])], color=[0,255,0], line_thickness=3)
                    self.distance = self.cal_object_dis(self.box)[0]

                else:

                    tmp=max(detect_candit,key=lambda x:x[4])
                    self.box=tmp

                    self.trackerbox=[(tmp[0]+tmp[2])/2,(tmp[1]+tmp[3])/2,tmp[2]-tmp[0],tmp[3]-tmp[1]]
                    self.track_obj=tmp[5]
                    self.mode=1
                    # print(type(img))
                    self.tracker.init(trackerimg,self.trackerbox)

                    detections = detect_candit
                    # label = '%s %.2f' % (names[int(cls)], conf)
                    # plot_one_box(xyxy, im0, label=label, color=[0,255,0], line_thickness=3)
                    plot_one_box(detections[0], im0, label=names[int(tmp[5])], color=[0,255,0], line_thickness=3)
                    self.distance = self.cal_object_dis(self.box)[0]
        # self.time2 = time.time()
        # timecost = self.time2 - self.time1
        # print(timecost)
        if detections:
            self.pbmsg(detections,img)
        out_img = im0[:, :, [2, 1, 0]]
        cv2.imshow('YOLOV5', out_img)
        a = cv2.waitKey(1)

    def pbmsg(self,detections,image):
        a=ArmorPoints()
        names = model.module.names if hasattr(model, 'module') else model.names
        for x,y,x1,y1,confidence,label in detections:
            confidence *=100
            if names[int(label)] == self.enemy_1 or names[int(label)] == self.enemy_2:

                if float(confidence)<60 and confidence != 1:
                    continue
                b=Box()
                b.xmin, b.ymin, b.xmax, b.ymax = x,y,x1,y1
                # print(b.xmin, b.ymin, b.xmax, b.ymax)
                b.confidence=round(float(confidence) / 100,2)
                # print(label)
                b.obj=names[int(label)]
                b.distance = self.distance*3
                a.bbox[a.numDetected]=b
                a.numDetected+=1
                print(b)
                # print(x,y,x1,y1) 
                # print(b.distance) 
                # print("______________________________")
        detect_res_pub.publish(a)
    
    def image_callback_1(self,image):
        global ros_image
        ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        with torch.no_grad():
            self.detect(ros_image)

if __name__ == '__main__':
    set_logging()

    #读取参数
    # CAR_ID=rospy.get_param('~CAR_ID')
    # enemy_color = rospy.get_param('~enemy_color')
    # enemy_1 = enemy_color + "_1"
    # enemy_2 = enemy_color + "_2"
    # CAR_ID=rospy.get_param('~CAR_ID')  
    # enemy_color = rospy.get_param('~enemy_color')
    CAR_ID="CAR1" 
    enemy_color = "green"
    device = ''
    device = select_device(device)
    half = device.type != 'cpu'  # half precision only supported on CUDA
    weights = 'best_m.pt'
    imgsz = 640
    model = attempt_load(weights, map_location=device)  # load FP32 model
    imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
    if half:
        model.half()  # to FP16
    '''
    模型初始化
    '''
    rospy.init_node('ros_yolo')
    image_topic_1 = "/image_view/image_raw"
    detect_res_pub = rospy.Publisher("/"+CAR_ID+"/detector/armormsg",ArmorPoints,queue_size=1)
    # draw_pub=rospy.Publisher("/"+CAR_ID+"/detector/draw",Image,queue_size=1)
    inferencer=Track_on_Detect()
    rospy.Subscriber(image_topic_1, Image, inferencer.image_callback_1, queue_size=1, buff_size=52428800)
    rospy.spin()
