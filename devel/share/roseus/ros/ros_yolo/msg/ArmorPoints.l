;; Auto-generated. Do not edit!


(when (boundp 'ros_yolo::ArmorPoints)
  (if (not (find-package "ROS_YOLO"))
    (make-package "ROS_YOLO"))
  (shadow 'ArmorPoints (find-package "ROS_YOLO")))
(unless (find-package "ROS_YOLO::ARMORPOINTS")
  (make-package "ROS_YOLO::ARMORPOINTS"))

(in-package "ROS")
;;//! \htmlinclude ArmorPoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ros_yolo::ArmorPoints
  :super ros::object
  :slots (_header _numDetected _bbox ))

(defmethod ros_yolo::ArmorPoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:numDetected __numDetected) 0)
    ((:bbox __bbox) (let (r) (dotimes (i 6) (push (instance ros_yolo::Box :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _numDetected (round __numDetected))
   (setq _bbox __bbox)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:numDetected
   (&optional __numDetected)
   (if __numDetected (setq _numDetected __numDetected)) _numDetected)
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _numDetected
    4
    ;; ros_yolo/Box[6] _bbox
    (apply #'+ (send-all _bbox :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _numDetected
       (write-long _numDetected s)
     ;; ros_yolo/Box[6] _bbox
     (dolist (elem _bbox)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _numDetected
     (setq _numDetected (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ros_yolo/Box[6] _bbox
   (dotimes (i 6)
     (send (elt _bbox i) :deserialize buf ptr-) (incf ptr- (send (elt _bbox i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get ros_yolo::ArmorPoints :md5sum-) "29bf3d3a8a44f27a0f27a1f9bc39e0c2")
(setf (get ros_yolo::ArmorPoints :datatype-) "ros_yolo/ArmorPoints")
(setf (get ros_yolo::ArmorPoints :definition-)
      "Header header
int32 numDetected
Box[6]  bbox
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: ros_yolo/Box
float32 xmin # xmin
float32 ymin  # ymin
float32 xmax #xmax
float32 ymax #ymax

float32 confidence 
string obj
float32 distance

")



(provide :ros_yolo/ArmorPoints "29bf3d3a8a44f27a0f27a1f9bc39e0c2")


