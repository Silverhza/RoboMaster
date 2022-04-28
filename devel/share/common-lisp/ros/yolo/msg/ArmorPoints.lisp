; Auto-generated. Do not edit!


(cl:in-package yolo-msg)


;//! \htmlinclude ArmorPoints.msg.html

(cl:defclass <ArmorPoints> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (numDetected
    :reader numDetected
    :initarg :numDetected
    :type cl:integer
    :initform 0)
   (bbox
    :reader bbox
    :initarg :bbox
    :type (cl:vector yolo-msg:Box)
   :initform (cl:make-array 6 :element-type 'yolo-msg:Box :initial-element (cl:make-instance 'yolo-msg:Box))))
)

(cl:defclass ArmorPoints (<ArmorPoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmorPoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmorPoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolo-msg:<ArmorPoints> is deprecated: use yolo-msg:ArmorPoints instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArmorPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:header-val is deprecated.  Use yolo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'numDetected-val :lambda-list '(m))
(cl:defmethod numDetected-val ((m <ArmorPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:numDetected-val is deprecated.  Use yolo-msg:numDetected instead.")
  (numDetected m))

(cl:ensure-generic-function 'bbox-val :lambda-list '(m))
(cl:defmethod bbox-val ((m <ArmorPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:bbox-val is deprecated.  Use yolo-msg:bbox instead.")
  (bbox m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmorPoints>) ostream)
  "Serializes a message object of type '<ArmorPoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'numDetected)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bbox))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmorPoints>) istream)
  "Deserializes a message object of type '<ArmorPoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numDetected) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'bbox) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'bbox)))
    (cl:dotimes (i 6)
    (cl:setf (cl:aref vals i) (cl:make-instance 'yolo-msg:Box))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmorPoints>)))
  "Returns string type for a message object of type '<ArmorPoints>"
  "yolo/ArmorPoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmorPoints)))
  "Returns string type for a message object of type 'ArmorPoints"
  "yolo/ArmorPoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmorPoints>)))
  "Returns md5sum for a message object of type '<ArmorPoints>"
  "29bf3d3a8a44f27a0f27a1f9bc39e0c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmorPoints)))
  "Returns md5sum for a message object of type 'ArmorPoints"
  "29bf3d3a8a44f27a0f27a1f9bc39e0c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmorPoints>)))
  "Returns full string definition for message of type '<ArmorPoints>"
  (cl:format cl:nil "Header header~%int32 numDetected~%Box[6]  bbox~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: yolo/Box~%float32 xmin # xmin~%float32 ymin  # ymin~%float32 xmax #xmax~%float32 ymax #ymax~%~%float32 confidence ~%string obj~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmorPoints)))
  "Returns full string definition for message of type 'ArmorPoints"
  (cl:format cl:nil "Header header~%int32 numDetected~%Box[6]  bbox~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: yolo/Box~%float32 xmin # xmin~%float32 ymin  # ymin~%float32 xmax #xmax~%float32 ymax #ymax~%~%float32 confidence ~%string obj~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmorPoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'bbox) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmorPoints>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmorPoints
    (cl:cons ':header (header msg))
    (cl:cons ':numDetected (numDetected msg))
    (cl:cons ':bbox (bbox msg))
))
