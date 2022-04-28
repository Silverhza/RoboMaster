; Auto-generated. Do not edit!


(cl:in-package modified_chassis-srv)


;//! \htmlinclude MoveMode-request.msg.html

(cl:defclass <MoveMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveMode-request (<MoveMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name modified_chassis-srv:<MoveMode-request> is deprecated: use modified_chassis-srv:MoveMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MoveMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader modified_chassis-srv:mode-val is deprecated.  Use modified_chassis-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveMode-request>) ostream)
  "Serializes a message object of type '<MoveMode-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mode) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveMode-request>) istream)
  "Deserializes a message object of type '<MoveMode-request>"
    (cl:setf (cl:slot-value msg 'mode) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveMode-request>)))
  "Returns string type for a service object of type '<MoveMode-request>"
  "modified_chassis/MoveModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveMode-request)))
  "Returns string type for a service object of type 'MoveMode-request"
  "modified_chassis/MoveModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveMode-request>)))
  "Returns md5sum for a message object of type '<MoveMode-request>"
  "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveMode-request)))
  "Returns md5sum for a message object of type 'MoveMode-request"
  "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveMode-request>)))
  "Returns full string definition for message of type '<MoveMode-request>"
  (cl:format cl:nil "bool mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveMode-request)))
  "Returns full string definition for message of type 'MoveMode-request"
  (cl:format cl:nil "bool mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude MoveMode-response.msg.html

(cl:defclass <MoveMode-response> (roslisp-msg-protocol:ros-message)
  ((received
    :reader received
    :initarg :received
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveMode-response (<MoveMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name modified_chassis-srv:<MoveMode-response> is deprecated: use modified_chassis-srv:MoveMode-response instead.")))

(cl:ensure-generic-function 'received-val :lambda-list '(m))
(cl:defmethod received-val ((m <MoveMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader modified_chassis-srv:received-val is deprecated.  Use modified_chassis-srv:received instead.")
  (received m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveMode-response>) ostream)
  "Serializes a message object of type '<MoveMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'received) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveMode-response>) istream)
  "Deserializes a message object of type '<MoveMode-response>"
    (cl:setf (cl:slot-value msg 'received) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveMode-response>)))
  "Returns string type for a service object of type '<MoveMode-response>"
  "modified_chassis/MoveModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveMode-response)))
  "Returns string type for a service object of type 'MoveMode-response"
  "modified_chassis/MoveModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveMode-response>)))
  "Returns md5sum for a message object of type '<MoveMode-response>"
  "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveMode-response)))
  "Returns md5sum for a message object of type 'MoveMode-response"
  "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveMode-response>)))
  "Returns full string definition for message of type '<MoveMode-response>"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveMode-response)))
  "Returns full string definition for message of type 'MoveMode-response"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveMode-response
    (cl:cons ':received (received msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveMode)))
  'MoveMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveMode)))
  'MoveMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveMode)))
  "Returns string type for a service object of type '<MoveMode>"
  "modified_chassis/MoveMode")