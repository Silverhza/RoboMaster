; Auto-generated. Do not edit!


(cl:in-package roborts_msgs-srv)


;//! \htmlinclude FricWhl-request.msg.html

(cl:defclass <FricWhl-request> (roslisp-msg-protocol:ros-message)
  ((open
    :reader open
    :initarg :open
    :type cl:boolean
    :initform cl:nil)
   (left
    :reader left
    :initarg :left
    :type cl:fixnum
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FricWhl-request (<FricWhl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FricWhl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FricWhl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roborts_msgs-srv:<FricWhl-request> is deprecated: use roborts_msgs-srv:FricWhl-request instead.")))

(cl:ensure-generic-function 'open-val :lambda-list '(m))
(cl:defmethod open-val ((m <FricWhl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roborts_msgs-srv:open-val is deprecated.  Use roborts_msgs-srv:open instead.")
  (open m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <FricWhl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roborts_msgs-srv:left-val is deprecated.  Use roborts_msgs-srv:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <FricWhl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roborts_msgs-srv:right-val is deprecated.  Use roborts_msgs-srv:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FricWhl-request>) ostream)
  "Serializes a message object of type '<FricWhl-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'open) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FricWhl-request>) istream)
  "Deserializes a message object of type '<FricWhl-request>"
    (cl:setf (cl:slot-value msg 'open) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FricWhl-request>)))
  "Returns string type for a service object of type '<FricWhl-request>"
  "roborts_msgs/FricWhlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FricWhl-request)))
  "Returns string type for a service object of type 'FricWhl-request"
  "roborts_msgs/FricWhlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FricWhl-request>)))
  "Returns md5sum for a message object of type '<FricWhl-request>"
  "fa14985976e9b1bf0f4e4ece48d5775a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FricWhl-request)))
  "Returns md5sum for a message object of type 'FricWhl-request"
  "fa14985976e9b1bf0f4e4ece48d5775a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FricWhl-request>)))
  "Returns full string definition for message of type '<FricWhl-request>"
  (cl:format cl:nil "bool open~%uint16 left~%uint16 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FricWhl-request)))
  "Returns full string definition for message of type 'FricWhl-request"
  (cl:format cl:nil "bool open~%uint16 left~%uint16 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FricWhl-request>))
  (cl:+ 0
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FricWhl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FricWhl-request
    (cl:cons ':open (open msg))
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
;//! \htmlinclude FricWhl-response.msg.html

(cl:defclass <FricWhl-response> (roslisp-msg-protocol:ros-message)
  ((received
    :reader received
    :initarg :received
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FricWhl-response (<FricWhl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FricWhl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FricWhl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roborts_msgs-srv:<FricWhl-response> is deprecated: use roborts_msgs-srv:FricWhl-response instead.")))

(cl:ensure-generic-function 'received-val :lambda-list '(m))
(cl:defmethod received-val ((m <FricWhl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roborts_msgs-srv:received-val is deprecated.  Use roborts_msgs-srv:received instead.")
  (received m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FricWhl-response>) ostream)
  "Serializes a message object of type '<FricWhl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'received) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FricWhl-response>) istream)
  "Deserializes a message object of type '<FricWhl-response>"
    (cl:setf (cl:slot-value msg 'received) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FricWhl-response>)))
  "Returns string type for a service object of type '<FricWhl-response>"
  "roborts_msgs/FricWhlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FricWhl-response)))
  "Returns string type for a service object of type 'FricWhl-response"
  "roborts_msgs/FricWhlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FricWhl-response>)))
  "Returns md5sum for a message object of type '<FricWhl-response>"
  "fa14985976e9b1bf0f4e4ece48d5775a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FricWhl-response)))
  "Returns md5sum for a message object of type 'FricWhl-response"
  "fa14985976e9b1bf0f4e4ece48d5775a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FricWhl-response>)))
  "Returns full string definition for message of type '<FricWhl-response>"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FricWhl-response)))
  "Returns full string definition for message of type 'FricWhl-response"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FricWhl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FricWhl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FricWhl-response
    (cl:cons ':received (received msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FricWhl)))
  'FricWhl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FricWhl)))
  'FricWhl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FricWhl)))
  "Returns string type for a service object of type '<FricWhl>"
  "roborts_msgs/FricWhl")