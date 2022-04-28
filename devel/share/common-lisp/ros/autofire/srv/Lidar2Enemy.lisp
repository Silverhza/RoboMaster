; Auto-generated. Do not edit!


(cl:in-package autofire-srv)


;//! \htmlinclude Lidar2Enemy-request.msg.html

(cl:defclass <Lidar2Enemy-request> (roslisp-msg-protocol:ros-message)
  ((setStatus
    :reader setStatus
    :initarg :setStatus
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Lidar2Enemy-request (<Lidar2Enemy-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Lidar2Enemy-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Lidar2Enemy-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autofire-srv:<Lidar2Enemy-request> is deprecated: use autofire-srv:Lidar2Enemy-request instead.")))

(cl:ensure-generic-function 'setStatus-val :lambda-list '(m))
(cl:defmethod setStatus-val ((m <Lidar2Enemy-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-srv:setStatus-val is deprecated.  Use autofire-srv:setStatus instead.")
  (setStatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Lidar2Enemy-request>) ostream)
  "Serializes a message object of type '<Lidar2Enemy-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'setStatus) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Lidar2Enemy-request>) istream)
  "Deserializes a message object of type '<Lidar2Enemy-request>"
    (cl:setf (cl:slot-value msg 'setStatus) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Lidar2Enemy-request>)))
  "Returns string type for a service object of type '<Lidar2Enemy-request>"
  "autofire/Lidar2EnemyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lidar2Enemy-request)))
  "Returns string type for a service object of type 'Lidar2Enemy-request"
  "autofire/Lidar2EnemyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Lidar2Enemy-request>)))
  "Returns md5sum for a message object of type '<Lidar2Enemy-request>"
  "5c20fa677eb90ce8ed6acc054aa17bb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Lidar2Enemy-request)))
  "Returns md5sum for a message object of type 'Lidar2Enemy-request"
  "5c20fa677eb90ce8ed6acc054aa17bb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Lidar2Enemy-request>)))
  "Returns full string definition for message of type '<Lidar2Enemy-request>"
  (cl:format cl:nil "bool setStatus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Lidar2Enemy-request)))
  "Returns full string definition for message of type 'Lidar2Enemy-request"
  (cl:format cl:nil "bool setStatus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Lidar2Enemy-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Lidar2Enemy-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Lidar2Enemy-request
    (cl:cons ':setStatus (setStatus msg))
))
;//! \htmlinclude Lidar2Enemy-response.msg.html

(cl:defclass <Lidar2Enemy-response> (roslisp-msg-protocol:ros-message)
  ((received
    :reader received
    :initarg :received
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Lidar2Enemy-response (<Lidar2Enemy-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Lidar2Enemy-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Lidar2Enemy-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autofire-srv:<Lidar2Enemy-response> is deprecated: use autofire-srv:Lidar2Enemy-response instead.")))

(cl:ensure-generic-function 'received-val :lambda-list '(m))
(cl:defmethod received-val ((m <Lidar2Enemy-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-srv:received-val is deprecated.  Use autofire-srv:received instead.")
  (received m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Lidar2Enemy-response>) ostream)
  "Serializes a message object of type '<Lidar2Enemy-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'received) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Lidar2Enemy-response>) istream)
  "Deserializes a message object of type '<Lidar2Enemy-response>"
    (cl:setf (cl:slot-value msg 'received) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Lidar2Enemy-response>)))
  "Returns string type for a service object of type '<Lidar2Enemy-response>"
  "autofire/Lidar2EnemyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lidar2Enemy-response)))
  "Returns string type for a service object of type 'Lidar2Enemy-response"
  "autofire/Lidar2EnemyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Lidar2Enemy-response>)))
  "Returns md5sum for a message object of type '<Lidar2Enemy-response>"
  "5c20fa677eb90ce8ed6acc054aa17bb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Lidar2Enemy-response)))
  "Returns md5sum for a message object of type 'Lidar2Enemy-response"
  "5c20fa677eb90ce8ed6acc054aa17bb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Lidar2Enemy-response>)))
  "Returns full string definition for message of type '<Lidar2Enemy-response>"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Lidar2Enemy-response)))
  "Returns full string definition for message of type 'Lidar2Enemy-response"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Lidar2Enemy-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Lidar2Enemy-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Lidar2Enemy-response
    (cl:cons ':received (received msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Lidar2Enemy)))
  'Lidar2Enemy-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Lidar2Enemy)))
  'Lidar2Enemy-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lidar2Enemy)))
  "Returns string type for a service object of type '<Lidar2Enemy>"
  "autofire/Lidar2Enemy")