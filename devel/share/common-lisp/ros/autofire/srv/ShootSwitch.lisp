; Auto-generated. Do not edit!


(cl:in-package autofire-srv)


;//! \htmlinclude ShootSwitch-request.msg.html

(cl:defclass <ShootSwitch-request> (roslisp-msg-protocol:ros-message)
  ((enableShoot
    :reader enableShoot
    :initarg :enableShoot
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ShootSwitch-request (<ShootSwitch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShootSwitch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShootSwitch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autofire-srv:<ShootSwitch-request> is deprecated: use autofire-srv:ShootSwitch-request instead.")))

(cl:ensure-generic-function 'enableShoot-val :lambda-list '(m))
(cl:defmethod enableShoot-val ((m <ShootSwitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-srv:enableShoot-val is deprecated.  Use autofire-srv:enableShoot instead.")
  (enableShoot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShootSwitch-request>) ostream)
  "Serializes a message object of type '<ShootSwitch-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enableShoot) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShootSwitch-request>) istream)
  "Deserializes a message object of type '<ShootSwitch-request>"
    (cl:setf (cl:slot-value msg 'enableShoot) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShootSwitch-request>)))
  "Returns string type for a service object of type '<ShootSwitch-request>"
  "autofire/ShootSwitchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShootSwitch-request)))
  "Returns string type for a service object of type 'ShootSwitch-request"
  "autofire/ShootSwitchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShootSwitch-request>)))
  "Returns md5sum for a message object of type '<ShootSwitch-request>"
  "5d377b7485c09603dc7fd050a7504601")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShootSwitch-request)))
  "Returns md5sum for a message object of type 'ShootSwitch-request"
  "5d377b7485c09603dc7fd050a7504601")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShootSwitch-request>)))
  "Returns full string definition for message of type '<ShootSwitch-request>"
  (cl:format cl:nil "bool enableShoot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShootSwitch-request)))
  "Returns full string definition for message of type 'ShootSwitch-request"
  (cl:format cl:nil "bool enableShoot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShootSwitch-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShootSwitch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ShootSwitch-request
    (cl:cons ':enableShoot (enableShoot msg))
))
;//! \htmlinclude ShootSwitch-response.msg.html

(cl:defclass <ShootSwitch-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ShootSwitch-response (<ShootSwitch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShootSwitch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShootSwitch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autofire-srv:<ShootSwitch-response> is deprecated: use autofire-srv:ShootSwitch-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ShootSwitch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-srv:status-val is deprecated.  Use autofire-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShootSwitch-response>) ostream)
  "Serializes a message object of type '<ShootSwitch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShootSwitch-response>) istream)
  "Deserializes a message object of type '<ShootSwitch-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShootSwitch-response>)))
  "Returns string type for a service object of type '<ShootSwitch-response>"
  "autofire/ShootSwitchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShootSwitch-response)))
  "Returns string type for a service object of type 'ShootSwitch-response"
  "autofire/ShootSwitchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShootSwitch-response>)))
  "Returns md5sum for a message object of type '<ShootSwitch-response>"
  "5d377b7485c09603dc7fd050a7504601")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShootSwitch-response)))
  "Returns md5sum for a message object of type 'ShootSwitch-response"
  "5d377b7485c09603dc7fd050a7504601")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShootSwitch-response>)))
  "Returns full string definition for message of type '<ShootSwitch-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShootSwitch-response)))
  "Returns full string definition for message of type 'ShootSwitch-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShootSwitch-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShootSwitch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ShootSwitch-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ShootSwitch)))
  'ShootSwitch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ShootSwitch)))
  'ShootSwitch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShootSwitch)))
  "Returns string type for a service object of type '<ShootSwitch>"
  "autofire/ShootSwitch")