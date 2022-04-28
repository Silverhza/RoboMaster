; Auto-generated. Do not edit!


(cl:in-package autofire-msg)


;//! \htmlinclude enemy_id.msg.html

(cl:defclass <enemy_id> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass enemy_id (<enemy_id>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <enemy_id>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'enemy_id)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autofire-msg:<enemy_id> is deprecated: use autofire-msg:enemy_id instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <enemy_id>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-msg:x-val is deprecated.  Use autofire-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <enemy_id>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-msg:y-val is deprecated.  Use autofire-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <enemy_id>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-msg:id-val is deprecated.  Use autofire-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <enemy_id>) ostream)
  "Serializes a message object of type '<enemy_id>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <enemy_id>) istream)
  "Deserializes a message object of type '<enemy_id>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<enemy_id>)))
  "Returns string type for a message object of type '<enemy_id>"
  "autofire/enemy_id")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'enemy_id)))
  "Returns string type for a message object of type 'enemy_id"
  "autofire/enemy_id")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<enemy_id>)))
  "Returns md5sum for a message object of type '<enemy_id>"
  "7993d0d3043d5146dda81224aff3279c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'enemy_id)))
  "Returns md5sum for a message object of type 'enemy_id"
  "7993d0d3043d5146dda81224aff3279c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<enemy_id>)))
  "Returns full string definition for message of type '<enemy_id>"
  (cl:format cl:nil "float32 x~%float32 y~%string id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'enemy_id)))
  "Returns full string definition for message of type 'enemy_id"
  (cl:format cl:nil "float32 x~%float32 y~%string id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <enemy_id>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <enemy_id>))
  "Converts a ROS message object to a list"
  (cl:list 'enemy_id
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':id (id msg))
))
