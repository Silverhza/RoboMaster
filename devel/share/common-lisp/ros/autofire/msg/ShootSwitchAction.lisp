; Auto-generated. Do not edit!


(cl:in-package autofire-msg)


;//! \htmlinclude ShootSwitchAction.msg.html

(cl:defclass <ShootSwitchAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type autofire-msg:ShootSwitchActionGoal
    :initform (cl:make-instance 'autofire-msg:ShootSwitchActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type autofire-msg:ShootSwitchActionResult
    :initform (cl:make-instance 'autofire-msg:ShootSwitchActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type autofire-msg:ShootSwitchActionFeedback
    :initform (cl:make-instance 'autofire-msg:ShootSwitchActionFeedback)))
)

(cl:defclass ShootSwitchAction (<ShootSwitchAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShootSwitchAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShootSwitchAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autofire-msg:<ShootSwitchAction> is deprecated: use autofire-msg:ShootSwitchAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <ShootSwitchAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-msg:action_goal-val is deprecated.  Use autofire-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <ShootSwitchAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-msg:action_result-val is deprecated.  Use autofire-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <ShootSwitchAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autofire-msg:action_feedback-val is deprecated.  Use autofire-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShootSwitchAction>) ostream)
  "Serializes a message object of type '<ShootSwitchAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShootSwitchAction>) istream)
  "Deserializes a message object of type '<ShootSwitchAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShootSwitchAction>)))
  "Returns string type for a message object of type '<ShootSwitchAction>"
  "autofire/ShootSwitchAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShootSwitchAction)))
  "Returns string type for a message object of type 'ShootSwitchAction"
  "autofire/ShootSwitchAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShootSwitchAction>)))
  "Returns md5sum for a message object of type '<ShootSwitchAction>"
  "df555fa4b6bad29ff607ae6ee4f12fcc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShootSwitchAction)))
  "Returns md5sum for a message object of type 'ShootSwitchAction"
  "df555fa4b6bad29ff607ae6ee4f12fcc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShootSwitchAction>)))
  "Returns full string definition for message of type '<ShootSwitchAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ShootSwitchActionGoal action_goal~%ShootSwitchActionResult action_result~%ShootSwitchActionFeedback action_feedback~%~%================================================================================~%MSG: autofire/ShootSwitchActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%ShootSwitchGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: autofire/ShootSwitchGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%uint32 cmd_id  # Specify which robot we want to recharge 1 enable 2 disable ~%~%================================================================================~%MSG: autofire/ShootSwitchActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ShootSwitchResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: autofire/ShootSwitchResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%bool is_finished #0 dead 1 no enemy~%~%================================================================================~%MSG: autofire/ShootSwitchActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ShootSwitchFeedback feedback~%~%================================================================================~%MSG: autofire/ShootSwitchFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%float32 percent_complete~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShootSwitchAction)))
  "Returns full string definition for message of type 'ShootSwitchAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ShootSwitchActionGoal action_goal~%ShootSwitchActionResult action_result~%ShootSwitchActionFeedback action_feedback~%~%================================================================================~%MSG: autofire/ShootSwitchActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%ShootSwitchGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: autofire/ShootSwitchGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%uint32 cmd_id  # Specify which robot we want to recharge 1 enable 2 disable ~%~%================================================================================~%MSG: autofire/ShootSwitchActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ShootSwitchResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: autofire/ShootSwitchResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%bool is_finished #0 dead 1 no enemy~%~%================================================================================~%MSG: autofire/ShootSwitchActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ShootSwitchFeedback feedback~%~%================================================================================~%MSG: autofire/ShootSwitchFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%float32 percent_complete~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShootSwitchAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShootSwitchAction>))
  "Converts a ROS message object to a list"
  (cl:list 'ShootSwitchAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))