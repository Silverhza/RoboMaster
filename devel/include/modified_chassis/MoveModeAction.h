// Generated by gencpp from file modified_chassis/MoveModeAction.msg
// DO NOT EDIT!


#ifndef MODIFIED_CHASSIS_MESSAGE_MOVEMODEACTION_H
#define MODIFIED_CHASSIS_MESSAGE_MOVEMODEACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <modified_chassis/MoveModeActionGoal.h>
#include <modified_chassis/MoveModeActionResult.h>
#include <modified_chassis/MoveModeActionFeedback.h>

namespace modified_chassis
{
template <class ContainerAllocator>
struct MoveModeAction_
{
  typedef MoveModeAction_<ContainerAllocator> Type;

  MoveModeAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  MoveModeAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::modified_chassis::MoveModeActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::modified_chassis::MoveModeActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::modified_chassis::MoveModeActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::modified_chassis::MoveModeAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::modified_chassis::MoveModeAction_<ContainerAllocator> const> ConstPtr;

}; // struct MoveModeAction_

typedef ::modified_chassis::MoveModeAction_<std::allocator<void> > MoveModeAction;

typedef boost::shared_ptr< ::modified_chassis::MoveModeAction > MoveModeActionPtr;
typedef boost::shared_ptr< ::modified_chassis::MoveModeAction const> MoveModeActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::modified_chassis::MoveModeAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::modified_chassis::MoveModeAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::modified_chassis::MoveModeAction_<ContainerAllocator1> & lhs, const ::modified_chassis::MoveModeAction_<ContainerAllocator2> & rhs)
{
  return lhs.action_goal == rhs.action_goal &&
    lhs.action_result == rhs.action_result &&
    lhs.action_feedback == rhs.action_feedback;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::modified_chassis::MoveModeAction_<ContainerAllocator1> & lhs, const ::modified_chassis::MoveModeAction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace modified_chassis

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::modified_chassis::MoveModeAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::modified_chassis::MoveModeAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::modified_chassis::MoveModeAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9e5f7897cfe9b4966af2e5808ba42b11";
  }

  static const char* value(const ::modified_chassis::MoveModeAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9e5f7897cfe9b496ULL;
  static const uint64_t static_value2 = 0x6af2e5808ba42b11ULL;
};

template<class ContainerAllocator>
struct DataType< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "modified_chassis/MoveModeAction";
  }

  static const char* value(const ::modified_chassis::MoveModeAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"MoveModeActionGoal action_goal\n"
"MoveModeActionResult action_result\n"
"MoveModeActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: modified_chassis/MoveModeActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"MoveModeGoal goal\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: modified_chassis/MoveModeGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the goal\n"
"bool mode  # Specify which robot we want to recharge\n"
"\n"
"================================================================================\n"
"MSG: modified_chassis/MoveModeActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"MoveModeResult result\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: modified_chassis/MoveModeResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the result\n"
"bool is_finished\n"
"\n"
"================================================================================\n"
"MSG: modified_chassis/MoveModeActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"MoveModeFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: modified_chassis/MoveModeFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define a feedback message\n"
"float32 state_keeping\n"
"\n"
;
  }

  static const char* value(const ::modified_chassis::MoveModeAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveModeAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::modified_chassis::MoveModeAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::modified_chassis::MoveModeAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::modified_chassis::MoveModeActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::modified_chassis::MoveModeActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::modified_chassis::MoveModeActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MODIFIED_CHASSIS_MESSAGE_MOVEMODEACTION_H
