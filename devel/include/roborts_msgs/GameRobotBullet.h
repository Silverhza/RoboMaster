// Generated by gencpp from file roborts_msgs/GameRobotBullet.msg
// DO NOT EDIT!


#ifndef ROBORTS_MSGS_MESSAGE_GAMEROBOTBULLET_H
#define ROBORTS_MSGS_MESSAGE_GAMEROBOTBULLET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace roborts_msgs
{
template <class ContainerAllocator>
struct GameRobotBullet_
{
  typedef GameRobotBullet_<ContainerAllocator> Type;

  GameRobotBullet_()
    : red1(0)
    , red2(0)
    , blue1(0)
    , blue2(0)  {
    }
  GameRobotBullet_(const ContainerAllocator& _alloc)
    : red1(0)
    , red2(0)
    , blue1(0)
    , blue2(0)  {
  (void)_alloc;
    }



   typedef uint16_t _red1_type;
  _red1_type red1;

   typedef uint16_t _red2_type;
  _red2_type red2;

   typedef uint16_t _blue1_type;
  _blue1_type blue1;

   typedef uint16_t _blue2_type;
  _blue2_type blue2;





  typedef boost::shared_ptr< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> const> ConstPtr;

}; // struct GameRobotBullet_

typedef ::roborts_msgs::GameRobotBullet_<std::allocator<void> > GameRobotBullet;

typedef boost::shared_ptr< ::roborts_msgs::GameRobotBullet > GameRobotBulletPtr;
typedef boost::shared_ptr< ::roborts_msgs::GameRobotBullet const> GameRobotBulletConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roborts_msgs::GameRobotBullet_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::roborts_msgs::GameRobotBullet_<ContainerAllocator1> & lhs, const ::roborts_msgs::GameRobotBullet_<ContainerAllocator2> & rhs)
{
  return lhs.red1 == rhs.red1 &&
    lhs.red2 == rhs.red2 &&
    lhs.blue1 == rhs.blue1 &&
    lhs.blue2 == rhs.blue2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::roborts_msgs::GameRobotBullet_<ContainerAllocator1> & lhs, const ::roborts_msgs::GameRobotBullet_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace roborts_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fdaec03d4604469fd40ee7049d826d6e";
  }

  static const char* value(const ::roborts_msgs::GameRobotBullet_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfdaec03d4604469fULL;
  static const uint64_t static_value2 = 0xd40ee7049d826d6eULL;
};

template<class ContainerAllocator>
struct DataType< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roborts_msgs/GameRobotBullet";
  }

  static const char* value(const ::roborts_msgs::GameRobotBullet_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#game robot bullet\n"
"uint16 red1\n"
"uint16 red2\n"
"uint16 blue1\n"
"uint16 blue2\n"
;
  }

  static const char* value(const ::roborts_msgs::GameRobotBullet_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.red1);
      stream.next(m.red2);
      stream.next(m.blue1);
      stream.next(m.blue2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GameRobotBullet_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roborts_msgs::GameRobotBullet_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roborts_msgs::GameRobotBullet_<ContainerAllocator>& v)
  {
    s << indent << "red1: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.red1);
    s << indent << "red2: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.red2);
    s << indent << "blue1: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.blue1);
    s << indent << "blue2: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.blue2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBORTS_MSGS_MESSAGE_GAMEROBOTBULLET_H
