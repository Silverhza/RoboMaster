// Generated by gencpp from file roborts_msgs/GyroEcdAngle.msg
// DO NOT EDIT!


#ifndef ROBORTS_MSGS_MESSAGE_GYROECDANGLE_H
#define ROBORTS_MSGS_MESSAGE_GYROECDANGLE_H


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
struct GyroEcdAngle_
{
  typedef GyroEcdAngle_<ContainerAllocator> Type;

  GyroEcdAngle_()
    : gyro_angle(0.0)
    , ecd_angle(0.0)  {
    }
  GyroEcdAngle_(const ContainerAllocator& _alloc)
    : gyro_angle(0.0)
    , ecd_angle(0.0)  {
  (void)_alloc;
    }



   typedef double _gyro_angle_type;
  _gyro_angle_type gyro_angle;

   typedef double _ecd_angle_type;
  _ecd_angle_type ecd_angle;





  typedef boost::shared_ptr< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> const> ConstPtr;

}; // struct GyroEcdAngle_

typedef ::roborts_msgs::GyroEcdAngle_<std::allocator<void> > GyroEcdAngle;

typedef boost::shared_ptr< ::roborts_msgs::GyroEcdAngle > GyroEcdAnglePtr;
typedef boost::shared_ptr< ::roborts_msgs::GyroEcdAngle const> GyroEcdAngleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator1> & lhs, const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator2> & rhs)
{
  return lhs.gyro_angle == rhs.gyro_angle &&
    lhs.ecd_angle == rhs.ecd_angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator1> & lhs, const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace roborts_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e67d66bb37dd1d2314499e5635fa43aa";
  }

  static const char* value(const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe67d66bb37dd1d23ULL;
  static const uint64_t static_value2 = 0x14499e5635fa43aaULL;
};

template<class ContainerAllocator>
struct DataType< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roborts_msgs/GyroEcdAngle";
  }

  static const char* value(const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#gimbal feedback gyro angle and encoder angledata\n"
"float64 gyro_angle\n"
"float64 ecd_angle\n"
;
  }

  static const char* value(const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gyro_angle);
      stream.next(m.ecd_angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GyroEcdAngle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roborts_msgs::GyroEcdAngle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roborts_msgs::GyroEcdAngle_<ContainerAllocator>& v)
  {
    s << indent << "gyro_angle: ";
    Printer<double>::stream(s, indent + "  ", v.gyro_angle);
    s << indent << "ecd_angle: ";
    Printer<double>::stream(s, indent + "  ", v.ecd_angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBORTS_MSGS_MESSAGE_GYROECDANGLE_H
