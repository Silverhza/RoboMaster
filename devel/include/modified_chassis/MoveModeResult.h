// Generated by gencpp from file modified_chassis/MoveModeResult.msg
// DO NOT EDIT!


#ifndef MODIFIED_CHASSIS_MESSAGE_MOVEMODERESULT_H
#define MODIFIED_CHASSIS_MESSAGE_MOVEMODERESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace modified_chassis
{
template <class ContainerAllocator>
struct MoveModeResult_
{
  typedef MoveModeResult_<ContainerAllocator> Type;

  MoveModeResult_()
    : is_finished(false)  {
    }
  MoveModeResult_(const ContainerAllocator& _alloc)
    : is_finished(false)  {
  (void)_alloc;
    }



   typedef uint8_t _is_finished_type;
  _is_finished_type is_finished;





  typedef boost::shared_ptr< ::modified_chassis::MoveModeResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::modified_chassis::MoveModeResult_<ContainerAllocator> const> ConstPtr;

}; // struct MoveModeResult_

typedef ::modified_chassis::MoveModeResult_<std::allocator<void> > MoveModeResult;

typedef boost::shared_ptr< ::modified_chassis::MoveModeResult > MoveModeResultPtr;
typedef boost::shared_ptr< ::modified_chassis::MoveModeResult const> MoveModeResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::modified_chassis::MoveModeResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::modified_chassis::MoveModeResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::modified_chassis::MoveModeResult_<ContainerAllocator1> & lhs, const ::modified_chassis::MoveModeResult_<ContainerAllocator2> & rhs)
{
  return lhs.is_finished == rhs.is_finished;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::modified_chassis::MoveModeResult_<ContainerAllocator1> & lhs, const ::modified_chassis::MoveModeResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace modified_chassis

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::modified_chassis::MoveModeResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::modified_chassis::MoveModeResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::modified_chassis::MoveModeResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "039e37637b9ee4ca0ff88874a3971b20";
  }

  static const char* value(const ::modified_chassis::MoveModeResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x039e37637b9ee4caULL;
  static const uint64_t static_value2 = 0x0ff88874a3971b20ULL;
};

template<class ContainerAllocator>
struct DataType< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "modified_chassis/MoveModeResult";
  }

  static const char* value(const ::modified_chassis::MoveModeResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the result\n"
"bool is_finished\n"
;
  }

  static const char* value(const ::modified_chassis::MoveModeResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_finished);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveModeResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::modified_chassis::MoveModeResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::modified_chassis::MoveModeResult_<ContainerAllocator>& v)
  {
    s << indent << "is_finished: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_finished);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MODIFIED_CHASSIS_MESSAGE_MOVEMODERESULT_H