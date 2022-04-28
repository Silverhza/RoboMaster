// Generated by gencpp from file obstacle_detector/Obstacles.msg
// DO NOT EDIT!


#ifndef OBSTACLE_DETECTOR_MESSAGE_OBSTACLES_H
#define OBSTACLE_DETECTOR_MESSAGE_OBSTACLES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <obstacle_detector/SegmentObstacle.h>
#include <obstacle_detector/CircleObstacle.h>

namespace obstacle_detector
{
template <class ContainerAllocator>
struct Obstacles_
{
  typedef Obstacles_<ContainerAllocator> Type;

  Obstacles_()
    : header()
    , segments()
    , circles()  {
    }
  Obstacles_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , segments(_alloc)
    , circles(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::obstacle_detector::SegmentObstacle_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::obstacle_detector::SegmentObstacle_<ContainerAllocator> >::other >  _segments_type;
  _segments_type segments;

   typedef std::vector< ::obstacle_detector::CircleObstacle_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::obstacle_detector::CircleObstacle_<ContainerAllocator> >::other >  _circles_type;
  _circles_type circles;





  typedef boost::shared_ptr< ::obstacle_detector::Obstacles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::obstacle_detector::Obstacles_<ContainerAllocator> const> ConstPtr;

}; // struct Obstacles_

typedef ::obstacle_detector::Obstacles_<std::allocator<void> > Obstacles;

typedef boost::shared_ptr< ::obstacle_detector::Obstacles > ObstaclesPtr;
typedef boost::shared_ptr< ::obstacle_detector::Obstacles const> ObstaclesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::obstacle_detector::Obstacles_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::obstacle_detector::Obstacles_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::obstacle_detector::Obstacles_<ContainerAllocator1> & lhs, const ::obstacle_detector::Obstacles_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.segments == rhs.segments &&
    lhs.circles == rhs.circles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::obstacle_detector::Obstacles_<ContainerAllocator1> & lhs, const ::obstacle_detector::Obstacles_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace obstacle_detector

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::obstacle_detector::Obstacles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::obstacle_detector::Obstacles_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obstacle_detector::Obstacles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obstacle_detector::Obstacles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obstacle_detector::Obstacles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obstacle_detector::Obstacles_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::obstacle_detector::Obstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3d18886bf3de3d0abbd8f4ab62b1ac0b";
  }

  static const char* value(const ::obstacle_detector::Obstacles_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3d18886bf3de3d0aULL;
  static const uint64_t static_value2 = 0xbbd8f4ab62b1ac0bULL;
};

template<class ContainerAllocator>
struct DataType< ::obstacle_detector::Obstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "obstacle_detector/Obstacles";
  }

  static const char* value(const ::obstacle_detector::Obstacles_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::obstacle_detector::Obstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"obstacle_detector/SegmentObstacle[] segments\n"
"obstacle_detector/CircleObstacle[] circles\n"
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
"MSG: obstacle_detector/SegmentObstacle\n"
"geometry_msgs/Point first_point  # First point of the segment [m]\n"
"geometry_msgs/Point last_point   # Last point of the segment [m]\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: obstacle_detector/CircleObstacle\n"
"geometry_msgs/Point center      # Central point [m]\n"
"geometry_msgs/Vector3 velocity  # Linear velocity [m/s]\n"
"float64 radius                  # Radius with added margin [m]\n"
"float64 true_radius             # True measured radius [m]\n"
"obstacle_detector/SegmentObstacle segment    # Keep track of the segment which the circle was spawned\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::obstacle_detector::Obstacles_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::obstacle_detector::Obstacles_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.segments);
      stream.next(m.circles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Obstacles_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::obstacle_detector::Obstacles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::obstacle_detector::Obstacles_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "segments[]" << std::endl;
    for (size_t i = 0; i < v.segments.size(); ++i)
    {
      s << indent << "  segments[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::obstacle_detector::SegmentObstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.segments[i]);
    }
    s << indent << "circles[]" << std::endl;
    for (size_t i = 0; i < v.circles.size(); ++i)
    {
      s << indent << "  circles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::obstacle_detector::CircleObstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.circles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBSTACLE_DETECTOR_MESSAGE_OBSTACLES_H