// Generated by gencpp from file _geometry_msgs/Point.msg
// DO NOT EDIT!


#ifndef _GEOMETRY_MSGS_MESSAGE_POINT_H
#define _GEOMETRY_MSGS_MESSAGE_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace _geometry_msgs
{
template <class ContainerAllocator>
struct Point_
{
  typedef Point_<ContainerAllocator> Type;

  Point_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Point_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;





  typedef std::shared_ptr< ::_geometry_msgs::Point_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_geometry_msgs::Point_<ContainerAllocator> const> ConstPtr;

}; // struct Point_

typedef ::_geometry_msgs::Point_<std::allocator<void> > Point;

typedef std::shared_ptr< ::_geometry_msgs::Point > PointPtr;
typedef std::shared_ptr< ::_geometry_msgs::Point const> PointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_geometry_msgs::Point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_geometry_msgs::Point_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_geometry_msgs::Point_<ContainerAllocator1> & lhs, const ::_geometry_msgs::Point_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_geometry_msgs::Point_<ContainerAllocator1> & lhs, const ::_geometry_msgs::Point_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _geometry_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::Point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::Point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::Point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::Point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::Point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::Point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_geometry_msgs::Point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a842b65f413084dc2b10fb484ea7f17";
  }

  static const char* value(const ::_geometry_msgs::Point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a842b65f413084dULL;
  static const uint64_t static_value2 = 0xc2b10fb484ea7f17ULL;
};

template<class ContainerAllocator>
struct DataType< ::_geometry_msgs::Point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_geometry_msgs/Point";
  }

  static const char* value(const ::_geometry_msgs::Point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_geometry_msgs::Point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::_geometry_msgs::Point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_geometry_msgs::Point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_geometry_msgs::Point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_geometry_msgs::Point_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_POINT_H
