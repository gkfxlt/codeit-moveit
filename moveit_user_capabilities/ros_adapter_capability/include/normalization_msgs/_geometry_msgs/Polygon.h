// Generated by gencpp from file _geometry_msgs/Polygon.msg
// DO NOT EDIT!


#ifndef _GEOMETRY_MSGS_MESSAGE_POLYGON_H
#define _GEOMETRY_MSGS_MESSAGE_POLYGON_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_geometry_msgs/Point32.h>

namespace _geometry_msgs
{
template <class ContainerAllocator>
struct Polygon_
{
  typedef Polygon_<ContainerAllocator> Type;

  Polygon_()
    : points()  {
    }
  Polygon_(const ContainerAllocator& _alloc)
    : points(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::_geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::_geometry_msgs::Point32_<ContainerAllocator> >::other >  _points_type;
  _points_type points;





  typedef std::shared_ptr< ::_geometry_msgs::Polygon_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_geometry_msgs::Polygon_<ContainerAllocator> const> ConstPtr;

}; // struct Polygon_

typedef ::_geometry_msgs::Polygon_<std::allocator<void> > Polygon;

typedef std::shared_ptr< ::_geometry_msgs::Polygon > PolygonPtr;
typedef std::shared_ptr< ::_geometry_msgs::Polygon const> PolygonConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_geometry_msgs::Polygon_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_geometry_msgs::Polygon_<ContainerAllocator1> & lhs, const ::_geometry_msgs::Polygon_<ContainerAllocator2> & rhs)
{
  return lhs.points == rhs.points;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_geometry_msgs::Polygon_<ContainerAllocator1> & lhs, const ::_geometry_msgs::Polygon_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _geometry_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::Polygon_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::Polygon_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::Polygon_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::Polygon_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::Polygon_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::Polygon_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_geometry_msgs::Polygon_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cd60a26494a087f577976f0329fa120e";
  }

  static const char* value(const ::_geometry_msgs::Polygon_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcd60a26494a087f5ULL;
  static const uint64_t static_value2 = 0x77976f0329fa120eULL;
};

template<class ContainerAllocator>
struct DataType< ::_geometry_msgs::Polygon_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_geometry_msgs/Polygon";
  }

  static const char* value(const ::_geometry_msgs::Polygon_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_geometry_msgs::Polygon_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::_geometry_msgs::Polygon_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_geometry_msgs::Polygon_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Polygon_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_geometry_msgs::Polygon_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_geometry_msgs::Polygon_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::_geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_POLYGON_H
