// Generated by gencpp from file _geometry_msgs/Vector3Stamped.msg
// DO NOT EDIT!


#ifndef _GEOMETRY_MSGS_MESSAGE_VECTOR3STAMPED_H
#define _GEOMETRY_MSGS_MESSAGE_VECTOR3STAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_std_msgs/Header.h>
#include <_geometry_msgs/Vector3.h>

namespace _geometry_msgs
{
template <class ContainerAllocator>
struct Vector3Stamped_
{
  typedef Vector3Stamped_<ContainerAllocator> Type;

  Vector3Stamped_()
    : header()
    , vector()  {
    }
  Vector3Stamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vector(_alloc)  {
  (void)_alloc;
    }



   typedef  ::_std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::_geometry_msgs::Vector3_<ContainerAllocator>  _vector_type;
  _vector_type vector;





  typedef std::shared_ptr< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> const> ConstPtr;

}; // struct Vector3Stamped_

typedef ::_geometry_msgs::Vector3Stamped_<std::allocator<void> > Vector3Stamped;

typedef std::shared_ptr< ::_geometry_msgs::Vector3Stamped > Vector3StampedPtr;
typedef std::shared_ptr< ::_geometry_msgs::Vector3Stamped const> Vector3StampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator1> & lhs, const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vector == rhs.vector;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator1> & lhs, const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _geometry_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7b324c7325e683bf02a9b14b01090ec7";
  }

  static const char* value(const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7b324c7325e683bfULL;
  static const uint64_t static_value2 = 0x02a9b14b01090ec7ULL;
};

template<class ContainerAllocator>
struct DataType< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_geometry_msgs/Vector3Stamped";
  }

  static const char* value(const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents a Vector3 with reference coordinate frame and timestamp\n"
"Header header\n"
"Vector3 vector\n"
"\n"
"================================================================================\n"
"MSG: _std_msgs/Header\n"
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
"MSG: _geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# _geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vector);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Vector3Stamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_geometry_msgs::Vector3Stamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_geometry_msgs::Vector3Stamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::_std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vector: ";
    s << std::endl;
    Printer< ::_geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vector);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_VECTOR3STAMPED_H
