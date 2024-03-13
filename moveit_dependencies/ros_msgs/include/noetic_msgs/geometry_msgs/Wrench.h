// Generated by gencpp from file geometry_msgs/Wrench.msg
// DO NOT EDIT!


#ifndef GEOMETRY_MSGS_MESSAGE_WRENCH_H
#define GEOMETRY_MSGS_MESSAGE_WRENCH_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Wrench_
{
  typedef Wrench_<ContainerAllocator> Type;

  Wrench_()
    : force()
    , torque()  {
    }
  Wrench_(const ContainerAllocator& _alloc)
    : force(_alloc)
    , torque(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _force_type;
  _force_type force;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _torque_type;
  _torque_type torque;





  typedef std::shared_ptr< ::geometry_msgs::Wrench_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::geometry_msgs::Wrench_<ContainerAllocator> const> ConstPtr;

}; // struct Wrench_

typedef ::geometry_msgs::Wrench_<std::allocator<void> > Wrench;

typedef std::shared_ptr< ::geometry_msgs::Wrench > WrenchPtr;
typedef std::shared_ptr< ::geometry_msgs::Wrench const> WrenchConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Wrench_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::geometry_msgs::Wrench_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::geometry_msgs::Wrench_<ContainerAllocator1> & lhs, const ::geometry_msgs::Wrench_<ContainerAllocator2> & rhs)
{
  return lhs.force == rhs.force &&
    lhs.torque == rhs.torque;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::geometry_msgs::Wrench_<ContainerAllocator1> & lhs, const ::geometry_msgs::Wrench_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace geometry_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::Wrench_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::Wrench_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::Wrench_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::Wrench_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::Wrench_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::Wrench_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geometry_msgs::Wrench_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f539cf138b23283b520fd271b567936";
  }

  static const char* value(const ::geometry_msgs::Wrench_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f539cf138b23283ULL;
  static const uint64_t static_value2 = 0xb520fd271b567936ULL;
};

template<class ContainerAllocator>
struct DataType< ::geometry_msgs::Wrench_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Wrench";
  }

  static const char* value(const ::geometry_msgs::Wrench_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geometry_msgs::Wrench_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents force in free space, separated into\n"
"# its linear and angular parts.\n"
"Vector3  force\n"
"Vector3  torque\n"
"\n"
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

  static const char* value(const ::geometry_msgs::Wrench_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geometry_msgs::Wrench_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.force);
      stream.next(m.torque);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Wrench_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geometry_msgs::Wrench_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geometry_msgs::Wrench_<ContainerAllocator>& v)
  {
    s << indent << "force: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.force);
    s << indent << "torque: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.torque);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_WRENCH_H
