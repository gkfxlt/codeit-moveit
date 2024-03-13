// Generated by gencpp from file _geometry_msgs/TwistWithCovariance.msg
// DO NOT EDIT!


#ifndef _GEOMETRY_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H
#define _GEOMETRY_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_geometry_msgs/Twist.h>

namespace _geometry_msgs
{
template <class ContainerAllocator>
struct TwistWithCovariance_
{
  typedef TwistWithCovariance_<ContainerAllocator> Type;

  TwistWithCovariance_()
    : twist()
    , covariance()  {
      covariance.assign(0.0);
  }
  TwistWithCovariance_(const ContainerAllocator& _alloc)
    : twist(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::_geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef std::array<double, 36>  _covariance_type;
  _covariance_type covariance;





  typedef std::shared_ptr< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct TwistWithCovariance_

typedef ::_geometry_msgs::TwistWithCovariance_<std::allocator<void> > TwistWithCovariance;

typedef std::shared_ptr< ::_geometry_msgs::TwistWithCovariance > TwistWithCovariancePtr;
typedef std::shared_ptr< ::_geometry_msgs::TwistWithCovariance const> TwistWithCovarianceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator1> & lhs, const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator2> & rhs)
{
  return lhs.twist == rhs.twist &&
    lhs.covariance == rhs.covariance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator1> & lhs, const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _geometry_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1fe8a28e6890a4cc3ae4c3ca5c7d82e6";
  }

  static const char* value(const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1fe8a28e6890a4ccULL;
  static const uint64_t static_value2 = 0x3ae4c3ca5c7d82e6ULL;
};

template<class ContainerAllocator>
struct DataType< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_geometry_msgs/TwistWithCovariance";
  }

  static const char* value(const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This expresses velocity in free space with uncertainty.\n"
"\n"
"Twist twist\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
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

  static const char* value(const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.twist);
      stream.next(m.covariance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TwistWithCovariance_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_geometry_msgs::TwistWithCovariance_<ContainerAllocator>& v)
  {
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::_geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "covariance[]" << std::endl;
    for (size_t i = 0; i < v.covariance.size(); ++i)
    {
      s << indent << "  covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.covariance[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H
