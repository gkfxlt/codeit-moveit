// Generated by gencpp from file _sensor_msgs/LaserEcho.msg
// DO NOT EDIT!


#ifndef _SENSOR_MSGS_MESSAGE_LASERECHO_H
#define _SENSOR_MSGS_MESSAGE_LASERECHO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace _sensor_msgs
{
template <class ContainerAllocator>
struct LaserEcho_
{
  typedef LaserEcho_<ContainerAllocator> Type;

  LaserEcho_()
    : echoes()  {
    }
  LaserEcho_(const ContainerAllocator& _alloc)
    : echoes(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _echoes_type;
  _echoes_type echoes;





  typedef std::shared_ptr< ::_sensor_msgs::LaserEcho_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_sensor_msgs::LaserEcho_<ContainerAllocator> const> ConstPtr;

}; // struct LaserEcho_

typedef ::_sensor_msgs::LaserEcho_<std::allocator<void> > LaserEcho;

typedef std::shared_ptr< ::_sensor_msgs::LaserEcho > LaserEchoPtr;
typedef std::shared_ptr< ::_sensor_msgs::LaserEcho const> LaserEchoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_sensor_msgs::LaserEcho_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_sensor_msgs::LaserEcho_<ContainerAllocator1> & lhs, const ::_sensor_msgs::LaserEcho_<ContainerAllocator2> & rhs)
{
  return lhs.echoes == rhs.echoes;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_sensor_msgs::LaserEcho_<ContainerAllocator1> & lhs, const ::_sensor_msgs::LaserEcho_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _sensor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::LaserEcho_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::LaserEcho_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::LaserEcho_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8bc5ae449b200fba4d552b4225586696";
  }

  static const char* value(const ::_sensor_msgs::LaserEcho_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8bc5ae449b200fbaULL;
  static const uint64_t static_value2 = 0x4d552b4225586696ULL;
};

template<class ContainerAllocator>
struct DataType< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_sensor_msgs/LaserEcho";
  }

  static const char* value(const ::_sensor_msgs::LaserEcho_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message is a submessage of MultiEchoLaserScan and is not intended\n"
"# to be used separately.\n"
"\n"
"float32[] echoes  # Multiple values of ranges or intensities.\n"
"                  # Each array represents data from the same angle increment.\n"
;
  }

  static const char* value(const ::_sensor_msgs::LaserEcho_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.echoes);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LaserEcho_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_sensor_msgs::LaserEcho_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_sensor_msgs::LaserEcho_<ContainerAllocator>& v)
  {
    s << indent << "echoes[]" << std::endl;
    for (size_t i = 0; i < v.echoes.size(); ++i)
    {
      s << indent << "  echoes[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.echoes[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_LASERECHO_H