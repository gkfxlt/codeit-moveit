// Generated by gencpp from file _sensor_msgs/Temperature.msg
// DO NOT EDIT!


#ifndef _SENSOR_MSGS_MESSAGE_TEMPERATURE_H
#define _SENSOR_MSGS_MESSAGE_TEMPERATURE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_std_msgs/Header.h>

namespace _sensor_msgs
{
template <class ContainerAllocator>
struct Temperature_
{
  typedef Temperature_<ContainerAllocator> Type;

  Temperature_()
    : header()
    , temperature(0.0)
    , variance(0.0)  {
    }
  Temperature_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , temperature(0.0)
    , variance(0.0)  {
  (void)_alloc;
    }



   typedef  ::_std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _temperature_type;
  _temperature_type temperature;

   typedef double _variance_type;
  _variance_type variance;





  typedef std::shared_ptr< ::_sensor_msgs::Temperature_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_sensor_msgs::Temperature_<ContainerAllocator> const> ConstPtr;

}; // struct Temperature_

typedef ::_sensor_msgs::Temperature_<std::allocator<void> > Temperature;

typedef std::shared_ptr< ::_sensor_msgs::Temperature > TemperaturePtr;
typedef std::shared_ptr< ::_sensor_msgs::Temperature const> TemperatureConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_sensor_msgs::Temperature_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_sensor_msgs::Temperature_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_sensor_msgs::Temperature_<ContainerAllocator1> & lhs, const ::_sensor_msgs::Temperature_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.temperature == rhs.temperature &&
    lhs.variance == rhs.variance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_sensor_msgs::Temperature_<ContainerAllocator1> & lhs, const ::_sensor_msgs::Temperature_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _sensor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::Temperature_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::Temperature_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::Temperature_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::Temperature_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::Temperature_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::Temperature_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_sensor_msgs::Temperature_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff71b307acdbe7c871a5a6d7ed359100";
  }

  static const char* value(const ::_sensor_msgs::Temperature_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff71b307acdbe7c8ULL;
  static const uint64_t static_value2 = 0x71a5a6d7ed359100ULL;
};

template<class ContainerAllocator>
struct DataType< ::_sensor_msgs::Temperature_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_sensor_msgs/Temperature";
  }

  static const char* value(const ::_sensor_msgs::Temperature_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_sensor_msgs::Temperature_<ContainerAllocator> >
{
  static const char* value()
  {
    return " # Single temperature reading.\n"
"\n"
" Header header           # timestamp is the time the temperature was measured\n"
"                         # frame_id is the location of the temperature reading\n"
"\n"
" float64 temperature     # Measurement of the Temperature in Degrees Celsius\n"
"\n"
" float64 variance        # 0 is interpreted as variance unknown\n"
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
;
  }

  static const char* value(const ::_sensor_msgs::Temperature_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_sensor_msgs::Temperature_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.temperature);
      stream.next(m.variance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Temperature_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_sensor_msgs::Temperature_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_sensor_msgs::Temperature_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::_std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "temperature: ";
    Printer<double>::stream(s, indent + "  ", v.temperature);
    s << indent << "variance: ";
    Printer<double>::stream(s, indent + "  ", v.variance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_TEMPERATURE_H
