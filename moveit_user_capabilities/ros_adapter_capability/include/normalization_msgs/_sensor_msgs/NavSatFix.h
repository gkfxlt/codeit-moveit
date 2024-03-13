// Generated by gencpp from file _sensor_msgs/NavSatFix.msg
// DO NOT EDIT!


#ifndef _SENSOR_MSGS_MESSAGE_NAVSATFIX_H
#define _SENSOR_MSGS_MESSAGE_NAVSATFIX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_std_msgs/Header.h>
#include <_sensor_msgs/NavSatStatus.h>

namespace _sensor_msgs
{
template <class ContainerAllocator>
struct NavSatFix_
{
  typedef NavSatFix_<ContainerAllocator> Type;

  NavSatFix_()
    : header()
    , status()
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , position_covariance()
    , position_covariance_type(0)  {
      position_covariance.assign(0.0);
  }
  NavSatFix_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , position_covariance()
    , position_covariance_type(0)  {
  (void)_alloc;
      position_covariance.assign(0.0);
  }



   typedef  ::_std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::_sensor_msgs::NavSatStatus_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef std::array<double, 9>  _position_covariance_type;
  _position_covariance_type position_covariance;

   typedef uint8_t _position_covariance_type_type;
  _position_covariance_type_type position_covariance_type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(COVARIANCE_TYPE_UNKNOWN)
  #undef COVARIANCE_TYPE_UNKNOWN
#endif
#if defined(_WIN32) && defined(COVARIANCE_TYPE_APPROXIMATED)
  #undef COVARIANCE_TYPE_APPROXIMATED
#endif
#if defined(_WIN32) && defined(COVARIANCE_TYPE_DIAGONAL_KNOWN)
  #undef COVARIANCE_TYPE_DIAGONAL_KNOWN
#endif
#if defined(_WIN32) && defined(COVARIANCE_TYPE_KNOWN)
  #undef COVARIANCE_TYPE_KNOWN
#endif

  enum {
    COVARIANCE_TYPE_UNKNOWN = 0u,
    COVARIANCE_TYPE_APPROXIMATED = 1u,
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2u,
    COVARIANCE_TYPE_KNOWN = 3u,
  };


  typedef std::shared_ptr< ::_sensor_msgs::NavSatFix_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_sensor_msgs::NavSatFix_<ContainerAllocator> const> ConstPtr;

}; // struct NavSatFix_

typedef ::_sensor_msgs::NavSatFix_<std::allocator<void> > NavSatFix;

typedef std::shared_ptr< ::_sensor_msgs::NavSatFix > NavSatFixPtr;
typedef std::shared_ptr< ::_sensor_msgs::NavSatFix const> NavSatFixConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_sensor_msgs::NavSatFix_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_sensor_msgs::NavSatFix_<ContainerAllocator1> & lhs, const ::_sensor_msgs::NavSatFix_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.status == rhs.status &&
    lhs.latitude == rhs.latitude &&
    lhs.longitude == rhs.longitude &&
    lhs.altitude == rhs.altitude &&
    lhs.position_covariance == rhs.position_covariance &&
    lhs.position_covariance_type == rhs.position_covariance_type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_sensor_msgs::NavSatFix_<ContainerAllocator1> & lhs, const ::_sensor_msgs::NavSatFix_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _sensor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::NavSatFix_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::NavSatFix_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::NavSatFix_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d3a8cd499b9b4a0249fb98fd05cfa48";
  }

  static const char* value(const ::_sensor_msgs::NavSatFix_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d3a8cd499b9b4a0ULL;
  static const uint64_t static_value2 = 0x249fb98fd05cfa48ULL;
};

template<class ContainerAllocator>
struct DataType< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_sensor_msgs/NavSatFix";
  }

  static const char* value(const ::_sensor_msgs::NavSatFix_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Navigation Satellite fix for any Global Navigation Satellite System\n"
"#\n"
"# Specified using the WGS 84 reference ellipsoid\n"
"\n"
"# header.stamp specifies the ROS time for this measurement (the\n"
"#        corresponding satellite time may be reported using the\n"
"#        _sensor_msgs/TimeReference message).\n"
"#\n"
"# header.frame_id is the frame of reference reported by the satellite\n"
"#        receiver, usually the location of the antenna.  This is a\n"
"#        Euclidean frame relative to the vehicle, not a reference\n"
"#        ellipsoid.\n"
"Header header\n"
"\n"
"# satellite fix status information\n"
"NavSatStatus status\n"
"\n"
"# Latitude [degrees]. Positive is north of equator; negative is south.\n"
"float64 latitude\n"
"\n"
"# Longitude [degrees]. Positive is east of prime meridian; negative is west.\n"
"float64 longitude\n"
"\n"
"# Altitude [m]. Positive is above the WGS 84 ellipsoid\n"
"# (quiet NaN if no altitude is available).\n"
"float64 altitude\n"
"\n"
"# Position covariance [m^2] defined relative to a tangential plane\n"
"# through the reported position. The components are East, North, and\n"
"# Up (ENU), in row-major order.\n"
"#\n"
"# Beware: this coordinate system exhibits singularities at the poles.\n"
"\n"
"float64[9] position_covariance\n"
"\n"
"# If the covariance of the fix is known, fill it in completely. If the\n"
"# GPS receiver provides the variance of each measurement, put them\n"
"# along the diagonal. If only Dilution of Precision is available,\n"
"# estimate an approximate covariance from that.\n"
"\n"
"uint8 COVARIANCE_TYPE_UNKNOWN = 0\n"
"uint8 COVARIANCE_TYPE_APPROXIMATED = 1\n"
"uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\n"
"uint8 COVARIANCE_TYPE_KNOWN = 3\n"
"\n"
"uint8 position_covariance_type\n"
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
"MSG: _sensor_msgs/NavSatStatus\n"
"# Navigation Satellite fix status for any Global Navigation Satellite System\n"
"\n"
"# Whether to output an augmented fix is determined by both the fix\n"
"# type and the last time differential corrections were received.  A\n"
"# fix is valid when status >= STATUS_FIX.\n"
"\n"
"int8 STATUS_NO_FIX =  -1        # unable to fix position\n"
"int8 STATUS_FIX =      0        # unaugmented fix\n"
"int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\n"
"int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n"
"\n"
"int8 status\n"
"\n"
"# Bits defining which Global Navigation Satellite System signals were\n"
"# used by the receiver.\n"
"\n"
"uint16 SERVICE_GPS =     1\n"
"uint16 SERVICE_GLONASS = 2\n"
"uint16 SERVICE_COMPASS = 4      # includes BeiDou.\n"
"uint16 SERVICE_GALILEO = 8\n"
"\n"
"uint16 service\n"
;
  }

  static const char* value(const ::_sensor_msgs::NavSatFix_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.position_covariance);
      stream.next(m.position_covariance_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavSatFix_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_sensor_msgs::NavSatFix_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_sensor_msgs::NavSatFix_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::_std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::_sensor_msgs::NavSatStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "position_covariance[]" << std::endl;
    for (size_t i = 0; i < v.position_covariance.size(); ++i)
    {
      s << indent << "  position_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position_covariance[i]);
    }
    s << indent << "position_covariance_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.position_covariance_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_NAVSATFIX_H
