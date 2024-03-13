// Generated by gencpp from file _sensor_msgs/JoyFeedback.msg
// DO NOT EDIT!


#ifndef _SENSOR_MSGS_MESSAGE_JOYFEEDBACK_H
#define _SENSOR_MSGS_MESSAGE_JOYFEEDBACK_H


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
struct JoyFeedback_
{
  typedef JoyFeedback_<ContainerAllocator> Type;

  JoyFeedback_()
    : type(0)
    , id(0)
    , intensity(0.0)  {
    }
  JoyFeedback_(const ContainerAllocator& _alloc)
    : type(0)
    , id(0)
    , intensity(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef uint8_t _id_type;
  _id_type id;

   typedef float _intensity_type;
  _intensity_type intensity;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(TYPE_LED)
  #undef TYPE_LED
#endif
#if defined(_WIN32) && defined(TYPE_RUMBLE)
  #undef TYPE_RUMBLE
#endif
#if defined(_WIN32) && defined(TYPE_BUZZER)
  #undef TYPE_BUZZER
#endif

  enum {
    TYPE_LED = 0u,
    TYPE_RUMBLE = 1u,
    TYPE_BUZZER = 2u,
  };


  typedef std::shared_ptr< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct JoyFeedback_

typedef ::_sensor_msgs::JoyFeedback_<std::allocator<void> > JoyFeedback;

typedef std::shared_ptr< ::_sensor_msgs::JoyFeedback > JoyFeedbackPtr;
typedef std::shared_ptr< ::_sensor_msgs::JoyFeedback const> JoyFeedbackConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_sensor_msgs::JoyFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_sensor_msgs::JoyFeedback_<ContainerAllocator1> & lhs, const ::_sensor_msgs::JoyFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.id == rhs.id &&
    lhs.intensity == rhs.intensity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_sensor_msgs::JoyFeedback_<ContainerAllocator1> & lhs, const ::_sensor_msgs::JoyFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _sensor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f4dcd73460360d98f36e55ee7f2e46f1";
  }

  static const char* value(const ::_sensor_msgs::JoyFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf4dcd73460360d98ULL;
  static const uint64_t static_value2 = 0xf36e55ee7f2e46f1ULL;
};

template<class ContainerAllocator>
struct DataType< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_sensor_msgs/JoyFeedback";
  }

  static const char* value(const ::_sensor_msgs::JoyFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Declare of the type of feedback\n"
"uint8 TYPE_LED    = 0\n"
"uint8 TYPE_RUMBLE = 1\n"
"uint8 TYPE_BUZZER = 2\n"
"\n"
"uint8 type\n"
"\n"
"# This will hold an id number for each type of each feedback.\n"
"# Example, the first led would be id=0, the second would be id=1\n"
"uint8 id\n"
"\n"
"# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is\n"
"# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.\n"
"float32 intensity\n"
"\n"
;
  }

  static const char* value(const ::_sensor_msgs::JoyFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.id);
      stream.next(m.intensity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JoyFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_sensor_msgs::JoyFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_sensor_msgs::JoyFeedback_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "intensity: ";
    Printer<float>::stream(s, indent + "  ", v.intensity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_JOYFEEDBACK_H
