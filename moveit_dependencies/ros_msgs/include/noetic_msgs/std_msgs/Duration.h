// Generated by gencpp from file std_msgs/Duration.msg
// DO NOT EDIT!


#ifndef STD_MSGS_MESSAGE_DURATION_H
#define STD_MSGS_MESSAGE_DURATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace std_msgs
{
template <class ContainerAllocator>
struct Duration_
{
  typedef Duration_<ContainerAllocator> Type;

  Duration_()
    : data()  {
    }
  Duration_(const ContainerAllocator& _alloc)
    : data()  {
  (void)_alloc;
    }



   typedef ros::Duration _data_type;
  _data_type data;





  typedef std::shared_ptr< ::std_msgs::Duration_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::std_msgs::Duration_<ContainerAllocator> const> ConstPtr;

}; // struct Duration_

typedef ::std_msgs::Duration_<std::allocator<void> > Duration;

typedef std::shared_ptr< ::std_msgs::Duration > DurationPtr;
typedef std::shared_ptr< ::std_msgs::Duration const> DurationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Duration_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::Duration_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_msgs::Duration_<ContainerAllocator1> & lhs, const ::std_msgs::Duration_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_msgs::Duration_<ContainerAllocator1> & lhs, const ::std_msgs::Duration_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Duration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Duration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Duration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Duration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Duration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Duration_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Duration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e286caf4241d664e55f3ad380e2ae46";
  }

  static const char* value(const ::std_msgs::Duration_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e286caf4241d664ULL;
  static const uint64_t static_value2 = 0xe55f3ad380e2ae46ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Duration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Duration";
  }

  static const char* value(const ::std_msgs::Duration_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Duration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duration data\n"
;
  }

  static const char* value(const ::std_msgs::Duration_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::Duration_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Duration_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Duration_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::Duration_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_DURATION_H
