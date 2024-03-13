// Generated by gencpp from file std_msgs/Empty.msg
// DO NOT EDIT!


#ifndef STD_MSGS_MESSAGE_EMPTY_H
#define STD_MSGS_MESSAGE_EMPTY_H


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
struct Empty_
{
  typedef Empty_<ContainerAllocator> Type;

  Empty_()
    {
    }
  Empty_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef std::shared_ptr< ::std_msgs::Empty_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::std_msgs::Empty_<ContainerAllocator> const> ConstPtr;

}; // struct Empty_

typedef ::std_msgs::Empty_<std::allocator<void> > Empty;

typedef std::shared_ptr< ::std_msgs::Empty > EmptyPtr;
typedef std::shared_ptr< ::std_msgs::Empty const> EmptyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Empty_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::Empty_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace std_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Empty_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Empty_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Empty_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Empty_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Empty_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Empty_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Empty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::std_msgs::Empty_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Empty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Empty";
  }

  static const char* value(const ::std_msgs::Empty_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Empty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::std_msgs::Empty_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::Empty_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Empty_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Empty_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::std_msgs::Empty_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_EMPTY_H
