// Generated by gencpp from file _moveit_msgs/PickupFeedback.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_PICKUPFEEDBACK_H
#define _MOVEIT_MSGS_MESSAGE_PICKUPFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace _moveit_msgs
{
template <class ContainerAllocator>
struct PickupFeedback_
{
  typedef PickupFeedback_<ContainerAllocator> Type;

  PickupFeedback_()
    : state()  {
    }
  PickupFeedback_(const ContainerAllocator& _alloc)
    : state(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _state_type;
  _state_type state;





  typedef std::shared_ptr< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct PickupFeedback_

typedef ::_moveit_msgs::PickupFeedback_<std::allocator<void> > PickupFeedback;

typedef std::shared_ptr< ::_moveit_msgs::PickupFeedback > PickupFeedbackPtr;
typedef std::shared_ptr< ::_moveit_msgs::PickupFeedback const> PickupFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_moveit_msgs::PickupFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_moveit_msgs::PickupFeedback_<ContainerAllocator1> & lhs, const ::_moveit_msgs::PickupFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_moveit_msgs::PickupFeedback_<ContainerAllocator1> & lhs, const ::_moveit_msgs::PickupFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _moveit_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "af6d3a99f0fbeb66d3248fa4b3e675fb";
  }

  static const char* value(const ::_moveit_msgs::PickupFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaf6d3a99f0fbeb66ULL;
  static const uint64_t static_value2 = 0xd3248fa4b3e675fbULL;
};

template<class ContainerAllocator>
struct DataType< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_moveit_msgs/PickupFeedback";
  }

  static const char* value(const ::_moveit_msgs::PickupFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# The internal state that the pickup action currently is in\n"
"string state\n"
"\n"
;
  }

  static const char* value(const ::_moveit_msgs::PickupFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PickupFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_moveit_msgs::PickupFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_moveit_msgs::PickupFeedback_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_PICKUPFEEDBACK_H
