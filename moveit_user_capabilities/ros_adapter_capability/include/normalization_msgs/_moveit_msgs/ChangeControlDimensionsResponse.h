// Generated by gencpp from file _moveit_msgs/ChangeControlDimensionsResponse.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_CHANGECONTROLDIMENSIONSRESPONSE_H
#define _MOVEIT_MSGS_MESSAGE_CHANGECONTROLDIMENSIONSRESPONSE_H


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
struct ChangeControlDimensionsResponse_
{
  typedef ChangeControlDimensionsResponse_<ContainerAllocator> Type;

  ChangeControlDimensionsResponse_()
    : success(false)  {
    }
  ChangeControlDimensionsResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef std::shared_ptr< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ChangeControlDimensionsResponse_

typedef ::_moveit_msgs::ChangeControlDimensionsResponse_<std::allocator<void> > ChangeControlDimensionsResponse;

typedef std::shared_ptr< ::_moveit_msgs::ChangeControlDimensionsResponse > ChangeControlDimensionsResponsePtr;
typedef std::shared_ptr< ::_moveit_msgs::ChangeControlDimensionsResponse const> ChangeControlDimensionsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator1> & lhs, const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator1> & lhs, const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _moveit_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_moveit_msgs/ChangeControlDimensionsResponse";
  }

  static const char* value(const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChangeControlDimensionsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_moveit_msgs::ChangeControlDimensionsResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_CHANGECONTROLDIMENSIONSRESPONSE_H
