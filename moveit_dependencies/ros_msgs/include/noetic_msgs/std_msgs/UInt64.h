// Generated by gencpp from file std_msgs/UInt64.msg
// DO NOT EDIT!


#ifndef STD_MSGS_MESSAGE_UINT64_H
#define STD_MSGS_MESSAGE_UINT64_H


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
struct UInt64_
{
  typedef UInt64_<ContainerAllocator> Type;

  UInt64_()
    : data(0)  {
    }
  UInt64_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef uint64_t _data_type;
  _data_type data;





  typedef std::shared_ptr< ::std_msgs::UInt64_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::std_msgs::UInt64_<ContainerAllocator> const> ConstPtr;

}; // struct UInt64_

typedef ::std_msgs::UInt64_<std::allocator<void> > UInt64;

typedef std::shared_ptr< ::std_msgs::UInt64 > UInt64Ptr;
typedef std::shared_ptr< ::std_msgs::UInt64 const> UInt64ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::UInt64_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::UInt64_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_msgs::UInt64_<ContainerAllocator1> & lhs, const ::std_msgs::UInt64_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_msgs::UInt64_<ContainerAllocator1> & lhs, const ::std_msgs::UInt64_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::std_msgs::UInt64_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::UInt64_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::UInt64_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::UInt64_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::UInt64_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::UInt64_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::UInt64_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b2a79973e8bf53d7b53acb71299cb57";
  }

  static const char* value(const ::std_msgs::UInt64_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b2a79973e8bf53dULL;
  static const uint64_t static_value2 = 0x7b53acb71299cb57ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::UInt64_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/UInt64";
  }

  static const char* value(const ::std_msgs::UInt64_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::UInt64_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 data\n"
;
  }

  static const char* value(const ::std_msgs::UInt64_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::UInt64_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UInt64_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::UInt64_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::UInt64_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_UINT64_H
