// Generated by gencpp from file _std_msgs/Float64.msg
// DO NOT EDIT!


#ifndef _STD_MSGS_MESSAGE_FLOAT64_H
#define _STD_MSGS_MESSAGE_FLOAT64_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace _std_msgs
{
template <class ContainerAllocator>
struct Float64_
{
  typedef Float64_<ContainerAllocator> Type;

  Float64_()
    : data(0.0)  {
    }
  Float64_(const ContainerAllocator& _alloc)
    : data(0.0)  {
  (void)_alloc;
    }



   typedef double _data_type;
  _data_type data;





  typedef std::shared_ptr< ::_std_msgs::Float64_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_std_msgs::Float64_<ContainerAllocator> const> ConstPtr;

}; // struct Float64_

typedef ::_std_msgs::Float64_<std::allocator<void> > Float64;

typedef std::shared_ptr< ::_std_msgs::Float64 > Float64Ptr;
typedef std::shared_ptr< ::_std_msgs::Float64 const> Float64ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_std_msgs::Float64_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_std_msgs::Float64_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_std_msgs::Float64_<ContainerAllocator1> & lhs, const ::_std_msgs::Float64_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_std_msgs::Float64_<ContainerAllocator1> & lhs, const ::_std_msgs::Float64_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _std_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_std_msgs::Float64_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_std_msgs::Float64_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_std_msgs::Float64_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_std_msgs::Float64_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_std_msgs::Float64_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_std_msgs::Float64_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_std_msgs::Float64_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fdb28210bfa9d7c91146260178d9a584";
  }

  static const char* value(const ::_std_msgs::Float64_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfdb28210bfa9d7c9ULL;
  static const uint64_t static_value2 = 0x1146260178d9a584ULL;
};

template<class ContainerAllocator>
struct DataType< ::_std_msgs::Float64_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_std_msgs/Float64";
  }

  static const char* value(const ::_std_msgs::Float64_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_std_msgs::Float64_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 data\n"
;
  }

  static const char* value(const ::_std_msgs::Float64_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_std_msgs::Float64_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Float64_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_std_msgs::Float64_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_std_msgs::Float64_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<double>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_FLOAT64_H