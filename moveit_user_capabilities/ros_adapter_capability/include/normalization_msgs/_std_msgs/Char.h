// Generated by gencpp from file _std_msgs/Char.msg
// DO NOT EDIT!


#ifndef _STD_MSGS_MESSAGE_CHAR_H
#define _STD_MSGS_MESSAGE_CHAR_H


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
struct Char_
{
  typedef Char_<ContainerAllocator> Type;

  Char_()
    : data(0)  {
    }
  Char_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef uint8_t _data_type;
  _data_type data;





  typedef std::shared_ptr< ::_std_msgs::Char_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_std_msgs::Char_<ContainerAllocator> const> ConstPtr;

}; // struct Char_

typedef ::_std_msgs::Char_<std::allocator<void> > Char;

typedef std::shared_ptr< ::_std_msgs::Char > CharPtr;
typedef std::shared_ptr< ::_std_msgs::Char const> CharConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_std_msgs::Char_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_std_msgs::Char_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_std_msgs::Char_<ContainerAllocator1> & lhs, const ::_std_msgs::Char_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_std_msgs::Char_<ContainerAllocator1> & lhs, const ::_std_msgs::Char_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _std_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_std_msgs::Char_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_std_msgs::Char_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_std_msgs::Char_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_std_msgs::Char_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_std_msgs::Char_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_std_msgs::Char_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_std_msgs::Char_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1bf77f25acecdedba0e224b162199717";
  }

  static const char* value(const ::_std_msgs::Char_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1bf77f25acecdedbULL;
  static const uint64_t static_value2 = 0xa0e224b162199717ULL;
};

template<class ContainerAllocator>
struct DataType< ::_std_msgs::Char_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_std_msgs/Char";
  }

  static const char* value(const ::_std_msgs::Char_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_std_msgs::Char_<ContainerAllocator> >
{
  static const char* value()
  {
    return "char data\n"
;
  }

  static const char* value(const ::_std_msgs::Char_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_std_msgs::Char_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Char_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_std_msgs::Char_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_std_msgs::Char_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_CHAR_H
