// Generated by gencpp from file _std_srvs/TriggerResponse.msg
// DO NOT EDIT!


#ifndef _STD_SRVS_MESSAGE_TRIGGERRESPONSE_H
#define _STD_SRVS_MESSAGE_TRIGGERRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace _std_srvs
{
template <class ContainerAllocator>
struct TriggerResponse_
{
  typedef TriggerResponse_<ContainerAllocator> Type;

  TriggerResponse_()
    : success(false)
    , message()  {
    }
  TriggerResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::_std_srvs::TriggerResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::_std_srvs::TriggerResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TriggerResponse_

typedef ::_std_srvs::TriggerResponse_<std::allocator<void> > TriggerResponse;

typedef boost::shared_ptr< ::_std_srvs::TriggerResponse > TriggerResponsePtr;
typedef boost::shared_ptr< ::_std_srvs::TriggerResponse const> TriggerResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_std_srvs::TriggerResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_std_srvs::TriggerResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_std_srvs::TriggerResponse_<ContainerAllocator1> & lhs, const ::_std_srvs::TriggerResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.message == rhs.message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_std_srvs::TriggerResponse_<ContainerAllocator1> & lhs, const ::_std_srvs::TriggerResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _std_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_std_srvs::TriggerResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_std_srvs::TriggerResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_std_srvs::TriggerResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::_std_srvs::TriggerResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x937c9679a518e3a1ULL;
  static const uint64_t static_value2 = 0x8d831e57125ea522ULL;
};

template<class ContainerAllocator>
struct DataType< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_std_srvs/TriggerResponse";
  }

  static const char* value(const ::_std_srvs::TriggerResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success   # indicate successful run of triggered service\n"
"string message # informational, e.g. for error messages\n"
"\n"
;
  }

  static const char* value(const ::_std_srvs::TriggerResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TriggerResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_std_srvs::TriggerResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_std_srvs::TriggerResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_SRVS_MESSAGE_TRIGGERRESPONSE_H
