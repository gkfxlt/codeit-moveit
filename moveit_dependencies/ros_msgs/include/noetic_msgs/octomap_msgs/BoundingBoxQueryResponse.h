// Generated by gencpp from file octomap_msgs/BoundingBoxQueryResponse.msg
// DO NOT EDIT!


#ifndef OCTOMAP_MSGS_MESSAGE_BOUNDINGBOXQUERYRESPONSE_H
#define OCTOMAP_MSGS_MESSAGE_BOUNDINGBOXQUERYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace octomap_msgs
{
template <class ContainerAllocator>
struct BoundingBoxQueryResponse_
{
  typedef BoundingBoxQueryResponse_<ContainerAllocator> Type;

  BoundingBoxQueryResponse_()
    {
    }
  BoundingBoxQueryResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef std::shared_ptr< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> const> ConstPtr;

}; // struct BoundingBoxQueryResponse_

typedef ::octomap_msgs::BoundingBoxQueryResponse_<std::allocator<void> > BoundingBoxQueryResponse;

typedef std::shared_ptr< ::octomap_msgs::BoundingBoxQueryResponse > BoundingBoxQueryResponsePtr;
typedef std::shared_ptr< ::octomap_msgs::BoundingBoxQueryResponse const> BoundingBoxQueryResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace octomap_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "octomap_msgs/BoundingBoxQueryResponse";
  }

  static const char* value(const ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
;
  }

  static const char* value(const ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BoundingBoxQueryResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::octomap_msgs::BoundingBoxQueryResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // OCTOMAP_MSGS_MESSAGE_BOUNDINGBOXQUERYRESPONSE_H
