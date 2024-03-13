// Generated by gencpp from file _moveit_msgs/QueryPlannerInterfacesResponse.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_QUERYPLANNERINTERFACESRESPONSE_H
#define _MOVEIT_MSGS_MESSAGE_QUERYPLANNERINTERFACESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_moveit_msgs/PlannerInterfaceDescription.h>

namespace _moveit_msgs
{
template <class ContainerAllocator>
struct QueryPlannerInterfacesResponse_
{
  typedef QueryPlannerInterfacesResponse_<ContainerAllocator> Type;

  QueryPlannerInterfacesResponse_()
    : planner_interfaces()  {
    }
  QueryPlannerInterfacesResponse_(const ContainerAllocator& _alloc)
    : planner_interfaces(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::_moveit_msgs::PlannerInterfaceDescription_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::_moveit_msgs::PlannerInterfaceDescription_<ContainerAllocator> >::other >  _planner_interfaces_type;
  _planner_interfaces_type planner_interfaces;





  typedef std::shared_ptr< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct QueryPlannerInterfacesResponse_

typedef ::_moveit_msgs::QueryPlannerInterfacesResponse_<std::allocator<void> > QueryPlannerInterfacesResponse;

typedef std::shared_ptr< ::_moveit_msgs::QueryPlannerInterfacesResponse > QueryPlannerInterfacesResponsePtr;
typedef std::shared_ptr< ::_moveit_msgs::QueryPlannerInterfacesResponse const> QueryPlannerInterfacesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator1> & lhs, const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator2> & rhs)
{
  return lhs.planner_interfaces == rhs.planner_interfaces;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator1> & lhs, const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _moveit_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5876081117e7cad85cc165e937798753";
  }

  static const char* value(const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5876081117e7cad8ULL;
  static const uint64_t static_value2 = 0x5cc165e937798753ULL;
};

template<class ContainerAllocator>
struct DataType< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_moveit_msgs/QueryPlannerInterfacesResponse";
  }

  static const char* value(const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"# The planning instances that could be used in the benchmark\n"
"PlannerInterfaceDescription[] planner_interfaces\n"
"\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/PlannerInterfaceDescription\n"
"# The name of the planner interface\n"
"string name\n"
"\n"
"# The name of the planning pipeline\n"
"string pipeline_id\n"
"\n"
"# The names of the planner ids within the interface\n"
"string[] planner_ids\n"
;
  }

  static const char* value(const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.planner_interfaces);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QueryPlannerInterfacesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_moveit_msgs::QueryPlannerInterfacesResponse_<ContainerAllocator>& v)
  {
    s << indent << "planner_interfaces[]" << std::endl;
    for (size_t i = 0; i < v.planner_interfaces.size(); ++i)
    {
      s << indent << "  planner_interfaces[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::_moveit_msgs::PlannerInterfaceDescription_<ContainerAllocator> >::stream(s, indent + "    ", v.planner_interfaces[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_QUERYPLANNERINTERFACESRESPONSE_H
