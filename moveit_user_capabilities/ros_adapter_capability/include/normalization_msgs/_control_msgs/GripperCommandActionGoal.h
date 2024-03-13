// Generated by gencpp from file _control_msgs/GripperCommandActionGoal.msg
// DO NOT EDIT!


#ifndef _CONTROL_MSGS_MESSAGE_GRIPPERCOMMANDACTIONGOAL_H
#define _CONTROL_MSGS_MESSAGE_GRIPPERCOMMANDACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_std_msgs/Header.h>
#include <_actionlib_msgs/GoalID.h>
#include <_control_msgs/GripperCommandGoal.h>

namespace _control_msgs
{
template <class ContainerAllocator>
struct GripperCommandActionGoal_
{
  typedef GripperCommandActionGoal_<ContainerAllocator> Type;

  GripperCommandActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  GripperCommandActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::_std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::_actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::_control_msgs::GripperCommandGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef std::shared_ptr< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct GripperCommandActionGoal_

typedef ::_control_msgs::GripperCommandActionGoal_<std::allocator<void> > GripperCommandActionGoal;

typedef std::shared_ptr< ::_control_msgs::GripperCommandActionGoal > GripperCommandActionGoalPtr;
typedef std::shared_ptr< ::_control_msgs::GripperCommandActionGoal const> GripperCommandActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator1> & lhs, const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.goal_id == rhs.goal_id &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator1> & lhs, const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _control_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aa581f648a35ed681db2ec0bf7a82bea";
  }

  static const char* value(const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaa581f648a35ed68ULL;
  static const uint64_t static_value2 = 0x1db2ec0bf7a82beaULL;
};

template<class ContainerAllocator>
struct DataType< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_control_msgs/GripperCommandActionGoal";
  }

  static const char* value(const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"_actionlib_msgs/GoalID goal_id\n"
"GripperCommandGoal goal\n"
"\n"
"================================================================================\n"
"MSG: _std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: _actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: _control_msgs/GripperCommandGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"GripperCommand command\n"
"\n"
"================================================================================\n"
"MSG: _control_msgs/GripperCommand\n"
"float64 position\n"
"float64 max_effort\n"
;
  }

  static const char* value(const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GripperCommandActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_control_msgs::GripperCommandActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::_std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::_actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::_control_msgs::GripperCommandGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_MSGS_MESSAGE_GRIPPERCOMMANDACTIONGOAL_H
