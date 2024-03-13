// Generated by gencpp from file _tf2_msgs/LookupTransformActionGoal.msg
// DO NOT EDIT!


#ifndef _TF2_MSGS_MESSAGE_LOOKUPTRANSFORMACTIONGOAL_H
#define _TF2_MSGS_MESSAGE_LOOKUPTRANSFORMACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_std_msgs/Header.h>
#include <_actionlib_msgs/GoalID.h>
#include <_tf2_msgs/LookupTransformGoal.h>

namespace _tf2_msgs
{
template <class ContainerAllocator>
struct LookupTransformActionGoal_
{
  typedef LookupTransformActionGoal_<ContainerAllocator> Type;

  LookupTransformActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  LookupTransformActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::_std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::_actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::_tf2_msgs::LookupTransformGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef std::shared_ptr< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct LookupTransformActionGoal_

typedef ::_tf2_msgs::LookupTransformActionGoal_<std::allocator<void> > LookupTransformActionGoal;

typedef std::shared_ptr< ::_tf2_msgs::LookupTransformActionGoal > LookupTransformActionGoalPtr;
typedef std::shared_ptr< ::_tf2_msgs::LookupTransformActionGoal const> LookupTransformActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator1> & lhs, const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.goal_id == rhs.goal_id &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator1> & lhs, const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _tf2_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f2e7bcdb75c847978d0351a13e699da5";
  }

  static const char* value(const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf2e7bcdb75c84797ULL;
  static const uint64_t static_value2 = 0x8d0351a13e699da5ULL;
};

template<class ContainerAllocator>
struct DataType< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_tf2_msgs/LookupTransformActionGoal";
  }

  static const char* value(const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"_actionlib_msgs/GoalID goal_id\n"
"LookupTransformGoal goal\n"
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
"MSG: _tf2_msgs/LookupTransformGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#Simple API\n"
"string target_frame\n"
"string source_frame\n"
"time source_time\n"
"duration timeout\n"
"\n"
"#Advanced API\n"
"time target_time\n"
"string fixed_frame\n"
"\n"
"#Whether or not to use the advanced API\n"
"bool advanced\n"
"\n"
;
  }

  static const char* value(const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LookupTransformActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_tf2_msgs::LookupTransformActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::_std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::_actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::_tf2_msgs::LookupTransformGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TF2_MSGS_MESSAGE_LOOKUPTRANSFORMACTIONGOAL_H