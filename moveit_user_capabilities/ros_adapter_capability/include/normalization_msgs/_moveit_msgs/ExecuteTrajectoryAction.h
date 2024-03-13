// Generated by gencpp from file _moveit_msgs/ExecuteTrajectoryAction.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_EXECUTETRAJECTORYACTION_H
#define _MOVEIT_MSGS_MESSAGE_EXECUTETRAJECTORYACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <_moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <_moveit_msgs/ExecuteTrajectoryActionFeedback.h>

namespace _moveit_msgs
{
template <class ContainerAllocator>
struct ExecuteTrajectoryAction_
{
  typedef ExecuteTrajectoryAction_<ContainerAllocator> Type;

  ExecuteTrajectoryAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  ExecuteTrajectoryAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::_moveit_msgs::ExecuteTrajectoryActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::_moveit_msgs::ExecuteTrajectoryActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::_moveit_msgs::ExecuteTrajectoryActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef std::shared_ptr< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> const> ConstPtr;

}; // struct ExecuteTrajectoryAction_

typedef ::_moveit_msgs::ExecuteTrajectoryAction_<std::allocator<void> > ExecuteTrajectoryAction;

typedef std::shared_ptr< ::_moveit_msgs::ExecuteTrajectoryAction > ExecuteTrajectoryActionPtr;
typedef std::shared_ptr< ::_moveit_msgs::ExecuteTrajectoryAction const> ExecuteTrajectoryActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator1> & lhs, const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator2> & rhs)
{
  return lhs.action_goal == rhs.action_goal &&
    lhs.action_result == rhs.action_result &&
    lhs.action_feedback == rhs.action_feedback;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator1> & lhs, const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _moveit_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ea125ee188e659d7d34771c3ffb01ec5";
  }

  static const char* value(const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xea125ee188e659d7ULL;
  static const uint64_t static_value2 = 0xd34771c3ffb01ec5ULL;
};

template<class ContainerAllocator>
struct DataType< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_moveit_msgs/ExecuteTrajectoryAction";
  }

  static const char* value(const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"ExecuteTrajectoryActionGoal action_goal\n"
"ExecuteTrajectoryActionResult action_result\n"
"ExecuteTrajectoryActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/ExecuteTrajectoryActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"_actionlib_msgs/GoalID goal_id\n"
"ExecuteTrajectoryGoal goal\n"
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
"MSG: _moveit_msgs/ExecuteTrajectoryGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# The trajectory to execute\n"
"RobotTrajectory trajectory\n"
"\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/RobotTrajectory\n"
"_trajectory_msgs/JointTrajectory joint_trajectory\n"
"_trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory\n"
"\n"
"================================================================================\n"
"MSG: _trajectory_msgs/JointTrajectory\n"
"Header header\n"
"string[] joint_names\n"
"JointTrajectoryPoint[] points\n"
"\n"
"================================================================================\n"
"MSG: _trajectory_msgs/JointTrajectoryPoint\n"
"# Each trajectory point specifies either positions[, velocities[, accelerations]]\n"
"# or positions[, effort] for the trajectory to be executed.\n"
"# All specified values are in the same order as the joint names in JointTrajectory.msg\n"
"\n"
"float64[] positions\n"
"float64[] velocities\n"
"float64[] accelerations\n"
"float64[] effort\n"
"duration time_from_start\n"
"\n"
"================================================================================\n"
"MSG: _trajectory_msgs/MultiDOFJointTrajectory\n"
"# The header is used to specify the coordinate frame and the reference time for the trajectory durations\n"
"Header header\n"
"\n"
"# A representation of a multi-dof joint trajectory (each point is a transformation)\n"
"# Each point along the trajectory will include an array of positions/velocities/accelerations\n"
"# that has the same length as the array of joint names, and has the same order of joints as \n"
"# the joint names array.\n"
"\n"
"string[] joint_names\n"
"MultiDOFJointTrajectoryPoint[] points\n"
"\n"
"================================================================================\n"
"MSG: _trajectory_msgs/MultiDOFJointTrajectoryPoint\n"
"# Each multi-dof joint can specify a transform (up to 6 DOF)\n"
"_geometry_msgs/Transform[] transforms\n"
"\n"
"# There can be a velocity specified for the origin of the joint \n"
"_geometry_msgs/Twist[] velocities\n"
"\n"
"# There can be an acceleration specified for the origin of the joint \n"
"_geometry_msgs/Twist[] accelerations\n"
"\n"
"duration time_from_start\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# _geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: _geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/ExecuteTrajectoryActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"_actionlib_msgs/GoalStatus status\n"
"ExecuteTrajectoryResult result\n"
"\n"
"================================================================================\n"
"MSG: _actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/ExecuteTrajectoryResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Error code - encodes the overall reason for failure\n"
"MoveItErrorCodes error_code\n"
"\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/MoveItErrorCodes\n"
"int32 val\n"
"\n"
"# overall behavior\n"
"int32 SUCCESS=1\n"
"int32 FAILURE=99999\n"
"\n"
"int32 PLANNING_FAILED=-1\n"
"int32 INVALID_MOTION_PLAN=-2\n"
"int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3\n"
"int32 CONTROL_FAILED=-4\n"
"int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5\n"
"int32 TIMED_OUT=-6\n"
"int32 PREEMPTED=-7\n"
"\n"
"# planning & kinematics request errors\n"
"int32 START_STATE_IN_COLLISION=-10\n"
"int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11\n"
"\n"
"int32 GOAL_IN_COLLISION=-12\n"
"int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13\n"
"int32 GOAL_CONSTRAINTS_VIOLATED=-14\n"
"\n"
"int32 INVALID_GROUP_NAME=-15\n"
"int32 INVALID_GOAL_CONSTRAINTS=-16\n"
"int32 INVALID_ROBOT_STATE=-17\n"
"int32 INVALID_LINK_NAME=-18\n"
"int32 INVALID_OBJECT_NAME=-19\n"
"\n"
"# system errors\n"
"int32 FRAME_TRANSFORM_FAILURE=-21\n"
"int32 COLLISION_CHECKING_UNAVAILABLE=-22\n"
"int32 ROBOT_STATE_STALE=-23\n"
"int32 SENSOR_INFO_STALE=-24\n"
"int32 COMMUNICATION_FAILURE=-25\n"
"\n"
"# kinematics errors\n"
"int32 NO_IK_SOLUTION=-31\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/ExecuteTrajectoryActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"_actionlib_msgs/GoalStatus status\n"
"ExecuteTrajectoryFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: _moveit_msgs/ExecuteTrajectoryFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# The internal state that the move group action currently is in\n"
"string state\n"
"\n"
;
  }

  static const char* value(const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ExecuteTrajectoryAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_moveit_msgs::ExecuteTrajectoryAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::_moveit_msgs::ExecuteTrajectoryActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::_moveit_msgs::ExecuteTrajectoryActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::_moveit_msgs::ExecuteTrajectoryActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_EXECUTETRAJECTORYACTION_H
