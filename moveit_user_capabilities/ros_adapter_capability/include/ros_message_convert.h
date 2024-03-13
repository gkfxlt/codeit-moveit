//
// Created by fan on 2021/11/22.
//

#ifndef CODEIT_MOVEIT_ROS_MESSAGE_CONVERT_H
#define CODEIT_MOVEIT_ROS_MESSAGE_CONVERT_H

#include "std_msgs/String.h"
#include "_std_msgs/String.h"

template <typename T1, typename T2>
struct MessageCompare
{
  static bool isSameType()
  {
    const char* p1 = ros::message_traits::md5sum<T1>();
    const char* p2 = ros::message_traits::md5sum<T2>();
    return strcmp(p1, p2) == 0;
  }
};

template <typename MsgType, typename Norm_MsgType>
struct MessageConverter
{
  static bool convert(Norm_MsgType& /*msg_out*/, const MsgType& /*msg_in*/)
  {
    const char* p = ros::message_traits::datatype<MsgType>();
    std::cerr << "*** no define convert for " << p << std::endl;
    return false;
  }

  static bool convertBack(MsgType& /*msg_out*/, const Norm_MsgType& /*msg_in*/)
  {
    const char* p = ros::message_traits::datatype<MsgType>();
    std::cerr << "*** no define convertBack for " << p << std::endl;
    return false;
  }
};

template <typename T_out, typename T_in>
void serialConvert(T_out& msg_out, const T_in& msg_in)
{
  namespace ser = ros::serialization;
  // 序列化
  uint32_t serial_size = ros::serialization::serializationLength(msg_in);
  std::shared_ptr<uint8_t> buffer(new uint8_t[serial_size]);
  ser::OStream oStream(buffer.get(), serial_size);
  ser::serialize(oStream, msg_in);

  // 反序列化
  ser::IStream iStream(buffer.get(), serial_size);
  ser::deserialize(iStream, msg_out);
}

template <typename T_out, typename T_in>
void serialConvertVector(std::vector<T_out>& msg_out, const std::vector<T_in>& msg_in)
{
  size_t size = msg_in.size();
  msg_out.resize(size);
  for (size_t i = 0; i < size; ++i)
  {
    serialConvert<T_out, T_in>(msg_out[i], msg_in[i]);
  }
}

template <typename T_out, typename T_in>
void messageConvert(T_out& msg_out, const T_in& msg_in)
{
  MessageConverter<typename std::decay<decltype(msg_in)>::type, typename std::decay<decltype(msg_out)>::type>::convert(
      msg_out, msg_in);
}

template <typename T_out, typename T_in>
void messageConvertVector(std::vector<T_out>& msg_out, const std::vector<T_in>& msg_in)
{
  size_t size = msg_in.size();
  msg_out.resize(size);
  for (size_t i = 0; i < size; ++i)
  {
    messageConvert<T_out, T_in>(msg_out[i], msg_in[i]);
  }
}

#define MirrorMessageConverter(T, _T)                                                                                  \
  template <>                                                                                                          \
  struct MessageConverter<_T, T>                                                                                       \
  {                                                                                                                    \
    static bool convert(T& msg_out, const _T& msg_in)                                                                  \
    {                                                                                                                  \
      return MessageConverter<T, _T>::convertBack(msg_out, msg_in);                                                    \
    }                                                                                                                  \
                                                                                                                       \
    static bool convertBack(_T& msg_out, const T& msg_in)                                                              \
    {                                                                                                                  \
      return MessageConverter<T, _T>::convert(msg_out, msg_in);                                                        \
    }                                                                                                                  \
  };

#include "std_msgs/String.h"
#include "_std_msgs/String.h"

#include "tf2_msgs/TFMessage.h"
#include "_tf2_msgs/TFMessage.h"

#include "moveit_msgs/PlanningScene.h"
#include "_moveit_msgs/PlanningScene.h"

#include "sensor_msgs/JointState.h"
#include "_sensor_msgs/JointState.h"

#include "moveit_msgs/AttachedCollisionObject.h"
#include "_moveit_msgs/AttachedCollisionObject.h"

#include "moveit_msgs/CollisionObject.h"
#include "_moveit_msgs/CollisionObject.h"

#include "sensor_msgs/PointCloud2.h"
#include "_sensor_msgs/PointCloud2.h"

#include "moveit_msgs/DisplayTrajectory.h"
#include "_moveit_msgs/DisplayTrajectory.h"

#include "visualization_msgs/MarkerArray.h"
#include "_visualization_msgs/MarkerArray.h"

#include "moveit_msgs/MotionPlanRequest.h"
#include "_moveit_msgs/MotionPlanRequest.h"

// DataType: std_msgs/String  &&  _std_msgs/String
// MD5:      992ce8a1687cec8c8bd883ec73ca41d1  &&  992ce8a1687cec8c8bd883ec73ca41d1 -> 0

// DataType: tf2_msgs/TFMessage  &&  _tf2_msgs/TFMessage
// MD5:      94810edda583a504dfda3829e70d7eec  &&  94810edda583a504dfda3829e70d7eec -> 0

// DataType: moveit_msgs/PlanningScene  &&  _moveit_msgs/PlanningScene
// MD5:      89aac6d20db967ba716cba5a86b1b9d5  &&  acfc50bcfd6c7b978066bfa7c786002c -> -1

// DataType: sensor_msgs/JointState  &&  _sensor_msgs/JointState
// MD5:      3066dcd76a6cfaef579bd0f34173e9fd  &&  3066dcd76a6cfaef579bd0f34173e9fd -> 0

// DataType: moveit_msgs/AttachedCollisionObject  &&  _moveit_msgs/AttachedCollisionObject
// MD5:      3ceac60b21e85bbd6c5b0ab9d476b752  &&  30199ef516f64c8bc1edb1084ce4584e -> 1

// DataType: moveit_msgs/CollisionObject  &&  _moveit_msgs/CollisionObject
// MD5:      568a161b26dc46c54a5a07621ce82cf3  &&  dbba710596087da521c07564160dfccb -> -1

// DataType: sensor_msgs/PointCloud2  &&  _sensor_msgs/PointCloud2
// MD5:      1158d486dd51d683ce2f1be655c3c181  &&  1158d486dd51d683ce2f1be655c3c181 -> 0

// DataType: moveit_msgs/DisplayTrajectory  &&  _moveit_msgs/DisplayTrajectory
// MD5:      c3c039261ab9e8a11457dac56b6316c8  &&  41936b74e168ba754279ae683ce3f121 -> 1

// DataType: visualization_msgs/MarkerArray  &&  _visualization_msgs/MarkerArray
// MD5:      d155b9ce5188fbaf89745847fd5882d7  &&  d155b9ce5188fbaf89745847fd5882d7 -> 0

// DataType: moveit_msgs/MotionPlanRequest  &&  _moveit_msgs/MotionPlanRequest
// MD5:      c3bec13a525a6ae66e0fc57b768fdca6  &&  9544d5f3b9cf69a0e1e7f8c75d87f54b -> 1

template <typename T1, typename T2>
struct TopicReporter
{
  static void report()
  {
    const char* p_datatype_1 = ros::message_traits::DataType<T1>::value();
    const char* p_md5_1 = ros::message_traits::MD5Sum<T1>::value();

    const char* p_datatype_2 = ros::message_traits::DataType<T2>::value();
    const char* p_md5_2 = ros::message_traits::MD5Sum<T2>::value();

    std::cerr << "DataType: " << p_datatype_1 << "  &&  " << p_datatype_2 << std::endl;
    std::cerr << "MD5:      " << p_md5_1 << "  &&  " << p_md5_2 << " -> " << strcmp(p_md5_1, p_md5_2) << std::endl;
    std::cerr << std::endl;
  }
};

/// CollisionObject
template <>
struct MessageConverter<moveit_msgs::CollisionObject, _moveit_msgs::CollisionObject>
{
  static bool convert(_moveit_msgs::CollisionObject& msg_out, const moveit_msgs::CollisionObject& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::CollisionObject& msg_out, const _moveit_msgs::CollisionObject& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    // "2176decaecbce78abc3b96ef049fabed";
    serialConvert(msg_out.header, msg_in.header);

    // geometry_msgs::Pose
    // serialConvert(msg_out.pose, msg_in.pose); // new

    // std::string
    msg_out.id = msg_in.id;

    // object_recognition_msgs::ObjectType
    // "ac757ec5be1998b0167e7efcda79e3cf";
    serialConvert(msg_out.type, msg_in.type);

    // std::vector<shape_msgs::SolidPrimitive>
    // "d8f8cbc74c5ff283fca29569ccefb45d";
    serialConvertVector(msg_out.primitives, msg_in.primitives);

    // std::vector<geometry_msgs::Pose>
    // "e45d45a5a1ce597b249e23fb30fc871f";
    serialConvertVector(msg_out.primitive_poses, msg_in.primitive_poses);

    // std::vector<shape_msgs::Mesh>
    // "1ffdae9486cd3316a121c578b47a85cc";
    serialConvertVector(msg_out.meshes, msg_in.meshes);

    // std::vector<geometry_msgs::Pose>
    // "e45d45a5a1ce597b249e23fb30fc871f";
    serialConvertVector(msg_out.mesh_poses, msg_in.mesh_poses);

    // std::vector<shape_msgs::Plane>
    // "2c1b92ed8f31492f8e73f6a4a44ca796";
    serialConvertVector(msg_out.planes, msg_in.planes);

    // std::vector<geometry_msgs::Pose>
    // "e45d45a5a1ce597b249e23fb30fc871f";
    serialConvertVector(msg_out.plane_poses, msg_in.plane_poses);

    // msg_out.subframe_names = msg_in.subframe_names; // new
    // msg_out.subframe_poses = msg_in.subframe_poses; // new

    // int8_t
    msg_out.operation = msg_in.operation;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::CollisionObject, _moveit_msgs::CollisionObject);

/// AttachedCollisionObject
template <>
struct MessageConverter<moveit_msgs::AttachedCollisionObject, _moveit_msgs::AttachedCollisionObject>
{
  static bool convert(_moveit_msgs::AttachedCollisionObject& msg_out, const moveit_msgs::AttachedCollisionObject& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::AttachedCollisionObject& msg_out,
                          const _moveit_msgs::AttachedCollisionObject& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.link_name = msg_in.link_name;

    // moveit_msgs::CollisionObject
    // "dbba710596087da521c07564160dfccb"; "568a161b26dc46c54a5a07621ce82cf3";
    messageConvert(msg_out.object, msg_in.object);

    // std::vector<std::string>
    msg_out.touch_links = msg_in.touch_links;

    // trajectory_msgs::JointTrajectory
    // "65b4f94a94d1ed67169da35a02f33d3f";
    serialConvert(msg_out.detach_posture, msg_in.detach_posture);

    // double
    msg_out.weight = msg_in.weight;
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::AttachedCollisionObject, _moveit_msgs::AttachedCollisionObject);

/// RobotState
template <>
struct MessageConverter<moveit_msgs::RobotState, _moveit_msgs::RobotState>
{
  static bool convert(_moveit_msgs::RobotState& msg_out, const moveit_msgs::RobotState& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::RobotState& msg_out, const _moveit_msgs::RobotState& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // sensor_msgs::JointState
    serialConvert(msg_out.joint_state, msg_in.joint_state);

    // sensor_msgs::MultiDOFJointState
    serialConvert(msg_out.multi_dof_joint_state, msg_in.multi_dof_joint_state);

    // std::vector<moveit_msgs::AttachedCollisionObject>
    messageConvertVector(msg_out.attached_collision_objects, msg_in.attached_collision_objects);

    // uint8_t
    msg_out.is_diff = msg_in.is_diff;
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::RobotState, _moveit_msgs::RobotState);

/// PlanningSceneWorld
template <>
struct MessageConverter<moveit_msgs::PlanningSceneWorld, _moveit_msgs::PlanningSceneWorld>
{
  static bool convert(_moveit_msgs::PlanningSceneWorld& msg_out, const moveit_msgs::PlanningSceneWorld& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlanningSceneWorld& msg_out, const _moveit_msgs::PlanningSceneWorld& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::vector<moveit_msgs::CollisionObject>
    messageConvertVector(msg_out.collision_objects, msg_in.collision_objects);

    // octomap_msgs::OctomapWithPose
    serialConvert(msg_out.octomap, msg_in.octomap);
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlanningSceneWorld, _moveit_msgs::PlanningSceneWorld);

/// PlanningScene
template <>
struct MessageConverter<moveit_msgs::PlanningScene, _moveit_msgs::PlanningScene>
{
  static bool convert(_moveit_msgs::PlanningScene msg_out, const moveit_msgs::PlanningScene& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlanningScene& msg_out, const _moveit_msgs::PlanningScene& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.name = msg_in.name;

    // moveit_msgs::RobotState
    // "968156f4aa4cb4018f1f2293eebcea8f"; "217a2e8e5547f4162b13a37db9cb4da4";
    messageConvert(msg_out.robot_state, msg_in.robot_state);

    // std::string
    msg_out.robot_model_name = msg_in.robot_model_name;

    // std::vector<geometry_msgs::TransformStamped>
    // "b5764a33bfeb3588febc2682852579b0";
    serialConvertVector(msg_out.fixed_frame_transforms, msg_in.fixed_frame_transforms);

    // moveit_msgs::AllowedCollisionMatrix
    // "aedce13587eef0d79165a075659c1879";
    serialConvert(msg_out.allowed_collision_matrix, msg_in.allowed_collision_matrix);

    // std::vector<moveit_msgs::LinkPadding>
    // "b3ea75670df55c696fedee97774d5947";
    serialConvertVector(msg_out.link_padding, msg_in.link_padding);

    // std::vector<moveit_msgs::LinkScale>
    // "19faf226446bfb2f645a4da6f2a56166";
    serialConvertVector(msg_out.link_scale, msg_in.link_scale);

    // std::vector<moveit_msgs::ObjectColor>
    // "ec3bd6f103430e64b2ae706a67d8488e";
    serialConvertVector(msg_out.object_colors, msg_in.object_colors);

    // moveit_msgs::PlanningSceneWorld
    // "79457311445f53d410ab4e3781de8447"; "373d88390d1db385335639f687723ee6";
    messageConvert(msg_out.world, msg_in.world);

    // uint8_t
    msg_out.is_diff = msg_in.is_diff;
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlanningScene, _moveit_msgs::PlanningScene);

/// DisplayTrajectory
template <>
struct MessageConverter<moveit_msgs::DisplayTrajectory, _moveit_msgs::DisplayTrajectory>
{
  static bool convert(_moveit_msgs::DisplayTrajectory& msg_out, const moveit_msgs::DisplayTrajectory& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::DisplayTrajectory& msg_out, const _moveit_msgs::DisplayTrajectory& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.model_id = msg_in.model_id;

    // std::vector<moveit_msgs::RobotTrajectory>
    // "dfa9556423d709a3729bcef664bddf67";
    serialConvertVector(msg_out.trajectory, msg_in.trajectory);

    // moveit_msgs::RobotState
    messageConvert(msg_out.trajectory_start, msg_in.trajectory_start);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::DisplayTrajectory, _moveit_msgs::DisplayTrajectory);

/// OrientationConstraint
template <>
struct MessageConverter<moveit_msgs::OrientationConstraint, _moveit_msgs::OrientationConstraint>
{
  static bool convert(_moveit_msgs::OrientationConstraint& msg_out, const moveit_msgs::OrientationConstraint& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::OrientationConstraint& msg_out, const _moveit_msgs::OrientationConstraint& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // geometry_msgs::Quaternion
    serialConvert(msg_out.orientation, msg_in.orientation);

    // std::string
    msg_out.link_name = msg_in.link_name;

    // double
    msg_out.absolute_x_axis_tolerance = msg_in.absolute_x_axis_tolerance;

    // double
    msg_out.absolute_y_axis_tolerance = msg_in.absolute_y_axis_tolerance;

    // double
    msg_out.absolute_z_axis_tolerance = msg_in.absolute_z_axis_tolerance;

    // msg_out.parameterization = msg_in.parameterization; // new

    // double
    msg_out.weight = msg_in.weight;
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::OrientationConstraint, _moveit_msgs::OrientationConstraint);

/// Constraints
template <>
struct MessageConverter<moveit_msgs::Constraints, _moveit_msgs::Constraints>
{
  static bool convert(_moveit_msgs::Constraints& msg_out, const moveit_msgs::Constraints& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::Constraints& msg_out, const _moveit_msgs::Constraints& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.name = msg_in.name;

    // std::vector<moveit_msgs::JointConstraint>
    // "c02a15146bec0ce13564807805b008f0";
    serialConvertVector(msg_out.joint_constraints, msg_in.joint_constraints);

    // std::vector<moveit_msgs::PositionConstraint>
    // "c83edf208d87d3aa3169f47775a58e6a";
    serialConvertVector(msg_out.position_constraints, msg_in.position_constraints);

    // std::vector<moveit_msgs::OrientationConstraint>
    // "183479d9281e5b4f23dc584f711d8a64"; "ab5cefb9bc4c0089620f5eb4caf4e59a";
    messageConvertVector(msg_out.orientation_constraints, msg_in.orientation_constraints);

    // std::vector<moveit_msgs::VisibilityConstraint>
    // "62cda903bfe31ff2e5fcdc3810d577ad";
    serialConvertVector(msg_out.visibility_constraints, msg_in.visibility_constraints);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::Constraints, _moveit_msgs::Constraints);

/// MotionPlanRequest
template <>
struct MessageConverter<moveit_msgs::MotionPlanRequest, _moveit_msgs::MotionPlanRequest>
{
  static bool convert(_moveit_msgs::MotionPlanRequest& msg_out, const moveit_msgs::MotionPlanRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MotionPlanRequest& msg_out, const _moveit_msgs::MotionPlanRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::WorkspaceParameters
    // "d639a834e7b1f927e9f1d6c30e920016";
    serialConvert(msg_out.workspace_parameters, msg_in.workspace_parameters);

    // moveit_msgs::RobotState
    messageConvert(msg_out.start_state, msg_in.start_state);

    // std::vector<moveit_msgs::Constraints>
    // "cfd22a10c51e0dc2b28d98772d2b55d5"; "8d5ce8d34ef26c65fb5d43c9e99bf6e0";
    messageConvertVector(msg_out.goal_constraints, msg_in.goal_constraints);

    // moveit_msgs::Constraints
    // "cfd22a10c51e0dc2b28d98772d2b55d5"; "8d5ce8d34ef26c65fb5d43c9e99bf6e0";
    messageConvert(msg_out.path_constraints, msg_in.path_constraints);

    // moveit_msgs::TrajectoryConstraints
    // "461e1a732dfebb01e7d6c75d51a51eac"; "461e1a732dfebb01e7d6c75d51a51eac";
    serialConvert(msg_out.trajectory_constraints, msg_in.trajectory_constraints);

    // std::string
    msg_out.planner_id = msg_in.planner_id;

    // std::string
    msg_out.group_name = msg_in.group_name;

    // int32_t
    msg_out.num_planning_attempts = msg_in.num_planning_attempts;

    // double
    msg_out.allowed_planning_time = msg_in.allowed_planning_time;

    // double
    msg_out.max_velocity_scaling_factor = msg_in.max_velocity_scaling_factor;

    // double
    msg_out.max_acceleration_scaling_factor = msg_in.max_acceleration_scaling_factor;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::MotionPlanRequest, _moveit_msgs::MotionPlanRequest);

///
/// service
///
#include "std_srvs/Empty.h"
#include "_std_srvs/Empty.h"

#include "moveit_msgs/ApplyPlanningScene.h"
#include "_moveit_msgs/ApplyPlanningScene.h"

#include "moveit_msgs/GetCartesianPath.h"
#include "_moveit_msgs/GetCartesianPath.h"

#include "moveit_msgs/GetPositionFK.h"
#include "_moveit_msgs/GetPositionFK.h"

#include "moveit_msgs/GetPositionIK.h"
#include "_moveit_msgs/GetPositionIK.h"

#include "moveit_msgs/GetMotionPlan.h"
#include "_moveit_msgs/GetMotionPlan.h"

#include "moveit_msgs/QueryPlannerInterfaces.h"
#include "_moveit_msgs/QueryPlannerInterfaces.h"

#include "moveit_msgs/GetPlannerParams.h"
#include "_moveit_msgs/GetPlannerParams.h"

#include "moveit_msgs/SetPlannerParams.h"
#include "_moveit_msgs/SetPlannerParams.h"

#include "moveit_msgs/GetStateValidity.h"
#include "_moveit_msgs/GetStateValidity.h"

#include "moveit_msgs/GetPlanningScene.h"
#include "_moveit_msgs/GetPlanningScene.h"

#include "moveit_msgs/SaveMap.h"
#include "_moveit_msgs/SaveMap.h"

#include "moveit_msgs/LoadMap.h"
#include "_moveit_msgs/LoadMap.h"

// DataType:     std_srvs/Empty  &&  _std_srvs/Empty
// MD5:          d41d8cd98f00b204e9800998ecf8427e  &&  d41d8cd98f00b204e9800998ecf8427e -> 0
// MD5_Request:  d41d8cd98f00b204e9800998ecf8427e  &&  d41d8cd98f00b204e9800998ecf8427e -> 0
// MD5_Response: d41d8cd98f00b204e9800998ecf8427e  &&  d41d8cd98f00b204e9800998ecf8427e -> 0

// DataType:     moveit_msgs/ApplyPlanningScene  &&  _moveit_msgs/ApplyPlanningScene
// MD5:          60a182de67a2bc514fbbc64e682bcaaa  &&  d4d1599e0b6f71a6a8a10cb34780619c -> -1
// MD5_Request:  7bedc4871b1d0af6ec8b8996db347e7f  &&  532b54e7c502b73178625025da63b084 -> 1
// MD5_Response: 358e233cde0c8a8bcfea4ce193f8fc15  &&  358e233cde0c8a8bcfea4ce193f8fc15 -> 0

// DataType:     moveit_msgs/GetCartesianPath  &&  _moveit_msgs/GetCartesianPath
// MD5:          5c9a54219f0d91a804e7670bc0e118f1  &&  2f81861c2a157da4ea888c0db053e2a1 -> 1
// MD5_Request:  b37c16ad7ed838d811a270a8054276b6  &&  721648200a81dbec003e9f1273aa2c80 -> 1
// MD5_Response: 45414110461a45eb0e273e013924ce59  &&  15ee95715c86802ada727b6361c6f4d2 -> 1

// DataType:     moveit_msgs/GetPositionFK  &&  _moveit_msgs/GetPositionFK
// MD5:          03d4858215085d70e74807025d68dc4c  &&  7e5228d4c9e3511cf5071ecf43c502b8 -> -1
// MD5_Request:  1d1ed72044ed56f6246c31b522781797  &&  0cc2c8039d5792659dd3a5a92f64c5bb -> 1
// MD5_Response: 297215cf4fdfe0008356995ae621dae6  &&  5d8c20528014017b172ce1f1d15ac982 -> -1

// DataType:     moveit_msgs/GetPositionIK  &&  _moveit_msgs/GetPositionIK
// MD5:          0661ea3324398c69f5a971d0ec55657e  &&  185001c352b2e96cb475d82048e9df9a -> -1
// MD5_Request:  a67dc7e99d15c1dca32a77c22bc2d93b  &&  8388b54598336654bca82763f918a740 -> 1
// MD5_Response: ad50fe5fa0ddb482909be313121ea148  &&  3943ba9ed5631a4f63888551da37cd16 -> 1

// DataType:     moveit_msgs/GetMotionPlan  &&  _moveit_msgs/GetMotionPlan
// MD5:          657e571ceabcb225c850c02c2249a1e1  &&  29c3c85c8a4c241f7588a9b4da235097 -> 1
// MD5_Request:  9dcb82c5daeb2ff8a7ab1a98b642871d  &&  5459d2db10f706c7e69e5485e0b58672 -> 1
// MD5_Response: 37fe7e8f0d4dfa55ccfa53d63c86ae15  &&  b87f68f66cd157fdfe416b1070b9ecb6 -> -1

// DataType:     moveit_msgs/QueryPlannerInterfaces  &&  _moveit_msgs/QueryPlannerInterfaces
// MD5:          acd3317a4c5631f33127fb428209db05  &&  5876081117e7cad85cc165e937798753 -> 1
// MD5_Request:  d41d8cd98f00b204e9800998ecf8427e  &&  d41d8cd98f00b204e9800998ecf8427e -> 0
// MD5_Response: acd3317a4c5631f33127fb428209db05  &&  5876081117e7cad85cc165e937798753 -> 1

// DataType:     moveit_msgs/GetPlannerParams  &&  _moveit_msgs/GetPlannerParams
// MD5:          b3ec1aca2b1471e3eea051c548c69810  &&  1b56b530c1107c60f0c9173e631d7bf5 -> 1
// MD5_Request:  f548a13784550d510d791867af53ef40  &&  f5065dceae6a10319c47163ab1012104 -> 1
// MD5_Response: 462b1bd59976ece800f6a48f2b0bf1a2  &&  462b1bd59976ece800f6a48f2b0bf1a2 -> 0

// DataType:     moveit_msgs/SetPlannerParams  &&  _moveit_msgs/SetPlannerParams
// MD5:          86762d89189c5f52cda7680fdbceb1db  &&  14bebee5d4d53a2df94b7f146d3eb2ff -> 1
// MD5_Request:  86762d89189c5f52cda7680fdbceb1db  &&  14bebee5d4d53a2df94b7f146d3eb2ff -> 1
// MD5_Response: d41d8cd98f00b204e9800998ecf8427e  &&  d41d8cd98f00b204e9800998ecf8427e -> 0

// DataType:     moveit_msgs/GetStateValidity  &&  _moveit_msgs/GetStateValidity
// MD5:          0c7c937b6a056e7ae5fded13d8e9b242  &&  06ea62db671e4dbf878eaca241db51ad -> 1
// MD5_Request:  b569c609cafad20ba7d0e46e70e7cab1  &&  a82b6cce5fd6d63051640aef60a848b1 -> 1
// MD5_Response: e326fb22b7448f29c0e726d9270d9929  &&  e326fb22b7448f29c0e726d9270d9929 -> 0

// DataType:     moveit_msgs/GetPlanningScene  &&  _moveit_msgs/GetPlanningScene
// MD5:          2745cf315b4eb5fb00e5befa8714d64d  &&  65cf69d3e7a0342e16374024d4eeef65 -> -1
// MD5_Request:  d81da6c0e5e015646a4efe344f33d7dc  &&  d81da6c0e5e015646a4efe344f33d7dc -> 0
// MD5_Response: 7bedc4871b1d0af6ec8b8996db347e7f  &&  532b54e7c502b73178625025da63b084 -> 1

// DataType:     moveit_msgs/SaveMap  &&  _moveit_msgs/SaveMap
// MD5:          93a4bc4c60dc17e2a69e3fcaaa25d69d  &&  93a4bc4c60dc17e2a69e3fcaaa25d69d -> 0
// MD5_Request:  030824f52a0628ead956fb9d67e66ae9  &&  030824f52a0628ead956fb9d67e66ae9 -> 0
// MD5_Response: 358e233cde0c8a8bcfea4ce193f8fc15  &&  358e233cde0c8a8bcfea4ce193f8fc15 -> 0

// DataType:     moveit_msgs/LoadMap  &&  _moveit_msgs/LoadMap
// MD5:          93a4bc4c60dc17e2a69e3fcaaa25d69d  &&  93a4bc4c60dc17e2a69e3fcaaa25d69d -> 0
// MD5_Request:  030824f52a0628ead956fb9d67e66ae9  &&  030824f52a0628ead956fb9d67e66ae9 -> 0
// MD5_Response: 358e233cde0c8a8bcfea4ce193f8fc15  &&  358e233cde0c8a8bcfea4ce193f8fc15 -> 0

template <typename T1, typename T2>
struct ServiceReporter
{
  static void report()
  {
    const char* p_datatype_1 = ros::service_traits::DataType<T1>::value();
    const char* p_md5_1 = ros::service_traits::MD5Sum<T1>::value();
    const char* p_md5_req_1 = ros::message_traits::MD5Sum<typename T1::Request>::value();
    const char* p_md5_res_1 = ros::message_traits::MD5Sum<typename T1::Response>::value();

    const char* p_datatype_2 = ros::service_traits::DataType<T2>::value();
    const char* p_md5_2 = ros::service_traits::MD5Sum<T2>::value();
    const char* p_md5_req_2 = ros::message_traits::MD5Sum<typename T2::Request>::value();
    const char* p_md5_res_2 = ros::message_traits::MD5Sum<typename T2::Response>::value();

    std::cerr << "DataType:     " << p_datatype_1 << "  &&  " << p_datatype_2 << std::endl;
    std::cerr << "MD5:          " << p_md5_1 << "  &&  " << p_md5_2 << " -> " << strcmp(p_md5_1, p_md5_2) << std::endl;
    std::cerr << "MD5_Request:  " << p_md5_req_1 << "  &&  " << p_md5_req_2 << " -> "
              << strcmp(p_md5_req_1, p_md5_req_2) << std::endl;
    std::cerr << "MD5_Response: " << p_md5_res_1 << "  &&  " << p_md5_res_2 << " -> "
              << strcmp(p_md5_res_1, p_md5_res_2) << std::endl;
    ;
    std::cerr << std::endl;
  }
};

/// MoveItErrorCodes
template <>
struct MessageConverter<moveit_msgs::MoveItErrorCodes, _moveit_msgs::MoveItErrorCodes>
{
  static bool convert(_moveit_msgs::MoveItErrorCodes& msg_out, const moveit_msgs::MoveItErrorCodes& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MoveItErrorCodes& msg_out, const _moveit_msgs::MoveItErrorCodes& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // int32_t
    msg_out.val = msg_in.val;
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::MoveItErrorCodes, _moveit_msgs::MoveItErrorCodes);

/// ApplyPlanningSceneRequest
template <>
struct MessageConverter<moveit_msgs::ApplyPlanningSceneRequest, _moveit_msgs::ApplyPlanningSceneRequest>
{
  static bool convert(_moveit_msgs::ApplyPlanningSceneRequest& msg_out,
                      const moveit_msgs::ApplyPlanningSceneRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::ApplyPlanningSceneRequest& msg_out,
                          const _moveit_msgs::ApplyPlanningSceneRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::PlanningScene
    messageConvert(msg_out.scene, msg_in.scene);
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::ApplyPlanningSceneRequest, _moveit_msgs::ApplyPlanningSceneRequest);

/// GetCartesianPathRequest
template <>
struct MessageConverter<moveit_msgs::GetCartesianPathRequest, _moveit_msgs::GetCartesianPathRequest>
{
  static bool convert(_moveit_msgs::GetCartesianPathRequest& msg_out, const moveit_msgs::GetCartesianPathRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetCartesianPathRequest& msg_out,
                          const _moveit_msgs::GetCartesianPathRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // moveit_msgs::RobotState
    messageConvert(msg_out.start_state, msg_in.start_state);

    // std::string
    msg_out.group_name = msg_in.group_name;

    // std::string
    msg_out.link_name = msg_in.link_name;

    // std::vector<geometry_msgs::Pose>
    serialConvertVector(msg_out.waypoints, msg_in.waypoints);

    // double
    msg_out.max_step = msg_in.max_step;

    // double
    msg_out.jump_threshold = msg_in.jump_threshold;

    // double
    // msg_out.prismatic_jump_threshold = msg_in.prismatic_jump_threshold; // new
    // msg_out.revolute_jump_threshold = msg_in.revolute_jump_threshold;  // new

    // uint8_t
    msg_out.avoid_collisions = msg_in.avoid_collisions;

    // moveit_msgs::Constraints
    messageConvert(msg_out.path_constraints, msg_in.path_constraints);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetCartesianPathRequest, _moveit_msgs::GetCartesianPathRequest);

/// GetCartesianPathResponse
template <>
struct MessageConverter<moveit_msgs::GetCartesianPathResponse, _moveit_msgs::GetCartesianPathResponse>
{
  static bool convert(_moveit_msgs::GetCartesianPathResponse& msg_out,
                      const moveit_msgs::GetCartesianPathResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetCartesianPathResponse& msg_out,
                          const _moveit_msgs::GetCartesianPathResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::RobotState
    messageConvert(msg_out.start_state, msg_in.start_state);

    // moveit_msgs::RobotTrajectory
    serialConvert(msg_out.start_state, msg_in.start_state);

    // double
    msg_out.fraction = msg_in.fraction;

    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetCartesianPathResponse, _moveit_msgs::GetCartesianPathResponse);

/// GetPositionFKRequest
template <>
struct MessageConverter<moveit_msgs::GetPositionFKRequest, _moveit_msgs::GetPositionFKRequest>
{
  static bool convert(_moveit_msgs::GetPositionFKRequest& msg_out, const moveit_msgs::GetPositionFKRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetPositionFKRequest& msg_out, const _moveit_msgs::GetPositionFKRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // std::vector<std::string>
    msg_out.fk_link_names = msg_in.fk_link_names;

    // moveit_msgs::RobotState
    messageConvert(msg_out.robot_state, msg_in.robot_state);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetPositionFKRequest, _moveit_msgs::GetPositionFKRequest);

/// GetPositionFKResponse
template <>
struct MessageConverter<moveit_msgs::GetPositionFKResponse, _moveit_msgs::GetPositionFKResponse>
{
  static bool convert(_moveit_msgs::GetPositionFKResponse& msg_out, const moveit_msgs::GetPositionFKResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetPositionFKResponse& msg_out, const _moveit_msgs::GetPositionFKResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::vector<geometry_msgs::PoseStamped>
    serialConvertVector(msg_out.pose_stamped, msg_in.pose_stamped);

    // std::vector<std::string>
    msg_out.fk_link_names = msg_in.fk_link_names;

    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetPositionFKResponse, _moveit_msgs::GetPositionFKResponse);

/// PositionIKRequest
template <>
struct MessageConverter<moveit_msgs::PositionIKRequest, _moveit_msgs::PositionIKRequest>
{
  static bool convert(_moveit_msgs::PositionIKRequest& msg_out, const moveit_msgs::PositionIKRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PositionIKRequest& msg_out, const _moveit_msgs::PositionIKRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.group_name = msg_in.group_name;

    // moveit_msgs::RobotState
    messageConvert(msg_out.robot_state, msg_in.robot_state);

    // moveit_msgs::Constraints
    messageConvert(msg_out.constraints, msg_in.constraints);

    // uint8_t
    msg_out.avoid_collisions = msg_in.avoid_collisions;

    // std::string
    msg_out.ik_link_name = msg_in.ik_link_name;

    // geometry_msgs::PoseStamped
    serialConvert(msg_out.pose_stamped, msg_in.pose_stamped);

    // std::vector<std::string>
    msg_out.ik_link_names = msg_in.ik_link_names;

    // std::vector<geometry_msgs::PoseStamped>
    serialConvertVector(msg_out.pose_stamped_vector, msg_in.pose_stamped_vector);

    // ros::Duration
    msg_out.timeout = msg_in.timeout;

    // int32_t
    // msg_out.attempts = msg_in.attempts; // old

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PositionIKRequest, _moveit_msgs::PositionIKRequest);

/// GetPositionIKRequest
template <>
struct MessageConverter<moveit_msgs::GetPositionIKRequest, _moveit_msgs::GetPositionIKRequest>
{
  static bool convert(_moveit_msgs::GetPositionIKRequest& msg_out, const moveit_msgs::GetPositionIKRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetPositionIKRequest& msg_out, const _moveit_msgs::GetPositionIKRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::PositionIKRequest
    // "cb7c3615ee4d29d023dfdc5950af0504"; "9936dc239c90af180ec94a51596c96f2";
    messageConvert(msg_out.ik_request, msg_in.ik_request);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetPositionIKRequest, _moveit_msgs::GetPositionIKRequest);

/// GetPositionIKResponse
template <>
struct MessageConverter<moveit_msgs::GetPositionIKResponse, _moveit_msgs::GetPositionIKResponse>
{
  static bool convert(_moveit_msgs::GetPositionIKResponse& msg_out, const moveit_msgs::GetPositionIKResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetPositionIKResponse& msg_out, const _moveit_msgs::GetPositionIKResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::RobotState
    messageConvert(msg_out.solution, msg_in.solution);

    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetPositionIKResponse, _moveit_msgs::GetPositionIKResponse);

/// GetMotionPlanRequest
template <>
struct MessageConverter<moveit_msgs::GetMotionPlanRequest, _moveit_msgs::GetMotionPlanRequest>
{
  static bool convert(_moveit_msgs::GetMotionPlanRequest& msg_out, const moveit_msgs::GetMotionPlanRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetMotionPlanRequest& msg_out, const _moveit_msgs::GetMotionPlanRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MotionPlanRequest
    messageConvert(msg_out.motion_plan_request, msg_in.motion_plan_request);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetMotionPlanRequest, _moveit_msgs::GetMotionPlanRequest);

/// MotionPlanResponse
template <>
struct MessageConverter<moveit_msgs::MotionPlanResponse, _moveit_msgs::MotionPlanResponse>
{
  static bool convert(_moveit_msgs::MotionPlanResponse& msg_out, const moveit_msgs::MotionPlanResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MotionPlanResponse& msg_out, const _moveit_msgs::MotionPlanResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::RobotState
    messageConvert(msg_out.trajectory_start, msg_in.trajectory_start);

    // std::string
    msg_out.group_name = msg_in.group_name;

    // moveit_msgs::RobotTrajectory
    serialConvert(msg_out.trajectory, msg_in.trajectory);

    // double
    msg_out.planning_time = msg_in.planning_time;

    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::MotionPlanResponse, _moveit_msgs::MotionPlanResponse);

/// GetMotionPlanResponse
template <>
struct MessageConverter<moveit_msgs::GetMotionPlanResponse, _moveit_msgs::GetMotionPlanResponse>
{
  static bool convert(_moveit_msgs::GetMotionPlanResponse& msg_out, const moveit_msgs::GetMotionPlanResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetMotionPlanResponse& msg_out, const _moveit_msgs::GetMotionPlanResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MotionPlanResponse
    messageConvert(msg_out.motion_plan_response, msg_in.motion_plan_response);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetMotionPlanResponse, _moveit_msgs::GetMotionPlanResponse);

/// PlannerInterfaceDescription
template <>
struct MessageConverter<moveit_msgs::PlannerInterfaceDescription, _moveit_msgs::PlannerInterfaceDescription>
{
  static bool convert(_moveit_msgs::PlannerInterfaceDescription& msg_out,
                      const moveit_msgs::PlannerInterfaceDescription& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlannerInterfaceDescription& msg_out,
                          const _moveit_msgs::PlannerInterfaceDescription& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.name = msg_in.name;

    // std::vector<std::string>
    msg_out.planner_ids = msg_in.planner_ids;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlannerInterfaceDescription, _moveit_msgs::PlannerInterfaceDescription);

/// QueryPlannerInterfacesResponse
template <>
struct MessageConverter<moveit_msgs::QueryPlannerInterfacesResponse, _moveit_msgs::QueryPlannerInterfacesResponse>
{
  static bool convert(_moveit_msgs::QueryPlannerInterfacesResponse& msg_out,
                      const moveit_msgs::QueryPlannerInterfacesResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::QueryPlannerInterfacesResponse& msg_out,
                          const _moveit_msgs::QueryPlannerInterfacesResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::PlannerInterfaceDescription
    messageConvertVector(msg_out.planner_interfaces, msg_in.planner_interfaces);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::QueryPlannerInterfacesResponse, _moveit_msgs::QueryPlannerInterfacesResponse);

/// GetPlannerParamsRequest
template <>
struct MessageConverter<moveit_msgs::GetPlannerParamsRequest, _moveit_msgs::GetPlannerParamsRequest>
{
  static bool convert(_moveit_msgs::GetPlannerParamsRequest& msg_out, const moveit_msgs::GetPlannerParamsRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetPlannerParamsRequest& msg_out,
                          const _moveit_msgs::GetPlannerParamsRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    // msg_out.pipeline_id = msg_in.pipeline_id; // new

    // std::string
    msg_out.planner_config = msg_in.planner_config;

    // std::string
    msg_out.group = msg_in.group;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetPlannerParamsRequest, _moveit_msgs::GetPlannerParamsRequest);

/// SetPlannerParamsRequest
template <>
struct MessageConverter<moveit_msgs::SetPlannerParamsRequest, _moveit_msgs::SetPlannerParamsRequest>
{
  static bool convert(_moveit_msgs::SetPlannerParamsRequest& msg_out, const moveit_msgs::SetPlannerParamsRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::SetPlannerParamsRequest& msg_out,
                          const _moveit_msgs::SetPlannerParamsRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    // msg_out.pipeline_id = msg_in.pipeline_id; // new

    // std::string
    msg_out.planner_config = msg_in.planner_config;

    // std::string
    msg_out.group = msg_in.group;

    // moveit_msgs::PlannerParams
    // "cebdf4927996b9026bcf59a160d64145";
    serialConvert(msg_out.params, msg_in.params);

    // uint8_t
    msg_out.replace = msg_in.replace;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::SetPlannerParamsRequest, _moveit_msgs::SetPlannerParamsRequest);

/// GetStateValidityRequest
template <>
struct MessageConverter<moveit_msgs::GetStateValidityRequest, _moveit_msgs::GetStateValidityRequest>
{
  static bool convert(_moveit_msgs::GetStateValidityRequest& msg_out, const moveit_msgs::GetStateValidityRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetStateValidityRequest& msg_out,
                          const _moveit_msgs::GetStateValidityRequest& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::RobotState
    messageConvert(msg_out.robot_state, msg_in.robot_state);

    // std::string
    msg_out.group_name = msg_in.group_name;

    // moveit_msgs::Constraints
    messageConvert(msg_out.constraints, msg_in.constraints);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetStateValidityRequest, _moveit_msgs::GetStateValidityRequest);

/// GetPlanningSceneResponse
template <>
struct MessageConverter<moveit_msgs::GetPlanningSceneResponse, _moveit_msgs::GetPlanningSceneResponse>
{
  static bool convert(_moveit_msgs::GetPlanningSceneResponse& msg_out,
                      const moveit_msgs::GetPlanningSceneResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::GetPlanningSceneResponse& msg_out,
                          const _moveit_msgs::GetPlanningSceneResponse& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::PlanningScene
    messageConvert(msg_out.scene, msg_in.scene);
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::GetPlanningSceneResponse, _moveit_msgs::GetPlanningSceneResponse);

///
/// action
///
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "_moveit_msgs/ExecuteTrajectoryAction.h"

#include "moveit_msgs/MoveGroupAction.h"
#include "_moveit_msgs/MoveGroupAction.h"

#include "moveit_msgs/PickupAction.h"
#include "_moveit_msgs/PickupAction.h"

#include "moveit_msgs/PlaceAction.h"
#include "_moveit_msgs/PlaceAction.h"

// DataType:     moveit_msgs/ExecuteTrajectoryAction  &&  _moveit_msgs/ExecuteTrajectoryAction
// MD5:          24e882ecd7f84f3e3299d504b8e3deae  &&  ea125ee188e659d7d34771c3ffb01ec5 -> -1
// MD5_Goal:     36f350977c67bc94e8cd408452bad0f0  &&  36f350977c67bc94e8cd408452bad0f0 -> 0
// MD5_Result:   8aaeab5c1cdb613e6a2913ebcc104c0d  &&  3aa24b8b6dd690aa38a02c1bea6f419f -> 1
// MD5_Feedback: 12232ef97486c7962f264c105aae2958  &&  12232ef97486c7962f264c105aae2958 -> 0

// DataType:     moveit_msgs/MoveGroupAction  &&  _moveit_msgs/MoveGroupAction
// MD5:          be9c7984423000efe94194063f359cfc  &&  6337a63f836aa32e02cee901b8c83a22 -> 1
// MD5_Goal:     df11ac1a643d87b6e6a6fe5af1823709  &&  152e336e337dce7cbe639f1bd9c65def -> 1
// MD5_Result:   6ee8682a508d60603228accdc4a2b30b  &&  e8f0259d9c66fae7820b6818cc47d807 -> -1
// MD5_Feedback: 12232ef97486c7962f264c105aae2958  &&  12232ef97486c7962f264c105aae2958 -> 0

// DataType:     moveit_msgs/PickupAction  &&  _moveit_msgs/PickupAction
// MD5:          f5aa574f57e5d9cf7d466d5913039489  &&  dffb706c5fad660e690608a93e87541c -> 1
// MD5_Goal:     9e12196da542c9a26bbc43e9655a1906  &&  83bb430d120a16fadf2ea3aad239f1d1 -> 1
// MD5_Result:   9a2192bdd4f78c9d7c479e4a43f2768f  &&  ab5e95d613f663fb48a9a67ea74b07c3 -> -1
// MD5_Feedback: 12232ef97486c7962f264c105aae2958  &&  12232ef97486c7962f264c105aae2958 -> 0

// DataType:     moveit_msgs/PlaceAction  &&  _moveit_msgs/PlaceAction
// MD5:          28cb4b6b7c2a211726c2c78386a9da69  &&  3504cede75e64e7bb6d8913483af0455 -> -1
// MD5_Goal:     facadaee390f685ed5e693ac12f5aa3d  &&  3f3419319ea8d2af4e18a600962d0cf5 -> 1
// MD5_Result:   d7a2ac2299b16bfd9120d347472b7cdc  &&  16248d37d9c712b8cfac7af69094a3e3 -> 1
// MD5_Feedback: 12232ef97486c7962f264c105aae2958  &&  12232ef97486c7962f264c105aae2958 -> 0

template <typename T1, typename T2>
struct ActionReporter
{
  static void report()
  {
    const char* p_datatype_1 = ros::message_traits::DataType<T1>::value();
    const char* p_md5_1 = ros::message_traits::MD5Sum<T1>::value();
    const char* p_md5_goal_1 = ros::message_traits::MD5Sum<typename T1::_action_goal_type>::value();
    const char* p_md5_result_1 = ros::message_traits::MD5Sum<typename T1::_action_result_type>::value();
    const char* p_md5_feedback_1 = ros::message_traits::MD5Sum<typename T1::_action_feedback_type>::value();

    const char* p_datatype_2 = ros::message_traits::DataType<T2>::value();
    const char* p_md5_2 = ros::message_traits::MD5Sum<T2>::value();
    const char* p_md5_goal_2 = ros::message_traits::MD5Sum<typename T2::_action_goal_type>::value();
    const char* p_md5_result_2 = ros::message_traits::MD5Sum<typename T2::_action_result_type>::value();
    const char* p_md5_feedback_2 = ros::message_traits::MD5Sum<typename T2::_action_feedback_type>::value();

    std::cerr << "DataType:     " << p_datatype_1 << "  &&  " << p_datatype_2 << std::endl;
    std::cerr << "MD5:          " << p_md5_1 << "  &&  " << p_md5_2 << " -> " << strcmp(p_md5_1, p_md5_2) << std::endl;
    std::cerr << "MD5_Goal:     " << p_md5_goal_1 << "  &&  " << p_md5_goal_2 << " -> "
              << strcmp(p_md5_goal_1, p_md5_goal_2) << std::endl;
    std::cerr << "MD5_Result:   " << p_md5_result_1 << "  &&  " << p_md5_result_2 << " -> "
              << strcmp(p_md5_result_1, p_md5_result_2) << std::endl;
    std::cerr << "MD5_Feedback: " << p_md5_feedback_1 << "  &&  " << p_md5_feedback_2 << " -> "
              << strcmp(p_md5_feedback_1, p_md5_feedback_2) << std::endl;
    std::cerr << std::endl;
  }
};

/// ExecuteTrajectoryResult
template <>
struct MessageConverter<moveit_msgs::ExecuteTrajectoryResult, _moveit_msgs::ExecuteTrajectoryResult>
{
  static bool convert(_moveit_msgs::ExecuteTrajectoryResult& msg_out, const moveit_msgs::ExecuteTrajectoryResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::ExecuteTrajectoryResult& msg_out,
                          const _moveit_msgs::ExecuteTrajectoryResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::ExecuteTrajectoryResult, _moveit_msgs::ExecuteTrajectoryResult);

/// ExecuteTrajectoryActionResult
template <>
struct MessageConverter<moveit_msgs::ExecuteTrajectoryActionResult, _moveit_msgs::ExecuteTrajectoryActionResult>
{
  static bool convert(_moveit_msgs::ExecuteTrajectoryActionResult& msg_out,
                      const moveit_msgs::ExecuteTrajectoryActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::ExecuteTrajectoryActionResult& msg_out,
                          const _moveit_msgs::ExecuteTrajectoryActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalStatus
    // "d388f9b87b3c471f784434d671988d4a";
    serialConvert(msg_out.status, msg_in.status);

    // moveit_msgs::ExecuteTrajectoryResult
    // "a367304b29bf35b99db616894f470bab"; "1f7ab918f5d0c5312f25263d3d688122";
    messageConvert(msg_out.result, msg_in.result);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::ExecuteTrajectoryActionResult, _moveit_msgs::ExecuteTrajectoryActionResult);

/// MoveGroupGoal
template <>
struct MessageConverter<moveit_msgs::PlanningOptions, _moveit_msgs::PlanningOptions>
{
  static bool convert(_moveit_msgs::PlanningOptions& msg_out, const moveit_msgs::PlanningOptions& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlanningOptions& msg_out, const _moveit_msgs::PlanningOptions& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::PlanningScene
    // "acfc50bcfd6c7b978066bfa7c786002c"; "89aac6d20db967ba716cba5a86b1b9d5";
    messageConvert(msg_out.planning_scene_diff, msg_in.planning_scene_diff);

    // uint8_t
    msg_out.plan_only = msg_in.plan_only;

    // uint8_t
    msg_out.look_around = msg_in.look_around;

    // int32_t
    msg_out.look_around_attempts = msg_in.look_around_attempts;

    // double
    msg_out.max_safe_execution_cost = msg_in.max_safe_execution_cost;

    // uint8_t
    msg_out.replan = msg_in.replan;

    // int32_t
    msg_out.replan_attempts = msg_in.replan_attempts;

    // double
    msg_out.replan_delay = msg_in.replan_delay;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlanningOptions, _moveit_msgs::PlanningOptions);

/// MoveGroupGoal
template <>
struct MessageConverter<moveit_msgs::MoveGroupGoal, _moveit_msgs::MoveGroupGoal>
{
  static bool convert(_moveit_msgs::MoveGroupGoal& msg_out, const moveit_msgs::MoveGroupGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MoveGroupGoal& msg_out, const _moveit_msgs::MoveGroupGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MotionPlanRequest
    // "9544d5f3b9cf69a0e1e7f8c75d87f54b"; "c3bec13a525a6ae66e0fc57b768fdca6";
    messageConvert(msg_out.request, msg_in.request);

    // moveit_msgs::PlanningOptions
    // "3134e041c806c7c2ff59948db4d57835"; "3934e50ede2ecea03e532aade900ab50";
    messageConvert(msg_out.planning_options, msg_in.planning_options);
    return true;
  }
};
MirrorMessageConverter(moveit_msgs::MoveGroupGoal, _moveit_msgs::MoveGroupGoal);

/// MoveGroupActionGoal
template <>
struct MessageConverter<moveit_msgs::MoveGroupActionGoal, _moveit_msgs::MoveGroupActionGoal>
{
  static bool convert(_moveit_msgs::MoveGroupActionGoal& msg_out, const moveit_msgs::MoveGroupActionGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MoveGroupActionGoal& msg_out, const _moveit_msgs::MoveGroupActionGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalID
    // "302881f31927c1df708a2dbab0e80ee8";
    serialConvert(msg_out.goal_id, msg_in.goal_id);

    // moveit_msgs::MoveGroupGoal
    // "995f602257ab6c41df8948daf6a3ed57"; ""a6de2db49c561a49babce1a8172e8906";";
    messageConvert(msg_out.goal, msg_in.goal);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::ExecuteTrajectoryActionGoal, _moveit_msgs::ExecuteTrajectoryActionGoal);

/// MoveGroupResult
template <>
struct MessageConverter<moveit_msgs::MoveGroupResult, _moveit_msgs::MoveGroupResult>
{
  static bool convert(_moveit_msgs::MoveGroupResult& msg_out, const moveit_msgs::MoveGroupResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MoveGroupResult& msg_out, const _moveit_msgs::MoveGroupResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    // moveit_msgs::RobotState
    messageConvert(msg_out.trajectory_start, msg_in.trajectory_start);

    // moveit_msgs::RobotTrajectory
    // "dfa9556423d709a3729bcef664bddf67";
    serialConvert(msg_out.planned_trajectory, msg_in.planned_trajectory);

    // moveit_msgs::RobotTrajectory
    serialConvert(msg_out.executed_trajectory, msg_in.executed_trajectory);

    // double
    msg_out.planning_time = msg_in.planning_time;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::MoveGroupResult, _moveit_msgs::MoveGroupResult);

/// MoveGroupActionResult
template <>
struct MessageConverter<moveit_msgs::MoveGroupActionResult, _moveit_msgs::MoveGroupActionResult>
{
  static bool convert(_moveit_msgs::MoveGroupActionResult& msg_out, const moveit_msgs::MoveGroupActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::MoveGroupActionResult& msg_out, const _moveit_msgs::MoveGroupActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalStatus
    // "d388f9b87b3c471f784434d671988d4a";
    serialConvert(msg_out.status, msg_in.status);

    // moveit_msgs::MoveGroupResult
    // "a367304b29bf35b99db616894f470bab"; "1f7ab918f5d0c5312f25263d3d688122";
    messageConvert(msg_out.result, msg_in.result);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::MoveGroupActionResult, _moveit_msgs::MoveGroupActionResult);

/// PickupGoal
template <>
struct MessageConverter<moveit_msgs::PickupGoal, _moveit_msgs::PickupGoal>
{
  static bool convert(_moveit_msgs::PickupGoal& msg_out, const moveit_msgs::PickupGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PickupGoal& msg_out, const _moveit_msgs::PickupGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.target_name = msg_in.target_name;

    // std::string
    msg_out.group_name = msg_in.group_name;

    // std::string
    msg_out.end_effector = msg_in.end_effector;

    // std::vector<moveit_msgs::Grasp>
    // "e26c8fb64f589c33c5d5e54bd7b5e4cb";
    serialConvertVector(msg_out.possible_grasps, msg_in.possible_grasps);

    // std::string
    msg_out.support_surface_name = msg_in.support_surface_name;

    // uint8_t
    msg_out.allow_gripper_support_collision = msg_in.allow_gripper_support_collision;

    // std::vector<std::string>
    msg_out.attached_object_touch_links = msg_in.attached_object_touch_links;

    // uint8_t
    msg_out.minimize_object_distance = msg_in.minimize_object_distance;

    // moveit_msgs::Constraints
    messageConvert(msg_out.path_constraints, msg_in.path_constraints);

    // std::string
    msg_out.planner_id = msg_in.planner_id;

    // std::vector<std::string>
    msg_out.allowed_touch_objects = msg_in.allowed_touch_objects;

    // double
    msg_out.allowed_planning_time = msg_in.allowed_planning_time;

    // moveit_msgs::PlanningOptions
    messageConvert(msg_out.planning_options, msg_in.planning_options);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PickupGoal, _moveit_msgs::PickupGoal);

/// PickupActionGoal
template <>
struct MessageConverter<moveit_msgs::PickupActionGoal, _moveit_msgs::PickupActionGoal>
{
  static bool convert(_moveit_msgs::PickupActionGoal& msg_out, const moveit_msgs::PickupActionGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PickupActionGoal& msg_out, const _moveit_msgs::PickupActionGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalID
    // "302881f31927c1df708a2dbab0e80ee8";
    serialConvert(msg_out.goal_id, msg_in.goal_id);

    // moveit_msgs::PickupGoal
    // "83bb430d120a16fadf2ea3aad239f1d1"; "458c6ab3761d73e99b070063f7b74c2a";
    messageConvert(msg_out.goal, msg_in.goal);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PickupActionGoal, _moveit_msgs::PickupActionGoal);

/// PickupResult
template <>
struct MessageConverter<moveit_msgs::PickupResult, _moveit_msgs::PickupResult>
{
  static bool convert(_moveit_msgs::PickupResult& msg_out, const moveit_msgs::PickupResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PickupResult& msg_out, const _moveit_msgs::PickupResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    // moveit_msgs::RobotState
    messageConvert(msg_out.trajectory_start, msg_in.trajectory_start);

    // moveit_msgs::RobotTrajectory
    serialConvertVector(msg_out.trajectory_stages, msg_in.trajectory_stages);

    // std::vector<std::string>
    msg_out.trajectory_descriptions = msg_in.trajectory_descriptions;

    // moveit_msgs::Grasp
    messageConvert(msg_out.grasp, msg_in.grasp);

    // msg_out.planning_time = msg_in.planning_time; // new

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PickupResult, _moveit_msgs::PickupResult);

/// PickupActionResult
template <>
struct MessageConverter<moveit_msgs::PickupActionResult, _moveit_msgs::PickupActionResult>
{
  static bool convert(_moveit_msgs::PickupActionResult& msg_out, const moveit_msgs::PickupActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PickupActionResult& msg_out, const _moveit_msgs::PickupActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalStatus
    // "d388f9b87b3c471f784434d671988d4a";
    serialConvert(msg_out.status, msg_in.status);

    // moveit_msgs::PickupResult
    // "10e27212825b55941c921761d6a21e16"; "9a2192bdd4f78c9d7c479e4a43f2768f";
    messageConvert(msg_out.result, msg_in.result);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PickupActionResult, _moveit_msgs::PickupActionResult);

/// PlaceLocation
template <>
struct MessageConverter<moveit_msgs::PlaceLocation, _moveit_msgs::PlaceLocation>
{
  static bool convert(_moveit_msgs::PlaceLocation& msg_out, const moveit_msgs::PlaceLocation& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlaceLocation& msg_out, const _moveit_msgs::PlaceLocation& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.id = msg_in.id;

    // trajectory_msgs::JointTrajectory
    // "65b4f94a94d1ed67169da35a02f33d3f";
    serialConvert(msg_out.post_place_posture, msg_in.post_place_posture);

    // geometry_msgs::PoseStamped
    // "d3812c3cbc69362b77dc0b19b345f8f5";
    serialConvert(msg_out.place_pose, msg_in.place_pose);

    // msg_out.quality = msg_in.quality; // new

    // moveit_msgs::GripperTranslation
    // "b53bc0ad0f717cdec3b0e42dec300121";
    serialConvert(msg_out.pre_place_approach, msg_in.pre_place_approach);

    // moveit_msgs::GripperTranslation
    serialConvert(msg_out.post_place_retreat, msg_in.post_place_retreat);

    // std::vector<std::string>
    msg_out.allowed_touch_objects = msg_in.allowed_touch_objects;

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlaceLocation, _moveit_msgs::PlaceLocation);

/// PlaceGoal
template <>
struct MessageConverter<moveit_msgs::PlaceGoal, _moveit_msgs::PlaceGoal>
{
  static bool convert(_moveit_msgs::PlaceGoal& msg_out, const moveit_msgs::PlaceGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlaceGoal& msg_out, const _moveit_msgs::PlaceGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std::string
    msg_out.group_name = msg_in.group_name;

    // std::string
    msg_out.attached_object_name = msg_in.attached_object_name;

    // moveit_msgs::PlaceLocation
    // "7b53f032c68481686026c3e9223d0713"; "f3dbcaca40fb29ede2af78b3e1831128";
    messageConvertVector(msg_out.place_locations, msg_in.place_locations);

    // uint8_t
    msg_out.place_eef = msg_in.place_eef;

    // std::string
    msg_out.support_surface_name = msg_in.support_surface_name;

    // uint8_t
    msg_out.allow_gripper_support_collision = msg_in.allow_gripper_support_collision;

    // moveit_msgs::Constraints
    messageConvert(msg_out.path_constraints, msg_in.path_constraints);

    // std::string
    msg_out.planner_id = msg_in.planner_id;

    // std::vector<std::string>
    msg_out.allowed_touch_objects = msg_in.allowed_touch_objects;

    // double
    msg_out.allowed_planning_time = msg_in.allowed_planning_time;

    // moveit_msgs::PlanningOptions
    messageConvert(msg_out.planning_options, msg_in.planning_options);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlaceGoal, _moveit_msgs::PlaceGoal);

/// PlaceActionGoal
template <>
struct MessageConverter<moveit_msgs::PlaceActionGoal, _moveit_msgs::PlaceActionGoal>
{
  static bool convert(_moveit_msgs::PlaceActionGoal& msg_out, const moveit_msgs::PlaceActionGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlaceActionGoal& msg_out, const _moveit_msgs::PlaceActionGoal& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalID
    // "302881f31927c1df708a2dbab0e80ee8";
    serialConvert(msg_out.goal_id, msg_in.goal_id);

    // moveit_msgs::PlaceGoal
    // "b5ff24625cebec440c18cd2e1d833764"; "e3f3e956e536ccd313fd8f23023f0a94";
    messageConvert(msg_out.goal, msg_in.goal);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlaceActionGoal, _moveit_msgs::PlaceActionGoal);

/// PlaceResult
template <>
struct MessageConverter<moveit_msgs::PlaceResult, _moveit_msgs::PlaceResult>
{
  static bool convert(_moveit_msgs::PlaceResult& msg_out, const moveit_msgs::PlaceResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlaceResult& msg_out, const _moveit_msgs::PlaceResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // moveit_msgs::MoveItErrorCodes
    messageConvert(msg_out.error_code, msg_in.error_code);

    // moveit_msgs::RobotState
    messageConvert(msg_out.trajectory_start, msg_in.trajectory_start);

    // moveit_msgs::RobotTrajectory
    serialConvertVector(msg_out.trajectory_stages, msg_in.trajectory_stages);

    // std::vector<std::string>
    msg_out.trajectory_descriptions = msg_in.trajectory_descriptions;

    // moveit_msgs::PlaceLocation
    messageConvert(msg_out.place_location, msg_in.place_location);

    // msg_out.planning_time = msg_in.planning_time; // new

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlaceResult, _moveit_msgs::PlaceResult);

/// PlaceActionResult
template <>
struct MessageConverter<moveit_msgs::PlaceActionResult, _moveit_msgs::PlaceActionResult>
{
  static bool convert(_moveit_msgs::PlaceActionResult& msg_out, const moveit_msgs::PlaceActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

  static bool convertBack(moveit_msgs::PlaceActionResult& msg_out, const _moveit_msgs::PlaceActionResult& msg_in)
  {
    return __convert(msg_out, msg_in);
  }

private:
  template <typename T1, typename T2>
  static bool __convert(T1& msg_out, const T2& msg_in)
  {
    // std_msgs::Header
    serialConvert(msg_out.header, msg_in.header);

    // actionlib_msgs::GoalStatus
    // "d388f9b87b3c471f784434d671988d4a";
    serialConvert(msg_out.status, msg_in.status);

    // moveit_msgs::PlaceResult
    // "978b357573c8ba8c31923f06ac62d8de"; "da2eea14de05cf0aa280f150a84ded50";
    messageConvert(msg_out.result, msg_in.result);

    return true;
  }
};
MirrorMessageConverter(moveit_msgs::PlaceActionResult, _moveit_msgs::PlaceActionResult);

#if defined(UBUNTU_16_04)
#include "ros_message_convert_16_04.h"
#elif defined(UBUNTU_18_04)
#include "ros_message_convert_18_04.h"
#elif defined(UBUNTU_20_04)
#include "ros_message_convert_20_04.h"
#endif

#endif  // CODEIT_MOVEIT_ROS_MESSAGE_CONVERT_H
