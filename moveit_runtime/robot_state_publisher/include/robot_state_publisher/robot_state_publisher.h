//
// Created by fan on 2021/7/19.
//

#ifndef CODEIT_MOVEIT_ROBOT_STATE_PUBLISHER_INTERFACE_H
#define CODEIT_MOVEIT_ROBOT_STATE_PUBLISHER_INTERFACE_H

#include <moveit/macros/class_forward.h>
#include <string>
#include <vector>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD(RobotModel);
}

struct FixedTransformsEulerAngles
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  std::string frame_id;
  std::string child_frame_id;
};

struct FixedTransformsQuaternion
{
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
  std::string frame_id;
  std::string child_frame_id;
};

class RobotStatePublisher
{
public:
  explicit RobotStatePublisher(const moveit::core::RobotModelConstPtr& robot_model);

  void addFixedTransforms(const std::string& transform);
  void addFixedTransforms(const std::vector<std::string>& transforms);

  void addFixedTransforms(const FixedTransformsEulerAngles& transform);
  void addFixedTransforms(const FixedTransformsQuaternion& transform);

  ~RobotStatePublisher();

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

#endif  // CODEIT_MOVEIT_ROBOT_STATE_PUBLISHER_INTERFACE_H
