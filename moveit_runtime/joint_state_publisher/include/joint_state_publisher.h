//
// Created by fan on 2021/7/16.
//

#ifndef CODEIT_MOVEIT_JOINT_STATE_PUBLISHER_H
#define CODEIT_MOVEIT_JOINT_STATE_PUBLISHER_H

#include <moveit/macros/class_forward.h>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD(RobotModel);
}

class JointStatePublisher
{
public:
  explicit JointStatePublisher(const moveit::core::RobotModelConstPtr& robot_model);

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};



#endif  // CODEIT_MOVEIT_JOINT_STATE_PUBLISHER_H
