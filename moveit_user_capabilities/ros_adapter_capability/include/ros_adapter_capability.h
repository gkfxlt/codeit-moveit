//
// Created by yjh on 6/28/21.
//

#ifndef CODEIT_MOVEIT_ROS_ADAPTER_CAPABILITY_H
#define CODEIT_MOVEIT_ROS_ADAPTER_CAPABILITY_H

#include <moveit/move_group_capability/move_group_capability.h>
#include "visibility_control.hpp"

namespace move_group
{
class ROS_ADAPTER_PUBLIC RosAdpater : public MoveGroupCapability
{
public:
  RosAdpater();
  ~RosAdpater() override;
  void initialize() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace move_group

#endif  // CODEIT_MOVEIT_ROS_ADAPTER_CAPABILITY_H
