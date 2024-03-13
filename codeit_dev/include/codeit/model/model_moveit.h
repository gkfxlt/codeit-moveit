//
// Created by fan on 2021/7/29.
//

#ifndef CODEIT_MOVEIT_MODEL_MOVEIT_H
#define CODEIT_MOVEIT_MODEL_MOVEIT_H

#include <codeit/core/data_type.hpp>
#include <codeit/model/model_planner.hpp>
#include "moveit/move_group/move_group.h"
#include <memory>


/// MoveitPlanner
namespace codeit::model
{

class MoveitPlanner
{
public:
  static MoveitPlanner* getInstance();

  move_group::MoveGroup& getMoveGroup();

  bool getCurrentJointState(move_group::JointState& jointState);

private:
  MoveitPlanner() = default;

  bool syncJointState(); //同步关节状态

  struct Imp;
  core::ImpPtr<Imp> imp_;
};

}  // namespace codeit::model

#endif  // CODEIT_MOVEIT_MODEL_MOVEIT_H
