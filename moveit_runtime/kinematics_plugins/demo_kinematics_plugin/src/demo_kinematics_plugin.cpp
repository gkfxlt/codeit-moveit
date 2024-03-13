//
// Created by Administrator on 2021/6/11.
//

#include "demo_kinematics_plugin/demo_kinematics_plugin.h"
#include "plugin_loader/plugin_loader.hpp"

bool DemoKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                   const kinematics::KinematicsQueryOptions& options) const
{
  return false;
}
bool DemoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                      double timeout, std::vector<double>& solution,
                                      moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  return false;
}
bool DemoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                      double timeout, const std::vector<double>& consistency_limits,
                                      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  return false;
}
bool DemoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                      double timeout, std::vector<double>& solution,
                                      const kinematics::KinematicsBase::IKCallbackFn& solution_callback,
                                      moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  return false;
}
bool DemoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                      double timeout, const std::vector<double>& consistency_limits,
                                      std::vector<double>& solution,
                                      const kinematics::KinematicsBase::IKCallbackFn& solution_callback,
                                      moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  return false;
}
bool DemoKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                   std::vector<geometry_msgs::Pose>& poses) const
{
  return false;
}
const std::vector<std::string>& DemoKinematicsPlugin::getJointNames() const
{
  std::vector<std::string> v;

  return v;
}
const std::vector<std::string>& DemoKinematicsPlugin::getLinkNames() const
{
  std::vector<std::string> v;

  return v;
}

PLUGIN_LOADER_REGISTER_CLASS(DemoKinematicsPlugin, kinematics::KinematicsBase);