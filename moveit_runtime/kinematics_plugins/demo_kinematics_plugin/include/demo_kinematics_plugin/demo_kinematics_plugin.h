//
// Created by Administrator on 2021/6/11.
//

#ifndef CODEIT_MOVEIT_DEMO_KINEMATICS_H
#define CODEIT_MOVEIT_DEMO_KINEMATICS_H

#include <moveit/kinematics_base/kinematics_base.h>

class DemoKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options) const override;
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override;
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override;
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override;
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override;
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;
  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getLinkNames() const override;
};

#endif  // CODEIT_MOVEIT_FAKE_PLANNER_H
