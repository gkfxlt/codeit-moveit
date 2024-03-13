//
// Created by Administrator on 2021/6/11.
//

#ifndef CODEIT_MOVEIT_DEMO_PLANNER_PLUGIN_H
#define CODEIT_MOVEIT_DEMO_PLANNER_PLUGIN_H

#include <moveit/planning_interface/planning_interface.h>

class DemoPlannerManager : public planning_interface::PlannerManager
{
public:
  DemoPlannerManager();
  ~DemoPlannerManager() override;

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;



  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

};

#endif  // CODEIT_MOVEIT_DEMO_PLANNER_PLUGIN_H
