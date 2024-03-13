//
// Created by Administrator on 2021/6/11.
//

#include "demo_planner_plugin/demo_planner_plugin.h"
#include "plugin_loader/plugin_loader.hpp"

DemoPlannerManager::DemoPlannerManager() : planning_interface::PlannerManager()
{
}

DemoPlannerManager::~DemoPlannerManager()
{
}

planning_interface::PlanningContextPtr
DemoPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                       const planning_interface::MotionPlanRequest& req,
                                       moveit_msgs::MoveItErrorCodes& error_code) const
{
  return planning_interface::PlanningContextPtr();
}

bool DemoPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
{
  return false;
}

bool DemoPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{
  return PlannerManager::initialize(model, ns);
}

std::string DemoPlannerManager::getDescription() const
{
  return PlannerManager::getDescription();
}

void DemoPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  PlannerManager::getPlanningAlgorithms(algs);
}

void DemoPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs)
{
  PlannerManager::setPlannerConfigurations(pcs);
}

PLUGIN_LOADER_REGISTER_CLASS(DemoPlannerManager, planning_interface::PlannerManager)
