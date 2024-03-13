/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/move_group_capability/move_group_context.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include "moveit/trajectory_execution_manager/trajectory_execution_manager.h"

#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "fake_node_handle/fake_node_handle.h"
#include "ros/time.h"
#include "log_helper/log_helper.h"


constexpr char LOGNAME[] = "moveit_cpp";
constexpr char PLANNING_PLUGIN_PARAM[] = "planning_plugin";

struct PlanningSceneMonitorOptions
{
  void load(/*const ros::NodeHandle& nh*/)
  {
    _ros::NodeHandle nh("~");

    std::string ns = "planning_scene_monitor_options/";
    nh.param<std::string>(ns + "name", name, "planning_scene_monitor");
    nh.param<std::string>(ns + "robot_description", robot_description, "robot_description");
    nh.param(ns + "joint_state_topic", joint_state_topic,
             planning_scene_monitor::PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC);
    nh.param(ns + "attached_collision_object_topic", attached_collision_object_topic,
             planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC);
    nh.param(ns + "monitored_planning_scene_topic", monitored_planning_scene_topic,
             planning_scene_monitor::PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC);
    nh.param(ns + "publish_planning_scene_topic", publish_planning_scene_topic,
             planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC);
    nh.param<double>(ns + "wait_for_initial_state_timeout", wait_for_initial_state_timeout, 0.0);
  }
  std::string name;
  std::string robot_description;
  std::string joint_state_topic;
  std::string attached_collision_object_topic;
  std::string monitored_planning_scene_topic;
  std::string publish_planning_scene_topic;
  double wait_for_initial_state_timeout;
};

struct PlanningPipelineOptions
{
  void load(/*const ros::NodeHandle& nh*/)
  {
    _ros::NodeHandle nh("~");

    std::string ns = "planning_pipelines/";
    nh.getParam(ns + "pipeline_names", pipeline_names);
    nh.getParam(ns + "namespace", parent_namespace);
  }
  std::vector<std::string> pipeline_names;
  std::string parent_namespace;
};

bool move_group::MoveGroupContext::loadPlanningSceneMonitor()
{
  PlanningSceneMonitorOptions options;
  options.load();

  planning_scene_monitor_.reset(
      new planning_scene_monitor::PlanningSceneMonitor(options.robot_description, options.name));
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Configuring Planning Scene Monitor");
  if (planning_scene_monitor_->getPlanningScene())
  {
    // Start state and scene monitors
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s' for joint states", options.joint_state_topic.c_str());
    // Subscribe to JointState sensor messages
    planning_scene_monitor_->startStateMonitor(options.joint_state_topic, options.attached_collision_object_topic);
    // Publish planning scene updates to remote monitors like RViz
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          options.monitored_planning_scene_topic);
    // Monitor and apply planning scene updates from remote publishers like the PlanningSceneInterface
    planning_scene_monitor_->startSceneMonitor(options.publish_planning_scene_topic);
    // Monitor requests for changes in the collision environment
    planning_scene_monitor_->startWorldGeometryMonitor();
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning scene not configured");
    return false;
  }

  // Wait for complete state to be recieved
  if (options.wait_for_initial_state_timeout > 0.0)
  {
    return planning_scene_monitor_->getStateMonitor()->waitForCurrentState(ros::Time::now(),
                                                                           options.wait_for_initial_state_timeout);
  }

  return true;
}

bool move_group::MoveGroupContext::loadPlanningPipelines()
{
  PlanningPipelineOptions options;
  options.load();

  for (const auto& planning_pipeline_name : options.pipeline_names)
  {
    if (planning_pipelines_.count(planning_pipeline_name) > 0)
    {
      ROS_WARN_NAMED(LOGNAME, "Skipping duplicate entry for planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }
    ROS_INFO_NAMED(LOGNAME, "Loading planning pipeline '%s'", planning_pipeline_name.c_str());

    planning_pipeline::PlanningPipelinePtr pipeline;
    _ros::NodeHandle child_nh("move_group/planning_pipelines/" + planning_pipeline_name);
    pipeline.reset(new planning_pipeline::PlanningPipeline(robot_model_, child_nh, PLANNING_PLUGIN_PARAM));

    if (!pipeline->getPlannerManager())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to initialize planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }

    planning_pipelines_[planning_pipeline_name] = pipeline;
  }

  if (planning_pipelines_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to load any planning pipelines.");
    return false;
  }

  // Retrieve group/pipeline mapping for faster lookup
  std::vector<std::string> group_names = robot_model_->getJointModelGroupNames();
  for (const auto& pipeline_entry : planning_pipelines_)
  {
    for (const auto& group_name : group_names)
    {
      groups_pipelines_map_[group_name] = {};
      const auto& pipeline = pipeline_entry.second;
      for (const auto& planner_configuration : pipeline->getPlannerManager()->getPlannerConfigurations())
      {
        if (planner_configuration.second.group == group_name)
        {
          groups_pipelines_map_[group_name].insert(pipeline_entry.first);
        }
      }
    }
  }

  return true;
}

move_group::MoveGroupContext::MoveGroupContext(const std::string& default_planning_pipeline,
                                               bool allow_trajectory_execution, bool debug)
  :allow_trajectory_execution_(allow_trajectory_execution)
  , debug_(debug)
{
  if (!loadPlanningSceneMonitor())
  {
    const std::string error = "Unable to configure planning scene monitor";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  robot_model_ = planning_scene_monitor_->getRobotModel();
  if (!robot_model_)
  {
    const std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                              "parameter server.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  bool load_planning_pipelines = true;
  if (load_planning_pipelines && !loadPlanningPipelines())
  {
    const std::string error = "Failed to load planning pipelines from parameter server";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  // Check if default planning pipeline has been initialized successfully
  const auto& pipelines = getPlanningPipelines();
  const auto default_pipeline_it = pipelines.find(default_planning_pipeline);
  if (default_pipeline_it != pipelines.end())
  {
    planning_pipeline_ = default_pipeline_it->second;

    // configure the planning pipeline
    planning_pipeline_->displayComputedMotionPlans(true);
    planning_pipeline_->checkSolutionPaths(true);

    if (debug_)
      planning_pipeline_->publishReceivedRequests(true);
  }
  else
  {
    ROS_ERROR(
        "Failed to find default PlanningPipeline '%s' - please check MoveGroup's planning pipeline configuration.",
        default_planning_pipeline.c_str());
  }

  if (allow_trajectory_execution_)
  {
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(
        robot_model_, planning_scene_monitor_->getStateMonitor()));
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));

    //      trajectory_execution_manager_ = moveit_cpp_->getTrajectoryExecutionManager();
    //      plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_,
    //      trajectory_execution_manager_)); plan_with_sensing_.reset(new
    //      plan_execution::PlanWithSensing(trajectory_execution_manager_)); if (debug)
    //        plan_with_sensing_->displayCostSources(true);
  }
}

move_group::MoveGroupContext::~MoveGroupContext()
{
  robot_model_.reset();
  planning_pipelines_.clear();
  groups_pipelines_map_.clear();

  //  plan_with_sensing_.reset();
  plan_execution_.reset();
  trajectory_execution_manager_.reset();
  planning_pipeline_.reset();
  planning_scene_monitor_.reset();
}

bool move_group::MoveGroupContext::status() const
{
  const planning_interface::PlannerManagerPtr& planner_interface = planning_pipeline_->getPlannerManager();
  if (planner_interface)
  {
    ROS_INFO_STREAM("MoveGroup context using planning plugin " << planning_pipeline_->getPlannerPluginName());
    ROS_INFO_STREAM("MoveGroup context initialization complete");
    return true;
  }
  else
  {
    ROS_WARN_STREAM("MoveGroup running was unable to load " << planning_pipeline_->getPlannerPluginName());
    return false;
  }
}

const std::map<std::string, planning_pipeline::PlanningPipelinePtr>&
move_group::MoveGroupContext::getPlanningPipelines() const
{
  return planning_pipelines_;
}
