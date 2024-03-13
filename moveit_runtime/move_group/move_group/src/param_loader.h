//
// Created by Administrator on 2021/6/10.
//
#pragma once
#ifndef CODEIT_MOVEIT_PARAM_LOADER_H
#define CODEIT_MOVEIT_PARAM_LOADER_H

#include <string>
#include <map>
#include <vector>
#include <filesystem>

struct MoveGroupParam
{
  std::string package_location;
  std::string urdf_path;
  std::string srdf_path;
  std::string robot_description;

  std::string capability_plugins_path;
  std::string kinematics_plugins_path;
  std::string planner_plugins_path;
  std::string planner_adapter_plugins_path;
  std::string controller_plugins_path;
  std::string perception_plugins_path;

  /// robot_description_planning
  struct
  {
    std::string joint_limits_config_path;
    std::string cartesian_limits_config_path;
  } robot_description_planning;

  /// robot_description_kinematics
  struct
  {
    std::string kinematics_config_path;
  } robot_description_kinematics;

  /// move_group
  struct MoveGroup
  {
    bool debug;
    bool allow_trajectory_execution;
    double max_safe_path_cost;
    double jiggle_fraction;

    std::string default_planning_pipeline;
    std::string capabilities;
    std::string disable_capabilities;

    /// octomap
    struct
    {
      double octomap_resolution;
      std::string octomap_frame;
    } octomap;

    struct
    {
      std::vector<std::string> transforms;
    } more_transform;

    /// planning_scene_monitor
    struct
    {
      bool publish_geometry_updates;
      bool publish_planning_scene;
      double publish_planning_scene_hz;
      bool publish_state_updates;
      bool publish_transforms_updates;
      std::string perception_config_path;
    } planning_scene_monitor;

    /// controller
    struct
    {
      bool moveit_manage_controllers;
      std::string moveit_controller_manager;
      std::string controller_config_path;
      std::string joint_motion_map_path;
    } controller;

    /// plan_execution
    struct
    {
      int max_replan_attempts;
      double record_trajectory_state_frequency;
    } plan_execution;

    /// trajectory_execution
    struct
    {
      double allowed_execution_duration_scaling;
      double allowed_goal_duration_margin;
      double allowed_start_tolerance;
      bool execution_duration_monitoring;
      double execution_velocity_scaling;
      bool wait_for_trajectory_completion;
    } trajectory_execution;

    /// planning_pipelines
    struct PIPELINE
    {
      std::string planning_plugin;
      std::string request_adapters;
      double start_state_max_bounds_error;
      std::string config_path;
    };
    std::map<std::string, PIPELINE> planning_pipelines;

  } move_group;
};

class ParamLoader
{
public:
  bool load();

  const std::map<std::string, unsigned int>& getJointMotionMap()
  {
    return joint_motion_map_;
  }
  const std::map<unsigned int, std::string>& getMotionJointMap()
  {
    return motion_joint_map_;
  }

  const std::vector<std::string>& getControllerJointNames()
  {
    return controller_joint_names_;
  }

  const std::vector<unsigned int>& getControllerMotionIds()
  {
    return controller_motion_ids_;
  }

private:
  bool load_from_xml_path(const std::string& xml_path);
  bool load_from_xml_string(const std::string& xml_str);

  bool apply();
  std::filesystem::path config_root_;
  std::filesystem::path plugin_root_;
  MoveGroupParam params_;

  std::map<std::string, unsigned int> joint_motion_map_;
  std::map<unsigned int, std::string> motion_joint_map_;

  std::vector<std::string> controller_joint_names_;
  std::vector<unsigned int> controller_motion_ids_;
};

#endif  // CODEIT_MOVEIT_PARAM_LOADER_H
