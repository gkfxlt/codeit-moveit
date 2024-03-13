//
// Created by Administrator on 2021/6/10.
//

#include "param_loader.h"
#include "fake_node_handle/fake_node_handle.h"
#include "stdboost/file_operation.h"
#include "stdboost/lexical_cast.h"
#include "stdboost/string_operation.h"
#include <tinyxml2.h>
#include "yaml-cpp/yaml.h"

using TiXmlElement = tinyxml2::XMLElement;
using TiXmlDocument = tinyxml2::XMLDocument;

tinyxml2::XMLElement* searchGroup(TiXmlElement* pRoot, const char* param_path)
{
  auto paths = stdboost::split(param_path, "/");

  // 参数组解析
  tinyxml2::XMLElement* pGroup = pRoot;
  for (unsigned int i = 0; i < paths.size() - 1; ++i)
  {
    bool bFoundGroup = false;
    for (TiXmlElement* param_group_xml = pGroup->FirstChildElement("group"); param_group_xml;
         param_group_xml = param_group_xml->NextSiblingElement("group"))
    {
      const char* group_name = param_group_xml->Attribute("name");
      if (strcmp(paths[i].c_str(), group_name) == 0)
      {
        bFoundGroup = true;
        pGroup = param_group_xml;
        break;
      }
    }

    if (!bFoundGroup)
    {
      return nullptr;
    }
  }

  return pGroup;
}

template <typename T>
bool parseParamFromXml(TiXmlElement* pRoot, const char* param_path, T& param_val)
{
  tinyxml2::XMLElement* pGroup = searchGroup(pRoot, param_path);
  if (!pGroup)
  {
    return false;
  }

  std::string temp = param_path;
  std::string param_name = temp.substr(temp.find_last_of('/') + 1);

  // 参数解析
  for (TiXmlElement* param_xml = pGroup->FirstChildElement("param"); param_xml;
       param_xml = param_xml->NextSiblingElement("param"))
  {
    const char* p = param_xml->Attribute("name");
    if (strcmp(param_name.c_str(), p) == 0)
    {
      try
      {
        param_val = stdboost::lexical_cast<T>(param_xml->Attribute("value"));
      }
      catch (...)
      {
        std::cerr << "parseParamFromXml Error: " << param_path << std::endl;
        return false;
      }
      return true;
    }
  }
  return false;
}

bool ParamLoader::load()
{
  try
  {
    std::filesystem::path bin_path = canonical(std::filesystem::path("."));
    std::string config_yaml_path = (bin_path / "moveit_config" / "config.yaml").string();

    YAML::Node config = YAML::LoadFile(config_yaml_path);
    config_root_ = bin_path / "moveit_config" / (config["config"].as<std::string>());

    plugin_root_ = canonical(std::filesystem::path(".")).lexically_normal();
    if (config["plugin_root_path"].IsDefined())
    {
      auto plugin_root_path_string = config["plugin_root_path"].as<std::string>();
      auto t = stdboost::split(plugin_root_path_string, ";");

      for (auto& p : t)
      {
        if (std::filesystem::exists(p))
        {
          plugin_root_ = canonical(std::filesystem::path(p)).lexically_normal();
          break;
        }
      }
    }
  }
  catch (...)
  {
    std::cerr << "LoadFile moveit_config/config.yaml Error! please check the file is exists. " << std::endl;
    return false;
  }

  return load_from_xml_path(std::filesystem::path(config_root_ / "config.xml").string());
}

bool ParamLoader::load_from_xml_path(const std::string& xml_path)
{
  std::string xml_str;
  if (std::filesystem::exists(xml_path) && loadFileToString(xml_path, xml_str))
  {
    return load_from_xml_string(xml_str);
  }
  return false;
}

bool ParamLoader::load_from_xml_string(const std::string& xml_str)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(xml_str.c_str());

  tinyxml2::XMLElement* pRoot = doc.RootElement();

  parseParamFromXml(pRoot, "package_location", params_.package_location);
  parseParamFromXml(pRoot, "urdf_path", params_.urdf_path);
  parseParamFromXml(pRoot, "srdf_path", params_.srdf_path);

  parseParamFromXml(pRoot, "capability_plugins_path", params_.capability_plugins_path);
  parseParamFromXml(pRoot, "kinematics_plugins_path", params_.kinematics_plugins_path);
  parseParamFromXml(pRoot, "planner_plugins_path", params_.planner_plugins_path);
  parseParamFromXml(pRoot, "planner_adapter_plugins_path", params_.planner_adapter_plugins_path);
  parseParamFromXml(pRoot, "controller_plugins_path", params_.controller_plugins_path);
  parseParamFromXml(pRoot, "perception_plugins_path", params_.perception_plugins_path);


  auto deal_path = [this](std::string& path) {
    std::string p_out;
    auto t = stdboost::split(path, " ");
    for (auto& tt : t)
    {
      p_out += " " + (plugin_root_ / tt).lexically_normal().string();
    }
    path = p_out;
  };

  deal_path(params_.capability_plugins_path);
  deal_path(params_.kinematics_plugins_path);
  deal_path(params_.planner_plugins_path);
  deal_path(params_.planner_adapter_plugins_path);
  deal_path(params_.controller_plugins_path);
  deal_path(params_.perception_plugins_path);


  /// robot_description_planning
  parseParamFromXml(pRoot, "robot_description_planning/joint_limits_config_path",
                    params_.robot_description_planning.joint_limits_config_path);
  parseParamFromXml(pRoot, "robot_description_planning/cartesian_limits_config_path",
                    params_.robot_description_planning.cartesian_limits_config_path);

  /// robot_description_kinematics
  parseParamFromXml(pRoot, "robot_description_kinematics/kinematics_config_path",
                    params_.robot_description_kinematics.kinematics_config_path);

  /// move_group
  parseParamFromXml(pRoot, "move_group/debug", params_.move_group.debug);
  parseParamFromXml(pRoot, "move_group/allow_trajectory_execution", params_.move_group.allow_trajectory_execution);
  parseParamFromXml(pRoot, "move_group/max_safe_path_cost", params_.move_group.max_safe_path_cost);
  parseParamFromXml(pRoot, "move_group/jiggle_fraction", params_.move_group.jiggle_fraction);

  parseParamFromXml(pRoot, "move_group/default_planning_pipeline", params_.move_group.default_planning_pipeline);
  parseParamFromXml(pRoot, "move_group/capabilities", params_.move_group.capabilities);
  parseParamFromXml(pRoot, "move_group/disable_capabilities", params_.move_group.disable_capabilities);

  /// octomap
  {
    parseParamFromXml(pRoot, "move_group/octomap/octomap_resolution", params_.move_group.octomap.octomap_resolution);
    parseParamFromXml(pRoot, "move_group/octomap/octomap_frame", params_.move_group.octomap.octomap_frame);
  }

  /// more_transform
  {
    std::string enable = "";
    parseParamFromXml(pRoot, "move_group/more_transform/enable", enable);

    auto transforms = stdboost::split(enable, " ");
    for (auto& trans : transforms)
    {
      std::string param_path = "move_group/more_transform/" + trans;
      std::string param_value = "";

      if (parseParamFromXml(pRoot, param_path.c_str(), param_value))
      {
        params_.move_group.more_transform.transforms.push_back(param_value);
      }
    }
  }

  /// planning_scene_monitor
  {
    parseParamFromXml(pRoot, "move_group/planning_scene_monitor/publish_geometry_updates",
                      params_.move_group.planning_scene_monitor.publish_geometry_updates);
    parseParamFromXml(pRoot, "move_group/planning_scene_monitor/publish_planning_scene",
                      params_.move_group.planning_scene_monitor.publish_planning_scene);
    parseParamFromXml(pRoot, "move_group/planning_scene_monitor/publish_planning_scene_hz",
                      params_.move_group.planning_scene_monitor.publish_planning_scene_hz);
    parseParamFromXml(pRoot, "move_group/planning_scene_monitor/publish_state_updates",
                      params_.move_group.planning_scene_monitor.publish_state_updates);
    parseParamFromXml(pRoot, "move_group/planning_scene_monitor/publish_transforms_updates",
                      params_.move_group.planning_scene_monitor.publish_transforms_updates);
    parseParamFromXml(pRoot, "move_group/planning_scene_monitor/perception_config_path",
                      params_.move_group.planning_scene_monitor.perception_config_path);
  }

  /// controller
  {
    parseParamFromXml(pRoot, "move_group/controller/moveit_manage_controllers",
                      params_.move_group.controller.moveit_manage_controllers);
    parseParamFromXml(pRoot, "move_group/controller/moveit_controller_manager",
                      params_.move_group.controller.moveit_controller_manager);
    parseParamFromXml(pRoot, "move_group/controller/controller_config_path",
                      params_.move_group.controller.controller_config_path);
    parseParamFromXml(pRoot, "move_group/controller/joint_motion_map_path",
                      params_.move_group.controller.joint_motion_map_path);
  }

  /// plan_execution
  {
    parseParamFromXml(pRoot, "move_group/plan_execution/max_replan_attempts",
                      params_.move_group.plan_execution.max_replan_attempts);
    parseParamFromXml(pRoot, "move_group/plan_execution/record_trajectory_state_frequency",
                      params_.move_group.plan_execution.record_trajectory_state_frequency);
  }

  /// trajectory_execution
  {
    parseParamFromXml(pRoot, "move_group/trajectory_execution/allowed_execution_duration_scaling",
                      params_.move_group.trajectory_execution.allowed_execution_duration_scaling);
    parseParamFromXml(pRoot, "move_group/trajectory_execution/allowed_goal_duration_margin",
                      params_.move_group.trajectory_execution.allowed_goal_duration_margin);
    parseParamFromXml(pRoot, "move_group/trajectory_execution/allowed_start_tolerance",
                      params_.move_group.trajectory_execution.allowed_start_tolerance);
    parseParamFromXml(pRoot, "move_group/trajectory_execution/execution_duration_monitoring",
                      params_.move_group.trajectory_execution.execution_duration_monitoring);
    parseParamFromXml(pRoot, "move_group/trajectory_execution/execution_velocity_scaling",
                      params_.move_group.trajectory_execution.execution_velocity_scaling);
    parseParamFromXml(pRoot, "move_group/trajectory_execution/wait_for_trajectory_completion",
                      params_.move_group.trajectory_execution.wait_for_trajectory_completion);
  }

  /// planning_pipelines
  tinyxml2::XMLElement* pGroup = searchGroup(pRoot, "move_group/planning_pipelines/*");
  for (TiXmlElement* param_group_xml = pGroup->FirstChildElement("group"); param_group_xml;
       param_group_xml = param_group_xml->NextSiblingElement("group"))
  {
    std::string group_name = param_group_xml->Attribute("name");
    auto& pipeline = params_.move_group.planning_pipelines[group_name];

    parseParamFromXml(pGroup, (group_name + "/planning_plugin").c_str(), pipeline.planning_plugin);
    parseParamFromXml(pGroup, (group_name + "/request_adapters").c_str(), pipeline.request_adapters);
    parseParamFromXml(pGroup, (group_name + "/start_state_max_bounds_error").c_str(),
                      pipeline.start_state_max_bounds_error);
    parseParamFromXml(pGroup, (group_name + "/config_path").c_str(), pipeline.config_path);
  }

  return apply();
}

template <typename T>
bool setValue(const std::string& param, const YAML::Node& value)
{
  _ros::NodeHandle nh;
  try
  {
    auto val = value.as<T>();
    nh.setParam(param, val);
  }
  catch (...)
  {
    return false;
  }
  return true;
}

void parseYaml(const std::string& ns, YAML::Node config)
{
  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
  {
    std::string key = it->first.as<std::string>();
    YAML::Node value = it->second;
    switch (value.Type())
    {
      case YAML::NodeType::Scalar:
      {
        std::string param = ns + "/" + key;
        if (setValue<int>(param, value) || setValue<double>(param, value) || setValue<bool>(param, value) ||
            setValue<std::string>(param, value))
        {
        }
      }
      break;
      case YAML::NodeType::Sequence:
      {
        _ros::NodeHandle nh;
        std::string param = ns + "/" + key;
        nh.setParam(param, value);
      }
      break;
      case YAML::NodeType::Map:
      {
        _ros::NodeHandle nh;
        std::string param = ns + "/" + key;
        nh.setParam(param, value);
        parseYaml(param, value);
      }
      break;
      case YAML::NodeType::Null:
        printf("key: %s Null\n", key.c_str());
        break;
      case YAML::NodeType::Undefined:
        printf("key: %s Undefined\n", key.c_str());
        break;
    }
  }
}

bool ParamLoader::apply()
{
  _ros::NodeHandle nh;

  params_.package_location = canonical(config_root_ / std::filesystem::path(params_.package_location)).string();

  params_.urdf_path = (config_root_ / params_.urdf_path).string();
  params_.srdf_path = (config_root_ / params_.srdf_path).string();
  params_.robot_description_planning.joint_limits_config_path =
      (config_root_ / params_.robot_description_planning.joint_limits_config_path).string();
  params_.robot_description_planning.cartesian_limits_config_path =
      (config_root_ / params_.robot_description_planning.cartesian_limits_config_path).string();
  params_.robot_description_kinematics.kinematics_config_path =
      (config_root_ / params_.robot_description_kinematics.kinematics_config_path).string();
  params_.move_group.planning_scene_monitor.perception_config_path =
      (config_root_ / params_.move_group.planning_scene_monitor.perception_config_path).string();
  params_.move_group.controller.controller_config_path =
      (config_root_ / params_.move_group.controller.controller_config_path).string();
  params_.move_group.controller.joint_motion_map_path =
      (config_root_ / params_.move_group.controller.joint_motion_map_path).string();

  for (auto& pipeline : params_.move_group.planning_pipelines)
  {
    pipeline.second.config_path = (config_root_ / pipeline.second.config_path).string();
  }

  nh.setParam("package_location", params_.package_location);
  nh.setParam("urdf_path", params_.urdf_path);
  nh.setParam("srdf_path", params_.srdf_path);

  nh.setParam("capability_plugins_path", params_.capability_plugins_path);
  nh.setParam("kinematics_plugins_path", params_.kinematics_plugins_path);
  nh.setParam("planner_plugins_path", params_.planner_plugins_path);
  nh.setParam("planner_adapter_plugins_path", params_.planner_adapter_plugins_path);
  nh.setParam("controller_plugins_path", params_.controller_plugins_path);
  nh.setParam("perception_plugins_path", params_.perception_plugins_path);

  /// robot_description_planning
  nh.setParam("robot_description_planning/joint_limits_config_path",
              params_.robot_description_planning.joint_limits_config_path);
  nh.setParam("robot_description_planning/cartesian_limits_config_path",
              params_.robot_description_planning.cartesian_limits_config_path);

  /// robot_description_kinematics
  nh.setParam("robot_description_kinematics/kinematics_config_path",
              params_.robot_description_kinematics.kinematics_config_path);

  /// move_group
  nh.setParam("move_group/debug", params_.move_group.debug);
  nh.setParam("move_group/allow_trajectory_execution", params_.move_group.allow_trajectory_execution);
  nh.setParam("move_group/max_safe_path_cost", params_.move_group.max_safe_path_cost);
  nh.setParam("move_group/jiggle_fraction", params_.move_group.jiggle_fraction);

  nh.setParam("move_group/default_planning_pipeline", params_.move_group.default_planning_pipeline);
  nh.setParam("move_group/capabilities", params_.move_group.capabilities);
  nh.setParam("move_group/disable_capabilities", params_.move_group.disable_capabilities);

  nh.setParam("move_group/planning_scene_monitor/perception_config_path",
              params_.move_group.planning_scene_monitor.perception_config_path);

  nh.setParam("move_group/controller/moveit_manage_controllers",
              params_.move_group.controller.moveit_manage_controllers);
  nh.setParam("move_group/controller/moveit_controller_manager",
              params_.move_group.controller.moveit_controller_manager);
  nh.setParam("move_group/controller/controller_config_path", params_.move_group.controller.controller_config_path);
  nh.setParam("move_group/controller/joint_motion_map_path", params_.move_group.controller.joint_motion_map_path);

  /// more_transform
  {
    nh.setParam("move_group/more_transform/transforms", params_.move_group.more_transform.transforms);
  }

  /// planning_scene_monitor
  {
    nh.setParam("move_group/planning_scene_monitor/publish_geometry_updates",
                params_.move_group.planning_scene_monitor.publish_geometry_updates);
    nh.setParam("move_group/planning_scene_monitor/publish_planning_scene",
                params_.move_group.planning_scene_monitor.publish_planning_scene);
    nh.setParam("move_group/planning_scene_monitor/publish_planning_scene_hz",
                params_.move_group.planning_scene_monitor.publish_planning_scene_hz);
    nh.setParam("move_group/planning_scene_monitor/publish_state_updates",
                params_.move_group.planning_scene_monitor.publish_state_updates);
    nh.setParam("move_group/planning_scene_monitor/publish_transforms_updates",
                params_.move_group.planning_scene_monitor.publish_transforms_updates);
  }

  /// plan_execution
  {
    nh.setParam("move_group/plan_execution/max_replan_attempts", params_.move_group.plan_execution.max_replan_attempts);
    nh.setParam("move_group/plan_execution/record_trajectory_state_frequency",
                params_.move_group.plan_execution.record_trajectory_state_frequency);
  }

  /// trajectory_execution
  {
    nh.setParam("move_group/trajectory_execution/allowed_execution_duration_scaling",
                params_.move_group.trajectory_execution.allowed_execution_duration_scaling);
    nh.setParam("move_group/trajectory_execution/allowed_goal_duration_margin",
                params_.move_group.trajectory_execution.allowed_goal_duration_margin);
    nh.setParam("move_group/trajectory_execution/allowed_start_tolerance",
                params_.move_group.trajectory_execution.allowed_start_tolerance);
    nh.setParam("move_group/trajectory_execution/execution_duration_monitoring",
                params_.move_group.trajectory_execution.execution_duration_monitoring);
    nh.setParam("move_group/trajectory_execution/execution_velocity_scaling",
                params_.move_group.trajectory_execution.execution_velocity_scaling);
    nh.setParam("move_group/trajectory_execution/wait_for_trajectory_completion",
                params_.move_group.trajectory_execution.wait_for_trajectory_completion);
  }

  /// planning_pipelines
  for (auto& pipeline : params_.move_group.planning_pipelines)
  {
    auto prefix = "move_group/planning_pipelines/" + pipeline.first + "/";

    nh.setParam(prefix + "planning_plugin", pipeline.second.planning_plugin);
    nh.setParam(prefix + "request_adapters", pipeline.second.request_adapters);
    nh.setParam(prefix + "start_state_max_bounds_error", pipeline.second.start_state_max_bounds_error);
    nh.setParam(prefix + "config_path", pipeline.second.config_path);
  }

  //////-------------------------------------------split line---------------------------------------------//////

  /// set robot_description
  std::string robot_description;
  if (!loadFileToString(params_.urdf_path, robot_description))
  {
    std::cerr << "urdf load error! path:" << params_.urdf_path << std::endl;
    return false;
  }
  // parse the <mesh filename="package://xxx"></mesh> to local filepath.
  robot_description =
      std::regex_replace(robot_description, std::regex("package://\\w+"), "file://" + params_.package_location);

  nh.setParam("robot_description", robot_description);

  /// set robot_description_semantic
  std::string robot_description_semantic;
  if (!loadFileToString(params_.srdf_path, robot_description_semantic))
  {
    std::cerr << "srdf load error! path:" << params_.srdf_path << std::endl;
    return false;
  }
  nh.setParam("robot_description_semantic", robot_description_semantic);

  /// joint_limits
  try
  {
    YAML::Node config = YAML::LoadFile(params_.robot_description_planning.joint_limits_config_path);

    parseYaml("robot_description_planning", config);
  }
  catch (...)
  {
    std::cerr << "joint_limits load error! path:" << params_.robot_description_planning.joint_limits_config_path
              << std::endl;
  }

  /// cartesian_limits
  try
  {
    YAML::Node config = YAML::LoadFile(params_.robot_description_planning.cartesian_limits_config_path);

    parseYaml("robot_description_planning", config);
  }
  catch (...)
  {
    std::cerr << "cartesian_limits load error! path:" << params_.robot_description_planning.cartesian_limits_config_path
              << std::endl;
  }

  /// kinematics
  try
  {
    YAML::Node config = YAML::LoadFile(params_.robot_description_kinematics.kinematics_config_path);

    parseYaml("robot_description_kinematics", config);
  }
  catch (...)
  {
    std::cerr << "kinematics load error! path:" << params_.robot_description_kinematics.kinematics_config_path
              << std::endl;
  }

  /// perception
  try
  {
    YAML::Node config = YAML::LoadFile(params_.move_group.planning_scene_monitor.perception_config_path);

    parseYaml("move_group", config);
  }
  catch (...)
  {
    std::cerr << "perception load error! path:" << params_.move_group.planning_scene_monitor.perception_config_path
              << std::endl;
  }

  /// controller
  try
  {
    YAML::Node config = YAML::LoadFile(params_.move_group.controller.controller_config_path);

    parseYaml("move_group", config);
  }
  catch (...)
  {
    std::cerr << "controller load error! path:" << params_.move_group.controller.controller_config_path << std::endl;
  }

  /// joint_motion_map
  try
  {
    YAML::Node config = YAML::LoadFile(params_.move_group.controller.joint_motion_map_path);

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      auto key = it->first.as<std::string>();
      auto value = it->second.as<int>();

      if (!key.empty() && (value >= 0) && (joint_motion_map_.count(key) == 0))
      {
        joint_motion_map_[key] = value;
        motion_joint_map_[value] = key;

        controller_joint_names_.push_back(key);
        controller_motion_ids_.push_back(value);
      }
    }
  }
  catch (...)
  {
    std::cerr << "controller joint_motion_map load error! path:" << params_.move_group.controller.joint_motion_map_path
              << std::endl;
  }

  /// planning_pipelines
  try
  {
    std::vector<std::string> pipeline_names;
    for (auto& pipeline : params_.move_group.planning_pipelines)
    {
      YAML::Node config = YAML::LoadFile(pipeline.second.config_path);

      parseYaml("move_group/planning_pipelines/" + pipeline.first, config);

      pipeline_names.push_back(pipeline.first);
    }
    nh.setParam("move_group/planning_pipelines/pipeline_names", pipeline_names);
  }
  catch (...)
  {
    std::cerr << "planning_pipelines load error!" << std::endl;
  }

  return true;
}
