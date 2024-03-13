/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

/* Author: Ioan Sucan, Dave Coleman, Robert Haschke */

#include "moveit_fake_controllers.h"
//#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>
//#include <pluginlib/class_list_macros.hpp>
#include <plugin_loader/plugin_loader.hpp>
#include <log_helper/log_helper.h>
#include <map>

#include "moveit_runtime/moveit_runtime.h"
#include "yaml-cpp/yaml.h"

namespace moveit_fake_controller_manager
{
static const std::string DEFAULT_TYPE = "interpolate";
static const std::string ROBOT_DESCRIPTION = "robot_description";

class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItFakeControllerManager() : node_handle_("~")
  {
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM_NAMED("MoveItFakeControllerManager", "No controller_list specified.");
      return;
    }

    YAML::Node controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.Type() != YAML::NodeType::Sequence)
    {
      ROS_ERROR_NAMED("MoveItFakeControllerManager", "controller_list should be specified as an array");
      return;
    }

    /* by setting latch to true we preserve the initial joint state while other nodes launch */
    bool latch = true;
    pub_ = node_handle_.advertise<sensor_msgs::JointState>(moveit_runtime::topic::CONTROLLER_JOINT_STATES_TOPIC_NAME, 100, latch);

    /* publish initial pose */
    YAML::Node initial;
    if (node_handle_.getParam("initial", initial))
    {
      sensor_msgs::JointState js = loadInitialJointValues(initial);
      js.header.stamp = ros::Time::now();
      pub_.publish(js);
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      std::set<std::string> keys;
      for (YAML::const_iterator it = controller_list[i].begin(); it != controller_list[i].end(); ++it) {
        keys.insert(it->first.as<std::string>());
      }

      if (!keys.count("name") || !keys.count("joints"))
      {
        ROS_ERROR_NAMED("MoveItFakeControllerManager", "Name and joints must be specified for each controller");
        continue;
      }

      try
      {
        const std::string name = controller_list[i]["name"].as<std::string>();

        if (controller_list[i]["joints"].Type() != YAML::NodeType::Sequence)
        {
          ROS_ERROR_STREAM_NAMED("MoveItFakeControllerManager",
                                 "The list of joints for controller " << name << " is not specified as an array");
          continue;
        }
        std::vector<std::string> joints;
        joints.reserve(controller_list[i]["joints"].size());
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          joints.emplace_back(controller_list[i]["joints"][j].as<std::string>());

        const std::string& type =
            keys.count("type") ? controller_list[i]["type"].as<std::string>() : DEFAULT_TYPE;
        if (type == "last point")
          controllers_[name].reset(new LastPointController(name, joints, pub_));
        else if (type == "via points")
          controllers_[name].reset(new ViaPointController(name, joints, pub_));
        else if (type == "interpolate")
          controllers_[name].reset(new InterpolatingController(name, joints, pub_));
        else
        {
          controllers_[name].reset(new InterpolatingController(name, joints, pub_));
          ROS_WARN_STREAM("Unknown fake controller type: " << type << " set type to 'interpolate'.");
          // ROS_ERROR_STREAM("Unknown fake controller type: " << type);
        }

        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        state.default_ = keys.count("default") ? (bool)controller_list[i]["default"] : false;
        state.active_ = true;

        controller_states_[name] = state;
      }
      catch (...)
      {
        ROS_ERROR_NAMED("MoveItFakeControllerManager", "Caught unknown exception while parsing controller information");
      }
    }
  }

  sensor_msgs::JointState loadInitialJointValues(YAML::Node& param) const
  {
    sensor_msgs::JointState js;

    if (param.Type() != YAML::NodeType::Sequence || param.size() == 0)
    {
      ROS_ERROR_ONCE_NAMED("loadInitialJointValues", "Parameter 'initial' should be an array of (group, pose) "
                                                     "structs.");
      return js;
    }

    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotState robot_state(robot_model);
    typedef std::map<std::string, double> JointPoseMap;
    JointPoseMap joints;

    robot_state.setToDefaultValues();  // initialize all joint values (just in case...)
    for (int i = 0, end = param.size(); i != end; ++i)
    {
      try
      {
        std::string group_name = param[i]["group"].as<std::string>();
        std::string pose_name = param[i]["pose"].as<std::string>();
        if (!robot_model->hasJointModelGroup(group_name))
        {
          ROS_WARN_STREAM_NAMED("loadInitialJointValues", "Unknown joint model group: " << group_name);
          continue;
        }
        moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
        const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();

        if (!robot_state.setToDefaultValues(jmg, pose_name))
        {
          ROS_WARN_NAMED("loadInitialJointValues", "Unknown pose '%s' for group '%s'.", pose_name.c_str(),
                         group_name.c_str());
          continue;
        }
        ROS_INFO_NAMED("loadInitialJointValues", "Set joints of group '%s' to pose '%s'.", group_name.c_str(),
                       pose_name.c_str());

        for (const std::string& joint_name : joint_names)
        {
          const moveit::core::JointModel* jm = robot_state.getJointModel(joint_name);
          if (!jm)
          {
            ROS_WARN_STREAM_NAMED("loadInitialJointValues", "Unknown joint: " << joint_name);
            continue;
          }
          if (jm->getVariableCount() != 1)
          {
            ROS_WARN_STREAM_NAMED("loadInitialJointValues", "Cannot handle multi-variable joint: " << joint_name);
            continue;
          }

          joints[joint_name] = robot_state.getJointPositions(jm)[0];
        }
      }
      catch (...)
      {
        ROS_ERROR_ONCE_NAMED("loadInitialJointValues", "Caught unknown exception while reading initial pose "
                                                       "information.");
      }
    }

    // fill the joint state
    for (const auto& name_pos_pair : joints)
    {
      js.name.push_back(name_pos_pair.first);
      js.position.push_back(name_pos_pair.second);
    }
    return js;
  }

  ~MoveItFakeControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
        ROS_FATAL_STREAM("No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    for (std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM("Returned " << names.size() << " controllers in list");
  }

  /*
   * Fake controllers are always active
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /*
   * Fake controllers are always loaded
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on "
               "the param server?",
               name.c_str());
      joints.clear();
    }
  }

  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    return controller_states_[name];
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& /*activate*/,
                         const std::vector<std::string>& /*deactivate*/) override
  {
    return false;
  }

protected:
  _ros::NodeHandle node_handle_;
  _ros::Publisher pub_;
  std::map<std::string, BaseFakeControllerPtr> controllers_;
  std::map<std::string, moveit_controller_manager::MoveItControllerManager::ControllerState> controller_states_;
};

}  // end namespace moveit_fake_controller_manager

PLUGIN_LOADER_REGISTER_CLASS(moveit_fake_controller_manager::MoveItFakeControllerManager,
    moveit_controller_manager::MoveItControllerManager);
