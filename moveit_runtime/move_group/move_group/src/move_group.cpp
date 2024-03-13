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

#include "moveit/move_group/move_group.h"

#include "moveit_msgs/ApplyPlanningScene.h"
#include "moveit_msgs/GetCartesianPath.h"
#include "std_srvs/Empty.h"
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "moveit_msgs/GetPlanningScene.h"
#include "moveit_msgs/GetPositionFK.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/PickupAction.h"
#include "moveit_msgs/PlaceAction.h"
#include "moveit_msgs/GetMotionPlan.h"
#include "moveit_msgs/GetStateValidity.h"
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/move_group_capability/capability_names.h>
#include <moveit/move_group_capability/move_group_capability.h>
#include <moveit/macros/console_colors.h>
#include <memory>
#include <set>
#include <moveit/kinematic_constraints/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"

#include "fake_node_handle/fake_node_handle.h"
#include "plugin_loader/plugin_loader.hpp"
#include "param_loader.h"
#include "log_helper/log_helper.h"
#include "ros/rate.h"

#include "moveit/trajectory_processing/iterative_time_parameterization.h"

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

namespace move_group
{
// These capabilities are loaded unless listed in disable_capabilities
// clang-format off
static const char* DEFAULT_CAPABILITIES[] = {
   "move_group/MoveGroupCartesianPathService",
   "move_group/MoveGroupKinematicsService",
   "move_group/MoveGroupExecuteTrajectoryAction",
   "move_group/MoveGroupMoveAction",
   "move_group/MoveGroupPickPlaceAction",
   "move_group/MoveGroupPlanService",
   "move_group/MoveGroupQueryPlannersService",
   "move_group/MoveGroupStateValidationService",
   "move_group/MoveGroupGetPlanningSceneService",
   "move_group/ApplyPlanningSceneService",
   "move_group/ClearOctomapService",
};
// clang-format on

class MoveGroupExe
{
public:
  MoveGroupExe(bool debug)
  {
    _ros::NodeHandle nh;
    std::string default_planning_pipeline = "ompl";
    nh.getParam("/move_group/default_planning_pipeline", default_planning_pipeline);

    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    bool allow_trajectory_execution;
    nh.param("/move_group/allow_trajectory_execution", allow_trajectory_execution, true);

    context_ = std::make_shared<MoveGroupContext>(default_planning_pipeline, allow_trajectory_execution, debug);

    // start the capabilities
    configureCapabilities();
  }

  ~MoveGroupExe()
  {
    capabilities_.clear();
    context_.reset();
    capability_plugin_loader_.reset();
  }

  MoveGroupContextPtr& getContext()
  {
    return context_;
  }

  void status()
  {
    if (context_)
    {
      if (context_->status())
      {
        if (capabilities_.empty())
          printf(MOVEIT_CONSOLE_COLOR_BLUE "\nmove_group is running but no capabilities are "
                                           "loaded.\n\n" MOVEIT_CONSOLE_COLOR_RESET);
        else
          printf(MOVEIT_CONSOLE_COLOR_GREEN "\nYou can start planning now!\n\n" MOVEIT_CONSOLE_COLOR_RESET);
        fflush(stdout);
      }
    }
    else
      ROS_ERROR("No MoveGroup context created. Nothing will work.");
  }

private:
  void configureCapabilities()
  {
    _ros::NodeHandle nh;
    std::string capability_plugins_path = "../lib/move_group_capabilities_default";
    if (!nh.getParam("/capability_plugins_path", capability_plugins_path))
    {
      ROS_WARN_ONCE("move_group", "PARAM NOT SET:capability_plugins_path");
    }
    capability_plugin_loader_.reset(new pluginloader::ClassLoader<MoveGroupCapability>(capability_plugins_path));

    std::set<std::string> capabilities;

    // add default capabilities
    for (const char* capability : DEFAULT_CAPABILITIES)
      capabilities.insert(capability);

    // add capabilities listed in ROS parameter
    std::string capability_plugins;
    if (nh.getParam("move_group/capabilities", capability_plugins))
    {
      auto caps = stdboost::resplit(capability_plugins);
      capabilities.insert(caps.begin(), caps.end());
    }

    // drop capabilities that have been explicitly disabled
    if (nh.getParam("move_group/disable_capabilities", capability_plugins))
    {
      auto caps = stdboost::resplit(capability_plugins);
      for (auto& cap : caps)
      {
        capabilities.erase(cap);
      }
    }

    for (const std::string& capability : capabilities)
    {
      printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, capability.c_str());
      MoveGroupCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(capability);
      if (cap)
      {
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(cap);
      }
      else
      {
        ROS_ERROR_STREAM("load move_group capability '" << capability << "' failed!");
      }
    }

    std::stringstream ss;
    ss << std::endl;
    ss << std::endl;
    ss << "********************************************************" << std::endl;
    ss << "* MoveGroup using: " << std::endl;
    for (const MoveGroupCapabilityPtr& cap : capabilities_)
      ss << "*     - " << cap->getName() << std::endl;
    ss << "********************************************************" << std::endl;
    ROS_INFO_STREAM(ss.str());
  }

  MoveGroupContextPtr context_;
  std::shared_ptr<pluginloader::ClassLoader<MoveGroupCapability>> capability_plugin_loader_;
  std::vector<MoveGroupCapabilityPtr> capabilities_;
};

geometry_msgs::Pose robotTarget2Pose(move_group::RobotTarget robot_target)
{
  geometry_msgs::Pose pose;
  pose.position.x = robot_target.x;
  pose.position.y = robot_target.y;
  pose.position.z = robot_target.z;
  pose.orientation.x = robot_target.q1;
  pose.orientation.y = robot_target.q2;
  pose.orientation.z = robot_target.q3;
  pose.orientation.w = robot_target.q4;
  return pose;
}

struct MoveGroup::Impl
{
  std::unique_ptr<ParamLoader> paramLoader;
  std::unique_ptr<move_group::MoveGroupExe> moveGroupExe;

  std::vector<std::string> controller_joint_names;
  std::vector<unsigned int> controller_motion_ids;

  /// topic
  _ros::Publisher joint_state_pub;
  _ros::Publisher collision_object_pub;
  _ros::Publisher point_cloud2_pub;

  /// services
  _ros::ServiceClient apply_planning_scene;
  _ros::ServiceClient get_cartesian_path;
  _ros::ServiceClient clear_octomap;
  _ros::ServiceClient get_planning_scene;
  _ros::ServiceClient compute_fk;
  _ros::ServiceClient compute_ik;
  _ros::ServiceClient get_motion_plan;
  _ros::ServiceClient get_state_validity;

  /// actions
  std::unique_ptr<_ros::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> execute_trajectory_action;
  std::unique_ptr<_ros::SimpleActionClient<moveit_msgs::MoveGroupAction>> move_group_action;
  std::unique_ptr<_ros::SimpleActionClient<moveit_msgs::PickupAction>> pickup_action;
  std::unique_ptr<_ros::SimpleActionClient<moveit_msgs::PlaceAction>> place_action;

  bool action_stop;
  double action_joint_ctl_freq;
  std::unique_ptr<_ros::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> follow_joint_trajectory_action;

  GetJointStateCallback getJointStateCallback;
  JointExecCallback jointExecCallback;
  TrajectoryExecCallback trajectoryExecCallback;
  ScreenUpdateCallback screenUpdateCallback;

  bool joint_state_pub_stop;
  std::thread joint_state_pub_thread;

  //add 2022-06-20
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;

  bool init(bool debug)
  {
    paramLoader = std::make_unique<ParamLoader>();
    if (!paramLoader->load()) {
      return false;
    }

    controller_joint_names = paramLoader->getControllerJointNames();
    controller_motion_ids = paramLoader->getControllerMotionIds();

    moveGroupExe = std::make_unique<move_group::MoveGroupExe>(debug);
    moveGroupExe->status();
    if (!moveGroupExe->getContext()->status()) {
      return false;
    }

    moveGroupExe->getContext()->planning_scene_monitor_->addUpdateCallback(
        std::bind(&Impl::screenUpdate, this, std::placeholders::_1));

    _ros::NodeHandle nh;
    /// topic
    joint_state_pub = nh.advertise<sensor_msgs::JointState>(moveit_runtime::topic::CONTROLLER_JOINT_STATES_TOPIC_NAME);
    collision_object_pub =
        nh.advertise<moveit_msgs::CollisionObject>(moveit_runtime::topic::COLLISION_OBJECT_TOPIC_NAME);
    point_cloud2_pub = nh.advertise<sensor_msgs::PointCloud2>(moveit_runtime::topic::INNER_POINT_CLOUD2_TOPIC_NAME);

    /// services
    apply_planning_scene =
        nh.serviceClient<moveit_msgs::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);
    get_cartesian_path = nh.serviceClient<moveit_msgs::GetCartesianPath>(move_group::CARTESIAN_PATH_SERVICE_NAME);
    clear_octomap = nh.serviceClient<std_srvs::Empty>(move_group::CLEAR_OCTOMAP_SERVICE_NAME);
    get_planning_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    compute_fk = nh.serviceClient<moveit_msgs::GetPositionFK>(move_group::FK_SERVICE_NAME);
    compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>(move_group::IK_SERVICE_NAME);
    get_motion_plan = nh.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);
    get_state_validity = nh.serviceClient<moveit_msgs::GetStateValidity>(move_group::STATE_VALIDITY_SERVICE_NAME);

    /// actions clients
    execute_trajectory_action = std::make_unique<_ros::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>>(
        move_group::EXECUTE_ACTION_NAME);
    move_group_action =
        std::make_unique<_ros::SimpleActionClient<moveit_msgs::MoveGroupAction>>(move_group::MOVE_ACTION);
    pickup_action = std::make_unique<_ros::SimpleActionClient<moveit_msgs::PickupAction>>(move_group::PICKUP_ACTION);
    place_action = std::make_unique<_ros::SimpleActionClient<moveit_msgs::PlaceAction>>(move_group::PLACE_ACTION);

    /// actions servers
    action_stop = false;
    action_joint_ctl_freq = 100;
    follow_joint_trajectory_action =
        std::make_unique<_ros::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
            moveit_runtime::action::CONTROLLER_FOLLOW_TRAJECTORY_ACTION_NAME,
            std::bind(&Impl::executeFollowJointTrajectory, this, _1), false);
    follow_joint_trajectory_action->start();

    joint_state_pub_stop = false;
    joint_state_pub_thread = std::thread(&Impl::publishJointState, this);

    return true;
  }

  ~Impl()
  {
    joint_state_pub_stop = true;
    if(joint_state_pub_thread.joinable())
      joint_state_pub_thread.join();
  }

  void publishJointState() const
  {
    sensor_msgs::JointState js;
    move_group::JointState jointState;
    js.name = controller_joint_names;
    jointState.joint_names = controller_joint_names;

    auto size = controller_joint_names.size();
    js.position.resize(size);
    jointState.positions.resize(size);

    ros::Rate rate(100);
    while (!joint_state_pub_stop)
    {
      if (getJointStateCallback && getJointStateCallback(jointState))
      {
        if (memcmp(js.position.data(), jointState.positions.data(), size * sizeof(double)) != 0)
        {
          memcpy(js.position.data(), jointState.positions.data(), size * sizeof(double));
          joint_state_pub.publish(js);
        }
      }
      rate.sleep();
    }
  }

  void screenUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type) const
  {
    if (update_type & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
    {
      if (screenUpdateCallback)
        screenUpdateCallback();
    }
  }

  static void interpolate(move_group::JointState& js, const trajectory_msgs::JointTrajectoryPoint& prev,
                          const trajectory_msgs::JointTrajectoryPoint& next, const ros::Duration& elapsed)
  {
    double duration = (next.time_from_start - prev.time_from_start).toSec();
    double alpha = 1.0;
    if (duration > std::numeric_limits<double>::epsilon())
      alpha = (elapsed - prev.time_from_start).toSec() / duration;

    js.positions.resize(prev.positions.size());
    for (std::size_t i = 0, end = prev.positions.size(); i < end; ++i)
    {
      js.positions[i] = prev.positions[i] + alpha * (next.positions[i] - prev.positions[i]);
    }
  }

  void executeFollowJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) const
  {
    static std::mutex mutex;
    std::scoped_lock<std::mutex> locker(mutex);

    control_msgs::FollowJointTrajectoryResult result;
    if (!trajectoryExecCallback)
    {
      follow_joint_trajectory_action->setAborted(result);
      return;
    }

    move_group::JointTrajectory jointTrajectory;
    auto point_num = goal->trajectory.points.size();
    jointTrajectory.joint_names = goal->trajectory.joint_names;
    jointTrajectory.points.resize(point_num);
    for (int i = 0; i < point_num; ++i)
    {
      jointTrajectory.points[i].time_from_start = goal->trajectory.points[i].time_from_start.toSec();
      jointTrajectory.points[i].positions = goal->trajectory.points[i].positions;
    }

    bool cancel = false;
    bool succeed = trajectoryExecCallback(jointTrajectory, cancel);

    if (succeed)
    {
      follow_joint_trajectory_action->setSucceeded(result);
    }
    else
    {
      follow_joint_trajectory_action->setAborted(result);
    }
  }

  moveit_msgs::Constraints constructGoalConstraints(const std::string& group_name, const geometry_msgs::Pose& pose,
                                                    double tolerance_pos = 1e-3, double tolerance_angle = 1e-2) const
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(moveGroupExe->getContext()->planning_scene_monitor_);

    if (const moveit::core::JointModelGroup* jmg = lscene->getCurrentState().getJointModelGroup(group_name))
    {
      if (!jmg->getLinkModelNames().empty())
      {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = lscene->getPlanningFrame();
        poseStamped.pose = pose;

        return kinematic_constraints::constructGoalConstraints(jmg->getLinkModelNames().back(), poseStamped,
                                                               tolerance_pos, tolerance_angle);
      }
    }

    return {};
  }
};

bool MoveGroup::init(bool debug)
{
  impl_ = std::make_shared<Impl>();
  return impl_->init(debug);
}

void MoveGroup::registerGetJointStateCallback(const GetJointStateCallback& getJointStateCallback)
{
  impl_->getJointStateCallback = getJointStateCallback;
}

void MoveGroup::registerJointExecCallback(const JointExecCallback& jointExecCallback)
{
  impl_->jointExecCallback = jointExecCallback;
}

void MoveGroup::registerTrajectoryExecCallback(const TrajectoryExecCallback& trajectoryExecCallback)
{
  impl_->trajectoryExecCallback = trajectoryExecCallback;
}

void MoveGroup::registerScreenUpdateCallback(const ScreenUpdateCallback& screenUpdateCallback)
{
  impl_->screenUpdateCallback = screenUpdateCallback;
}

bool MoveGroup::checkCollision(const std::string& group_name, const move_group::JointTrajectory& jointTrajectory,
                               int current_index, int& collision_index)
{
  auto& psm = impl_->moveGroupExe->getContext()->planning_scene_monitor_;
  auto& planning_scene_ = psm->getPlanningScene();
  auto& robot_model = psm->getRobotModel();
  moveit::core::RobotState robot_state(robot_model);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = group_name;
  res.collision = false;

  planning_scene_monitor::LockedPlanningSceneRO lscene(psm);  // lock the scene so that it
                                                              // does not modify the world
                                                              // representation while
                                                              // isStateValid() is called

  const collision_detection::AllowedCollisionMatrix* acm = &planning_scene_->getAllowedCollisionMatrix();
  for (int i = current_index; i < jointTrajectory.points.size(); ++i)
  {
    robot_state.setJointGroupPositions(group_name, jointTrajectory.points[i].positions);

    if (acm)
      planning_scene_->checkCollisionUnpadded(req, res, robot_state, *acm);
    else
      planning_scene_->checkCollisionUnpadded(req, res, robot_state);

    if (res.collision || !planning_scene_->isStateFeasible(robot_state, false))
    {
      // call the same functions again, in verbose mode, to show what issues have been detected
      planning_scene_->isStateFeasible(robot_state, true);
      req.verbose = true;
      res.clear();
      if (acm)
        planning_scene_->checkCollisionUnpadded(req, res, robot_state, *acm);
      else
        planning_scene_->checkCollisionUnpadded(req, res, robot_state);
    }

    if (res.collision)
    {
      collision_index = i;
      return true;
    }
  }
  return false;
}

// 计算机器人自碰撞距离
double MoveGroup::getDistanceRobotSelf(const move_group::JointState& jointState, std::string& out_link1,
                                       std::string& out_link2)
{
  auto& psm = impl_->moveGroupExe->getContext()->planning_scene_monitor_;
  auto& planning_scene_ = psm->getPlanningScene();

  moveit::core::RobotState robotState = planning_scene_->getCurrentState();
  robotState.setVariablePositions(jointState.joint_names, jointState.positions);

  collision_detection::DistanceRequest req;
  req.type = collision_detection::DistanceRequestTypes::SINGLE;
  req.acm = &planning_scene_->getAllowedCollisionMatrix();
  req.enable_signed_distance = true;

  collision_detection::DistanceResult res;
  planning_scene_->getCollisionEnv()->distanceSelf(req, res, robotState);
  out_link1 = res.minimum_distance.link_names[0];
  out_link2 = res.minimum_distance.link_names[1];
  return res.minimum_distance.distance;
}

// 规划到指定状态
bool MoveGroup::planToState(const move_group::JointState& startState, const move_group::JointState& goalState,
                            JointTrajectory& out_joint_trajectory, const std::string& group_name,
                            const std::string& pipeline_id, double allowed_planning_time)
{
  moveit_msgs::GetMotionPlan plan;
  plan.request.motion_plan_request.group_name = group_name;
  plan.request.motion_plan_request.pipeline_id = pipeline_id;
  plan.request.motion_plan_request.allowed_planning_time = allowed_planning_time;

  if ((startState.positions.size() != 0) && (startState.joint_names.size() == startState.positions.size()))
  {
    plan.request.motion_plan_request.start_state.joint_state.header.frame_id = "world";
    plan.request.motion_plan_request.start_state.joint_state.name = startState.joint_names;
    plan.request.motion_plan_request.start_state.joint_state.position = startState.positions;
  }

  // goal_constraints
  plan.request.motion_plan_request.goal_constraints.resize(1);
  moveit_msgs::Constraints& goal = plan.request.motion_plan_request.goal_constraints[0];
  int joint_num = goalState.joint_names.size();
  goal.joint_constraints.resize(joint_num);
  for (std::size_t i = 0; i < joint_num; ++i)
  {
    goal.joint_constraints[i].joint_name = goalState.joint_names[i];
    goal.joint_constraints[i].position = goalState.positions[i];
    goal.joint_constraints[i].tolerance_above = 1e-2;
    goal.joint_constraints[i].tolerance_below = 1e-2;
    goal.joint_constraints[i].weight = 1.0;
  }

  /// plan
  impl_->get_motion_plan.call(plan.request, plan.response);
  if (plan.response.motion_plan_response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    return false;
  }

  /// convert
  auto& joint_trajectory = plan.response.motion_plan_response.trajectory.joint_trajectory;
  auto point_num = joint_trajectory.points.size();
  out_joint_trajectory.joint_names = joint_trajectory.joint_names;
  out_joint_trajectory.points.resize(joint_trajectory.points.size());
  for (int i = 0; i < point_num; ++i)
  {
    out_joint_trajectory.points[i].time_from_start = joint_trajectory.points[i].time_from_start.toSec();
    out_joint_trajectory.points[i].positions = joint_trajectory.points[i].positions;
    out_joint_trajectory.points[i].velocities = joint_trajectory.points[i].velocities;
    out_joint_trajectory.points[i].accelerations = joint_trajectory.points[i].accelerations;
  }

  return true;
}

void MoveGroup::inComingPointCloud(const char* data, int point_size, int point_num, const std::string& frame_id,
                                   int x_offset, int y_offset, int z_offset)
{
  sensor_msgs::PointCloud2 cloud_out;
  cloud_out.width = point_num;
  cloud_out.height = 1;
  cloud_out.header.frame_id = frame_id;
  cloud_out.fields.resize(3);
  cloud_out.fields[0].name = "x";
  cloud_out.fields[0].offset = 0;
  cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[0].count = 1;
  cloud_out.fields[1].name = "y";
  cloud_out.fields[1].offset = 4;
  cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[1].count = 1;
  cloud_out.fields[2].name = "z";
  cloud_out.fields[2].offset = 8;
  cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[2].count = 1;

  cloud_out.point_step = 12;  // 3*4=12
  cloud_out.row_step = cloud_out.point_step * cloud_out.width;
  cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
  cloud_out.is_dense = false;

  for (int i = 0; i < point_num; ++i)
  {
    float* pstep = (float*)&cloud_out.data[i * cloud_out.point_step];
    pstep[0] = *(float*)&data[i * point_size + x_offset];
    pstep[1] = *(float*)&data[i * point_size + y_offset];
    pstep[2] = *(float*)&data[i * point_size + z_offset];
  }

  impl_->point_cloud2_pub.publish(cloud_out);
}

bool MoveGroup::publishJointState(const move_group::JointState& jointState)
{
  sensor_msgs::JointState js;
  js.name = jointState.joint_names;
  js.position = jointState.positions;
  impl_->joint_state_pub.publish(js);
  return true;
}

const std::vector<std::string>& MoveGroup::getControllerJointNames()
{
  return impl_->controller_joint_names;
}

const std::vector<unsigned int>& MoveGroup::getControllerMotionIds()
{
  return impl_->controller_motion_ids;
}

bool MoveGroup::getJointNameByMotionId(const std::vector<unsigned int>& motion_ids,
                                       std::vector<std::string>& joint_names)
{
  if (motion_ids.size() == 0)
  {
    return false;
  }

  auto& m = impl_->paramLoader->getMotionJointMap();
  for (auto& motion_id : motion_ids)
  {
    if (m.find(motion_id) == m.end())
    {
      return false;
    }
    joint_names.emplace_back(m.at(motion_id));
  }
  return true;
}

bool MoveGroup::getMotionIdByJointName(const std::vector<std::string>& joint_names,
                                       std::vector<unsigned int>& motion_ids)
{
  if (joint_names.size() == 0)
  {
    return false;
  }

  auto& m = impl_->paramLoader->getJointMotionMap();
  for (auto& joint_name : joint_names)
  {
    if (m.find(joint_name) == m.end())
    {
      return false;
    }
    motion_ids.emplace_back(m.at(joint_name));
  }
  return true;
}

move_group::Pose Isometry3d_to_pose(const Eigen::Isometry3d& in)
{
  move_group::Pose p;
  p.x = in.translation().x();
  p.y = in.translation().y();
  p.z = in.translation().z();
  Eigen::Quaterniond q(in.linear());
  p.q1 = q.x();
  p.q2 = q.y();
  p.q3 = q.z();
  p.q4 = q.w();
  if (p.q4 < 0)
  {
    p.q1 *= -1;
    p.q2 *= -1;
    p.q3 *= -1;
    p.q4 *= -1;
  }
  return p;
}

bool MoveGroup::getGroupJointTransforms(const std::string& group, std::map<std::string, move_group::Pose>& poses)
{
  moveit::core::RobotState rs =
      planning_scene_monitor::LockedPlanningSceneRO(impl_->moveGroupExe->getContext()->planning_scene_monitor_)
          ->getCurrentState();
  moveit::core::RobotModelConstPtr rm = rs.getRobotModel();

  const moveit::core::JointModelGroup* jmg = rm->getJointModelGroup(group);
  auto& names = (jmg != nullptr) ? jmg->getJointModelNames() : rm->getJointModelNames();
  for (auto& name : names)
  {
    poses[name] = Isometry3d_to_pose(rs.getJointTransform(name));
  }
  return true;
}

bool MoveGroup::getGroupGlobalLinkTransforms(const std::string& group, std::map<std::string, move_group::Pose>& poses)
{
  moveit::core::RobotState rs =
      planning_scene_monitor::LockedPlanningSceneRO(impl_->moveGroupExe->getContext()->planning_scene_monitor_)
          ->getCurrentState();
  moveit::core::RobotModelConstPtr rm = rs.getRobotModel();

  const moveit::core::JointModelGroup* jmg = rm->getJointModelGroup(group);
  auto& names = (jmg != nullptr) ? jmg->getLinkModelNames() : rm->getLinkModelNames();
  for (auto& name : names)
  {
    poses[name] = Isometry3d_to_pose(rs.getGlobalLinkTransform(name));
  }
  return true;
}

bool MoveGroup::planIt(move_group::PlanIt::Request& request, move_group::PlanIt::Response& response)
{
  moveit_msgs::GetMotionPlan plan;
  plan.request.motion_plan_request.group_name = request.group_name;
  plan.request.motion_plan_request.pipeline_id = request.pipeline_id;
  plan.request.motion_plan_request.allowed_planning_time = request.allowed_planning_time;

  if ((request.joint_start.joint_names.size() != 0) &&
      (request.joint_start.joint_names.size() == request.joint_start.positions.size()))
  {
    plan.request.motion_plan_request.start_state.joint_state.header.frame_id = "world";
    plan.request.motion_plan_request.start_state.joint_state.name = request.joint_start.joint_names;
    plan.request.motion_plan_request.start_state.joint_state.position = request.joint_start.positions;
  }

  plan.request.motion_plan_request.goal_constraints.resize(1);
  plan.request.motion_plan_request.goal_constraints[0] =
      impl_->constructGoalConstraints(request.group_name, robotTarget2Pose(request.robot_target));

  /// plan
  impl_->get_motion_plan.call(plan.request, plan.response);
  if (plan.response.motion_plan_response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    return false;
  }

  /// convert
  response.group_name = plan.response.motion_plan_response.group_name;
  response.planning_time = plan.response.motion_plan_response.planning_time;

  auto& joint_trajectory = plan.response.motion_plan_response.trajectory.joint_trajectory;
  auto point_num = joint_trajectory.points.size();
  response.joint_trajectory.joint_names = joint_trajectory.joint_names;
  response.joint_trajectory.points.resize(joint_trajectory.points.size());
  for (int i = 0; i < point_num; ++i)
  {
    response.joint_trajectory.points[i].time_from_start = joint_trajectory.points[i].time_from_start.toSec();
    response.joint_trajectory.points[i].positions = joint_trajectory.points[i].positions;
    response.joint_trajectory.points[i].velocities = joint_trajectory.points[i].velocities;
    response.joint_trajectory.points[i].accelerations = joint_trajectory.points[i].accelerations;
  }

  return true;
}

void MoveGroup::processCollisionObject(const move_group::ProcessCollisionObjectParam& param)
{
  moveit_msgs::CollisionObject collisionObject;

  collisionObject.operation = param.operation;

  collisionObject.id = param.id;
  collisionObject.header.frame_id = param.frame_id;
  collisionObject.primitives.resize(1);
  collisionObject.primitives.at(0).type = param.type;
  collisionObject.primitives.at(0).dimensions = param.dimensions;
  collisionObject.primitive_poses.resize(1);

  auto& pose = collisionObject.primitive_poses.at(0);
  pose.position.x = param.pose.x;
  pose.position.y = param.pose.y;
  pose.position.z = param.pose.z;
  pose.orientation.x = param.pose.q1;
  pose.orientation.y = param.pose.q2;
  pose.orientation.z = param.pose.q3;
  pose.orientation.w = param.pose.q4;

  impl_->collision_object_pub.publish(collisionObject);
}

bool MoveGroup::checkJointStateValid(const move_group::JointState& jointState)
{
  moveit_msgs::GetStateValidity getStateValidity;

  getStateValidity.request.robot_state.joint_state.name = jointState.joint_names;
  getStateValidity.request.robot_state.joint_state.position = jointState.positions;

  impl_->get_state_validity.call(getStateValidity.request, getStateValidity.response);

  return getStateValidity.response.valid;
}

void MoveGroup::clearCollisionObject()
{
  std_srvs::Empty empty;
  impl_->clear_octomap.call(empty.request, empty.response);
}


void MoveGroup::setTrajectoryFilterParam(double max_velocity_scaling_factor, double max_acceleration_scaling_factor){
	impl_->max_velocity_scaling_factor_ =  max_velocity_scaling_factor;
	impl_->max_acceleration_scaling_factor_ = max_acceleration_scaling_factor;
}

void MoveGroup::trajectoryBlend(const std::string & group_name, const std::vector<JointTrajectory> &multi_trajectory_series,
		JointTrajectory &goal_trajectory){
	// build connect trajectory
	JointTrajectory connect_plan = multi_trajectory_series[0];
	moveit_msgs::RobotTrajectory trajectory, trajectory_filter;
	trajectory.joint_trajectory.joint_names = connect_plan.joint_names;
	trajectory.joint_trajectory.points.resize(connect_plan.points.size());
	for (size_t ni = 1; ni < multi_trajectory_series.size(); ni++) {
		ros::Duration start_time =
				trajectory.joint_trajectory.points[trajectory.joint_trajectory.points.size()
						- 1].time_from_start;
		for (size_t nj = 1;
				nj < multi_trajectory_series[ni].points.size();
				nj++) {
			trajectory_msgs::JointTrajectoryPoint new_point;
			new_point.accelerations =  multi_trajectory_series[ni].points[nj].accelerations;
			new_point.positions = multi_trajectory_series[ni].points[nj].positions;
			new_point.velocities = multi_trajectory_series[ni].points[nj].velocities;
			new_point.time_from_start += start_time;
			trajectory.joint_trajectory.points.push_back(new_point);
		}
	}
	// trajectory filter
	robot_trajectory::RobotTrajectory rt(
			impl_->moveGroupExe->getContext()->planning_scene_monitor_->getRobotModel(), group_name);
	rt.setRobotTrajectoryMsg(*(impl_->moveGroupExe->getContext()->planning_scene_monitor_->getStateMonitor()->getCurrentState()), trajectory);
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	iptp.computeTimeStamps(rt, impl_->max_velocity_scaling_factor_, impl_->max_acceleration_scaling_factor_);
	rt.getRobotTrajectoryMsg(trajectory_filter);
	//build the goal trajectory
	goal_trajectory.joint_names.clear();
	goal_trajectory.points.clear();
	for(unsigned int i = 0; i!= trajectory_filter.joint_trajectory.points.size(); ++i){
		JointTrajectoryPoint joint_series;
		joint_series.accelerations = trajectory_filter.joint_trajectory.points[i].accelerations;
		joint_series.positions = trajectory_filter.joint_trajectory.points[i].positions;
		joint_series.velocities = trajectory_filter.joint_trajectory.points[i].velocities;
		joint_series.time_from_start = trajectory_filter.joint_trajectory.points[i].time_from_start.toSec();
		goal_trajectory.points.push_back(joint_series);
	}

}



bool MoveGroup::checkCollision(const std::string& group_name, const move_group::JointState& joint_state)
{
  auto& psm = impl_->moveGroupExe->getContext()->planning_scene_monitor_;
  auto& planning_scene_ = psm->getPlanningScene();
  auto& robot_model = psm->getRobotModel();
  moveit::core::RobotState robot_state(robot_model);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = group_name;
  res.collision = false;

  planning_scene_monitor::LockedPlanningSceneRO lscene(psm);  // lock the scene so that it
                                                              // does not modify the world
                                                              // representation while
                                                              // isStateValid() is called
  robot_state.setJointGroupPositions(group_name, joint_state.positions);
  
  const collision_detection::AllowedCollisionMatrix* acm = &planning_scene_->getAllowedCollisionMatrix();
  
  if (acm)
    planning_scene_->checkCollisionUnpadded(req, res, robot_state, *acm);
  else
    planning_scene_->checkCollisionUnpadded(req, res, robot_state);
  
  if (res.collision || !planning_scene_->isStateFeasible(robot_state, false))
  {
    // call the same functions again, in verbose mode, to show what issues have been detected
    planning_scene_->isStateFeasible(robot_state, true);
    req.verbose = true;
    res.clear();
    if (acm)
      planning_scene_->checkCollisionUnpadded(req, res, robot_state, *acm);
    else
      planning_scene_->checkCollisionUnpadded(req, res, robot_state);
  }

  if (res.collision)
  {
    return true;
  }
  
  return false;
}

}  // namespace move_group
