//
// Created by fan on 2021/7/6.
//

#pragma once

#include "moveit/move_group/visibility_control.hpp"
#include <functional>
#include <memory>
#include <map>
#include<string>
#include<vector>
namespace move_group
{
struct Pose
{
  double x, y, z;
  double q1, q2, q3, q4;
};
typedef Pose RobotTarget;

struct JointState
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;
};

struct JointTrajectoryPoint
{
  double time_from_start;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct JointTrajectory
{
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
};

struct PlanIt
{
  struct Request
  {
    JointState joint_start;  // size()==0,以当前点开始

    Pose robot_target;  // 目标点
    std::vector<Pose> waypoints;
    std::string group_name;
    std::string pipeline_id;
    double allowed_planning_time;
  };

  struct Response
  {
    double planning_time;
    std::string group_name;
    JointTrajectory joint_trajectory;
  };

  Request request;
  Response response;
};

struct ProcessCollisionObjectParam
{
  enum Operation
  {
    ADD = 0,
    REMOVE = 1,
    APPEND = 2,
    MOVE = 3,
  };

  enum Type
  {
    BOX = 1u,       // 盒   dimensions: 3维  长  宽  高
    SPHERE = 2u,    // 球   dimensions: 1维  半径
    CYLINDER = 3u,  // 圆柱 dimensions: 2维  高  半径
    CONE = 4u,      // 圆锥 dimensions: 2维  高  半径
  };

  std::string id;        // id：障碍物名字
  std::string frame_id;  // 四元素位姿的tf基
  Pose pose;             // 四元素位姿

  int32_t operation;  // 操作：ADD、REMOVE、APPEND、MOVE
  int32_t type;       // 障碍物类型：盒、球、圆柱、圆锥
  std::vector<double> dimensions;
};

using GetJointStateCallback = std::function<bool(move_group::JointState& jointState)>;
using JointExecCallback = std::function<bool(const move_group::JointState& jointState)>;
using TrajectoryExecCallback = std::function<bool(const move_group::JointTrajectory& jointTrajectory, bool& cancel)>;
using ScreenUpdateCallback = std::function<void(void)>;

class MOVE_GROUP_PUBLIC MoveGroup
{
public:
  bool init(bool debug = false);

  ///  注册回调函数
  void registerGetJointStateCallback(const GetJointStateCallback& getJointStateCallback);
  void registerJointExecCallback(const JointExecCallback& jointExecCallback);                 //执行关节回调
  void registerTrajectoryExecCallback(const TrajectoryExecCallback& trajectoryExecCallback);  //执行轨迹回调
  void registerScreenUpdateCallback(const ScreenUpdateCallback& screenUpdateCallback);        //场景更新回调

  /// 轨迹碰撞检测
  bool checkCollision(const std::string& group_name, const move_group::JointTrajectory& jointTrajectory,
                      int current_index, int& collision_index);  //场景更新后计算轨迹碰撞

  // 计算机器人自碰撞距离
  double getDistanceRobotSelf(const move_group::JointState& jointState, std::string& out_link1, std::string& out_link2);
  // 规划到指定状态
  bool planToState(const move_group::JointState& startState, const move_group::JointState& goalState,
                   JointTrajectory& out_joint_trajectory, const std::string& group_name = "all",
                   const std::string& pipeline_id = "ompl", double allowed_planning_time = 5.0);

  ///  点云处理
  /// \param data 数据区
  /// \param point_size 每个点占用数据空间的大小
  /// \param point_num  点的个数
  void inComingPointCloud(const char* data, int point_size, int point_num, const std::string& frame_id,
                          int x_offset = 0, int y_offset = 4, int z_offset = 8);

  ///  常规接口
  bool publishJointState(const move_group::JointState& jointState);
  const std::vector<std::string>& getControllerJointNames();  //已关联motion_id的关节名
  const std::vector<unsigned int>& getControllerMotionIds();  //已关联关节名的motion_id
  bool getJointNameByMotionId(const std::vector<unsigned int>& motion_ids, std::vector<std::string>& joint_names);
  bool getMotionIdByJointName(const std::vector<std::string>& joint_names, std::vector<unsigned int>& motion_ids);

  bool getGroupJointTransforms(const std::string& group, std::map<std::string, move_group::Pose>& poses);
  bool getGroupGlobalLinkTransforms(const std::string& group, std::map<std::string, move_group::Pose>& poses);
  bool planIt(move_group::PlanIt::Request& request, move_group::PlanIt::Response& response);
  void processCollisionObject(const move_group::ProcessCollisionObjectParam& param);
  bool checkJointStateValid(const move_group::JointState& jointState);
  void clearCollisionObject();

  // add 2022-06-20
  void setTrajectoryFilterParam(double max_velocity_scaling_factor, double max_acceleration_scaling_factor);
  void trajectoryBlend(const std::string &group_name, const std::vector<JointTrajectory> &multi_trajectory_series,
		  JointTrajectory &goal_trajectory);
  
  // add 2023-12-27
  bool checkCollision(const std::string& group_name, const move_group::JointState& joint_state);  //判断状态点是否发生碰撞

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace move_group
