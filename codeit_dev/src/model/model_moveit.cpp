//
// Created by fan on 2021/7/29.
//

#include "codeit/model/model.hpp"
#include "codeit/model/model_moveit.h"
#include <functional>

#include "codeit/system/server.hpp"

namespace codeit::model
{
/// MoveitPlanner::Imp
struct MoveitPlanner::Imp
{
  move_group::MoveGroup move_group;

  bool init()
  {
    bool result = move_group.init();
    move_group.registerGetJointStateCallback(std::bind(&Imp::GetJointStateCallback, this, std::placeholders::_1));
    move_group.registerJointExecCallback(std::bind(&Imp::JointExecCallback, this, std::placeholders::_1));
    move_group.registerTrajectoryExecCallback(
        std::bind(&Imp::TrajectoryExecCallback, this, std::placeholders::_1, std::placeholders::_2));

    return result;
  }

  bool getCurrentJointState(move_group::JointState& jointState)
  {
    jointState.joint_names = move_group.getControllerJointNames();
    return GetJointStateCallback(jointState);
  }

  bool syncJointState()
  {
    move_group::JointState jointState;
    if(getCurrentJointState(jointState))
    {
      return move_group.publishJointState(jointState);
    }
    return false;
  }

  bool getValidMotionIdByJointName(const std::vector<std::string>& joint_names, std::vector<unsigned int>& motion_ids)
  {
    auto& cs = codeit::system::ControlSystem::instance();
    auto& motionPool = cs.controller().motionPool();

    if (!cs.running())
    {
      return false;
    }

    if (!move_group.getMotionIdByJointName(joint_names, motion_ids))
    {
      return false;
    }

    unsigned int max_index = *std::max_element(motion_ids.begin(), motion_ids.end());
    if (max_index >= motionPool.size())
    {
      return false;
    }
    return true;
  }

  bool GetJointStateCallback(move_group::JointState& jointState)
  {
    std::vector<unsigned int> motion_ids;
    if (!getValidMotionIdByJointName(jointState.joint_names, motion_ids))
    {
      return false;
    }

    jointState.positions.resize(jointState.joint_names.size());

    auto& motionPool = codeit::system::ControlSystem::instance().controller().motionPool();
    for (int i = 0; i < motion_ids.size(); ++i)
    {
      jointState.positions[i] = motionPool[motion_ids[i]].actualPos();
    }
    return true;
  }

  bool JointExecCallback(const move_group::JointState& jointState)
  {
    std::vector<unsigned int> motion_ids;
    if (!getValidMotionIdByJointName(jointState.joint_names, motion_ids))
    {
      return false;
    }

    auto& motionPool = codeit::system::ControlSystem::instance().controller().motionPool();
    for (int i = 0; i < motion_ids.size(); ++i)
    {
      motionPool[motion_ids[i]].setTargetPos(jointState.positions[i]);
    }
    return true;
  }

  bool TrajectoryExecCallback(const move_group::JointTrajectory& jointTrajectory, bool& cancel)
  {
    std::vector<unsigned int> motion_ids;
    if (!getValidMotionIdByJointName(jointTrajectory.joint_names, motion_ids))
    {
      return false;
    }

    char buff[256];
    std::string joint_series;
    std::string time_series;
    std::vector<std::string> cmds;
    auto& trajectory_points = jointTrajectory.points;
    auto trajectory_point_num = trajectory_points.size();
    for (int i = 0; i < trajectory_point_num; ++i)
    {
      auto& j_time = trajectory_points[i].time_from_start;
      std::vector<double> j_pos(motion_ids.size());
      for (int j = 0; j < motion_ids.size(); ++j)
      {
        j_pos[motion_ids[j]] = trajectory_points[i].positions[j];
      }

      memset(buff, 0, sizeof(buff));
      sprintf(buff,
              "ReVarValue --type=jointtarget --name=j%d --value={%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f}", i,
              j_pos[0], j_pos[1], j_pos[2], j_pos[3], j_pos[4], j_pos[5], j_pos[6], j_pos[7], j_pos[8], j_pos[9]);

      cmds.emplace_back(buff);

      joint_series.append("j" + std::to_string(i) + ",");
      time_series.append(std::to_string(j_time) + ",");
    }
    joint_series.pop_back();
    time_series.pop_back();

    {
      auto len = joint_series.size() + time_series.size() + 128;
      auto buff2 = new char[len];
      memset(buff2, 0, len);
      sprintf(buff2, "ServoSeries --joint_set={%s} --time={%s} --scale=1", joint_series.c_str(), time_series.c_str());
      cmds.emplace_back(buff2);
      delete[] buff2;
    }

    auto& cs = codeit::system::ControlSystem::instance();
    for (auto& cmd : cmds)
    {
      std::cerr << cmd.c_str() << std::endl;
      cs.executeCmd(cmd.c_str());
    }
    return true;
  }
};


MoveitPlanner* MoveitPlanner::getInstance() {
    static MoveitPlanner moveit_planner;
    static MoveitPlanner* ret_planner = nullptr;
    static once_flag flag;
    std::call_once(flag, [&]() {
            if (moveit_planner.imp_->init()) {
                ret_planner = &moveit_planner;
            }
        }
    );

    return ret_planner;
}

move_group::MoveGroup& MoveitPlanner::getMoveGroup()
{
  return imp_->move_group;
}

bool MoveitPlanner::getCurrentJointState(move_group::JointState& jointState)
{
  return imp_->getCurrentJointState(jointState);
}

bool MoveitPlanner::syncJointState()
{
  return imp_->syncJointState();
}

}  // namespace codeit::model