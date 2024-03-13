//
// Created by fan on 2021/7/16.
//

#include "joint_state_publisher.h"
#include "moveit/robot_model/robot_model.h"

#include <thread>
#include "ros/rate.h"
#include "ros/time.h"
#include "fake_node_handle/fake_node_handle.h"

#include "sensor_msgs/JointState.h"
#include "moveit_runtime/moveit_runtime.h"

struct FreeJointsMore
{
  const moveit::core::JointModel* self;
  int joint_index;
};

struct MimicJointsMore
{
  const moveit::core::JointModel* self;
  int joint_index;

  const moveit::core::JointModel* mimic;
  double mimic_factor;
  double mimic_offset;
};

struct JointStatePublisher::Impl
{
  bool stop_;
  std::mutex mutex_;
  std::thread thread_;

  sensor_msgs::JointState msg_;
  _ros::Publisher joint_state_publisher_;
  _ros::Subscriber controller_joint_state_subscriber_;

  // key:name value:index
  std::map<std::string, FreeJointsMore> free_joints;
  std::map<std::string, MimicJointsMore> mimc_joints;

  Impl() : stop_(false)
  {
  }

  ~Impl()
  {
    stop_ = true;
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model)
  {
    msg_.name.clear();
    msg_.position.clear();
    msg_.velocity.clear();
    msg_.effort.clear();

    auto& jms = robot_model->getJointModels();
    for (auto& jm : jms)
    {
      if (jm->getType() == moveit::core::JointModel::FIXED || jm->getType() == moveit::core::JointModel::FLOATING ||
          jm->getType() == moveit::core::JointModel::PLANAR)
      {
        continue;
      }

      auto& name = jm->getName();
      int joint_index = msg_.name.size();
      if (jm->getMimic() == nullptr)
      {
        FreeJointsMore more{};
        more.self = jm;
        more.joint_index = joint_index;

        free_joints[name] = more;
      }
      else
      {
        MimicJointsMore more{};
        more.self = jm;
        more.joint_index = joint_index;
        more.mimic = jm->getMimic();
        more.mimic_factor = jm->getMimicFactor();
        more.mimic_offset = jm->getMimicOffset();

        mimc_joints[name] = more;
      }

      msg_.name.push_back(name);
    }

    auto size = msg_.name.size();
    msg_.position.resize(size);
    msg_.velocity.resize(size);
    msg_.effort.resize(size);

    // sub and pub
    _ros::NodeHandle nh;
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>(moveit_runtime::topic::JOINT_STATES_TOPIC_NAME);
    controller_joint_state_subscriber_ =
        nh.subscribe(moveit_runtime::topic::CONTROLLER_JOINT_STATES_TOPIC_NAME, 1, &Impl::callback, this);
    thread_ = std::thread(&Impl::loop, this);
  }

  void callback(const std::shared_ptr<const sensor_msgs::JointState>& msg)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    auto num = msg->name.size();
    bool has_position = msg->position.size() == num;
    bool has_velocity = msg->velocity.size() == num;
    bool has_effort = msg->effort.size() == num;

    if ((num == 0) || !has_position)
    {
      return;
    }

    for (int i = 0; i < num; ++i)
    {
      auto& name = msg->name[i];
      if (free_joints.find(name) == free_joints.end())
      {
        continue;
      }

      {
        int index = free_joints[name].joint_index;
        msg_.position[index] = msg->position[i];

        if (has_velocity)
        {
          msg_.velocity[index] = msg->velocity[i];
        }

        if (has_effort)
        {
          msg_.effort[index] = msg->effort[i];
        }
      }

      for (auto& mj : mimc_joints)
      {
        if (mj.second.mimic == free_joints[name].self)
        {
          int joint_index = mj.second.joint_index;
          msg_.position[joint_index] = msg->position[i] * mj.second.mimic_factor + mj.second.mimic_offset;

          if (has_velocity)
          {
            msg_.velocity[joint_index] = msg->velocity[i] * mj.second.mimic_factor;
          }

          if (has_effort)
          {
            msg_.effort[joint_index] = msg->effort[i];
          }
        }
      }
    }
  }

  void loop()
  {
    ros::Rate rate(10);

    while (!stop_)
    {
      {
        std::scoped_lock<std::mutex> lock(mutex_);
        msg_.header.stamp = ros::Time::now();
        joint_state_publisher_.publish(msg_);
      }
      rate.sleep();
    }
  }
};

JointStatePublisher::JointStatePublisher(const moveit::core::RobotModelConstPtr& robot_model)
{
  impl_ = std::make_shared<Impl>();
  impl_->init(robot_model);
}
