/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"
#include "moveit/robot_model/robot_model.h"
#include <kdl_parser/kdl_parser.hpp>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include "stdboost/string_operation.h"
#include "log_helper/log_helper.h"

using namespace robot_state_publisher;

struct RobotStatePublisher::Impl
{
  urdf::ModelInterfaceSharedPtr model;
  KDL::Tree tree;
  MimicMap mimic;
  std::unique_ptr<JointStateListener> listener;

  explicit Impl(const moveit::core::RobotModelConstPtr& robot_model)
  {
    model = robot_model->getURDF();

    if (!kdl_parser::treeFromUrdfModel(*model, tree))
    {
      ROS_ERROR("Failed to extract kdl tree from xml robot description");
      return;
    }

    for (auto & joint : model->joints_)
    {
      if (joint.second->mimic)
      {
        mimic.insert(std::make_pair(joint.first, joint.second->mimic));
      }
    }

    listener = std::make_unique<JointStateListener>(tree, mimic);
  }
};

RobotStatePublisher::RobotStatePublisher(const moveit::core::RobotModelConstPtr& robot_model)
{
  impl_ = std::make_shared<Impl>(robot_model);
}

void RobotStatePublisher::addFixedTransforms(const std::string& transform)
{
  auto t = stdboost::split(transform, " ");
  if (t.size() == 8)
  {
    FixedTransformsEulerAngles tf;
    tf.x = strtod(t[0].c_str(), nullptr);
    tf.y = strtod(t[1].c_str(), nullptr);
    tf.z = strtod(t[2].c_str(), nullptr);
    tf.yaw = strtod(t[3].c_str(), nullptr);
    tf.pitch = strtod(t[4].c_str(), nullptr);
    tf.roll = strtod(t[5].c_str(), nullptr);
    tf.frame_id = t[6];
    tf.child_frame_id = t[7];
    addFixedTransforms(tf);
  }
  else if (t.size() == 9)
  {
    FixedTransformsQuaternion tf;
    tf.x = strtod(t[0].c_str(), nullptr);
    tf.y = strtod(t[1].c_str(), nullptr);
    tf.z = strtod(t[2].c_str(), nullptr);
    tf.qx = strtod(t[3].c_str(), nullptr);
    tf.qy = strtod(t[4].c_str(), nullptr);
    tf.qz = strtod(t[5].c_str(), nullptr);
    tf.qw = strtod(t[6].c_str(), nullptr);
    tf.frame_id = t[7];
    tf.child_frame_id = t[8];
    addFixedTransforms(tf);
  }
}

void RobotStatePublisher::addFixedTransforms(const std::vector<std::string>& transforms)
{
  for (auto& transform : transforms)
  {
    addFixedTransforms(transform);
  }
}

void RobotStatePublisher::addFixedTransforms(const FixedTransformsEulerAngles& transform)
{
  geometry_msgs::TransformStamped tf_transform;
  tf_transform.transform.translation.x = transform.x;
  tf_transform.transform.translation.y = transform.y;
  tf_transform.transform.translation.z = transform.z;

  tf2::Quaternion quat;
  quat.setRPY(transform.roll, transform.pitch, transform.yaw);
  tf_transform.transform.rotation.x = quat.x();
  tf_transform.transform.rotation.y = quat.y();
  tf_transform.transform.rotation.z = quat.z();
  tf_transform.transform.rotation.w = quat.w();

  tf_transform.header.stamp = ros::Time::now();
  tf_transform.header.frame_id = transform.frame_id;
  tf_transform.child_frame_id = transform.child_frame_id;

  impl_->listener->addFixedTransforms(tf_transform);
}

void RobotStatePublisher::addFixedTransforms(const FixedTransformsQuaternion& transform)
{
  geometry_msgs::TransformStamped tf_transform;
  tf_transform.transform.translation.x = transform.x;
  tf_transform.transform.translation.y = transform.y;
  tf_transform.transform.translation.z = transform.z;

  tf_transform.transform.rotation.x = transform.qx;
  tf_transform.transform.rotation.y = transform.qy;
  tf_transform.transform.rotation.z = transform.qz;
  tf_transform.transform.rotation.w = transform.qw;

  tf_transform.header.stamp = ros::Time::now();
  tf_transform.header.frame_id = transform.frame_id;
  tf_transform.child_frame_id = transform.child_frame_id;

  impl_->listener->addFixedTransforms(tf_transform);
}

RobotStatePublisher::~RobotStatePublisher()
{
}
