/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#include "tf2_ros/transform_listener.h"
#include "log_helper/log_helper.h"
#include "moveit_runtime/moveit_runtime.h"

using namespace tf2_ros;

TransformListener::TransformListener(tf2::BufferCore& buffer)
    :buffer_(buffer)
{
  init();
}

void TransformListener::init()
{
  _ros::NodeHandle nh;
  message_subscriber_tf_ = nh.subscribe(moveit_runtime::topic::TF_TOPIC_NAME, 100, &TransformListener::subscription_callback, this);
  message_subscriber_tf_static_ = nh.subscribe(moveit_runtime::topic::TF_STATIC_TOPIC_NAME, 100, &TransformListener::static_subscription_callback, this);
}

void TransformListener::subscription_callback(const tf2_msgs::TFMessage& msg)
{
  subscription_callback_impl(msg, false);
}

void TransformListener::static_subscription_callback(const tf2_msgs::TFMessage& msg)
{
  subscription_callback_impl(msg, true);
}

void TransformListener::subscription_callback_impl(const tf2_msgs::TFMessage& msg, bool is_static)
{
  ros::Time now = ros::Time::now();
  if(now < last_update_){
    ROS_WARN_STREAM("Detected jump back in time of " << (last_update_ - now).toSec() << "s. Clearing TF buffer.");
    buffer_.clear();
  }
  last_update_ = now;

  const tf2_msgs::TFMessage& msg_in = msg; // *(msg_evt.getConstMessage());
  std::string authority = ""; // msg_evt.getPublisherName(); // lookup the authority
  for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
  {
    try
    {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    }

    catch (tf2::TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
};




