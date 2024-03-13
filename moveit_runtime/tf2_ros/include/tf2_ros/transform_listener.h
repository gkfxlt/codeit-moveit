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

#ifndef TF2_ROS_TRANSFORMLISTENER_H
#define TF2_ROS_TRANSFORMLISTENER_H

#include "std_msgs/Empty.h"
#include "tf2_msgs/TFMessage.h"

#include "tf2_ros/buffer.h"
#include "fake_node_handle/fake_node_handle.h"

namespace tf2_ros{

/** \brief This class provides an easy way to request and receive coordinate frame transform information.
 */
class TransformListener
{

public:
  /**@brief Constructor for transform listener */
  TransformListener(tf2::BufferCore& buffer);

private:

  /// Initialize this transform listener, subscribing, advertising services, etc.
  void init();

  /// Callback function for ros message subscription
  void subscription_callback(const tf2_msgs::TFMessage& msg);
  void static_subscription_callback(const tf2_msgs::TFMessage& msg);
  void subscription_callback_impl(const tf2_msgs::TFMessage& msg, bool is_static);

  _ros::Subscriber message_subscriber_tf_;
  _ros::Subscriber message_subscriber_tf_static_;
  tf2::BufferCore& buffer_;
  ros::Time last_update_;

};
}

#endif //TF_TRANSFORMLISTENER_H
