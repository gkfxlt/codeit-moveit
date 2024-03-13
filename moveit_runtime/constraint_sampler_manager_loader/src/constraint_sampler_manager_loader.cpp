/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <plugin_loader/plugin_loader.hpp>
#include <memory>

#include "fake_node_handle/fake_node_handle.h"
#include "log_helper/log_helper.h"

namespace constraint_sampler_manager_loader
{
class ConstraintSamplerManagerLoader::Helper
{
public:
  Helper(const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  {
    std::string constraint_samplers_lib_path;
    constraint_sampler_plugin_loader_.reset(
        new pluginloader::ClassLoader<constraint_samplers::ConstraintSamplerAllocator>(constraint_samplers_lib_path));

    std::string constraint_samplers;

    if (nh_.getParam("/move_group/constraint_samplers", constraint_samplers))
    {
      auto samplers = stdboost::resplit(constraint_samplers);
      for(const auto& sampler : samplers)
      {
        constraint_samplers::ConstraintSamplerAllocatorPtr csa =
            constraint_sampler_plugin_loader_->createUniqueInstance(sampler);
        csm->registerSamplerAllocator(csa);

        ROS_INFO("Loaded constraint sampling plugin %s", sampler.c_str());
      }
    }

  }

private:
  _ros::NodeHandle nh_;
  std::unique_ptr<pluginloader::ClassLoader<constraint_samplers::ConstraintSamplerAllocator> >
      constraint_sampler_plugin_loader_;
};

ConstraintSamplerManagerLoader::ConstraintSamplerManagerLoader(
    const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  : constraint_sampler_manager_(
        csm ? csm :
              constraint_samplers::ConstraintSamplerManagerPtr(new constraint_samplers::ConstraintSamplerManager()))
  , impl_(new Helper(constraint_sampler_manager_))
{
}
}  // namespace constraint_sampler_manager_loader
