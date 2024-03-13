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

#include <moveit/move_group_capability/move_group_capability.h>
//#include <pluginlib/class_loader.hpp>
#include <plugin_loader/plugin_loader.hpp>
//#include <boost/algorithm/string/join.hpp>

namespace move_group{
class A{
public:
  A(){
    std::string capability_lib_path = "../lib/moveit_move_group_default_capabilities ../lib/ros_adapter_capability";

    std::shared_ptr<pluginloader::ClassLoader<MoveGroupCapability> > capability_plugin_loader;
    capability_plugin_loader.reset(new pluginloader::ClassLoader<MoveGroupCapability>(capability_lib_path));

    //pluginloader::ClassLoader<move_group::MoveGroupCapability> capability_plugin_loader(capability_lib_path);

    std::cout << "Available capabilities:" << std::endl;
    for (auto cc : capability_plugin_loader->getDeclaredClasses())
    {
      std::cout << cc << std::endl;
    }
  }
};
}


int main(int /*argc*/, char** /*argv*/)
{
  //  try
  //  {

  move_group::A a;
//  std::string capability_lib_path = "../lib/moveit_move_group_default_capabilities";
//
//  std::shared_ptr<pluginloader::ClassLoader<move_group::MoveGroupCapability> > capability_plugin_loader;
//  capability_plugin_loader.reset(new pluginloader::ClassLoader<move_group::MoveGroupCapability>(capability_lib_path));
//
//  //pluginloader::ClassLoader<move_group::MoveGroupCapability> capability_plugin_loader(capability_lib_path);
//
//  std::cout << "Available capabilities:" << std::endl;
//  for (auto cc : capability_plugin_loader->getDeclaredClasses())
//  {
//    std::cout << cc << std::endl;
//  }

  //    std::cout << "Available capabilities:\n"
  //              << std::algorithm::join(capability_plugin_loader.getDeclaredClasses(), "\n") << std::endl;
  //  }
  //  catch (pluginlib::PluginlibException& ex)
  //  {
  //    std::cerr << "Exception while creating plugin loader for move_group capabilities: " << ex.what() << std::endl;
  //  }

  return 0;
}
