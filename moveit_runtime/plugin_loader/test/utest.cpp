/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "gtest/gtest.h"
#include "plugin_loader/plugin_loader.hpp"
#include "base.hpp"
#include <iostream>
#include <set>

TEST(PluginLoader, PluginLoader)
{
  plugin_loader::PluginLoader loader("../lib/plugin_loader_test_plugins", false);

  auto names = loader.getAvailableClasses<Base>();
  std::set<std::string> names_set(names.begin(), names.end());

  EXPECT_TRUE(names_set.find("animal::Cat") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Cow") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Dog") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Duck") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Sheep") != names_set.end());

  for (const auto& name : names)
  {
    std::cout << name << " --> say:";
    loader.createInstance<Base>(name)->saySomething();
  }
}

TEST(NodeHandle, ClassLoader)
{
  pluginloader::ClassLoader<Base> loader(" ../lib/plugin_loader_test_plugins ../lib/test_lib_a ");

  auto names = loader.getAvailableClasses();
  std::set<std::string> names_set(names.begin(), names.end());

  EXPECT_TRUE(names_set.find("animal::Cat") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Cow") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Dog") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Duck") != names_set.end());
  EXPECT_TRUE(names_set.find("animal::Sheep") != names_set.end());

  for (const auto& name : names)
  {
    std::cout << name << " --> say:";
    loader.createUniqueInstance(name)->saySomething();
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}