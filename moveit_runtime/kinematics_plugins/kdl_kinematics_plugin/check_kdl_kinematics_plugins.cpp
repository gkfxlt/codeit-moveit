//
// Created by Administrator on 2021/6/18.
//

#include "include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h"

int main()
{
  auto plugin = std::make_shared<kdl_kinematics_plugin::KDLKinematicsPlugin>();

  return 0;
}