
#pragma once

#include <iostream>
#include "libBase.h"
#include "fake_node_handle/fake_node_handle.h"

class WIN_EXPORT LibA : public LibBase
{
public:
  LibA()
  {
    _ros::NodeHandle nh;
    nh.setParam("/lib/name", "LibA");
  }
};

#include "plugin_loader/plugin_loader.hpp"
PLUGIN_LOADER_REGISTER_CLASS(LibA, LibBase);