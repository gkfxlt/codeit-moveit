
#pragma once

#include <iostream>
#include "libBase.h"
#include "fake_node_handle/fake_node_handle.h"

class WIN_EXPORT LibB : public LibBase
{
public:
  LibB()
  {
    _ros::NodeHandle nh;
    nh.setParam("/lib/name", "LibB");
  }
};

#include "plugin_loader/plugin_loader.hpp"
PLUGIN_LOADER_REGISTER_CLASS(LibB, LibBase);