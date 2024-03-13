//
// Created by yjh on 6/28/21.
//

#ifndef CODEIT_MOVEIT_SERVICE_ADAPTER_H
#define CODEIT_MOVEIT_SERVICE_ADAPTER_H

#include "ros/node_handle.h"

struct ServiceAdapterBase
{
  void shutdown()
  {
    shut_down_ = true;
  }

  static bool shut_down_;
};

class ServiceAdapter
{
public:
  void init(ros::NodeHandle* pNode);

private:
  std::vector<std::unique_ptr<ServiceAdapterBase>> service_adapters_;
};

#endif  // CODEIT_MOVEIT_SERVICE_ADAPTER_H
