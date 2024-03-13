//
// Created by yjh on 6/28/21.
//

#ifndef CODEIT_MOVEIT_TOPIC_ADAPTER_H
#define CODEIT_MOVEIT_TOPIC_ADAPTER_H

#include "ros/node_handle.h"

struct TopicAdapterBase
{
  void shutdown()
  {
    shut_down_ = true;
  }

  static bool shut_down_;
};

class TopicAdapter
{
public:
  void init(ros::NodeHandle* pNode);
  void shutdown();

private:
  std::vector<std::unique_ptr<TopicAdapterBase>> topic_adapters_;
};

#endif  // CODEIT_MOVEIT_TOPIC_ADAPTER_H
