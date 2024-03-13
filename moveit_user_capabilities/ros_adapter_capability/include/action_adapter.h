//
// Created by yjh on 6/28/21.
//

#ifndef CODEIT_MOVEIT_ACTION_ADAPTER_H
#define CODEIT_MOVEIT_ACTION_ADAPTER_H

#include <memory>
#include <vector>

struct ActionAdapterBase
{
  void shutdown()
  {
    shut_down_ = true;
  }

  static bool shut_down_;
};

class ActionAdapter
{
public:
  void init();

private:
  std::vector<std::unique_ptr<ActionAdapterBase>> action_adapters_;
};

#endif  // CODEIT_MOVEIT_ACTION_ADAPTER_H
