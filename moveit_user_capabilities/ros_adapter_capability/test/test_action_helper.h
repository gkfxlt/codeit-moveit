//
// Created by fan on 2021/7/5.
//

#ifndef CODEIT_MOVEIT_TEST_ACTION_HELPER_H
#define CODEIT_MOVEIT_TEST_ACTION_HELPER_H

#include <memory>

class ActionTestServer
{
public:
  ActionTestServer();
  ~ActionTestServer();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // CODEIT_MOVEIT_TEST_ACTION_HELPER_H
