//
// Created by fan on 2021/7/5.
//

#include "test_action_helper.h"
#include "fake_node_handle/action.h"

#include "moveit_msgs/MoveGroupAction.h"

#include <unistd.h>

struct ActionTestServer::Impl
{
  Impl()
  {
    server_.reset(new _ros::SimpleActionServer<moveit_msgs::MoveGroupAction>(
        "move_group", std::bind(&Impl::executeCallback, this, std::placeholders::_1), false));

    server_->registerPreemptCallback(std::bind(&Impl::preemptCallback, this));
    server_->start();
  }

  void preemptCallback()
  {
    std::cerr << "preemptCallback..." << std::endl;
  }

  // void executeCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
  void executeCallback(const std::shared_ptr<const moveit_msgs::MoveGroupGoal>& goal)
  {
    for (int i = 0; i < 6; ++i)
    {
      std::cerr << "executeCallback..." << std::endl;

      moveit_msgs::MoveGroupFeedback feedback;
      feedback.state = "555";
      server_->publishFeedback(feedback);
      sleep(0.5);

      if (i == 3)
      {
        moveit_msgs::MoveGroupResult result;
        result.error_code.val = 555;
        result.planning_time = 555;

        server_->setSucceeded(result, "test setSucceeded...");
        break;
      }
    }
  }

  std::unique_ptr<_ros::SimpleActionServer<moveit_msgs::MoveGroupAction>> server_;
};

ActionTestServer::ActionTestServer()
{
  impl_.reset(new Impl);
}

ActionTestServer::~ActionTestServer()
{
}
