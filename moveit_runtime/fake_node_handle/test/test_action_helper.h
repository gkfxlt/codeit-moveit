//
// Created by fan on 2021/7/2.
//

#ifndef CODEIT_MOVEIT_TEST_ACTION_HELPER_H
#define CODEIT_MOVEIT_TEST_ACTION_HELPER_H

#include "fake_node_handle/fake_node_handle.h"
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include "ros/rate.h"

using namespace _ros;

class ActionTestServer {
public:
  ActionTestServer() {
    server_.reset(new _ros::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction>(
        "execute_trajectory",
        std::bind(&ActionTestServer::executeCallback, this, std::placeholders::_1), false));

    server_->registerPreemptCallback(
        std::bind(&ActionTestServer::preemptCallback, this));
    server_->start();
  }

  void executeCallback(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal) {
    ros::Rate rate(2);
    for (int i = 0; i < 6; ++i) {
      std::cerr << "executeCallback..." << std::endl;

      moveit_msgs::ExecuteTrajectoryFeedback feedback;
      feedback.state = "555";
      server_->publishFeedback(feedback);
      rate.sleep();

      if(i == 3)
      {
        moveit_msgs::ExecuteTrajectoryResult result;
        server_->setSucceeded(result, "test setSucceeded...");
      }
    }
  }

  void preemptCallback() {
    std::cerr << "preemptCallback..." << std::endl;
  }

private:
  std::unique_ptr<_ros::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction>> server_;
};

class ActionTestClient {
public:
  ActionTestClient() {
    client_.reset(new SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>("execute_trajectory"));
  }

  void sendGoal(const moveit_msgs::ExecuteTrajectoryGoal &goal) {
    client_->sendGoal(goal, std::bind(&ActionTestClient::done, this, std::placeholders::_1, std::placeholders::_2),
                      std::bind(&ActionTestClient::active, this),
                      std::bind(&ActionTestClient::feedback, this, std::placeholders::_1));
  }

  void waitFinish()
  {
    client_->waitForResult();
  }

private:
  void done(const SimpleClientGoalState &state, const moveit_msgs::ExecuteTrajectoryResultConstPtr &result) {
    std::cerr << "SimpleDoneCallback" << std::endl;
  }

  void active() {
    std::cerr << "SimpleActiveCallback" << std::endl;
  }

  void feedback(const moveit_msgs::ExecuteTrajectoryFeedbackConstPtr &feedback) {
    std::cerr << "SimpleFeedbackCallback: " << feedback->state << std::endl;
  }

private:
  std::unique_ptr<SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> client_;
};


#endif  // CODEIT_MOVEIT_TEST_ACTION_HELPER_H
