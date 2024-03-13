//
// Created by fan on 2021/7/2.
//

#ifndef CODEIT_MOVEIT_TEST_TOPIC_HELPER_H
#define CODEIT_MOVEIT_TEST_TOPIC_HELPER_H

#include "std_msgs/String.h"


class TopicTestClass
{
public:
  TopicTestClass()
  {
    _ros::NodeHandle nh;
    sub_ = nh.subscribe("test_topic", 1, &TopicTestClass::callback, this);
  }

  std::string getRecivedMsg()
  {
    return msg_;
  }

private:
  void callback(const std_msgs::String& msg)
  {
    msg_ = msg.data;
  }

  _ros::Subscriber sub_;
  std::string msg_;
};

#endif  // CODEIT_MOVEIT_TEST_TOPIC_HELPER_H
