//
// Created by Administrator on 2021/6/7.
//

#pragma once

#ifndef CODEIT_MOVEIT_TOPIC_H
#define CODEIT_MOVEIT_TOPIC_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <mutex>

#include "fake_node_handle/topic_advertise_options.h"
#include "fake_node_handle/topic_subscribe_options.h"

#include "fake_node_handle/visibility_control.hpp"

namespace _ros
{
//订阅者
class FAKE_NODE_HANDLE_PUBLIC Subscriber
{
public:
  Subscriber() = default;

  Subscriber(const std::string& topic, SubscribeOptions* subscribeOptions)
    : topic_(topic), shutdown_(false), subscribeOptions_(subscribeOptions)
  {
  }

  explicit operator bool() const
  {
    return subscribeOptions_ != nullptr;
  }

  const std::string& getTopic() const
  {
    return topic_;
  }

  void shutdown();

private:
  bool shutdown_{ false };
  std::string topic_{ "" };
  SubscribeOptions* subscribeOptions_{ nullptr };
};

//发布者
class FAKE_NODE_HANDLE_PUBLIC Publisher
{
public:
  Publisher() = default;

  explicit Publisher(const std::string& topic, bool async = false) : topic_(topic), async_(async), shutdown_(false)
  {
  }

  template <typename Msg>
  void publish(const Msg& msg) const
  {
    if (async_)
    {
      static Msg s_msg;
      s_msg = msg;
      publish(ros::message_traits::md5sum<Msg>(), &s_msg);
    }
    else
    {
      publish(ros::message_traits::md5sum<Msg>(), &msg);
    }
  }

  void publish(const std::string& topic_md5sum, const void* param) const;

  void shutdown()
  {
    shutdown_ = true;
  }

private:
  bool async_;
  bool shutdown_;
  std::string topic_;
};

class SubjectCenter
{
public:
  static SubjectCenter* Instance();

  Publisher advertise(AdvertiseOptions& ops);
  Subscriber subscribe(SubscribeOptions& ops);

  void publish(std::string topic, const std::string& topic_md5sum, const void* param, bool async);

  void unsubscribe(const std::string& topic, SubscribeOptions* subscribeOptions);

  void clearAll();

private:
  std::mutex mutex_;
  SubjectCenter() = default;
  std::map<std::string, std::vector<std::shared_ptr<AdvertiseOptions>>> publishers_;
  std::map<std::string, std::vector<std::shared_ptr<SubscribeOptions>>> subscribers_;
};

}  // namespace _ros

#endif  // CODEIT_MOVEIT_TOPIC_H
