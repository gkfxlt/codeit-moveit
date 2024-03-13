//
// Created by Administrator on 2021/6/16.
//
#include <future>
#include "fake_node_handle/topic.h"

using namespace _ros;

void Subscriber::shutdown()
{
  SubjectCenter::Instance()->unsubscribe(topic_, subscribeOptions_);
  shutdown_ = true;
}

void Publisher::publish(const std::string& topic_md5sum, const void* param) const
{
  if(!shutdown_)
  {
    SubjectCenter::Instance()->publish(topic_, topic_md5sum, param, async_);
  }
}

SubjectCenter* SubjectCenter::Instance()
{
  static SubjectCenter instance;
  return &instance;
}
void SubjectCenter::publish(std::string topic, const std::string& topic_md5sum, const void* param, bool async)
{
  if (subscribers_.find(topic) != subscribers_.end())
  {
    auto& subs = subscribers_[topic];
    for (auto& sub : subs)
    {
      if (sub->md5sum == topic_md5sum)
      {
        if (!async)
        {
          sub->helper->call(const_cast<void*>(param));
        }
        else
        {
          std::async(std::launch::async, [&](){sub->helper->call(const_cast<void*>(param));});
        }
      }
    }
  }
}
Publisher SubjectCenter::advertise(AdvertiseOptions& ops)
{
  std::scoped_lock lock(mutex_);
  publishers_[ops.topic].emplace_back(new AdvertiseOptions(ops));
  return Publisher(ops.topic);
}
Subscriber SubjectCenter::subscribe(SubscribeOptions& ops)
{
  std::scoped_lock lock(mutex_);
  subscribers_[ops.topic].emplace_back(new SubscribeOptions(ops));
  return Subscriber(ops.topic, subscribers_[ops.topic].back().get());
}
void SubjectCenter::unsubscribe(const std::string& topic, SubscribeOptions* subscribeOptions)
{
  std::scoped_lock lock(mutex_);
  if (subscribers_.find(topic) != subscribers_.end())
  {
    auto& subs = subscribers_[topic];

    for (auto it = subs.begin(); it != subs.end(); ++it)
    {
      if (it->get() == subscribeOptions)
      {
        subs.erase(it);
        return;
      }
    }
  }
}

void SubjectCenter::clearAll()
{
    publishers_.clear();
    subscribers_.clear();
}
