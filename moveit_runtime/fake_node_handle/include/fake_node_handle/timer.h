//
// Created by Administrator on 2021/6/21.
//

#pragma once

#include <ros/time.h>
#include <thread>
#include <functional>
#include <atomic>

#include "fake_node_handle/visibility_control.hpp"

namespace _ros
{
using TimerCB = std::function<void()>;

template <typename T, typename D>
class TimerBaseImp
{
public:
  TimerBaseImp()
  {
    callback_ = {};
    oneshot_ = false;
    autostart_ = true;
    stop_ = true;
  }

  TimerBaseImp(D period, TimerCB callback, bool oneshot, bool autostart)
  {
    period_ = period;
    callback_ = callback;
    oneshot_ = oneshot;
    autostart_ = autostart;
    stop_ = !autostart_;

    if (autostart)
    {
      start();
    }
  }

  ~TimerBaseImp()
  {
    stop();
  }

  void start()
  {
    stop();

    if (oneshot_)
    {
      callback_();
      return;
    }

    stop_ = false;
    thread_ = std::thread([this]() {
      while (!stop_)
      {
        auto next = T::now() + period_;

        callback_();

        T::sleepUntil(next);
      }
    });
  }

  void stop()
  {
    stop_ = true;
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  void setPeriod(const D& period, bool reset = true)
  {
    period_ = period;
  }

private:
  D period_;
  TimerCB callback_;
  bool oneshot_;
  bool autostart_;
  bool stop_;

  std::thread thread_;
};

template<typename T, typename D>
class TimerBase
{
public:
  TimerBase()
  {
    impl_ = std::make_shared<TimerBaseImp<T, D>>();
  }

  TimerBase(D period, TimerCB callback, bool oneshot, bool autostart)
  {
    impl_ = std::make_shared<TimerBaseImp<T, D>>(period, callback, oneshot, autostart);
  }

  TimerBase(const TimerBase& rhs)
  {
    impl_ = rhs.impl_;
  }

  virtual ~TimerBase()
  {
    impl_->stop();
  }

  void start()
  {
    impl_->start();
  }

  void stop()
  {
    impl_->stop();
  }

  void setPeriod(const D& period, bool reset = true)
  {
    impl_->setPeriod(period, reset);
  }

protected:
  std::shared_ptr<TimerBaseImp<T, D>> impl_;
};

class FAKE_NODE_HANDLE_PUBLIC Timer : public TimerBase<ros::Time, ros::Duration>
{
public:
  Timer() : TimerBase<ros::Time, ros::Duration>()
  {
  }

  Timer(ros::Duration period, TimerCB callback, bool oneshot, bool autostart)
    : TimerBase<ros::Time, ros::Duration>(period, callback, oneshot, autostart)
  {
  }
};

class FAKE_NODE_HANDLE_PUBLIC WallTimer : public TimerBase<ros::WallTime, ros::WallDuration>
{
public:
  WallTimer() : TimerBase<ros::WallTime, ros::WallDuration>()
  {
  }
  WallTimer(ros::WallDuration period, TimerCB callback, bool oneshot, bool autostart)
    : TimerBase<ros::WallTime, ros::WallDuration>(period, callback, oneshot, autostart)
  {
  }
};

// using Timer = TimerBase<ros::Time, ros::Duration>;
// using WallTimer = TimerBase<ros::WallTime, ros::WallDuration>;

class TimerCenter
{
public:
  static TimerCenter* Instance();

  Timer createTimer(ros::Duration period, TimerCB cb, bool oneshot, bool autostart);
  WallTimer createWallTimer(ros::WallDuration period, TimerCB cb, bool oneshot, bool autostart);

private:
  TimerCenter() = default;
  std::vector<std::shared_ptr<Timer>> timers_;
  std::vector<std::shared_ptr<WallTimer>> wall_timers_;
};

}  // namespace _ros
