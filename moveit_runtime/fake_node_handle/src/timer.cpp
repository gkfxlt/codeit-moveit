//
// Created by Administrator on 2021/6/21.
//

#include "fake_node_handle/timer.h"

using namespace _ros;

TimerCenter* TimerCenter::Instance()
{
  static TimerCenter instance;
  return &instance;
}

Timer TimerCenter::createTimer(ros::Duration period, TimerCB cb, bool oneshot, bool autostart)
{
  timers_.emplace_back(new Timer(period, cb, oneshot, autostart));
  return *timers_.back();
}

WallTimer TimerCenter::createWallTimer(ros::WallDuration period, TimerCB cb, bool oneshot, bool autostart)
{
  wall_timers_.emplace_back(new WallTimer(period, cb, oneshot, autostart));
  return *wall_timers_.back();
}
