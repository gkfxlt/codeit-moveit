//
// Created by Administrator on 2021/6/23.
//

#ifndef CODEIT_MOVEIT_ACTION_H
#define CODEIT_MOVEIT_ACTION_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <functional>

#include "fake_node_handle/action_helper.h"
#include "fake_node_handle/visibility_control.hpp"

namespace _ros
{
class FAKE_NODE_HANDLE_PUBLIC SimpleActionServerStatic
{
public:
  static void init(const std::string& action_name, const std::shared_ptr<ActionCallbackOptions>& ops);

  static void publishFeedback(const std::string& action_name, const void* feedback);

  static bool setGoalState(const std::string& action_name, SimpleClientGoalState& state, void* result,
                           const std::string& text);

  static void registerPreemptCallback(const std::string& action_name, const std::function<void()>& cb);
};

template <typename ActionSpec>
class SimpleActionServer
{
  using GoalConstPtr = typename AT<ActionSpec>::GoalConstPtr;
  using Result = typename AT<ActionSpec>::Result;
  using Feedback = typename AT<ActionSpec>::Feedback;
  using FeedbackConstPtr = typename AT<ActionSpec>::FeedbackConstPtr;
  using ExecuteCallback = typename AT<ActionSpec>::ExecuteCallback;

  using PreemptCallback = typename std::function<void()>;

public:
  SimpleActionServer(const std::string& action_name, ExecuteCallback execute_callback, bool auto_start)
  {
    action_name_ = action_name;
    auto ops = std::make_shared<ActionCallbackOptions>();
    ops->template init<ActionSpec>(action_name_, execute_callback, auto_start);
    SimpleActionServerStatic::init(action_name_, ops);
  }

  bool setAborted(const Result& result = Result(), const std::string& text = std::string(""))
  {
    SimpleClientGoalState state = SimpleClientGoalState(SimpleClientGoalState::StateEnum::ABORTED);
    return setGoalState(state, result, text);
  }

  bool setSucceeded(const Result& result = Result(), const std::string& text = std::string(""))
  {
    SimpleClientGoalState state = SimpleClientGoalState(SimpleClientGoalState::StateEnum::SUCCEEDED);
    return setGoalState(state, result, text);
  }

  bool setPreempted(const Result& result = Result(), const std::string& text = std::string(""))
  {
    SimpleClientGoalState state = SimpleClientGoalState(SimpleClientGoalState::StateEnum::PREEMPTED);
    return setGoalState(state, result, text);
  }

  bool setGoalState(SimpleClientGoalState& state, const Result& result, const std::string& text)
  {
    state_ = state;
    result_ = result;
    return SimpleActionServerStatic::setGoalState(action_name_, state_, static_cast<void*>(&result_), text);
  }

  void publishFeedback(const FeedbackConstPtr& feedback)
  {
    publishFeedback(*feedback);
  }

  void publishFeedback(const Feedback& feedback)
  {
    const void* p = static_cast<const void*>(&feedback);
    SimpleActionServerStatic::publishFeedback(action_name_, const_cast<void*>(p));
  }

  void registerPreemptCallback(const PreemptCallback& cb)
  {
    SimpleActionServerStatic::registerPreemptCallback(action_name_, cb);
  }

  void start()
  {
  }

  void shutdown()
  {
  }

private:
  std::string action_name_;
  SimpleClientGoalState state_{ SimpleClientGoalState::PENDING };
  Result result_;
};

class FAKE_NODE_HANDLE_PUBLIC SimpleActionClientStatic
{
public:
  static void sendGoal(const std::string& action_name, const std::shared_ptr<ActionGoalBindOptions>& ops);

  static void waitForResult(const std::string& action_name);
};

template <typename ActionSpec>
class SimpleActionClient
{
  using SimpleDoneCallback = typename AT<ActionSpec>::SimpleDoneCallback;
  using SimpleActiveCallback = typename AT<ActionSpec>::SimpleActiveCallback;
  using SimpleFeedbackCallback = typename AT<ActionSpec>::SimpleFeedbackCallback;

  using Goal = typename AT<ActionSpec>::Goal;

public:
  SimpleActionClient(const std::string& name)
  {
    action_name_ = name;
  }

  bool waitForServer()
  {
    return true;
  }

  void sendGoal(const Goal& goal, SimpleDoneCallback done_cb = SimpleDoneCallback(),
                SimpleActiveCallback active_cb = SimpleActiveCallback(),
                SimpleFeedbackCallback feedback_cb = SimpleFeedbackCallback())
  {
    auto ops = std::make_shared<ActionGoalBindOptions>();
    ops->template init<ActionSpec>(goal, done_cb, active_cb, feedback_cb);
    SimpleActionClientStatic::sendGoal(action_name_, ops);
  }

  void cancelGoal()
  {
    // fan mask todo:
  }

  void waitForResult()
  {
    SimpleActionClientStatic::waitForResult(action_name_);
  }

private:
  std::string action_name_;
};

}  // namespace _ros

#endif  // CODEIT_MOVEIT_ACTION_H
