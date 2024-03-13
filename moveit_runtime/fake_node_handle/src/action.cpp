//
// Created by Administrator on 2021/6/23.
//

#include "fake_node_handle/action.h"
#include <queue>
#include <future>
#include <iostream>

using namespace _ros;

class ActionRun
{
public:
  ActionRun(const std::string& action_name, const std::shared_ptr<ActionCallbackOptions>& callback)
  {
    is_done = true;
    action_name_ = action_name;
    callback_ = callback;
    start();
  }

  void registerPreemptCallback(const std::function<void()>& cb)
  {
    callback_->registerPreemptCallback(cb);
  }

  bool addGoal(const std::shared_ptr<ActionGoalBindOptions>& goalbind)
  {
    std::unique_lock<std::mutex> locker(mutex_);

    if (!is_done)
    {
      current_goalbind->helper_->preempt_callback(callback_->helper_);
    }

    is_done = false;
    goalbinds_.push(goalbind);
    condition_.notify_all();

    return true;
  }

  void publishFeedback(const void* feedback)
  {
    if (!is_done)
    {
      current_goalbind->helper_->feedback_callback(feedback);
    }
  }

  bool setGoalState(SimpleClientGoalState& state, void* result, const std::string& text)
  {
    if (!is_done)
    {
      state_ = state;
      result_ = result;
    }
    return true;
  }

  void waitForResult()
  {
    while(!is_done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  ~ActionRun()
  {
    stop();
  }

  void start()
  {
    stop();
    stop_ = false;
    thread_ = std::thread(&ActionRun::run, this);
  }

  void stop()
  {
    stop_ = true;
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  void run()
  {
    while (!stop_)
    {
      {
        std::unique_lock<std::mutex> locker(mutex_);
        while (goalbinds_.empty())
        {
          is_done = true;
          condition_.wait(locker);  // Unlock _mutex and wait to be notified
        }

        is_done = false;
        current_goalbind = goalbinds_.front();
        goalbinds_.pop();
      }

      current_goalbind->helper_->active_callback();
      current_goalbind->helper_->execute_callback(callback_->helper_);

      is_done = true;
      current_goalbind->helper_->done_callback(state_, result_);
    }
  }

private:
  std::string action_name_;
  std::atomic_bool is_done;
  std::shared_ptr<ActionCallbackOptions> callback_;

  std::queue<std::shared_ptr<ActionGoalBindOptions>> goalbinds_;
  std::shared_ptr<ActionGoalBindOptions> current_goalbind;

  SimpleClientGoalState state_{ SimpleClientGoalState::PENDING };
  void* result_{ nullptr };

  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable condition_;

  bool stop_{ false };
};

struct Impl
{
  std::map<std::string, std::shared_ptr<ActionRun>> action_runs;
};

static auto local_data = std::make_shared<Impl>();

/// SimpleActionServerStatic
void SimpleActionServerStatic::init(const std::string& action_name, const std::shared_ptr<ActionCallbackOptions>& ops)
{
  local_data->action_runs[action_name] = std::make_shared<ActionRun>(action_name, ops);
}

void SimpleActionServerStatic::publishFeedback(const std::string& action_name, const void* feedback)
{
  auto& action_runs = local_data->action_runs;
  if (action_runs.find(action_name) != action_runs.end())
  {
    action_runs[action_name]->publishFeedback(feedback);
  }
}

bool SimpleActionServerStatic::setGoalState(const std::string& action_name, SimpleClientGoalState& state, void* result,
                                            const std::string& text)
{
  auto& action_runs = local_data->action_runs;
  if (action_runs.find(action_name) != action_runs.end())
  {
    return action_runs[action_name]->setGoalState(state, result, text);
  }
  return false;
}
void SimpleActionServerStatic::registerPreemptCallback(const std::string& action_name, const std::function<void()>& cb)
{
  auto& action_runs = local_data->action_runs;
  if (action_runs.find(action_name) != action_runs.end())
  {
    action_runs[action_name]->registerPreemptCallback(cb);
  }
}

/// SimpleActionClientStatic
void SimpleActionClientStatic::sendGoal(const std::string& action_name,
                                        const std::shared_ptr<ActionGoalBindOptions>& ops)
{
  auto& action_runs = local_data->action_runs;
  if (action_runs.find(action_name) != action_runs.end())
  {
    action_runs[action_name]->addGoal(ops);
  }
}

void SimpleActionClientStatic::waitForResult(const std::string& action_name)
{
  auto& action_runs = local_data->action_runs;
  if (action_runs.find(action_name) != action_runs.end())
  {
    action_runs[action_name]->waitForResult();
  }
}
