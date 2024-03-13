//
// Created by fan on 2021/7/2.
//

#ifndef CODEIT_MOVEIT_ACTION_HELPER_H
#define CODEIT_MOVEIT_ACTION_HELPER_H

#include <string>
#include <memory>
#include <functional>

namespace _ros
{
/// SimpleClientGoalState
class SimpleClientGoalState
{
public:
  enum StateEnum
  {
    PENDING,
    ACTIVE,
    RECALLED,
    REJECTED,
    PREEMPTED,
    ABORTED,
    SUCCEEDED,
    LOST
  };

  StateEnum state_;
  std::string text_;

  explicit SimpleClientGoalState(const StateEnum& state, const std::string& text = std::string(""))
    : state_(state), text_(text)
  {
  }

  inline bool operator==(const SimpleClientGoalState& rhs) const
  {
    return state_ == rhs.state_;
  }

  inline bool operator==(const SimpleClientGoalState::StateEnum& rhs) const
  {
    return state_ == rhs;
  }

  inline bool operator!=(const SimpleClientGoalState::StateEnum& rhs) const
  {
    return !(*this == rhs);
  }

  inline bool operator!=(const SimpleClientGoalState& rhs) const
  {
    return !(*this == rhs);
  }

  /**
   * \brief Determine if goal is done executing (ie. reached a terminal state)
   * \return True if in RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, or LOST. False otherwise
   */
  inline bool isDone() const
  {
    switch (state_)
    {
      case RECALLED:
      case REJECTED:
      case PREEMPTED:
      case ABORTED:
      case SUCCEEDED:
      case LOST:
        return true;
      default:
        return false;
    }
  }

  std::string getText() const
  {
    return text_;
  }

  //! \brief Convert the state to a string. Useful when printing debugging information
  std::string toString() const
  {
    switch (state_)
    {
      case PENDING:
        return "PENDING";
      case ACTIVE:
        return "ACTIVE";
      case RECALLED:
        return "RECALLED";
      case REJECTED:
        return "REJECTED";
      case PREEMPTED:
        return "PREEMPTED";
      case ABORTED:
        return "ABORTED";
      case SUCCEEDED:
        return "SUCCEEDED";
      case LOST:
        return "LOST";
      default:
        // ROS_ERROR_NAMED("actionlib", "BUG: Unhandled SimpleGoalState: %u", state_);
        break;
    }
    return "BUG-UNKNOWN";
  }
};

/// Action Type Define
template <typename ActionSpec>
struct AT
{
  using ActionGoal = typename ActionSpec::_action_goal_type;
  using Goal = typename ActionGoal::_goal_type;
  using ActionResult = typename ActionSpec::_action_result_type;
  using Result = typename ActionResult::_result_type;
  using ActionFeedback = typename ActionSpec::_action_feedback_type;
  using Feedback = typename ActionFeedback::_feedback_type;

  using ActionGoalConstPtr = typename std::shared_ptr<const ActionGoal>;
  using ActionGoalPtr = typename std::shared_ptr<ActionGoal>;
  using GoalConstPtr = typename std::shared_ptr<const Goal>;
  using GoalPtr = typename std::shared_ptr<Goal>;

  using ActionResultConstPtr = typename std::shared_ptr<const ActionResult>;
  using ActionResultPtr = typename std::shared_ptr<ActionResult>;
  using ResultConstPtr = typename std::shared_ptr<const Result>;
  using ResultPtr = typename std::shared_ptr<Result>;

  using ActionFeedbackConstPtr = typename std::shared_ptr<const ActionFeedback>;
  using ActionFeedbackPtr = typename std::shared_ptr<ActionFeedback>;
  using FeedbackConstPtr = typename std::shared_ptr<const Feedback>;
  using FeedbackPtr = typename std::shared_ptr<Feedback>;

  using ExecuteCallback = typename std::function<void(const GoalConstPtr&)>;
  using SimpleDoneCallback =
      typename std::function<void(const SimpleClientGoalState& state, const ResultConstPtr& result)>;
  using SimpleActiveCallback = typename std::function<void()>;
  using SimpleFeedbackCallback = typename std::function<void(const FeedbackConstPtr& feedback)>;
};

/// ActionServerCallbackHelper
struct ActionCallbackHelper
{
  using PreemptCallback = typename std::function<void()>;

  virtual ~ActionCallbackHelper() = default;
  virtual void execute_callback(const void* param) = 0;

  void registerPreemptCallback(const PreemptCallback& cb)
  {
    preempt_callback_ = cb;
  }

  void preempt_callback()
  {
    preempt_callback_();
  }

private:
  PreemptCallback preempt_callback_;
};
typedef std::shared_ptr<ActionCallbackHelper> ActionExecuteCallbackHelperPtr;

/// ActionCallbackHelperT
template <typename ActionSpec>
struct ActionCallbackHelperT : public ActionCallbackHelper
{
  using Goal = typename AT<ActionSpec>::Goal;
  using GoalConstPtr = typename AT<ActionSpec>::GoalConstPtr;
  using ExecuteCallback = typename AT<ActionSpec>::ExecuteCallback;

  explicit ActionCallbackHelperT(const ExecuteCallback& callback) : callback_(callback)
  {
  }

  void execute_callback(const void* param) override
  {
    const Goal* goal = static_cast<const Goal*>(param);
    GoalConstPtr p = std::shared_ptr<const Goal>(goal, [](const Goal*) {});
    callback_(p);
  }

private:
  ExecuteCallback callback_;
};

/// ActionServerOptions
struct ActionCallbackOptions
{
  using PreemptCallback = typename std::function<void()>;

  template <typename ActionSpec, typename Callback = typename AT<ActionSpec>::ExecuteCallback>
  void init(const std::string& name, const Callback callback, bool auto_start)
  {
    name_ = name;
    helper_ = std::make_shared<ActionCallbackHelperT<ActionSpec>>(callback);
  }

  void registerPreemptCallback(const PreemptCallback& cb)
  {
    helper_->registerPreemptCallback(cb);
  }

  std::string name_;
  ActionExecuteCallbackHelperPtr helper_;
};

/// ActionGoalBindHelper
struct ActionGoalBindHelper
{
  virtual ~ActionGoalBindHelper() = default;
  virtual void execute_callback(ActionExecuteCallbackHelperPtr callbackHelper) = 0;
  virtual void preempt_callback(ActionExecuteCallbackHelperPtr callbackHelper) = 0;

  virtual void done_callback(const SimpleClientGoalState& state, const void* param) = 0;
  virtual void active_callback() = 0;
  virtual void feedback_callback(const void* param) = 0;
};
typedef std::shared_ptr<ActionGoalBindHelper> ActionGoalBindHelperPtr;

/// ActionGoalBindHelperT
template <typename ActionSpec>
struct ActionGoalBindHelperT : public ActionGoalBindHelper
{
  using Goal = typename AT<ActionSpec>::Goal;

  using Feedback = typename AT<ActionSpec>::Feedback;
  using FeedbackConstPtr = typename AT<ActionSpec>::FeedbackConstPtr;

  using Result = typename AT<ActionSpec>::Result;
  using ResultConstPtr = typename AT<ActionSpec>::ResultConstPtr;

  using SimpleDoneCallback = typename AT<ActionSpec>::SimpleDoneCallback;
  using SimpleActiveCallback = typename AT<ActionSpec>::SimpleActiveCallback;
  using SimpleFeedbackCallback = typename AT<ActionSpec>::SimpleFeedbackCallback;

  explicit ActionGoalBindHelperT(const Goal goal, const SimpleDoneCallback& done_cb,
                                 const SimpleActiveCallback& active_cb, const SimpleFeedbackCallback& feedback_cb)
    : goal_(goal), done_cb_(done_cb), active_cb_(active_cb), feedback_cb_(feedback_cb)
  {
  }

  void execute_callback(ActionExecuteCallbackHelperPtr callbackHelper)
  {
    callbackHelper->execute_callback(&goal_);
  }

  void preempt_callback(ActionExecuteCallbackHelperPtr callbackHelper) override
  {
    callbackHelper->preempt_callback();
  }

  void done_callback(const SimpleClientGoalState& state, const void* param) override
  {
    if (done_cb_)
    {
      result_ = *static_cast<const Result*>(param);
      ResultConstPtr p = std::shared_ptr<const Result>(&result_, [](const Result*) {});
      done_cb_(state, p);
    }
  }

  void active_callback() override
  {
    if (active_cb_)
    {
      active_cb_();
    }
  }

  void feedback_callback(const void* param) override
  {
    if (feedback_cb_)
    {
      feedback_ = *static_cast<const Feedback*>(param);
      FeedbackConstPtr p = std::shared_ptr<const Feedback>(&feedback_, [](const Feedback*) {});
      feedback_cb_(p);
    }
  }

private:
  Goal goal_;
  Result result_;
  Feedback feedback_;
  SimpleDoneCallback done_cb_;
  SimpleActiveCallback active_cb_;
  SimpleFeedbackCallback feedback_cb_;
};

/// ActionGoalBindOptions
struct ActionGoalBindOptions
{
  template <typename ActionSpec, typename Goal = typename AT<ActionSpec>::Goal,
            typename SimpleDoneCallback = typename AT<ActionSpec>::SimpleDoneCallback,
            typename SimpleActiveCallback = typename AT<ActionSpec>::SimpleActiveCallback,
            typename SimpleFeedbackCallback = typename AT<ActionSpec>::SimpleFeedbackCallback>
  void init(const Goal& goal, const SimpleDoneCallback& done_cb, const SimpleActiveCallback& active_cb,
            const SimpleFeedbackCallback& feedback_cb)
  {
    helper_ = std::make_shared<ActionGoalBindHelperT<ActionSpec>>(goal, done_cb, active_cb, feedback_cb);
  }

  ActionGoalBindHelperPtr helper_;
};

}  // namespace _ros

#endif  // CODEIT_MOVEIT_ACTION_HELPER_H
