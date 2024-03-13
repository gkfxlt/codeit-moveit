//
// Created by yjh on 6/28/21.
//

#include "action_adapter.h"
#include "ros_message_convert.h"

#include "fake_node_handle/fake_node_handle.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "moveit_runtime/moveit_runtime.h"

bool ActionAdapterBase::shut_down_ = false;

template <typename MsgType, typename Norm_MsgType>
class ActionServerAdapterBase : public ActionAdapterBase
{
  using Goal = typename _ros::AT<MsgType>::Goal;
  using Result = typename _ros::AT<MsgType>::Result;
  using ResultConstPtr = typename _ros::AT<MsgType>::ResultConstPtr;
  using Feedback = typename _ros::AT<MsgType>::Feedback;
  using FeedbackConstPtr = typename _ros::AT<MsgType>::FeedbackConstPtr;

  using _Goal = typename _ros::AT<Norm_MsgType>::Goal;
  using _Result = typename _ros::AT<Norm_MsgType>::Result;
  using _ResultConstPtr = typename _ros::AT<Norm_MsgType>::ResultConstPtr;
  using _Feedback = typename _ros::AT<Norm_MsgType>::Feedback;
  using _FeedbackConstPtr = typename _ros::AT<Norm_MsgType>::FeedbackConstPtr;

public:
  ActionServerAdapterBase(const std::string& action_name)
  {
    action_name_ = action_name;

    //    server_.reset(new actionlib::SimpleActionServer<MsgType>(
    //        action_name, boost::bind(&ActionServerAdapterBase::execute, this, boost::placeholders::_1), false));

    server_.reset(new actionlib::SimpleActionServer<MsgType>(action_name, false));
    server_->registerGoalCallback(boost::bind(&ActionServerAdapterBase::goalCallback, this));

    server_->registerPreemptCallback(boost::bind(&ActionServerAdapterBase::preemptCallback, this));

    server_->start();

    fake_client_.reset(new _ros::SimpleActionClient<Norm_MsgType>(action_name));
  }

  /// for actionlib::SimpleActionServer<MsgType>
  bool execute(const boost::shared_ptr<const Goal>& goal)
  {
    if (shut_down_)
    {
      return false;
    }

    static _Goal _goal;
    if (MessageCompare<Goal, _Goal>::isSameType())
    {
      serialConvert(_goal, goal);
    }
    else
    {
      messageConvert(_goal, goal);
    }

    fake_client_->sendGoal(_goal, std::bind(&ActionServerAdapterBase::done, this, std::placeholders::_1,
                                            std::placeholders::_2),
                           std::bind(&ActionServerAdapterBase::active, this),
                           std::bind(&ActionServerAdapterBase::feedback, this, std::placeholders::_1));

    fake_client_->waitForResult();
    return true;
  }

  void goalCallback()
  {
    if (shut_down_)
    {
      return;
    }

    std::cerr << "**  SimpleGoalCallback Begin: " << action_name_ << std::endl;

    static _Goal _goal;
    auto& goal = *server_->acceptNewGoal();
    if (MessageCompare<Goal, _Goal>::isSameType())
    {
      serialConvert(_goal, goal);
    }
    else
    {
      messageConvert(_goal, goal);
    }

    fake_client_->sendGoal(_goal, std::bind(&ActionServerAdapterBase::done, this, std::placeholders::_1,
                                            std::placeholders::_2),
                           std::bind(&ActionServerAdapterBase::active, this),
                           std::bind(&ActionServerAdapterBase::feedback, this, std::placeholders::_1));

    fake_client_->waitForResult();

    std::cerr << "**  SimpleGoalCallback End: " << action_name_ << std::endl;
  }

  void preemptCallback()
  {
    if (shut_down_)
    {
      return;
    }

    std::cerr << "**  SimplePreemptCallback: " << action_name_ << std::endl;
  }

  /// for _ros::SimpleActionClient<MsgType>
  void done(const _ros::SimpleClientGoalState& state, const _ResultConstPtr& _result)
  {
    if (shut_down_)
    {
      return;
    }

    std::cerr << "**  SimpleDoneCallback: " << action_name_ << ": " << state.toString() << std::endl;

    static Result result;
    const _Result& __result = *_result;
    if (MessageCompare<Result, _Result>::isSameType())
    {
      serialConvert(result, __result);
    }
    else
    {
      messageConvert(result, __result);
    }

    if (state == state.SUCCEEDED)
    {
      server_->setSucceeded(result);
    }
    else if (state == state.PREEMPTED)
    {
      server_->setPreempted(result);
    }
    else if (true || (state == state.ABORTED))
    {
      server_->setAborted(result);
    }
  }

  void active()
  {
    if (shut_down_)
    {
      return;
    }

    std::cerr << "**  SimpleActiveCallback: " << action_name_ << std::endl;
  }

  void feedback(const _FeedbackConstPtr& _feedback)
  {
    if (shut_down_)
    {
      return;
    }

    std::cerr << "**  SimpleFeedbackCallback: " << action_name_ << ": " << _feedback->state << std::endl;

    static Feedback feedback;
    if (MessageCompare<Feedback, _Feedback>::isSameType())
    {
      serialConvert(feedback, *_feedback);
    }
    else
    {
      messageConvert(feedback, *_feedback);
    }

    server_->publishFeedback(feedback);
  }

protected:
  std::string action_name_;
  std::unique_ptr<actionlib::SimpleActionServer<MsgType>> server_;
  std::unique_ptr<_ros::SimpleActionClient<Norm_MsgType>> fake_client_;
};

template <typename MsgType>
class ActionClientAdapterBase : public ActionAdapterBase
{
};

///
///
///

// class ActionServerAdapterTest : public ActionServerAdapterBase<moveit_msgs::ExecuteTrajectoryAction>
//{
// public:
//   ActionServerAdapterTest(const std::string &action_name)
//       : ActionServerAdapterBase<moveit_msgs::ExecuteTrajectoryAction>(action_name){}
// };

#define DEFINE_ACTION_SERVER(class_name, msg_type, norm_msg_type)                                                      \
  class class_name : public ActionServerAdapterBase<msg_type, norm_msg_type>                                           \
  {                                                                                                                    \
  public:                                                                                                              \
    class_name(const std::string& action_name) : ActionServerAdapterBase<msg_type, norm_msg_type>(action_name)         \
    {                                                                                                                  \
    }                                                                                                                  \
  };

#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "_moveit_msgs/ExecuteTrajectoryAction.h"
DEFINE_ACTION_SERVER(ActionServerAdapterTest, moveit_msgs::ExecuteTrajectoryAction,
                     _moveit_msgs::ExecuteTrajectoryAction)

#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "_moveit_msgs/ExecuteTrajectoryAction.h"
DEFINE_ACTION_SERVER(ActionServerExecuteTrajectory, moveit_msgs::ExecuteTrajectoryAction,
                     _moveit_msgs::ExecuteTrajectoryAction)
// static const std::string EXECUTE_ACTION_NAME = "execute_trajectory";  // name of 'execute' action
// execute_action_server_.reset(new _ros::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction>(
//     EXECUTE_ACTION_NAME, std::bind(&MoveGroupExecuteTrajectoryAction::executePathCallback, this, _1), false));

#include "moveit_msgs/MoveGroupAction.h"
#include "_moveit_msgs/MoveGroupAction.h"
DEFINE_ACTION_SERVER(ActionServerMoveGroup, moveit_msgs::MoveGroupAction, _moveit_msgs::MoveGroupAction)
// static const std::string MOVE_ACTION = "move_group";      // name of 'move' action
// move_action_server_.reset(new _ros::SimpleActionServer<moveit_msgs::MoveGroupAction>(
//     MOVE_ACTION, std::bind(&MoveGroupMoveAction::executeMoveCallback, this, _1), false));

#include "moveit_msgs/PickupAction.h"
#include "_moveit_msgs/PickupAction.h"
DEFINE_ACTION_SERVER(ActionServerPickup, moveit_msgs::PickupAction, _moveit_msgs::PickupAction)
// static const std::string PICKUP_ACTION = "pickup";  // name of 'pickup' action
// pickup_action_server_.reset(new _ros::SimpleActionServer<moveit_msgs::PickupAction>(
//     PICKUP_ACTION, std::bind(&MoveGroupPickPlaceAction::executePickupCallback, this, _1), false));

#include "moveit_msgs/PlaceAction.h"
#include "_moveit_msgs/PlaceAction.h"
DEFINE_ACTION_SERVER(ActionServerPlace, moveit_msgs::PlaceAction, _moveit_msgs::PlaceAction)
// static const std::string PLACE_ACTION = "place";    // name of 'place' action
// place_action_server_.reset(new _ros::SimpleActionServer<moveit_msgs::PlaceAction>(
//     PLACE_ACTION, std::bind(&MoveGroupPickPlaceAction::executePlaceCallback, this, _1), false));

void report_action()
{
  ActionReporter<moveit_msgs::ExecuteTrajectoryAction, _moveit_msgs::ExecuteTrajectoryAction>::report();
  ActionReporter<moveit_msgs::MoveGroupAction, _moveit_msgs::MoveGroupAction>::report();
  ActionReporter<moveit_msgs::PickupAction, _moveit_msgs::PickupAction>::report();
  ActionReporter<moveit_msgs::PlaceAction, _moveit_msgs::PlaceAction>::report();
}

void ActionAdapter::init()
{
  // report_action();

  /// for test
  action_adapters_.push_back(std::make_unique<ActionServerAdapterTest>("action_test"));

  /// action_servers
  action_adapters_.push_back(
      std::make_unique<ActionServerExecuteTrajectory>(moveit_runtime::action::EXECUTE_ACTION_NAME));
  action_adapters_.push_back(std::make_unique<ActionServerMoveGroup>(moveit_runtime::action::MOVE_ACTION_NAME));
  action_adapters_.push_back(std::make_unique<ActionServerPickup>(moveit_runtime::action::PICKUP_ACTION_NAME));
  action_adapters_.push_back(std::make_unique<ActionServerPlace>(moveit_runtime::action::PLACE_ACTION_NAME));

  /// action_clients
}
