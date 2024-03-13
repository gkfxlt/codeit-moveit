//
// Created by yjh on 6/28/21.
//
#include "param_adapter.h"
#include "topic_adapter.h"
#include "service_adapter.h"
#include "action_adapter.h"
#include "ros/node_handle.h"
#include <memory>
#include <thread>

#include "ros_adapter_capability.h"


using namespace move_group;

struct RosAdpater::Impl
{
  std::unique_ptr<ParamAdapter> paramAdapter;
  std::unique_ptr<TopicAdapter> topicAdapter;
  std::unique_ptr<ServiceAdapter> serviceAdapter;
  std::unique_ptr<ActionAdapter> actionAdapter;

  bool stop;
  std::thread thread;

  ~Impl()
  {
    stop = true;
    if (thread.joinable())
    {
      thread.join();
    }
  }

  void initialize()
  {
    thread = std::thread([this]() {

      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "ros_adapter", ros::init_options::NoSigintHandler);

      ros::NodeHandle nh;

      paramAdapter = std::make_unique<ParamAdapter>();
      topicAdapter = std::make_unique<TopicAdapter>();
      serviceAdapter = std::make_unique<ServiceAdapter>();
      actionAdapter = std::make_unique<ActionAdapter>();

      paramAdapter->init(&nh);
      topicAdapter->init(&nh);
      serviceAdapter->init(&nh);
      actionAdapter->init();

      ros::Rate rate(100);
      while (!stop)
      {
        ros::spinOnce();
        rate.sleep();
      }

      topicAdapter->shutdown();
      ros::shutdown();
    });
  }
};

void RosAdpater::initialize()
{
  impl_->initialize();
}

RosAdpater::RosAdpater() : MoveGroupCapability("RosAdpater"), impl_(std::make_unique<Impl>())
{
}

RosAdpater::~RosAdpater()
{
}

#include <plugin_loader/plugin_loader.hpp>
PLUGIN_LOADER_REGISTER_CLASS(move_group::RosAdpater, move_group::MoveGroupCapability)
