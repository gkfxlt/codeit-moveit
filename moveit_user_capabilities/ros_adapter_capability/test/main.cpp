//
// Created by yjh on 6/28/21.
//

#include "ros/node_handle.h"
#include "fake_node_handle/fake_node_handle.h"

#include "ros_adapter_capability.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "test_action_helper.h"

class ST
{
public:
  ST()
  {
    _ros::NodeHandle nh;
    server_ = nh.advertiseService("server_adapter_test", &ST::callback, this);
  }

private:
  bool callback(std_srvs::Empty::RequestType& req, std_srvs::Empty::RequestType& res)
  {
    std::cout << "server_adapter_test" << std::endl;
    return true;
  }

  _ros::ServiceServer server_;
};

int main(int argc, char** argv)
{
  move_group::RosAdpater adpt;
  adpt.initialize();

  _ros::NodeHandle nh;

  std::function<void(const sensor_msgs::JointState&)> ff = [](const sensor_msgs::JointState& msg) {
    static int i = 0;
    std::cout << "===sensor_msgs::JointState  Recived:" << i++ << " ======" << std::endl;
  };
  _ros::Subscriber sub_ = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, ff);

  std::thread thread([&]() {
    _ros::Publisher pub_;
    pub_ = nh.advertise<std_msgs::String>("pub_adapter_test");

    std_msgs::String msg;
    msg.data = "are you ok?";
    while (ros::ok())
    {
      pub_.publish(msg);
      sleep(1);
    }
  });

  ST st;

  ActionTestServer actionTestServer;

  while (ros::ok())
  {
    sleep(1);
  }

  return 0;
}