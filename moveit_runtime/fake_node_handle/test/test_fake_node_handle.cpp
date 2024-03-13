

#include <gtest/gtest.h>
#include "fake_node_handle/fake_node_handle.h"
#include "plugin_loader/plugin_loader.hpp"
#include "libBase.h"

TEST(NodeHandle, test_int)
{
  _ros::NodeHandle nh;

  int int_val = 555;
  nh.setParam("/move_group/kiniect/int", 666);
  nh.getParam("move_group/kiniect/int", int_val);
  ASSERT_EQ(int_val, 666);
}

TEST(NodeHandle, test_str)
{
  _ros::NodeHandle nh;

  std::string str_val = "555";
  nh.setParam("/move_group/kiniect/str", "666");
  nh.getParam("/move_group/kiniect/str", str_val);
  ASSERT_EQ(str_val, "666");
}

 TEST(NodeHandle, test_lib)
{
   _ros::NodeHandle nh;
   nh.setParam("/lib/name", "nomame");

   std::string val;
   nh.getParam("/lib/name", val);
   ASSERT_EQ(val, "nomame");

   plugin_loader::PluginLoader loader_a("../lib/test_lib_a");
   auto lib_a = loader_a.createUniqueInstance<LibBase>("LibA");
   nh.getParam("/lib/name", val);
   ASSERT_EQ(val, "LibA");

   plugin_loader::PluginLoader loader_b("../lib/test_lib_b");
   auto lib_b = loader_b.createUniqueInstance<LibBase>("LibB");
   nh.getParam("/lib/name", val);
   ASSERT_EQ(val, "LibB");

   nh.clear();
 }

#include "test_topic_helper.h"
static std::string g_topic_recived_msg = "";
void topic_test_func(const std_msgs::String& msg)
{
  g_topic_recived_msg = msg.data;
}

TEST(NodeHandle, test_topic)
{
  _ros::NodeHandle nh;

  TopicTestClass topicTestClass;
  std::string lamda_topic_recived_msg = "";
  auto sub1 = nh.subscribe("test_topic", 1, &topic_test_func);
  auto sub2 = nh.subscribe("test_topic", 1,
                           std::function<void(const std_msgs::String&)>(
                               [&](const std_msgs::String& msg) -> void { lamda_topic_recived_msg = msg.data; }));

  std_msgs::String msg;
  msg.data = "hello world";
  auto pub = nh.advertise<std_msgs::String>("test_topic");
  pub.publish(msg);

  ASSERT_EQ(topicTestClass.getRecivedMsg(), msg.data);
  ASSERT_EQ(g_topic_recived_msg, msg.data);
  ASSERT_EQ(lamda_topic_recived_msg, msg.data);
}

#include "test_service_helper.h"
TEST(NodeHandle, test_service)
{
  class ServiceServerClass
  {
  public:
    ServiceServerClass()
    {
      _ros::NodeHandle nh;
      auto sev = nh.advertiseService("test_service", &ServiceServerClass::callback, this);
    }

    bool callback(AddRequest& req, AddResponse& res)
    {
      res.result = req.add1 + req.add2;
      return true;
    }
  };
  ServiceServerClass serviceServerClass;

  _ros::NodeHandle nh;
  auto client = nh.serviceClient<GetAddResult>(std::string("test_service"));

  GetAddResult add{};
  add.request.add1 = 12;
  add.request.add2 = 34;

  ASSERT_TRUE(client.call(add));
  ASSERT_EQ(add.response.result, 46);
}

#include "moveit_msgs/GetPositionFK.h"
TEST(NodeHandle, test_service_GetPositionFK)
{
  class ServiceServerClass
  {
  public:
    ServiceServerClass()
    {
      _ros::NodeHandle nh;
      auto sev = nh.advertiseService("test_service_GetPositionFK", &ServiceServerClass::callback, this);
    }

    bool callback(moveit_msgs::GetPositionFKRequest& req, moveit_msgs::GetPositionFKResponse& res)
    {
      res.fk_link_names = req.fk_link_names;
      return true;
    }
  };
  ServiceServerClass cls;

  _ros::NodeHandle nh;
  auto client = nh.serviceClient<moveit_msgs::GetPositionFK>(std::string("test_service_GetPositionFK"));

  moveit_msgs::GetPositionFK msg;
  msg.request.fk_link_names.push_back("qwe");
  msg.request.fk_link_names.push_back("asd");
  msg.request.fk_link_names.push_back("zxc");

  ASSERT_TRUE(client.call(msg));
  ASSERT_EQ(msg.response.fk_link_names, msg.request.fk_link_names);
  ASSERT_EQ(msg.response.fk_link_names[0], msg.request.fk_link_names[0]);
  ASSERT_EQ(msg.response.fk_link_names[1], msg.request.fk_link_names[1]);
  ASSERT_EQ(msg.response.fk_link_names[2], msg.request.fk_link_names[2]);
}

//#include "test_action_helper.h"
//TEST(NodeHandle, test_action)
//{
//  ActionTestServer actionTestServer;
//  ActionTestClient actionTestClient;
//
//  moveit_msgs::ExecuteTrajectoryGoal goal;
//  goal.trajectory.joint_trajectory.joint_names.push_back("joint_1");
//  goal.trajectory.joint_trajectory.joint_names.push_back("joint_2");
//  goal.trajectory.joint_trajectory.joint_names.push_back("joint_3");
//  goal.trajectory.joint_trajectory.joint_names.push_back("joint_4");
//  goal.trajectory.joint_trajectory.joint_names.push_back("joint_5");
//  actionTestClient.sendGoal(goal);
//
//  actionTestClient.waitFinish();
//
//  ASSERT_EQ(1000, 1000);
//}

TEST(NodeHandle, test_timer)
{
  int total = 0;
  int failed = 0;

  _ros::NodeHandle nh;
  _ros::Timer t = nh.createTimer(ros::Duration(0.01), [&]() {
    static ros::Time last = ros::Time::now();
    ros::Time now = ros::Time::now();
    total++;
    if(fabs((now - last).toSec() - 0.01) > 0.01)
    {
      failed ++;
    }
    last = now;
  });
  t.start();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  t.stop();

  // 百分之一
  std::cout << "Failed: " << failed << std::endl;
  EXPECT_LE(failed, 1);
}

TEST(NodeHandle, test_walltimer)
{
  int total = 0;
  int failed = 0;

  _ros::NodeHandle nh;
  _ros::WallTimer t = nh.createWallTimer(ros::WallDuration(0.01), [&]() {
    static ros::WallTime last = ros::WallTime::now();
    ros::WallTime now = ros::WallTime::now();
    total++;
    if(fabs((now - last).toSec() - 0.01) > 0.01)
    {
      failed ++;
    }
    last = now;
  });
  t.start();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  t.stop();

  // 百分之一
  std::cout << "Failed: " << failed << std::endl;
  EXPECT_LE(failed, 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
