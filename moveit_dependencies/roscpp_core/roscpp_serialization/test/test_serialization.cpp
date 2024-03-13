
#include "JointTrajectory.h"
#include "gtest/gtest.h"


TEST(Serializer, deserialize_1)
{
  namespace ser = ros::serialization;

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names.push_back("joint_1");
  msg.joint_names.push_back("joint_2");
  msg.joint_names.push_back("joint_3");
  msg.joint_names.push_back("joint_4");
  msg.points.resize(1);

  auto& point = msg.points[0];
  point.positions.push_back(1.0);
  point.positions.push_back(2.0);
  point.positions.push_back(3.0);
  point.positions.push_back(4.0);

  // 序列化
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  std::shared_ptr<uint8_t> buffer(new uint8_t[serial_size]);
  ser::OStream oStream(buffer.get(), serial_size);
  ser::serialize(oStream, msg);

  // 反序列化1
  trajectory_msgs::JointTrajectory msg_deserialize;
  ser::IStream iStream(buffer.get(), serial_size);
  ser::deserialize(iStream, msg_deserialize);


  ASSERT_EQ(msg_deserialize.joint_names.size(), 4);
  ASSERT_EQ(msg_deserialize.joint_names[0], "joint_1");
  ASSERT_EQ(msg_deserialize.joint_names[1], "joint_2");
  ASSERT_EQ(msg_deserialize.joint_names[2], "joint_3");
  ASSERT_EQ(msg_deserialize.joint_names[3], "joint_4");

  ASSERT_EQ(msg_deserialize.points.size(), 1);
  ASSERT_EQ(msg_deserialize.points[0].positions.size(), 4);
  ASSERT_EQ(msg_deserialize.points[0].positions[0], 1);
  ASSERT_EQ(msg_deserialize.points[0].positions[1], 2);
  ASSERT_EQ(msg_deserialize.points[0].positions[2], 3);
  ASSERT_EQ(msg_deserialize.points[0].positions[3], 4);
}

TEST(Serializer, deserialize_2)
{
  namespace ser = ros::serialization;

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names.push_back("joint_1");
  msg.joint_names.push_back("joint_2");
  msg.joint_names.push_back("joint_3");
  msg.joint_names.push_back("joint_4");
  msg.points.resize(1);

  auto& point = msg.points[0];
  point.positions.push_back(1.0);
  point.positions.push_back(2.0);
  point.positions.push_back(3.0);
  point.positions.push_back(4.0);

  // 序列化
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  std::shared_ptr<uint8_t> buffer(new uint8_t[serial_size]);
  ser::OStream oStream(buffer.get(), serial_size);
  ser::serialize(oStream, msg);

  // 反序列化1
  trajectory_msgs::JointTrajectory msg_deserialize;
  ser::IStream iStream(buffer.get(), serial_size);
  ros::serialization::Serializer<trajectory_msgs::JointTrajectory>::read(iStream, msg_deserialize);


  ASSERT_EQ(msg_deserialize.joint_names.size(), 4);
  ASSERT_EQ(msg_deserialize.joint_names[0], "joint_1");
  ASSERT_EQ(msg_deserialize.joint_names[1], "joint_2");
  ASSERT_EQ(msg_deserialize.joint_names[2], "joint_3");
  ASSERT_EQ(msg_deserialize.joint_names[3], "joint_4");

  ASSERT_EQ(msg_deserialize.points.size(), 1);
  ASSERT_EQ(msg_deserialize.points[0].positions.size(), 4);
  ASSERT_EQ(msg_deserialize.points[0].positions[0], 1);
  ASSERT_EQ(msg_deserialize.points[0].positions[1], 2);
  ASSERT_EQ(msg_deserialize.points[0].positions[2], 3);
  ASSERT_EQ(msg_deserialize.points[0].positions[3], 4);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}