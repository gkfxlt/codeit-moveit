//
// Created by yjh on 6/28/21.
//

#include "topic_adapter.h"
#include "ros_message_convert.h"

#include "fake_node_handle/fake_node_handle.h"
#include "moveit_runtime/moveit_runtime.h"

bool TopicAdapterBase::shut_down_ = false;

template <typename MsgType, typename Norm_MsgType>
class TopicPubAdapterBase : public TopicAdapterBase
{
public:
  TopicPubAdapterBase(ros::NodeHandle* pNode, const std::string& topic, uint32_t size)
  {
    publisher_ = pNode->advertise<MsgType>(topic, size);

    _ros::NodeHandle nh;
    fake_subscriber_ = nh.subscribe(topic, size, &TopicPubAdapterBase::callback, this);
  }

  void callback(const std::shared_ptr<const Norm_MsgType>& msg_in)
  {
    if (shut_down_)
    {
      return;
    }

    if (MessageCompare<MsgType, Norm_MsgType>::isSameType())
    {
      publisher_.publish(*msg_in);
    }
    else
    {
      static MsgType msg_out;
      MessageConverter<Norm_MsgType, MsgType>::convert(msg_out, *msg_in);
      publisher_.publish(msg_out);
    }
  }

protected:
  ros::Publisher publisher_;
  _ros::Subscriber fake_subscriber_;
};

template <typename MsgType, typename Norm_MsgType>
class TopicSubAdapterBase : public TopicAdapterBase
{
public:
  TopicSubAdapterBase(ros::NodeHandle* pNode, const std::string& topic, uint32_t size)
  {
    subscriber_ = pNode->subscribe<MsgType>(topic, size, boost::bind(&TopicSubAdapterBase::callback, this, _1));

    _ros::NodeHandle nh;
    fake_publisher_ = nh.advertise<Norm_MsgType>(topic);
  }

  void callback(const boost::shared_ptr<const MsgType>& msg_in)
  {
    if (!shut_down_)
    {
      if (MessageCompare<MsgType, Norm_MsgType>::isSameType())
      {
        fake_publisher_.publish(*msg_in);
      }
      else
      {
        static Norm_MsgType msg_out;
        MessageConverter<MsgType, Norm_MsgType>::convert(msg_out, *msg_in);
        fake_publisher_.publish(msg_out);
      }
    }
  }

protected:
  ros::Subscriber subscriber_;
  _ros::Publisher fake_publisher_;
};

// class TopicPubAdapterTest : public TopicPubAdapterBase<std_msgs::String>
//{
// public:
//   TopicPubAdapterTest(ros::NodeHandle* pNode, const std::string &topic_name, int size)
//       : TopicPubAdapterBase<std_msgs::String>(pNode, topic_name, size){}
// };
//
// class TopicSubAdapterTest : public TopicSubAdapterBase<std_msgs::String>
//{
// public:
//   TopicSubAdapterTest(ros::NodeHandle* pNode, const std::string &topic_name, int size)
//         : TopicSubAdapterBase<std_msgs::String>(pNode, topic_name, size){}
// };

#define DEFINE_TOPIC_PUB(class_name, msg_type, norm_msg_type)                                                          \
  class class_name : public TopicPubAdapterBase<msg_type, norm_msg_type>                                               \
  {                                                                                                                    \
  public:                                                                                                              \
    class_name(ros::NodeHandle* pNode, const std::string& topic_name, uint32_t size = 1)                               \
      : TopicPubAdapterBase<msg_type, norm_msg_type>(pNode, topic_name, size)                                          \
    {                                                                                                                  \
    }                                                                                                                  \
  };

#define DEFINE_TOPIC_SUB(class_name, msg_type, norm_msg_type)                                                          \
  class class_name : public TopicSubAdapterBase<msg_type, norm_msg_type>                                               \
  {                                                                                                                    \
  public:                                                                                                              \
    class_name(ros::NodeHandle* pNode, const std::string& topic_name, uint32_t size = 1)                               \
      : TopicSubAdapterBase<msg_type, norm_msg_type>(pNode, topic_name, size)                                          \
    {                                                                                                                  \
    }                                                                                                                  \
  };

/// for test
#include "std_msgs/String.h"
#include "_std_msgs/String.h"
DEFINE_TOPIC_PUB(TopicPubAdapterTest, std_msgs::String, _std_msgs::String)
DEFINE_TOPIC_SUB(TopicSubAdapterTest, std_msgs::String, _std_msgs::String)

/// subs
#include "tf2_msgs/TFMessage.h"
#include "_tf2_msgs/TFMessage.h"
DEFINE_TOPIC_SUB(TopicSubTF, tf2_msgs::TFMessage, _tf2_msgs::TFMessage);
DEFINE_TOPIC_SUB(TopicSubTFStatic, tf2_msgs::TFMessage, _tf2_msgs::TFMessage);

#include "moveit_msgs/PlanningScene.h"
#include "_moveit_msgs/PlanningScene.h"
DEFINE_TOPIC_SUB(TopicSubPlanningScene, moveit_msgs::PlanningScene, _moveit_msgs::PlanningScene);
DEFINE_TOPIC_SUB(TopicSubPlanningSceneWord, moveit_msgs::PlanningSceneWorld, _moveit_msgs::PlanningScene);

#include "sensor_msgs/JointState.h"
#include "_sensor_msgs/JointState.h"
DEFINE_TOPIC_SUB(TopicSubJointState, sensor_msgs::JointState, _sensor_msgs::JointState)

#include "moveit_msgs/AttachedCollisionObject.h"
#include "_moveit_msgs/AttachedCollisionObject.h"
DEFINE_TOPIC_SUB(TopicSubAttachedCollisionObject, moveit_msgs::AttachedCollisionObject,
                 _moveit_msgs::AttachedCollisionObject)

#include "moveit_msgs/CollisionObject.h"
#include "_moveit_msgs/CollisionObject.h"
DEFINE_TOPIC_SUB(TopicSubCollisionObject, moveit_msgs::CollisionObject, _moveit_msgs::CollisionObject)

#include "std_msgs/String.h"
#include "_std_msgs/String.h"
DEFINE_TOPIC_SUB(TopicSubTrajEvent, std_msgs::String, _std_msgs::String)

#include "sensor_msgs/PointCloud2.h"
#include "_sensor_msgs/PointCloud2.h"
DEFINE_TOPIC_SUB(TopicSubPointCloud2, sensor_msgs::PointCloud2, _sensor_msgs::PointCloud2)

/// pubs
#include "tf2_msgs/TFMessage.h"
#include "_tf2_msgs/TFMessage.h"
DEFINE_TOPIC_PUB(TopicPubTF, tf2_msgs::TFMessage, _tf2_msgs::TFMessage);
DEFINE_TOPIC_PUB(TopicPubTFStatic, tf2_msgs::TFMessage, _tf2_msgs::TFMessage);

#include "moveit_msgs/DisplayTrajectory.h"
#include "_moveit_msgs/DisplayTrajectory.h"
DEFINE_TOPIC_PUB(TopicPubDisplayPath, moveit_msgs::DisplayTrajectory, _moveit_msgs::DisplayTrajectory)

#include "visualization_msgs/MarkerArray.h"
#include "_visualization_msgs/MarkerArray.h"
DEFINE_TOPIC_PUB(TopicPubDisplayGrasp, visualization_msgs::MarkerArray, _visualization_msgs::MarkerArray)

#include "moveit_msgs/MotionPlanRequest.h"
#include "_moveit_msgs/MotionPlanRequest.h"
DEFINE_TOPIC_PUB(TopicPubReceivedRequest, moveit_msgs::MotionPlanRequest, _moveit_msgs::MotionPlanRequest)
// const std::string planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC = "motion_plan_request";
// received_request_publisher_ = nh.advertise<moveit_msgs::MotionPlanRequest>(MOTION_PLAN_REQUEST_TOPIC, 10, true);

#include "visualization_msgs/MarkerArray.h"
#include "_visualization_msgs/MarkerArray.h"
DEFINE_TOPIC_PUB(TopicPubContacts, visualization_msgs::MarkerArray, _visualization_msgs::MarkerArray)
// const std::string planning_pipeline::PlanningPipeline::MOTION_CONTACTS_TOPIC = "display_contacts";
// contacts_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(MOTION_CONTACTS_TOPIC, 100, true);

#include "moveit_msgs/PlanningScene.h"
#include "_moveit_msgs/PlanningScene.h"
DEFINE_TOPIC_PUB(TopicPubPlanningScene, moveit_msgs::PlanningScene, _moveit_msgs::PlanningScene)
// const std::string PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC = "monitored_planning_scene";
// planning_scene_publisher_ = nh.advertise<moveit_msgs::PlanningScene>(planning_scene_topic);

#include "sensor_msgs/JointState.h"
#include "_sensor_msgs/JointState.h"
DEFINE_TOPIC_PUB(TopicPubJointState, sensor_msgs::JointState, _sensor_msgs::JointState)

void report_topic()
{
  TopicReporter<std_msgs::String, _std_msgs::String>::report();
  TopicReporter<tf2_msgs::TFMessage, _tf2_msgs::TFMessage>::report();
  TopicReporter<moveit_msgs::PlanningScene, _moveit_msgs::PlanningScene>::report();
  TopicReporter<sensor_msgs::JointState, _sensor_msgs::JointState>::report();
  TopicReporter<moveit_msgs::AttachedCollisionObject, _moveit_msgs::AttachedCollisionObject>::report();
  TopicReporter<moveit_msgs::CollisionObject, _moveit_msgs::CollisionObject>::report();
  TopicReporter<sensor_msgs::PointCloud2, _sensor_msgs::PointCloud2>::report();
  TopicReporter<moveit_msgs::DisplayTrajectory, _moveit_msgs::DisplayTrajectory>::report();
  TopicReporter<visualization_msgs::MarkerArray, _visualization_msgs::MarkerArray>::report();
  TopicReporter<moveit_msgs::MotionPlanRequest, _moveit_msgs::MotionPlanRequest>::report();
}

void TopicAdapter::init(ros::NodeHandle* pNode)
{
  // report_topic();

  /// for test
  topic_adapters_.push_back(std::make_unique<TopicPubAdapterTest>(pNode, "pub_adapter_test", 1));
  topic_adapters_.push_back(std::make_unique<TopicSubAdapterTest>(pNode, "sub_adapter_test", 1));

  /// subs
  // fan mask for already has pub. this will take a bug.
  // topic_adapters_.push_back(std::make_unique<TopicSubTF>(pNode, moveit_runtime::topic::TF_TOPIC_NAME, 1));
  // topic_adapters_.push_back(std::make_unique<TopicSubTFStatic>(pNode, moveit_runtime::topic::TF_STATIC_TOPIC_NAME,
  // 1));

  topic_adapters_.push_back(
      std::make_unique<TopicSubPlanningScene>(pNode, moveit_runtime::topic::PLANNING_SCENE_TOPIC_NAME, 1));
  topic_adapters_.push_back(
      std::make_unique<TopicSubPlanningSceneWord>(pNode, moveit_runtime::topic::PLANNING_SCENE_WORLD_TOPIC_NAME, 1));

  // fan mask for already has pub. this will take a bug.
  // topic_adapters_.push_back(std::make_unique<TopicSubJointState>(pNode,
  // moveit_runtime::topic::JOINT_STATES_TOPIC_NAME, 1));

  topic_adapters_.push_back(std::make_unique<TopicSubAttachedCollisionObject>(
      pNode, moveit_runtime::topic::ATTACHED_COLLISION_OBJECT_TOPIC_NAME, 1));
  topic_adapters_.push_back(
      std::make_unique<TopicSubCollisionObject>(pNode, moveit_runtime::topic::COLLISION_OBJECT_TOPIC_NAME, 1));
  topic_adapters_.push_back(
      std::make_unique<TopicSubTrajEvent>(pNode, moveit_runtime::topic::TRAJECTORY_EXECUTION_EVENT_TOPIC_NAME, 1));
  topic_adapters_.push_back(std::make_unique<TopicSubPointCloud2>(pNode, "/camera/depth/color/points", 1));

  /// pubs
  topic_adapters_.push_back(std::make_unique<TopicPubTF>(pNode, moveit_runtime::topic::TF_TOPIC_NAME, 1));
  topic_adapters_.push_back(std::make_unique<TopicPubTFStatic>(pNode, moveit_runtime::topic::TF_STATIC_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubDisplayPath>(pNode, moveit_runtime::topic::DISPLAY_PATH_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubDisplayGrasp>(pNode, moveit_runtime::topic::DISPLAY_GRASP_MARKERS_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubReceivedRequest>(pNode, moveit_runtime::topic::MOTION_PLAN_REQUEST_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubContacts>(pNode, moveit_runtime::topic::DISPLAY_CONTACTS_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubPlanningScene>(pNode, moveit_runtime::topic::MONITORED_PLANNING_SCENE_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubJointState>(pNode, moveit_runtime::topic::CONTROLLER_JOINT_STATES_TOPIC_NAME, 1));

  topic_adapters_.push_back(
      std::make_unique<TopicPubJointState>(pNode, moveit_runtime::topic::JOINT_STATES_TOPIC_NAME, 1));
}

void TopicAdapter::shutdown()
{
  TopicAdapterBase::shut_down_ = true;
}
