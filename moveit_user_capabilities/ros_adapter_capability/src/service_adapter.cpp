//
// Created by yjh on 6/28/21.
//

#include "service_adapter.h"
#include "fake_node_handle/fake_node_handle.h"
#include "moveit_runtime/moveit_runtime.h"

#include "ros_message_convert.h"

bool ServiceAdapterBase::shut_down_ = false;

template <typename MsgType, typename Norm_MsgType>
class ServiceServerAdapterBase : public ServiceAdapterBase
{
  using ReqType = typename MsgType::Request;
  using ResType = typename MsgType::Response;

  using _ReqType = typename Norm_MsgType::Request;
  using _ResType = typename Norm_MsgType::Response;

public:
  ServiceServerAdapterBase(ros::NodeHandle* pNode, const std::string& serviceName)
  {
    serviceName_ = serviceName;

    server_ = pNode->advertiseService(serviceName, &ServiceServerAdapterBase::callback, this);

    _ros::NodeHandle nh;
    fake_client_ = nh.serviceClient<Norm_MsgType>(serviceName);
  }

  bool callback(ReqType& req, ResType& res)
  {
    if (shut_down_)
    {
      return false;
    }

    std::cerr << "**  ServiceServer Call Begin: " << serviceName_ << std::endl;

    static _ReqType _req;
    static _ResType _res;

    if (MessageCompare<ReqType, _ReqType>::isSameType())
    {
      serialConvert(_req, req);
    }
    else
    {
      messageConvert(_req, req);
    }

    bool flag = fake_client_.call(_req, _res);

    if (MessageCompare<ResType, _ResType>::isSameType())
    {
      serialConvert(res, _res);
    }
    else
    {
      messageConvert(res, _res);
    }

    std::cerr << "**  ServiceServer Call End: " << serviceName_ << std::endl;

    return flag;
  }

protected:
  std::string serviceName_;
  ros::ServiceServer server_;
  _ros::ServiceClient fake_client_;
};

template <typename MsgType, typename Norm_MsgType>
class ServiceClientAdapterBase : public ServiceAdapterBase
{
  using ReqType = typename MsgType::Request;
  using ResType = typename MsgType::Response;

  using _ReqType = typename Norm_MsgType::Request;
  using _ResType = typename Norm_MsgType::Response;

public:
  ServiceClientAdapterBase(ros::NodeHandle* pNode, const std::string& serviceName)
  {
    _ros::NodeHandle nh;
    fake_server_ = nh.advertiseService(serviceName, &ServiceClientAdapterBase::callback, this);

    client_ = pNode->serviceClient<MsgType>(serviceName);
  }

  bool callback(_ReqType& _req, _ResType& _res)
  {
    if (shut_down_)
    {
      return false;
    }

    static ReqType req;
    static ResType res;

    if (MessageCompare<ReqType, _ReqType>::isSameType())
    {
      serialConvert(req, _req);
    }
    else
    {
      messageConvert(req, _req);
    }

    bool flag = client_.call(req, res);

    if (MessageCompare<ResType, _ResType>::isSameType())
    {
      serialConvert(_res, res);
    }
    else
    {
      messageConvert(_res, res);
    }

    return flag;
  }

protected:
  ros::ServiceClient client_;
  _ros::ServiceServer fake_server_;
};

// class ServiceServerAdapterTest : public ServiceServerAdapterBase<std_srvs::Empty>
//{
// public:
//   ServiceServerAdapterTest(ros::NodeHandle* pNode, const std::string &service_name)
//       : ServiceServerAdapterBase<std_srvs::Empty>(pNode, service_name){}
// };
//
// class ServiceClientAdapterTest : public ServiceClientAdapterBase<std_srvs::Empty>
//{
// public:
//   ServiceClientAdapterTest(ros::NodeHandle* pNode, const std::string &service_name)
//       : ServiceClientAdapterBase<std_srvs::Empty>(pNode, service_name){}
// };

#define DEFINE_SERVICE_SERVER(class_name, msg_type, norm_msg_type)                                                     \
  class class_name : public ServiceServerAdapterBase<msg_type, norm_msg_type>                                          \
  {                                                                                                                    \
  public:                                                                                                              \
    class_name(ros::NodeHandle* pNode, const std::string& service_name)                                                \
      : ServiceServerAdapterBase<msg_type, norm_msg_type>(pNode, service_name)                                         \
    {                                                                                                                  \
    }                                                                                                                  \
  };

#define DEFINE_SERVICE_CLIENT(class_name, msg_type, norm_msg_type)                                                     \
  class class_name : public ServiceClientAdapterBase<msg_type, norm_msg_type>                                          \
  {                                                                                                                    \
  public:                                                                                                              \
    class_name(ros::NodeHandle* pNode, const std::string& service_name)                                                \
      : ServiceClientAdapterBase<msg_type, norm_msg_type>(pNode, service_name)                                         \
    {                                                                                                                  \
    }                                                                                                                  \
  };

#include "std_srvs/Empty.h"
DEFINE_SERVICE_SERVER(ServiceServerAdapterTest, std_srvs::Empty, _std_srvs::Empty)
DEFINE_SERVICE_CLIENT(ServiceClientAdapterTest, std_srvs::Empty, _std_srvs::Empty)

/// service_servers
#include "std_srvs/Empty.h"
DEFINE_SERVICE_SERVER(ServiceServerClearOctomap, std_srvs::Empty, _std_srvs::Empty)
// static const std::string CLEAR_OCTOMAP_SERVICE_NAME =
//     "clear_octomap";  // name of the service that can be used to clear the octomap
// service_ = root_node_handle_.advertiseService(CLEAR_OCTOMAP_SERVICE_NAME, &ClearOctomapService::clearOctomap, this);

#include "moveit_msgs/ApplyPlanningScene.h"
DEFINE_SERVICE_SERVER(ServiceServerApplyPlanningScene, moveit_msgs::ApplyPlanningScene, _moveit_msgs::ApplyPlanningScene)
// static const std::string APPLY_PLANNING_SCENE_SERVICE_NAME =
//     "apply_planning_scene";  // name of the service that applies a given planning scene
// service_ = root_node_handle_.advertiseService(APPLY_PLANNING_SCENE_SERVICE_NAME,
//                                               &ApplyPlanningSceneService::applyScene, this);

#include "moveit_msgs/GetCartesianPath.h"
DEFINE_SERVICE_SERVER(ServiceServerCartesianPath, moveit_msgs::GetCartesianPath, _moveit_msgs::GetCartesianPath)
// static const std::string CARTESIAN_PATH_SERVICE_NAME =
//     "compute_cartesian_path";  // name of the service that computes cartesian paths
// cartesian_path_service_ = root_node_handle_.advertiseService(CARTESIAN_PATH_SERVICE_NAME,
//                                                              &MoveGroupCartesianPathService::computeService, this);

#include "moveit_msgs/GetPositionFK.h"
DEFINE_SERVICE_SERVER(ServiceServerFK, moveit_msgs::GetPositionFK, _moveit_msgs::GetPositionFK)
// static const std::string FK_SERVICE_NAME = "compute_fk";  // name of fk service
// fk_service_ =
// root_node_handle_.advertiseService(FK_SERVICE_NAME, &MoveGroupKinematicsService::computeFKService, this);

#include "moveit_msgs/GetPositionIK.h"
DEFINE_SERVICE_SERVER(ServiceServerIK, moveit_msgs::GetPositionIK, _moveit_msgs::GetPositionIK)
// static const std::string IK_SERVICE_NAME = "compute_ik";  // name of ik service
// ik_service_ =
// root_node_handle_.advertiseService(IK_SERVICE_NAME, &MoveGroupKinematicsService::computeIKService, this);

#include "moveit_msgs/GetMotionPlan.h"
DEFINE_SERVICE_SERVER(ServiceServerGetMotionPlan, moveit_msgs::GetMotionPlan, _moveit_msgs::GetMotionPlan)
// static const std::string PLANNER_SERVICE_NAME =
//     "plan_kinematic_path";  // name of the advertised service (within the ~ namespace)
// plan_service_ =
// root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupPlanService::computePlanService, this);

#include "moveit_msgs/QueryPlannerInterfaces.h"
DEFINE_SERVICE_SERVER(ServiceServerQueryPlanners, moveit_msgs::QueryPlannerInterfaces,
                      _moveit_msgs::QueryPlannerInterfaces)
// static const std::string QUERY_PLANNERS_SERVICE_NAME =
//     "query_planner_interface";  // name of the advertised query planners service
// query_service_ = root_node_handle_.advertiseService(QUERY_PLANNERS_SERVICE_NAME,
//                                                     &MoveGroupQueryPlannersService::queryInterface, this);

#include "moveit_msgs/GetPlannerParams.h"
DEFINE_SERVICE_SERVER(ServiceServerGetPlannerParams, moveit_msgs::GetPlannerParams, _moveit_msgs::GetPlannerParams)
// static const std::string GET_PLANNER_PARAMS_SERVICE_NAME =
//     "get_planner_params";  // service name to retrieve planner parameters
// get_service_ = root_node_handle_.advertiseService(GET_PLANNER_PARAMS_SERVICE_NAME,
//                                                   &MoveGroupQueryPlannersService::getParams, this);

#include "moveit_msgs/SetPlannerParams.h"
DEFINE_SERVICE_SERVER(ServiceServerSetPlannerParams, moveit_msgs::SetPlannerParams, _moveit_msgs::SetPlannerParams)
// static const std::string SET_PLANNER_PARAMS_SERVICE_NAME =
//     "set_planner_params";                                 // service name to set planner parameters
// set_service_ = root_node_handle_.advertiseService(SET_PLANNER_PARAMS_SERVICE_NAME,
//                                                   &MoveGroupQueryPlannersService::setParams, this);

#include "moveit_msgs/GetStateValidity.h"
DEFINE_SERVICE_SERVER(ServiceServerGetStateValidity, moveit_msgs::GetStateValidity, _moveit_msgs::GetStateValidity)
// static const std::string STATE_VALIDITY_SERVICE_NAME =
//     "check_state_validity";  // name of the service that validates states
// validity_service_ = root_node_handle_.advertiseService(STATE_VALIDITY_SERVICE_NAME,
//                                                        &MoveGroupStateValidationService::computeService, this);

#include "moveit_msgs/GetPlanningScene.h"
DEFINE_SERVICE_SERVER(ServiceServerGetPlanningScene, moveit_msgs::GetPlanningScene, _moveit_msgs::GetPlanningScene)
// static const std::string GET_PLANNING_SCENE_SERVICE_NAME =
//     "get_planning_scene";  // name of the service that can be used to query the planning scene
// get_scene_service_ = nh.advertiseService(service_name, &PlanningSceneMonitor::getPlanningSceneServiceCallback, this);

#include "moveit_msgs/SaveMap.h"
DEFINE_SERVICE_SERVER(ServiceServerSaveMap, moveit_msgs::SaveMap, _moveit_msgs::SaveMap)

#include "moveit_msgs/LoadMap.h"
DEFINE_SERVICE_SERVER(ServiceServerLoadMap, moveit_msgs::LoadMap, _moveit_msgs::LoadMap)

void report_service()
{
  ServiceReporter<std_srvs::Empty, _std_srvs::Empty>::report();
  ServiceReporter<moveit_msgs::ApplyPlanningScene, _moveit_msgs::ApplyPlanningScene>::report();
  ServiceReporter<moveit_msgs::GetCartesianPath, _moveit_msgs::GetCartesianPath>::report();
  ServiceReporter<moveit_msgs::GetPositionFK, _moveit_msgs::GetPositionFK>::report();
  ServiceReporter<moveit_msgs::GetPositionIK, _moveit_msgs::GetPositionIK>::report();
  ServiceReporter<moveit_msgs::GetMotionPlan, _moveit_msgs::GetMotionPlan>::report();
  ServiceReporter<moveit_msgs::QueryPlannerInterfaces, _moveit_msgs::QueryPlannerInterfaces>::report();
  ServiceReporter<moveit_msgs::GetPlannerParams, _moveit_msgs::GetPlannerParams>::report();
  ServiceReporter<moveit_msgs::SetPlannerParams, _moveit_msgs::SetPlannerParams>::report();
  ServiceReporter<moveit_msgs::GetStateValidity, _moveit_msgs::GetStateValidity>::report();
  ServiceReporter<moveit_msgs::GetPlanningScene, _moveit_msgs::GetPlanningScene>::report();
  ServiceReporter<moveit_msgs::SaveMap, _moveit_msgs::SaveMap>::report();
  ServiceReporter<moveit_msgs::LoadMap, _moveit_msgs::LoadMap>::report();
}

void ServiceAdapter::init(ros::NodeHandle* pNode)
{
  // report_service();

  /// for test
  service_adapters_.push_back(std::make_unique<ServiceServerAdapterTest>(pNode, "server_adapter_test"));
  service_adapters_.push_back(std::make_unique<ServiceClientAdapterTest>(pNode, "client_adapter_test"));

  /// service_servers
  service_adapters_.push_back(
      std::make_unique<ServiceServerClearOctomap>(pNode, moveit_runtime::service::CLEAR_OCTOMAP_SERVICE_NAME));
  service_adapters_.push_back(std::make_unique<ServiceServerApplyPlanningScene>(
      pNode, moveit_runtime::service::APPLY_PLANNING_SCENE_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerCartesianPath>(pNode, moveit_runtime::service::CARTESIAN_PATH_SERVICE_NAME));

  service_adapters_.push_back(std::make_unique<ServiceServerFK>(pNode, moveit_runtime::service::FK_SERVICE_NAME));
  service_adapters_.push_back(std::make_unique<ServiceServerIK>(pNode, moveit_runtime::service::IK_SERVICE_NAME));

  service_adapters_.push_back(
      std::make_unique<ServiceServerGetMotionPlan>(pNode, moveit_runtime::service::PLANNER_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerQueryPlanners>(pNode, moveit_runtime::service::QUERY_PLANNERS_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerGetPlannerParams>(pNode, moveit_runtime::service::GET_PLANNER_PARAMS_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerSetPlannerParams>(pNode, moveit_runtime::service::SET_PLANNER_PARAMS_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerGetStateValidity>(pNode, moveit_runtime::service::STATE_VALIDITY_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerGetPlanningScene>(pNode, moveit_runtime::service::GET_PLANNING_SCENE_SERVICE_NAME));

  service_adapters_.push_back(
      std::make_unique<ServiceServerSaveMap>(pNode, moveit_runtime::service::SAVE_MAP_SERVICE_NAME));
  service_adapters_.push_back(
      std::make_unique<ServiceServerLoadMap>(pNode, moveit_runtime::service::LOAD_MAP_SERVICE_NAME));

  /// service_clients
}
