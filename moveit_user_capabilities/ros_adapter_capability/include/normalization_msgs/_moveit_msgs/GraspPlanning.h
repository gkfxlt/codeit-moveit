// Generated by gencpp from file _moveit_msgs/GraspPlanning.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_GRASPPLANNING_H
#define _MOVEIT_MSGS_MESSAGE_GRASPPLANNING_H

#include <ros/service_traits.h>


#include <_moveit_msgs/GraspPlanningRequest.h>
#include <_moveit_msgs/GraspPlanningResponse.h>


namespace _moveit_msgs
{

struct GraspPlanning
{

typedef GraspPlanningRequest Request;
typedef GraspPlanningResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GraspPlanning
} // namespace _moveit_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::_moveit_msgs::GraspPlanning > {
  static const char* value()
  {
    return "63d5775dc1a243968785b3d481b6bb24";
  }

  static const char* value(const ::_moveit_msgs::GraspPlanning&) { return value(); }
};

template<>
struct DataType< ::_moveit_msgs::GraspPlanning > {
  static const char* value()
  {
    return "_moveit_msgs/GraspPlanning";
  }

  static const char* value(const ::_moveit_msgs::GraspPlanning&) { return value(); }
};


// service_traits::MD5Sum< ::_moveit_msgs::GraspPlanningRequest> should match
// service_traits::MD5Sum< ::_moveit_msgs::GraspPlanning >
template<>
struct MD5Sum< ::_moveit_msgs::GraspPlanningRequest>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::GraspPlanning >::value();
  }
  static const char* value(const ::_moveit_msgs::GraspPlanningRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::GraspPlanningRequest> should match
// service_traits::DataType< ::_moveit_msgs::GraspPlanning >
template<>
struct DataType< ::_moveit_msgs::GraspPlanningRequest>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::GraspPlanning >::value();
  }
  static const char* value(const ::_moveit_msgs::GraspPlanningRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::_moveit_msgs::GraspPlanningResponse> should match
// service_traits::MD5Sum< ::_moveit_msgs::GraspPlanning >
template<>
struct MD5Sum< ::_moveit_msgs::GraspPlanningResponse>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::GraspPlanning >::value();
  }
  static const char* value(const ::_moveit_msgs::GraspPlanningResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::GraspPlanningResponse> should match
// service_traits::DataType< ::_moveit_msgs::GraspPlanning >
template<>
struct DataType< ::_moveit_msgs::GraspPlanningResponse>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::GraspPlanning >::value();
  }
  static const char* value(const ::_moveit_msgs::GraspPlanningResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_GRASPPLANNING_H