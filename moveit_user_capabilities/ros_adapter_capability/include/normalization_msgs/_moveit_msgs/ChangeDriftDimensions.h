// Generated by gencpp from file _moveit_msgs/ChangeDriftDimensions.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_CHANGEDRIFTDIMENSIONS_H
#define _MOVEIT_MSGS_MESSAGE_CHANGEDRIFTDIMENSIONS_H

#include <ros/service_traits.h>


#include <_moveit_msgs/ChangeDriftDimensionsRequest.h>
#include <_moveit_msgs/ChangeDriftDimensionsResponse.h>


namespace _moveit_msgs
{

struct ChangeDriftDimensions
{

typedef ChangeDriftDimensionsRequest Request;
typedef ChangeDriftDimensionsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ChangeDriftDimensions
} // namespace _moveit_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::_moveit_msgs::ChangeDriftDimensions > {
  static const char* value()
  {
    return "0d34c8d563fea2efff65829c37132a15";
  }

  static const char* value(const ::_moveit_msgs::ChangeDriftDimensions&) { return value(); }
};

template<>
struct DataType< ::_moveit_msgs::ChangeDriftDimensions > {
  static const char* value()
  {
    return "_moveit_msgs/ChangeDriftDimensions";
  }

  static const char* value(const ::_moveit_msgs::ChangeDriftDimensions&) { return value(); }
};


// service_traits::MD5Sum< ::_moveit_msgs::ChangeDriftDimensionsRequest> should match
// service_traits::MD5Sum< ::_moveit_msgs::ChangeDriftDimensions >
template<>
struct MD5Sum< ::_moveit_msgs::ChangeDriftDimensionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::ChangeDriftDimensions >::value();
  }
  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::ChangeDriftDimensionsRequest> should match
// service_traits::DataType< ::_moveit_msgs::ChangeDriftDimensions >
template<>
struct DataType< ::_moveit_msgs::ChangeDriftDimensionsRequest>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::ChangeDriftDimensions >::value();
  }
  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::_moveit_msgs::ChangeDriftDimensionsResponse> should match
// service_traits::MD5Sum< ::_moveit_msgs::ChangeDriftDimensions >
template<>
struct MD5Sum< ::_moveit_msgs::ChangeDriftDimensionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::ChangeDriftDimensions >::value();
  }
  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::ChangeDriftDimensionsResponse> should match
// service_traits::DataType< ::_moveit_msgs::ChangeDriftDimensions >
template<>
struct DataType< ::_moveit_msgs::ChangeDriftDimensionsResponse>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::ChangeDriftDimensions >::value();
  }
  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_CHANGEDRIFTDIMENSIONS_H
