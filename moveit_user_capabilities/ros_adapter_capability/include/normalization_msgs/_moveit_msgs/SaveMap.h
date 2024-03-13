// Generated by gencpp from file _moveit_msgs/SaveMap.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_SAVEMAP_H
#define _MOVEIT_MSGS_MESSAGE_SAVEMAP_H

#include <ros/service_traits.h>


#include <_moveit_msgs/SaveMapRequest.h>
#include <_moveit_msgs/SaveMapResponse.h>


namespace _moveit_msgs
{

struct SaveMap
{

typedef SaveMapRequest Request;
typedef SaveMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SaveMap
} // namespace _moveit_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::_moveit_msgs::SaveMap > {
  static const char* value()
  {
    return "93a4bc4c60dc17e2a69e3fcaaa25d69d";
  }

  static const char* value(const ::_moveit_msgs::SaveMap&) { return value(); }
};

template<>
struct DataType< ::_moveit_msgs::SaveMap > {
  static const char* value()
  {
    return "_moveit_msgs/SaveMap";
  }

  static const char* value(const ::_moveit_msgs::SaveMap&) { return value(); }
};


// service_traits::MD5Sum< ::_moveit_msgs::SaveMapRequest> should match
// service_traits::MD5Sum< ::_moveit_msgs::SaveMap >
template<>
struct MD5Sum< ::_moveit_msgs::SaveMapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::SaveMap >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveMapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::SaveMapRequest> should match
// service_traits::DataType< ::_moveit_msgs::SaveMap >
template<>
struct DataType< ::_moveit_msgs::SaveMapRequest>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::SaveMap >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveMapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::_moveit_msgs::SaveMapResponse> should match
// service_traits::MD5Sum< ::_moveit_msgs::SaveMap >
template<>
struct MD5Sum< ::_moveit_msgs::SaveMapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::SaveMap >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveMapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::SaveMapResponse> should match
// service_traits::DataType< ::_moveit_msgs::SaveMap >
template<>
struct DataType< ::_moveit_msgs::SaveMapResponse>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::SaveMap >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveMapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_SAVEMAP_H