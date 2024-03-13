// Generated by gencpp from file _moveit_msgs/SaveRobotStateToWarehouse.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_SAVEROBOTSTATETOWAREHOUSE_H
#define _MOVEIT_MSGS_MESSAGE_SAVEROBOTSTATETOWAREHOUSE_H

#include <ros/service_traits.h>


#include <_moveit_msgs/SaveRobotStateToWarehouseRequest.h>
#include <_moveit_msgs/SaveRobotStateToWarehouseResponse.h>


namespace _moveit_msgs
{

struct SaveRobotStateToWarehouse
{

typedef SaveRobotStateToWarehouseRequest Request;
typedef SaveRobotStateToWarehouseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SaveRobotStateToWarehouse
} // namespace _moveit_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouse > {
  static const char* value()
  {
    return "6e80fdee0b8efd0e265bd085def4d166";
  }

  static const char* value(const ::_moveit_msgs::SaveRobotStateToWarehouse&) { return value(); }
};

template<>
struct DataType< ::_moveit_msgs::SaveRobotStateToWarehouse > {
  static const char* value()
  {
    return "_moveit_msgs/SaveRobotStateToWarehouse";
  }

  static const char* value(const ::_moveit_msgs::SaveRobotStateToWarehouse&) { return value(); }
};


// service_traits::MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouseRequest> should match
// service_traits::MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouse >
template<>
struct MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouse >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveRobotStateToWarehouseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::SaveRobotStateToWarehouseRequest> should match
// service_traits::DataType< ::_moveit_msgs::SaveRobotStateToWarehouse >
template<>
struct DataType< ::_moveit_msgs::SaveRobotStateToWarehouseRequest>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::SaveRobotStateToWarehouse >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveRobotStateToWarehouseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouseResponse> should match
// service_traits::MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouse >
template<>
struct MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::_moveit_msgs::SaveRobotStateToWarehouse >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveRobotStateToWarehouseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::_moveit_msgs::SaveRobotStateToWarehouseResponse> should match
// service_traits::DataType< ::_moveit_msgs::SaveRobotStateToWarehouse >
template<>
struct DataType< ::_moveit_msgs::SaveRobotStateToWarehouseResponse>
{
  static const char* value()
  {
    return DataType< ::_moveit_msgs::SaveRobotStateToWarehouse >::value();
  }
  static const char* value(const ::_moveit_msgs::SaveRobotStateToWarehouseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_SAVEROBOTSTATETOWAREHOUSE_H