//
// Created by fan on 2021/7/2.
//

#ifndef CODEIT_MOVEIT_TEST_SERVICE_HELPER_H
#define CODEIT_MOVEIT_TEST_SERVICE_HELPER_H

struct AddRequest
{
  int add1;
  int add2;
};

struct AddResponse
{
  int result;
};

struct GetAddResult
{
  typedef AddRequest Request;
  typedef AddResponse Response;
  Request request;
  Response response;

  typedef Request RequestType;
  typedef Response ResponseType;

};  // struct GetMotionPlan

namespace ros::service_traits
{
template <>
struct MD5Sum<AddRequest>
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }
  static const char* value(const AddRequest&)
  {
    return value();
  }
};

template <>
struct MD5Sum<AddResponse>
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }
  static const char* value(const AddResponse&)
  {
    return value();
  }
};

template <>
struct MD5Sum<GetAddResult>
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }
  static const char* value(const GetAddResult&)
  {
    return value();
  }
};
}  // namespace ros::service_traits

#endif  // CODEIT_MOVEIT_TEST_SERVICE_HELPER_H
