// Generated by gencpp from file _octomap_msgs/BoundingBoxQuery.msg
// DO NOT EDIT!


#ifndef _OCTOMAP_MSGS_MESSAGE_BOUNDINGBOXQUERY_H
#define _OCTOMAP_MSGS_MESSAGE_BOUNDINGBOXQUERY_H

#include <ros/service_traits.h>


#include <_octomap_msgs/BoundingBoxQueryRequest.h>
#include <_octomap_msgs/BoundingBoxQueryResponse.h>


namespace _octomap_msgs
{

struct BoundingBoxQuery
{

typedef BoundingBoxQueryRequest Request;
typedef BoundingBoxQueryResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct BoundingBoxQuery
} // namespace _octomap_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::_octomap_msgs::BoundingBoxQuery > {
  static const char* value()
  {
    return "93aa3d73b866f04880927745f4aab303";
  }

  static const char* value(const ::_octomap_msgs::BoundingBoxQuery&) { return value(); }
};

template<>
struct DataType< ::_octomap_msgs::BoundingBoxQuery > {
  static const char* value()
  {
    return "_octomap_msgs/BoundingBoxQuery";
  }

  static const char* value(const ::_octomap_msgs::BoundingBoxQuery&) { return value(); }
};


// service_traits::MD5Sum< ::_octomap_msgs::BoundingBoxQueryRequest> should match
// service_traits::MD5Sum< ::_octomap_msgs::BoundingBoxQuery >
template<>
struct MD5Sum< ::_octomap_msgs::BoundingBoxQueryRequest>
{
  static const char* value()
  {
    return MD5Sum< ::_octomap_msgs::BoundingBoxQuery >::value();
  }
  static const char* value(const ::_octomap_msgs::BoundingBoxQueryRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::_octomap_msgs::BoundingBoxQueryRequest> should match
// service_traits::DataType< ::_octomap_msgs::BoundingBoxQuery >
template<>
struct DataType< ::_octomap_msgs::BoundingBoxQueryRequest>
{
  static const char* value()
  {
    return DataType< ::_octomap_msgs::BoundingBoxQuery >::value();
  }
  static const char* value(const ::_octomap_msgs::BoundingBoxQueryRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::_octomap_msgs::BoundingBoxQueryResponse> should match
// service_traits::MD5Sum< ::_octomap_msgs::BoundingBoxQuery >
template<>
struct MD5Sum< ::_octomap_msgs::BoundingBoxQueryResponse>
{
  static const char* value()
  {
    return MD5Sum< ::_octomap_msgs::BoundingBoxQuery >::value();
  }
  static const char* value(const ::_octomap_msgs::BoundingBoxQueryResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::_octomap_msgs::BoundingBoxQueryResponse> should match
// service_traits::DataType< ::_octomap_msgs::BoundingBoxQuery >
template<>
struct DataType< ::_octomap_msgs::BoundingBoxQueryResponse>
{
  static const char* value()
  {
    return DataType< ::_octomap_msgs::BoundingBoxQuery >::value();
  }
  static const char* value(const ::_octomap_msgs::BoundingBoxQueryResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OCTOMAP_MSGS_MESSAGE_BOUNDINGBOXQUERY_H
