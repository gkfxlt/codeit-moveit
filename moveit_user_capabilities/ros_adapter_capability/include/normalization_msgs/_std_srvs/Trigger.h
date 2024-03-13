// Generated by gencpp from file _std_srvs/Trigger.msg
// DO NOT EDIT!


#ifndef _STD_SRVS_MESSAGE_TRIGGER_H
#define _STD_SRVS_MESSAGE_TRIGGER_H

#include <ros/service_traits.h>


#include <_std_srvs/TriggerRequest.h>
#include <_std_srvs/TriggerResponse.h>


namespace _std_srvs
{

struct Trigger
{

typedef TriggerRequest Request;
typedef TriggerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Trigger
} // namespace _std_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::_std_srvs::Trigger > {
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::_std_srvs::Trigger&) { return value(); }
};

template<>
struct DataType< ::_std_srvs::Trigger > {
  static const char* value()
  {
    return "_std_srvs/Trigger";
  }

  static const char* value(const ::_std_srvs::Trigger&) { return value(); }
};


// service_traits::MD5Sum< ::_std_srvs::TriggerRequest> should match
// service_traits::MD5Sum< ::_std_srvs::Trigger >
template<>
struct MD5Sum< ::_std_srvs::TriggerRequest>
{
  static const char* value()
  {
    return MD5Sum< ::_std_srvs::Trigger >::value();
  }
  static const char* value(const ::_std_srvs::TriggerRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::_std_srvs::TriggerRequest> should match
// service_traits::DataType< ::_std_srvs::Trigger >
template<>
struct DataType< ::_std_srvs::TriggerRequest>
{
  static const char* value()
  {
    return DataType< ::_std_srvs::Trigger >::value();
  }
  static const char* value(const ::_std_srvs::TriggerRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::_std_srvs::TriggerResponse> should match
// service_traits::MD5Sum< ::_std_srvs::Trigger >
template<>
struct MD5Sum< ::_std_srvs::TriggerResponse>
{
  static const char* value()
  {
    return MD5Sum< ::_std_srvs::Trigger >::value();
  }
  static const char* value(const ::_std_srvs::TriggerResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::_std_srvs::TriggerResponse> should match
// service_traits::DataType< ::_std_srvs::Trigger >
template<>
struct DataType< ::_std_srvs::TriggerResponse>
{
  static const char* value()
  {
    return DataType< ::_std_srvs::Trigger >::value();
  }
  static const char* value(const ::_std_srvs::TriggerResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // STD_SRVS_MESSAGE_TRIGGER_H
