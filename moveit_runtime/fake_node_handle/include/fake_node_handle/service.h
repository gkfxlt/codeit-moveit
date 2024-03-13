//
// Created by Administrator on 2021/6/7.
//

#pragma once

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <cassert>
#include "ros/duration.h"

#include "fake_node_handle/service_advertise_options.h"
#include "fake_node_handle/service_client_options.h"

#include "fake_node_handle/visibility_control.hpp"


namespace _ros
{
class FAKE_NODE_HANDLE_PUBLIC ServiceServer
{
public:
  ServiceServer() = default;

  explicit ServiceServer(const std::string& service_name) : service_name_(service_name)
  {
  }

  const std::string& getService()
  {
    return service_name_;
  }

private:
  std::string service_name_;
};

class FAKE_NODE_HANDLE_PUBLIC ServiceClient
{
public:
  ServiceClient() = default;

  explicit ServiceClient(const std::string& service_name) : service_name_(service_name)
  {
  }

  template <typename Service>
  bool call(Service& service)
  {
    namespace st = ros::service_traits;
    return call(st::md5sum(service), &service.request, &service.response);
  }

  template <typename MReq, typename MRes>
  bool call(MReq& req, MRes& res)
  {
    namespace st = ros::service_traits;
    if (strcmp(st::md5sum(req), st::md5sum(res)))
    {
      assert("ServiceClient::call(...) MD5 check error! please check you param.");
      return false;
    }

    return call(st::md5sum(req), &req, &res);
  }

  bool call(const std::string& service_md5sum, void* req, void* res);

  bool exists();

  void waitForExistence(ros::Duration d)
  {
  }

private:
  std::string service_name_;
};

class ServiceCenter
{
public:
  static ServiceCenter* Instance();

  ServiceServer advertiseService(AdvertiseServiceOptions& ops);
  ServiceClient serviceClient(ServiceClientOptions& ops);

  bool exists(const std::string& service);
  bool call(const std::string& service, const std::string& service_md5sum, void* req, void* res);
  void clearAll();

private:
  ServiceCenter() = default;
  std::map<std::string, std::shared_ptr<AdvertiseServiceOptions>> servers_;
  std::map<std::string, std::vector<std::shared_ptr<ServiceClientOptions>>> clients_;
};

}  // namespace _ros
