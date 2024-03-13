//
// Created by Administrator on 2021/6/16.
//
#include "fake_node_handle/service.h"

using namespace _ros;

bool ServiceClient::exists()
{
  return ServiceCenter::Instance()->exists(service_name_);
}

bool ServiceClient::call(const std::string& service_md5sum, void* req, void* res)
{
  return ServiceCenter::Instance()->call(service_name_, service_md5sum, req, res);
}

ServiceCenter* ServiceCenter::Instance()
{
  static ServiceCenter instance;
  return &instance;
}

ServiceServer ServiceCenter::advertiseService(AdvertiseServiceOptions& ops)
{
  servers_[ops.service] = std::make_shared<AdvertiseServiceOptions>(ops);
  return ServiceServer(ops.service);
}
ServiceClient ServiceCenter::serviceClient(ServiceClientOptions& ops)
{
  clients_[ops.service].emplace_back(new ServiceClientOptions(ops));
  return ServiceClient(ops.service);
}
bool ServiceCenter::exists(const std::string& service)
{
  if (servers_.find(service) == servers_.end())
  {
    return false;
  }
  return false;
}
bool ServiceCenter::call(const std::string& service, const std::string& service_md5sum, void* req, void* res)
{
  if (servers_.find(service) == servers_.end())
  {
    return false;
  }

  auto& server = servers_[service];
  if (server->md5sum != service_md5sum)
  {
    assert("ServiceCenter::call(...) MD5 check error! please check you param.");
    return false;
  }

  return server->helper->call(req, res);
}

void ServiceCenter::clearAll()
{
  servers_.clear();
  clients_.clear();
}
