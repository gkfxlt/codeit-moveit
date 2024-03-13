
#include "fake_node_handle/fake_node_handle.h"
#include <stdboost/string_operation.h>

#include <fake_node_handle/param.h>

using namespace _ros;

struct NodeHandleBackingCollection
{
  ParamCenter* paramCenter;
  SubjectCenter* subjectCenter;
  ServiceCenter* serviceCenter;
  TimerCenter* timerCenter;

  NodeHandleBackingCollection()
    : paramCenter(ParamCenter::Instance())
    , subjectCenter(SubjectCenter::Instance())
    , serviceCenter(ServiceCenter::Instance())
    , timerCenter(TimerCenter::Instance())
  {
  }
};
static NodeHandleBackingCollection collection;


NodeHandle::NodeHandle(const std::string& ns)
{
  std::string s = ns;
  stdboost::trim(s);

  while (stdboost::starts_with(s, "/"))
    s.erase(s.begin());

  while (stdboost::ends_with(s, "/"))
    s.erase(--s.end());

  ns_ = s.empty() ? "/" : "/" + s + "/";
}


///
/// \param param
/// \param ops
void NodeHandle::setParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops) const
{
  collection.paramCenter->setParam(resolveName(param), ops);
}

bool NodeHandle::getParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops) const
{
  return collection.paramCenter->getParam(resolveName(param), ops);
}

bool NodeHandle::hasParam(const std::string& param) const
{
  return collection.paramCenter->hasParam(resolveName(param));
}

bool NodeHandle::searchParam(const std::string& param_name, std::string& full_param_name) const
{
  return collection.paramCenter->searchParam(param_name, full_param_name);
}

std::map<std::string, std::shared_ptr<ParamValueOptions>>& NodeHandle::getAllParam()
{
  return collection.paramCenter->getAllParam();
}

std::string NodeHandle::resolveName(const std::string& name) const
{
  std::string s = (*name.begin() == '/') ? name : ns_ + name;
  stdboost::replace_str_once(s, "~", "move_group");
  return s;
}

///
/// \param ops
/// \return
Publisher NodeHandle::advertise(AdvertiseOptions& ops)
{
  ops.topic = resolveName(ops.topic);
  return collection.subjectCenter->advertise(ops);
}
Subscriber NodeHandle::subscribe(SubscribeOptions& ops)
{
  ops.topic = resolveName(ops.topic);
  return collection.subjectCenter->subscribe(ops);
}

///
/// \param ops
/// \return
ServiceServer NodeHandle::advertiseService(AdvertiseServiceOptions& ops)
{
  ops.service = resolveName(ops.service);
  return collection.serviceCenter->advertiseService(ops);
}

ServiceClient NodeHandle::serviceClient(ServiceClientOptions& ops)
{
  ops.service = resolveName(ops.service);
  return collection.serviceCenter->serviceClient(ops);
}

Timer NodeHandle::createTimer(ros::Duration period, std::function<void()> callback, bool oneshot, bool autostart)
{
  return collection.timerCenter->createTimer(period, callback, oneshot, autostart);
}

WallTimer NodeHandle::createWallTimer(ros::WallDuration period, std::function<void()> callback, bool oneshot,
                                      bool autostart)
{
  return collection.timerCenter->createWallTimer(period, callback, oneshot, autostart);
}

void NodeHandle::clear() const
{
  collection.paramCenter->clearAll();
  collection.subjectCenter->clearAll();
  collection.serviceCenter->clearAll();
}

std::string NodeHandle::getNamespace()
{
    return ns_;
}

