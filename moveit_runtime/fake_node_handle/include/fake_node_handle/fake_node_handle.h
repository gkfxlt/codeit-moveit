#pragma once

#include "fake_node_handle/param.h"
#include "fake_node_handle/topic.h"
#include "fake_node_handle/service.h"
#include "fake_node_handle/action.h"
#include "fake_node_handle/timer.h"
#include "fake_node_handle/visibility_control.hpp"

namespace _ros
{
class FAKE_NODE_HANDLE_PUBLIC NodeHandle
{
public:
  explicit NodeHandle(const std::string& ns = "");

  //参数
  template <typename T>
  void setParam(const std::string& key, const T& val) const
  {
    auto ops = std::make_shared<ParamValueOptions>();
    ops->template init<T>(key, val);
    setParam(key, ops);
  }

  void setParam(const std::string& key, const char* val) const
  {
    setParam(key, std::string(val));
  }

  void setParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops) const;

  template <typename T>
  bool getParam(const std::string& param, T& val) const
  {
    std::shared_ptr<ParamValueOptions> ops;
    if (!getParam(param, ops))
    {
      return false;
    }

    if (ops->type_ != typeid(T).name())
    {
      assert("NodeHandle::getParam(...) param value type error! please check you param.");
      return false;
    }

    try
    {
      val = std::any_cast<T>(ops->val_);
    }
    catch (std::bad_any_cast& e)
    {
      return false;
    }
    return true;
  }

  bool getParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops) const;

  template <typename T>
  bool param(const std::string& param_name, T& param_val, const T& default_val) const
  {
    if (hasParam(param_name))
    {
      if (getParam(param_name, param_val))
      {
        return true;
      }
    }

    param_val = default_val;
    return false;
  }

  template <typename T>
  T param(const std::string& param_name, const T& default_val) const
  {
    T param_val;
    param(param_name, param_val, default_val);
    return param_val;
  }

  bool hasParam(const std::string& param) const;
  bool searchParam(const std::string& param_name, std::string& full_param_name) const;
  std::map<std::string, std::shared_ptr<ParamValueOptions>>& getAllParam();
  std::string resolveName(const std::string& name) const;

  /// Topic
  ///
  /// advertise
  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size = 1, bool latch = false)
  {
    AdvertiseOptions ops;
    ops.template init<M>(topic, queue_size);
    return advertise(ops);
  }

  Publisher advertise(AdvertiseOptions& ops);

  ///
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj)
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, std::bind(fp, obj, std::placeholders::_1));
    return subscribe(ops);
  }
  /// and the const version
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M) const, T* obj)
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, std::bind(fp, obj, std::placeholders::_1));
    return subscribe(ops);
  }

  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(const std::shared_ptr<M const>&),
                       T* obj)
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, std::bind(fp, obj, std::placeholders::_1));
    return subscribe(ops);
  }
  /// and the const version
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       void (T::*fp)(const std::shared_ptr<M const>&) const, T* obj)
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, std::bind(fp, obj, std::placeholders::_1));
    return subscribe(ops);
  }

  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), const std::shared_ptr<T>& obj)
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, std::bind(fp, obj.get(), std::placeholders::_1));
    return subscribe(ops);
  }
  /// and the const version
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M) const,
                       const std::shared_ptr<T>& obj)
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, std::bind(fp, obj.get(), std::placeholders::_1));
    return subscribe(ops);
  }

  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(const std::shared_ptr<M const>&),
                       const std::shared_ptr<T>& obj)
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, std::bind(fp, obj.get(), std::placeholders::_1));
    return subscribe(ops);
  }
  /// and the const version
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       void (T::*fp)(const std::shared_ptr<M const>&) const, const std::shared_ptr<T>& obj)
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, std::bind(fp, obj.get(), std::placeholders::_1));
    return subscribe(ops);
  }

  template <class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (*fp)(M))
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, fp);
    return subscribe(ops);
  }

  template <class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (*fp)(const std::shared_ptr<M const>&))
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, fp);
    return subscribe(ops);
  }

  template <class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       const std::function<void(const std::shared_ptr<M const>&)>& callback)
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, callback);
    return subscribe(ops);
  }

  //  template <class M, class C>
  template <class C>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, const std::function<void(C)>& callback)
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<C>(topic, queue_size, callback);
    return subscribe(ops);
  }

  Subscriber subscribe(SubscribeOptions& ops);

  ///
  ///
  /// advertiseService
  template <class T, class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool (T::*srv_func)(MReq&, MRes&), T* obj)
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, std::bind(srv_func, obj, std::placeholders::_1, std::placeholders::_2));
    return advertiseService(ops);
  }

  template <class T, class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool (T::*srv_func)(MReq&, MRes&),
                                 const std::shared_ptr<T>& obj)
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, std::bind(srv_func, obj.get(), std::placeholders::_1, std::placeholders::_2));
    return advertiseService(ops);
  }

  template <class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool (*srv_func)(MReq&, MRes&))
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, srv_func);
    return advertiseService(ops);
  }

  template <class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, const std::function<bool(MReq&, MRes&)>& callback)
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, callback);
    return advertiseService(ops);
  }

  template <class S>
  ServiceServer advertiseService(const std::string& service, const std::function<bool(S&)>& callback)
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<S>(service, callback);
    return advertiseService(ops);
  }

  ServiceServer advertiseService(AdvertiseServiceOptions& ops);

  /// serviceClient
  template <class MReq, class MRes>
  ServiceClient serviceClient(const std::string& service_name)
  {
    ServiceClientOptions ops;
    ops.template init<MReq, MRes>(service_name);
    return serviceClient(ops);
  }

  template <class Service>
  ServiceClient serviceClient(const std::string& service_name)
  {
    ServiceClientOptions ops;
    ops.template init<Service>(service_name);
    return serviceClient(ops);
  }

  ServiceClient serviceClient(ServiceClientOptions& ops);


  /// createTimer
  template <class T>
  Timer createTimer(ros::Duration period, void (T::*callback)(), T* obj, bool oneshot = false,
                    bool autostart = true)
  {
    return createTimer(period, std::bind(callback, obj), oneshot, autostart);
  }

  Timer createTimer(ros::Duration period, std::function<void()> callback, bool oneshot = false,
                    bool autostart = true);

  /// createWallTimer
  template <class T>
  WallTimer createWallTimer(ros::WallDuration period, void (T::*callback)(), T* obj, bool oneshot = false,
                            bool autostart = true)
  {
    return createWallTimer(period, std::bind(callback, obj), oneshot, autostart);
  }

  WallTimer createWallTimer(ros::WallDuration period, std::function<void()> callback, bool oneshot = false,
                            bool autostart = true);


  void clear() const;

  std::string getNamespace();

private:
  std::string ns_;
};

}  // namespace _ros
