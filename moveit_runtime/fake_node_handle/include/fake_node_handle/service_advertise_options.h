/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "ros/service_traits.h"

namespace _ros
{
template <typename MReq, typename MRes>
struct ServiceSpec
{
  typedef MReq RequestType;
  typedef MRes ResponseType;
  typedef std::shared_ptr<RequestType> RequestPtr;
  typedef std::shared_ptr<ResponseType> ResponsePtr;
  typedef std::function<bool(RequestType&, ResponseType&)> CallbackType;

  static bool call(const CallbackType& cb, RequestType& req, ResponseType& res)
  {
    return cb(req, res);
  }
};

class ServiceCallbackHelper
{
public:
  virtual ~ServiceCallbackHelper()
  {
  }
  virtual bool call(void* pReq, void* pRes) = 0;
};

typedef std::shared_ptr<ServiceCallbackHelper> ServiceCallbackHelperPtr;

template <typename Spec>
class ServiceCallbackHelperT : public ServiceCallbackHelper
{
public:
  typedef typename Spec::RequestType RequestType;
  typedef typename Spec::ResponseType ResponseType;
  typedef typename Spec::CallbackType Callback;

  ServiceCallbackHelperT(const Callback& callback) : callback_(callback)
  {
  }

  virtual bool call(void* pReq, void* pRes)
  {
    RequestType& req = *((RequestType*)pReq);
    ResponseType& res = *((ResponseType*)pRes);

    return Spec::call(callback_, req, res);
  }

private:
  Callback callback_;
};

/**
 * \brief Encapsulates all options available for creating a ServiceServer
 */
struct AdvertiseServiceOptions
{
  /**
   * \brief Templated convenience method for filling out md5sum/etc. based on the service request/response types
   * \param _service Service name to advertise on
   * \param _callback Callback to call when this service is called
   */
  template <class MReq, class MRes>
  void init(const std::string& _service, const std::function<bool(MReq&, MRes&)>& _callback)
  {
    namespace st = ros::service_traits;
    if (st::md5sum<MReq>() != st::md5sum<MRes>())
    {
      assert("AdvertiseServiceOptions::init(...) MD5 check error! please check you param.");
    }
    service = _service;
    md5sum = st::md5sum<MReq>();
    helper = std::make_shared<ServiceCallbackHelperT<ServiceSpec<MReq, MRes> > >(_callback);
  }

  /**
   * \brief Templated convenience method for filling out md5sum/etc. based on the service type
   * \param _service Service name to advertise on
   * \param _callback Callback to call when this service is called
   */
  template <class Service>
  void init(const std::string& _service,
            const std::function<bool(typename Service::Request&, typename Service::Response&)>& _callback)
  {
    namespace st = ros::service_traits;
    typedef typename Service::Request Request;
    typedef typename Service::Response Response;
    service = _service;
    md5sum = st::md5sum<Service>();
    helper = std::make_shared<ServiceCallbackHelperT<ServiceSpec<Request, Response> > >(_callback);
  }

  /**
   * \brief Templated convenience method for filling out md5sum/etc. based on the service spec type
   * \param _service Service name to advertise on
   * \param _callback Callback to call when this service is called
   */
  template <class Spec>
  void initBySpecType(const std::string& _service, const typename Spec::CallbackType& _callback)
  {
    namespace st = ros::service_traits;
    typedef typename Spec::RequestType Request;
    typedef typename Spec::ResponseType Response;
    service = _service;
    md5sum = st::md5sum<Request>();
    helper = std::make_shared<ServiceCallbackHelperT<Spec> >(_callback);
  }

  std::string service;              ///< Service name
  std::string md5sum;               ///< MD5 of the service
  ServiceCallbackHelperPtr helper;  ///< Helper object used for creating messages and calling callbacks
};

}  // namespace _ros
