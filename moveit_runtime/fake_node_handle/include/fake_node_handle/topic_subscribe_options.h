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

#include "ros/message_traits.h"

namespace _ros
{
template <typename M>
struct ParameterAdapter
{
  typedef typename std::remove_reference<typename std::remove_const<M>::type>::type Message;
  typedef M Parameter;
  static Parameter getParameter(void* param)
  {
    Message* msg = static_cast<Message*>(param);
    return *msg;
  }
};

template <typename M>
struct ParameterAdapter<const std::shared_ptr<M const>&>
{
  typedef typename std::remove_reference<typename std::remove_const<M>::type>::type Message;
  typedef const std::shared_ptr<Message const> Parameter;
  static Parameter getParameter(void* param)
  {
    Message const* msg = static_cast<Message const*>(param);
    return std::shared_ptr<Message const>(msg, [](Message const*) {});
  }
};

template <typename M>
struct ParameterAdapter<const std::shared_ptr<M>&>
{
  typedef typename std::remove_reference<typename std::remove_const<M>::type>::type Message;
  typedef std::shared_ptr<Message> Parameter;
  static Parameter getParameter(void* param)
  {
    Message* msg = static_cast<Message*>(param);
    return std::shared_ptr<Message>(msg, [](Message*) {});
  }
};

template <typename M>
struct ParameterAdapter<const M&>
{
  typedef typename std::remove_reference<typename std::remove_const<M>::type>::type Message;
  typedef const M& Parameter;
  static Parameter getParameter(void* param)
  {
    Message* msg = static_cast<Message*>(param);
    return *msg;
  }
};

template <typename M>
struct ParameterAdapter<std::shared_ptr<M const> >
{
  typedef typename std::remove_reference<typename std::remove_const<M>::type>::type Message;
  typedef std::shared_ptr<Message const> Parameter;
  static Parameter getParameter(void* param)
  {
    Message const* msg = static_cast<Message const*>(param);
    return std::shared_ptr<Message const>(msg, [](Message const*) {});
  }
};

template <typename M>
struct ParameterAdapter<std::shared_ptr<M> >
{
  typedef typename std::remove_reference<typename std::remove_const<M>::type>::type Message;
  typedef std::shared_ptr<Message> Parameter;
  static Parameter getParameter(void* param)
  {
    Message* msg = static_cast<Message*>(param);
    return std::shared_ptr<Message>(msg, [](Message*) {});
  }
};

class SubscriptionCallbackHelper
{
public:
  virtual ~SubscriptionCallbackHelper()
  {
  }

  virtual void call(void* param) = 0;
};
typedef std::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

/**
 * \brief Concrete generic implementation of
 * SubscriptionCallbackHelper for any normal message type.  Use
 * directly with care, this is mostly for internal use.
 */
template <typename P, typename Enabled = void>
class SubscriptionCallbackHelperT : public SubscriptionCallbackHelper
{
public:
  typedef ParameterAdapter<P> Adapter;
  typedef std::function<void(typename Adapter::Parameter)> Callback;

  explicit SubscriptionCallbackHelperT(const Callback& callback) : callback_(callback)
  {
  }

  virtual void call(void* param)
  {
    callback_(ParameterAdapter<P>::getParameter(param));
  }

private:
  Callback callback_;
};

/**
 * \brief Encapsulates all options available for creating a Subscriber
 */
struct SubscribeOptions
{
  /**
   * \brief Templated initialization, templated on callback parameter type.  Supports any callback parameters supported
   * by the SubscriptionCallbackAdapter \param _topic Topic to subscribe on \param _queue_size Number of incoming
   * messages to queue up for processing (messages in excess of this queue capacity will be discarded). \param _callback
   * Callback to call when a message arrives on this topic
   */
  template <class P>
  void initByFullCallbackType(const std::string& _topic, uint32_t _queue_size, const std::function<void(P)>& _callback)
  {
    typedef typename ParameterAdapter<P>::Message MessageType;
    topic = _topic;
    md5sum = ros::message_traits::md5sum<MessageType>();
    helper = std::make_shared<SubscriptionCallbackHelperT<P> >(_callback);
  }

  /**
   * \brief Templated initialization, templated on message type.  Only supports "const std::shared_ptr<M const>&"
   * callback types \param _topic Topic to subscribe on \param _queue_size Number of incoming messages to queue up for
   *        processing (messages in excess of this queue capacity will be
   *        discarded).
   * \param _callback Callback to call when a message arrives on this topic
   */
  template <class M>
  void init(const std::string& _topic, uint32_t _queue_size,
            const std::function<void(const std::shared_ptr<M const>&)>& _callback)
  {
    typedef typename ParameterAdapter<M>::Message MessageType;
    topic = _topic;
    md5sum = ros::message_traits::md5sum<MessageType>();
    helper = std::make_shared<SubscriptionCallbackHelperT<const std::shared_ptr<MessageType const>&> >(_callback);
  }

  std::string topic;                     ///< Topic to subscribe to
  std::string md5sum;                    ///< MD5 of the message datatype
  SubscriptionCallbackHelperPtr helper;  ///< Helper object used to get create messages and call callbacks
};

}  // namespace _ros
