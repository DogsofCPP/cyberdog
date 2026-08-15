// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef CYBERDOG_COMMON__CYBERDOG_LCM_HPP_
#define CYBERDOG_COMMON__CYBERDOG_LCM_HPP_
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <atomic>

#define  LCM_ADDR  "udpm://239.255.76.67:8689?ttl=1"

namespace cyberdog
{
namespace common
{
/**
 * @brief RPC 调用的客户端类
 *
 * @tparam Req 请求数据模板
 * @tparam Res 返回数据模板
 */
template<typename Req, typename Res>
class LcmClient final
{
public:
  LcmClient(const LcmClient &) = delete;
  LcmClient(LcmClient &&) = delete;

  /**
   * @brief Construct a new Lcm Client object
   *
   * @param name RPC调用的通道名， 需要与服务端对应
   */
  explicit LcmClient(const std::string & name)
  {
    if (!name.empty()) {
      request_name_ = name + std::string("_request");
      response_name_ = name + std::string("_response");
    } else {
      request_name_ = std::string("_request");
      response_name_ = std::string("_response");
    }

    lcm_ptr_ = std::make_shared<lcm::LCM>(LCM_ADDR);
    is_service_valid_ = true;
    lcm_ptr_->subscribe(response_name_, &LcmClient<Req, Res>::ResponseCallback, this);
  }

  ~LcmClient()
  {
    // this->lcm_ptr_->unsubscribe();
  }

public:
  /**
   * @brief 发起一个RPC调用
   *
   * @param request 请求的数据
   * @param response 返回的数据
   * @param mill_time 超时设置， 单位毫秒
   * @return true 调用成
   * @return false 调用失败， 此时reponse的后续使用需注意内存访问安全问题
   */
  bool Request(const Req & request, Res & response, int mill_time)
  {
    if (!is_service_valid_) {
      return false;
    }

    is_service_valid_ = false;
    bool result = false;

    lcm_ptr_->publish(request_name_, &request);
    is_service_wait_ = true;

    if (this->lcm_ptr_->handleTimeout(mill_time) <= 0) {
      result = false;
    } else {
      std::lock_guard<std::mutex> lk(mutex_);
      response = response_data_;
      result = true;
    }

    is_service_wait_ = false;
    is_service_valid_ = true;
    return result;
  }

private:
  void ResponseCallback(
    const lcm::ReceiveBuffer * rBuf, const std::string & topic_name,
    const Res * response)
  {
    (void) rBuf;
    (void) topic_name;

    if (!is_service_wait_) {
      return;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    response_data_ = *response;
  }

private:
  std::shared_ptr<lcm::LCM> lcm_ptr_ {nullptr};
  std::string request_name_ {""};
  std::string response_name_ {""};
  std::atomic_bool is_service_valid_ {false};
  std::atomic_bool is_service_wait_ {false};
  std::mutex mutex_;
  Res response_data_;
};  // class LcmClient

/**
 * @brief RPC调用服务端类
 *
 * @tparam Req 请求数据模板
 * @tparam Res 返回数据模板
 */
template<typename Req, typename Res>
class LcmServer final
{
public:
  LcmServer(const LcmServer &) = delete;
  LcmServer(LcmServer &&) = delete;

  /**
   * @brief Construct a new Lcm Server object
   *
   * @param name 调用通道名称，需要与客户端对应
   * @param func 回调函数，需要实例化代码中自行实现
   */
  LcmServer(const std::string & name, std::function<void(const Req &, Res &)> func)
  : request_name_(name + std::string("_request")),
    response_name_(name + std::string("_response")),
    exit_(false)
  {
    lcm_ptr_ = std::make_shared<lcm::LCM>(LCM_ADDR);
    callback_ = func;
    lcm_ptr_->subscribe(request_name_, &LcmServer<Req, Res>::RequestCallback, this);
  }

  ~LcmServer()
  {
  }

  /**
   * @brief 接收消息的启动代码
   *          1. 要求必须调用该函数，否则无法接收消息并给出响应
   *          2. 该函数会产生阻塞，需要注意
   *
   */
  void Spin()
  {
    // while (exit_ && 0 == lcm_ptr_->handleTimeout(10)) {}
    while (0 == lcm_ptr_->handle()) {
      if (exit_) {
        break;
      }
    }
  }

  void Exit()
  {
    exit_ = true;
    lcm_ptr_ = nullptr;
  }

public:
  void RequestCallback(
    const lcm::ReceiveBuffer * rBuf, const std::string & topic_name,
    const Req * request)
  {
    (void) rBuf;
    (void) topic_name;
    Res response;
    callback_(*request, response);
    lcm_ptr_->publish(response_name_, &response);
  }

private:
  std::shared_ptr<lcm::LCM> lcm_ptr_ {nullptr};
  std::string request_name_ {""};
  std::string response_name_ {""};
  std::function<void(const Req &, Res &)> callback_;
  bool exit_;
};  // class LcmServer

}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_LCM_HPP_
