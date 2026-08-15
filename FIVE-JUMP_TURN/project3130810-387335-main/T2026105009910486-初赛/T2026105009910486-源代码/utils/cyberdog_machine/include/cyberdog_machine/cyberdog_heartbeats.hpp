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
#ifndef CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
#define CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <utility>
#include <thread>
#include <condition_variable>
#include "rclcpp/node.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/msg/heartbeats.hpp"

namespace cyberdog
{
namespace machine
{
struct HeartClick
{
  uint8_t Push()
  {
    uint8_t tmp_val = count_.fetch_add(1);
    return tmp_val;
  }

  void Pop()
  {
    count_.store(0);
  }

  std::atomic_uint8_t count_ {0};
};  // struct HeartClick

/**
 * @brief 心跳类，具体参数及定义请参考心跳设计文档：
 *        https://xiaomi.f.mioffice.cn/docs/dock43ed3sAGiJmwgnPFIm7I5Kf
 */
class HeartBeats
{
  using BeatsMap = std::map<std::string, std::shared_ptr<HeartClick>>;

public:
  /**
   * @brief Construct a new Heart Beats object
   *
   * @param duration 心跳发送、检测间隔，
   * @param lost_limit 丢失/超时 限制，达到该次数会触发心跳异常
   */
  HeartBeats(
    int32_t duration, int8_t lost_limit,
    std::function<void()> on_keep = nullptr,
    std::function<void(const std::string &, bool)> on_lost = nullptr)
  : beats_duration_(duration), lost_limit_(lost_limit),
    keep_callback_(on_keep), lost_callback_(on_lost),
    notify_callback_(std::function<void()>())
  {}
  explicit HeartBeats(int32_t duration, std::function<void()> on_keep = nullptr)
  : beats_duration_(duration),
    keep_callback_(on_keep), lost_callback_(nullptr),
    notify_callback_(std::function<void()>())
  {}
  virtual ~HeartBeats()
  {
    {
      std::lock_guard<std::mutex> lck(exit_mut_);
      exit_ = true;
      exit_cond_.notify_all();
    }
    if (thread_check_.joinable()) {
      thread_check_.join();
    }
    if (thread_cycle_.joinable()) {
      thread_cycle_.join();
    }
  }

  /**
   * @brief 配置心跳监听对象，单一发送模块不需要配置
   *
   * @param target_vec 监听对象name容器
   */
  void HeartConfig(
    const std::vector<std::string> & target_vec,
    std::function<void()> on_notify = std::function<void()>(),
    int32_t notify_duration = 0)
  {
    if (target_vec.empty()) {
      return;
    }
    for (auto iter = target_vec.begin(); iter != target_vec.end(); ++iter) {
      std::string tmp_name = std::string(*iter);
      auto tmp_ptr = std::make_shared<HeartClick>();
      beats_map_.insert(std::make_pair(tmp_name, tmp_ptr));
    }
    notify_callback_ = on_notify;
    notify_interval_ = notify_duration / beats_duration_;
  }

  /**
   * @brief 注册监听异常回调函数
   *
   * @param callback 回调函数以引用形式得到异常模块的名字
   * @return true
   * @return false
   */
  bool RegisterLostCallback(std::function<void(const std::string &, bool)> on_lost)
  {
    if (beats_map_.empty()) {
      return false;
    }
    if (on_lost == nullptr) {
      return false;
    } else {
      lost_callback_ = on_lost;
    }
    return true;
  }

  /**
   * @brief 注册发送回调函数
   *
   * @param callback 回调函数以void(void)形式引用，即允许不同的通信方式发送心跳数据
   * @return true
   * @return false
   */
  bool RegisterKeepCallback(std::function<void()> on_keep)
  {
    if (on_keep == nullptr) {
      return false;
    } else {
      keep_callback_ = on_keep;
    }
    return true;
  }

  /**
   * @brief 心跳更新函数，监听方以ros或lcm、grpc等监听心跳，将name信息更新至该函数用于心跳刷新
   *
   * @param name
   */
  void HeartUpdate(const std::string & name)
  {
    auto iter = beats_map_.find(name);
    if (iter != beats_map_.end()) {
      // std::cout << "receive once" << std::endl;
      iter->second->Pop();
    }
  }

  /**
   * @brief 心跳启动函数，要求必须执行
   *
   */
  void HeartRun()
  {
    if (lost_callback_ != nullptr) {
      std::cout << "heartrun check\n";
      // auto t = std::thread(std::bind(&HeartBeats::BeatsCheck, this));
      // t.detach();
      thread_check_ = std::thread(std::bind(&HeartBeats::BeatsCheck, this));
    }
    if (keep_callback_ != nullptr) {
      std::cout << "heartrun keep\n";
      // auto t = std::thread(std::bind(&HeartBeats::BeatsCycle, this));
      // t.detach();
      thread_cycle_ = std::thread(std::bind(&HeartBeats::BeatsCycle, this));
    }
  }

private:
  void BeatsCycle()
  {
    while (!exit_) {
      keep_callback_();
      {
        std::unique_lock<std::mutex> lck(exit_mut_);
        exit_cond_.wait_for(
          lck, std::chrono::milliseconds(beats_duration_), [this] {
            return exit_ == true;
          });
      }
    }
  }

  void BeatsCheck()
  {
    int8_t interval_cnt = 0;
    while (!exit_) {
      for (auto iter = beats_map_.begin(); iter != beats_map_.end(); ++iter) {
        if (iter->second->Push() >= lost_limit_) {
          lost_callback_(iter->first, true);
        } else {
          lost_callback_(iter->first, false);
        }
      }
      if (++interval_cnt >= notify_interval_) {
        interval_cnt = 0;
        notify_callback_();
      }
      {
        std::unique_lock<std::mutex> lck(exit_mut_);
        exit_cond_.wait_for(
          lck, std::chrono::milliseconds(beats_duration_), [this] {
            return exit_ == true;
          });
      }
    }
  }

private:
  std::function<void(const std::string &, bool)> lost_callback_;
  std::function<void()> keep_callback_;
  std::function<void()> notify_callback_;

private:
  int32_t beats_duration_ {1000};       // 心跳间隔
  int8_t lost_limit_ {5};               // 连续丢失容忍上限
  int8_t notify_interval_ {0};           // 通知间隔
  BeatsMap beats_map_;                  // 心跳刷新及异常检测机制实现map
  bool exit_ {false};                   // 退出标志
  std::thread thread_check_;
  std::thread thread_cycle_;
  std::mutex exit_mut_;
  std::condition_variable exit_cond_;
};  // class HeartBeats

class HeartBeatsActuator
{
public:
  HeartBeatsActuator(std::string name)
  : name_(name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name_ + "_heartbeat_keep");
    heartbeats_pub_ = node_ptr_->create_publisher<protocol::msg::Heartbeats>(
      "manager_heartbeats",
      rclcpp::SystemDefaultsQoS()
    );
    heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeats>(
      500,
      std::bind(&HeartBeatsActuator::HeartKeep, this));
    std::thread(
      [this]() {
        rclcpp::spin(node_ptr_);
      }).detach();
  }

public:
  void HeartBeatRun()
  {
    heart_beats_ptr_->HeartRun();
  }

private:
  void HeartKeep()
  {
    protocol::msg::Heartbeats msg;
    // msg.name = "audio";
    msg.name = name_;
    heartbeats_pub_->publish(msg);
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::unique_ptr<HeartBeats> heart_beats_ptr_;
  rclcpp::Publisher<protocol::msg::Heartbeats>::SharedPtr heartbeats_pub_;
};  // class HeartBeatsActuator
}  // namespace machine
}  // namespace cyberdog

#endif  // CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
