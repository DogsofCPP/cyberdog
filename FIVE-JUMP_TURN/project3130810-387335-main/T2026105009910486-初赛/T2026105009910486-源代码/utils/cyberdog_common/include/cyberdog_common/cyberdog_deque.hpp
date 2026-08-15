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
#ifndef CYBERDOG_COMMON__CYBERDOG_DEQUE_HPP_
#define CYBERDOG_COMMON__CYBERDOG_DEQUE_HPP_
#include <mutex>
#include <chrono>
#include <deque>
#include <queue>
#include <condition_variable>

namespace cyberdog
{
namespace common
{
template<class T>
/**
 * @brief  双向队列API
 *         消息阻塞式出队列
 *
 */
class MsgDeque
{
public:
  MsgDeque() {}
  ~MsgDeque() {}
  /**
   * @brief 入队函数
   *        1. 入队列尾
   *        2. 唤醒一次
   * @param T 只支持构造该消息队列类时选择实例化的模板
   */
  int PushBack(const T & msg)
  {
    std::unique_lock<std::mutex> lock(mux_);
    queue_.push_back(msg);
    cv_.notify_one();
    return 0;
  }
  /**
   * @brief 入队函数
   *        1. 入队列首
   *        2.唤醒一次
   * @param T 只支持构造该消息队列类时选择实例化的模板
   */
  int PushFront(const T & msg)
  {
    std::unique_lock<std::mutex> lock(mux_);
    queue_.push_front(msg);
    cv_.notify_one();
    return 0;
  }
  /**
   * @brief 出队函数
   *        1. 如果队列中有数据，则从队首返回一个；
   *        2. 如果队列无数据，阻塞等待；
   */
  T PopFront()
  {
    std::unique_lock<std::mutex> lock(mux_);
    if (queue_.empty()) {
      cv_.wait(lock);
    }
    T msg = queue_.front();
    queue_.pop_front();
    return msg;
  }
  /**
  * @brief 出队函数
  *        1. 如果队列中有数据，则从队首返回一个引用；
  *        2. 如果队列无数据，阻塞等待；
  * @param T 只支持构造该消息队列类时选择实例化的模板
  */
  void PopFront(T & msg)
  {
    std::unique_lock<std::mutex> lock(mux_);
    if (queue_.empty()) {
      cv_.wait(lock);
    }
    msg = queue_.front();
    queue_.pop_front();
  }
  /**
   * @brief 出队函数
   *        1. 如果队列中有数据，则从队尾返回一个；
   *        2. 如果队列无数据，阻塞等待；
   */
  T PopBack()
  {
    std::unique_lock<std::mutex> lock(mux_);
    if (queue_.empty()) {
      cv_.wait(lock);
    }
    T msg = queue_.back();
    queue_.pop_back();
    return msg;
  }
  /**
  * @brief 出队函数
  *        1. 如果队列中有数据，则从队尾返回一个引用；
  *        2. 如果队列无数据，阻塞等待；
  * @param T 只支持构造该消息队列类时选择实例化的模板
  */
  void PopBack(T & msg)
  {
    std::unique_lock<std::mutex> lock(mux_);
    if (queue_.empty()) {
      cv_.wait(lock);
    }
    msg = queue_.back();
    queue_.pop_back();
  }
  /**
   * @brief 队列大小函数
   *        1、返回队列长度
   *
   */
  int Size()
  {
    std::unique_lock<std::mutex> lock(mux_);
    return queue_.size();
  }
  /**
   * @brief 队列为空判断函数
   *        1. 队列为空返回true.否则返回false
   */
  bool Empty()
  {
    std::unique_lock<std::mutex> lock(mux_);
    return queue_.empty();
  }
  /**
   * @brief 队列清空函数
   *        1. 清空队列消息
   */
  int Clear()
  {
    std::unique_lock<std::mutex> lock(mux_);
    return queue_.Clear();
  }

private:
  std::deque<T> queue_;
  std::mutex mux_;
  std::condition_variable cv_;
};  // class MsgDeque
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_DEQUE_HPP_
