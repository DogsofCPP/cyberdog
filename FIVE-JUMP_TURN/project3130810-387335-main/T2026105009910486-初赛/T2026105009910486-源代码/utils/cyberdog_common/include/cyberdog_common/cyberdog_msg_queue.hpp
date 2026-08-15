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
#ifndef CYBERDOG_COMMON__CYBERDOG_MSG_QUEUE_HPP_
#define CYBERDOG_COMMON__CYBERDOG_MSG_QUEUE_HPP_

#include <list>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace cyberdog
{
namespace common
{
template<typename T>
class MsgQueue
{
public:
  MsgQueue()
  {
    ResetWait();
  }

  ~MsgQueue()
  {
    Clear();
    if (IsWait()) {
      read_signal_.notify_all();
    }
  }

/* Msg Queue opened API */

public:
  /**
   * @brief 入队函数
   *        1. 会在队首原址构造
   *        2. 如果有等待出队线程，会唤醒一次
   *
   * @param t 只支持构造该消息队列类时选择实例化的模板
   */
  void EnQueue(const T & t)
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    data_list_.emplace_front(t);
    if (IsWait()) {
      GetWait();
      read_signal_.notify_one();
    }
  }

  /**
   * @brief 入队函数
   *        1. 会在队首原址构造
   *        2. 如果有等待出队线程，会唤醒一次
   *        3. 会保持队列不超过一个的数据，用于click场景
   *
   * @param t 只支持构造该消息队列类时选择实例化的模板
   */
  void EnQueueOne(const T & t)
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    if (!IsEmpty()) {
      Clear();
    } else {
      data_list_.emplace_front(t);
      if (IsWait()) {
        GetWait();
        read_signal_.notify_one();
      }
    }
  }

  /**
   * @brief 出队函数
   *          1. 如果队列非空，则消耗掉一个数据，并将该数据引用返回
   *          2. 如果队列为空，则进入条件等待，直到有数据入队
   *          3. 如果等待状态中发生析构，则会返回失败，此时引用参数不可用，其行为是未定义的
   *
   * @param t 只支持构造该消息队列类时选择实例化的模板
   * @return true 获取数据成功
   * @return false 获取数据失败
   */
  bool DeQueue(T & t)
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    if (data_list_.empty()) {
      SetWait();
      read_signal_.wait(lk);
    }
    if (IsEmpty()) {
      return false;
    } else {
      t = data_list_.back();
      data_list_.pop_back();
      return true;
    }
  }

  /**
   * @brief 重置队列，此时所有等待函数会得到失败的返回值并解锁
   *
   */
  void Reset()
  {
    Clear();
    if (IsWait()) {
      ResetWait();
      read_signal_.notify_all();
    }
  }

  int Size()
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    return static_cast<int>(data_list_.size());
  }

  bool IsEmpty()
  {
    // std::unique_lock<std::mutex> lk(data_lock_);
    return data_list_.empty();
  }

/* Internal API */

private:
  bool IsWait()
  {
    return is_wait_ == 0 ? false : true;
  }

  void SetWait()
  {
    is_wait_++;
  }

  void GetWait()
  {
    if (is_wait_ > 0) {
      is_wait_--;
    }
  }

  void ResetWait()
  {
    is_wait_ = 0;
  }

  void Clear()
  {
    // std::unique_lock<std::mutex> lk(data_lock_);
    while (!data_list_.empty()) {
      data_list_.pop_front();
    }
  }

private:
  std::condition_variable read_signal_;
  std::mutex data_lock_;
  std::list<T> data_list_;
  std::atomic<int32_t> is_wait_;
};  // class MsgQueue
}  // namespace common
}  // namespace cyberdog

#endif  // CYBERDOG_COMMON__CYBERDOG_MSG_QUEUE_HPP_
