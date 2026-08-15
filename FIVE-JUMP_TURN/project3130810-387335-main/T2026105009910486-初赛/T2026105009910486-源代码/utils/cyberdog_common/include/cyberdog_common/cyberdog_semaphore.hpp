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
#ifndef CYBERDOG_COMMON__CYBERDOG_SEMAPHORE_HPP_
#define CYBERDOG_COMMON__CYBERDOG_SEMAPHORE_HPP_
#include <mutex>
#include <chrono>
#include <condition_variable>
namespace cyberdog
{
namespace common
{
/**
 * @brief 进程内信号量
 */
class Semaphore
{
public:
  Semaphore() {}
  ~Semaphore() {}
  /**
   * @brief 信号等待函数
   *        1. 毫秒级别等待
   * @param time  等待时间，单位毫秒，ms
   */
  int WaitFor(int time)
  {
    std::unique_lock<std::mutex> lock(mux_);
    return static_cast<int>(cv_.wait_for(lock, std::chrono::milliseconds(time)));
  }
  /**
   * @brief 信号等待函数
   *        1.  阻塞直到获取到消息
   */
  void Wait(void)
  {
    std::unique_lock<std::mutex> lock(mux_);
    cv_.wait(lock);
  }
  /**
   * @brief 唤醒信号
   *        1. 唤醒一次
   */
  void Give()
  {
    std::unique_lock<std::mutex> lock(mux_);
    cv_.notify_one();
  }
  /**
   * @brief 唤醒信号
   *        1. 唤醒所有等待
   */
  void GiveAll()
  {
    std::unique_lock<std::mutex> lock(mux_);
    cv_.notify_all();
  }

private:
  std::mutex mux_;
  std::condition_variable cv_;
};
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_SEMAPHORE_HPP_
