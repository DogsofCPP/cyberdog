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
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

cyberdog::common::MsgQueue<std::string> msg_queue;

void EnQueueThread()
{
  int i = 0;
  while(i++ < 10) {
    auto data = std::to_string(i);
    msg_queue.EnQueue(data);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void DeQueueThread()
{
  std::string data;
  int i = 0;
  while(i++ < 10) {
    if(msg_queue.DeQueue(data)) {
      INFO("DeQueue OK with data: %s", data.c_str());
    }
  }
}

TEST(twothread, msgqueue)
{
  auto t1 = std::thread(EnQueueThread);
  auto t2 = std::thread(DeQueueThread);
  t1.join();
  t2.join();
}

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  LOGGER_MAIN_INSTANCE("MsgQueueTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
