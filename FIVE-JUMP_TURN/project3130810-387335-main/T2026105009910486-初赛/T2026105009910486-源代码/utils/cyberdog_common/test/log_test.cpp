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
#include <chrono>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_log.hpp"

bool test()
{
  static int i = 0;
  return (++i) % 2;
}

class CustomClass
{
public:
  CustomClass()
  {
    INFO("CustomClass has own namespace.");
    INFO_STREAM("CustomClass start " << 22 << 33 << " end");
  }
  void output()
  {
    int num = 2;
    DEBUG_EXPRESSION(num % 2 == 0, "CustomClass debug");
    INFO_EXPRESSION(num % 2 == 1, "CustomClass info");
    WARN_EXPRESSION(num % 2 == 0, "CustomClass warn");
    ERROR_EXPRESSION(num % 2 == 1, "CustomClass error");
    FATAL_EXPRESSION(num % 2 == 0, "CustomClass fatal");
    DEBUG_STREAM_EXPRESSION(num % 2 == 1, "CustomClass debug" << "stream");
    INFO_STREAM_EXPRESSION(num % 2 == 0, "CustomClass info" << "stream");
    WARN_STREAM_EXPRESSION(num % 2 == 1, "CustomClass warn" << "stream");
    ERROR_STREAM_EXPRESSION(num % 2 == 0, "CustomClass error" << "stream");
    FATAL_STREAM_EXPRESSION(num % 2 == 1, "CustomClass fatal" << "stream");
  }

private:
  LOGGER_MINOR_INSTANCE("CustomClass");
};

TEST(cyberdoglog, test1)
{
  INFO("cyberdog log test1");
}

TEST(cyberdoglog, test2)
{
  LOGGER_MAIN_INSTANCE("test");
  INFO("cyberdog log test2");
}

TEST(cyberdoglog, test3)
{
  DEBUG("cyberdog log test3 debug");
  WARN("cyberdog log test3 warn");
  ERROR("cyberdog log test3 error");
  FATAL("cyberdog log test3 fatal");
}

TEST(cyberdoglog, test4)
{
  for (auto i = 0; i < 10; ++i) {
    DEBUG_ONCE("cyberdog log test4 debug");
    INFO_ONCE("cyberdog log test4 info");
    WARN_ONCE("cyberdog log test4 warn");
    ERROR_ONCE("cyberdog log test4 error");
    FATAL_ONCE("cyberdog log test4 fatal");
    DEBUG_STREAM_ONCE("cyberdog log test4 debug" << "stream");
    INFO_STREAM_ONCE("cyberdog log test4 info" << "stream");
    WARN_STREAM_ONCE("cyberdog log test4 warn" << "stream");
    ERROR_STREAM_ONCE("cyberdog log test4 error" << "stream");
    FATAL_STREAM_ONCE("cyberdog log test4 fatal" << "stream");
  }
}

TEST(cyberdoglog, test5)
{
  CustomClass cc;
  cc.output();
}

TEST(cyberdoglog, test6)
{
  DEBUG_FUNCTION(test, "test6 debug");
  INFO_FUNCTION(test, "test6 info");
  WARN_FUNCTION(test, "test6 warn");
  ERROR_FUNCTION(test, "test6 error");
  FATAL_FUNCTION(test, "test6 fatal");
  DEBUG_STREAM_FUNCTION(test, "test6 debug" << "stream");
  INFO_STREAM_FUNCTION(test, "test6 info" << "stream");
  WARN_STREAM_FUNCTION(test, "test6 warn" << "stream");
  ERROR_STREAM_FUNCTION(test, "test6 error" << "stream");
  FATAL_STREAM_FUNCTION(test, "test6 fatal" << "stream");
}

TEST(cyberdoglog, test7)
{
  for (auto i = 0; i < 2; ++i) {
    DEBUG("cyberdog log test7 debug");
    INFO("cyberdog log test7 info");
    WARN("cyberdog log test7 warn");
    ERROR("cyberdog log test7 error");
    FATAL("cyberdog log test7 fatal");
    DEBUG_STREAM("cyberdog log test7 debug" << "stream");
    INFO_STREAM("cyberdog log test7 info" << "stream");
    WARN_STREAM("cyberdog log test7 warn" << "stream");
    ERROR_STREAM("cyberdog log test7 error" << "stream");
    FATAL_STREAM("cyberdog log test7 fatal" << "stream");
  }
}

TEST(cyberdoglog, test8)
{
  auto func_ = [] {
      auto last_clock = std::chrono::high_resolution_clock::now();
      auto current_clock = last_clock;
      while ((std::chrono::duration_cast<std::chrono::milliseconds>(
          current_clock -
          last_clock).count()) < 5050)
      {
        current_clock = std::chrono::high_resolution_clock::now();
        DEBUG_MILLSECONDS(1000, "cyberdog log test8 debug");
        INFO_MILLSECONDS(100, "cyberdog log test8 info");
        WARN_MILLSECONDS(200, "cyberdog log test8 warn");
        ERROR_MILLSECONDS(300, "cyberdog log test8 error");
        FATAL_MILLSECONDS(500, "cyberdog log test8 fatal");
        DEBUG_STREAM_MILLSECONDS(1000, "cyberdog log test8 debug" << "stream");
        INFO_STREAM_MILLSECONDS(100, "cyberdog log test8 info" << "stream");
        WARN_STREAM_MILLSECONDS(200, "cyberdog log test8 warn" << "stream");
        ERROR_STREAM_MILLSECONDS(300, "cyberdog log test8 error" << "stream");
        FATAL_STREAM_MILLSECONDS(500, "cyberdog log test8 fatal" << "stream");
      }
    };
  std::thread t1(func_);
  t1.join();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
