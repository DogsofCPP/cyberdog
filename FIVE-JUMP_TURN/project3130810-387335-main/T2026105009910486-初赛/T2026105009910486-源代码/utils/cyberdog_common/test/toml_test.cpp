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

// Test toml package functions and performance,
// Under googletest.
// Should not be compiled in OTA or DailyBuild version

#include <dirent.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <algorithm>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
using cyberdog::common::CyberdogToml;

TEST(hello, hello__Test)
{
  INFO_STREAM("hello, gtest!");
  EXPECT_EQ('a', 97);
}

TEST(file, parser)
{
  toml::value value;
  auto result = CyberdogToml::ParseFile("nofile", value);
  EXPECT_FALSE(result);

  result = CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value);
  EXPECT_TRUE(result);
}

TEST(reader, table)
{
  toml::value value;
  if (!CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value)) {
    return;
  }

  toml::value a;
  auto result = CyberdogToml::Get(value, "a", a);
  EXPECT_TRUE(result);
  EXPECT_TRUE(a.is_table());

  int aa;
  result = CyberdogToml::Get(a, "a", aa);
  EXPECT_TRUE(result);
  EXPECT_EQ(aa, 1);

  std::string ab;
  result = CyberdogToml::Get(a, "b", ab);
  EXPECT_TRUE(result);
  EXPECT_EQ(ab, std::string("cyberdog"));

  bool ac;
  result = CyberdogToml::Get(a, "c", ac);
  EXPECT_TRUE(result);
  EXPECT_EQ(ac, true);

  float ad;
  result = CyberdogToml::Get(a, "d", ad);
  EXPECT_TRUE(result);
  EXPECT_EQ(ad, static_cast<float>(0.1));
}

TEST(reader, array)
{
  toml::value value;
  if (!CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value)) {
    return;
  }

  toml::value b;
  auto result = CyberdogToml::Get(value, "b", b);
  EXPECT_TRUE(result);
  EXPECT_TRUE(b.is_array());
  EXPECT_TRUE(b.as_array().at(0).is_table());

  toml::value b0;
  result = CyberdogToml::Get(b, 0, b0);
  EXPECT_TRUE(result);
  EXPECT_TRUE(b0.is_table());
  result = CyberdogToml::Get(b, 10, b0);
  EXPECT_FALSE(result);
}

TEST(writer, modify)
{
  toml::value value;
  if (!CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value)) {
    return;
  }

  toml::value b;
  auto result = CyberdogToml::Get(value, "b", b);
  EXPECT_TRUE(result);
  result = CyberdogToml::Set(b, "a", 0);
  EXPECT_FALSE(result);

  toml::value b1;
  result = CyberdogToml::Get(b, 1, b1);
  EXPECT_TRUE(result);
  result = CyberdogToml::Set(b1, "c", true);
  EXPECT_TRUE(result);
  bool b1c;
  result = CyberdogToml::Get(b1, "c", b1c);
  EXPECT_TRUE(b1c);
}

TEST(writer, add)
{
  toml::value a;
  INFO_STREAM(a.is_uninitialized());
  INFO_STREAM(a.is_table());
  auto result = CyberdogToml::Set(a, "b", std::string("xiaomi"));
  EXPECT_TRUE(result);

  std::string ab;
  result = CyberdogToml::Get(a, "b", ab);
  EXPECT_TRUE(result);
  EXPECT_EQ(ab, std::string("xiaomi"));

  toml::value b;
  result = CyberdogToml::Set(b, 0, std::string("xiaomi"));
  EXPECT_FALSE(result);

  result = CyberdogToml::Set(b, std::string("xiaomi"));
  EXPECT_TRUE(result);
  EXPECT_TRUE(b.is_array());

  std::string b0;
  result = CyberdogToml::Get(b, 0, b0);
  EXPECT_TRUE(result);
  EXPECT_EQ(b0, std::string("xiaomi"));
}

int main(int argc, char ** argv)
{
  // INFO_STEAM(BenchmarkPath);
  ::testing::InitGoogleTest(&argc, argv);
  LOGGER_MAIN_INSTANCE("TomlTest");
  return RUN_ALL_TESTS();
}
