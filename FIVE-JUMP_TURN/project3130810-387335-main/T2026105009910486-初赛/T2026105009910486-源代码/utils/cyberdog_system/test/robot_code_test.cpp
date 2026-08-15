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
#include <memory>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_system/robot_code.hpp"

/* 设置模块独有的错误码
   推荐放在模块内部头文件中
 */
enum class RobotCode : int32_t
{
  kRobotError1 = 21,
  kRobotError2 = 22,
  kRobotError3 = 23
};

/* 系统预留code使用方法 */
TEST(get_code, basic_using)
{
  using KeyCode_T = cyberdog::system::KeyCode;
  auto code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<RobotCode>>(
    cyberdog::system::ModuleCode::kRobot);
  auto result = code_ptr_->GetKeyCode(KeyCode_T::kOK);
  EXPECT_EQ(0, result);

  result = code_ptr_->GetCode(RobotCode::kRobotError1);
  EXPECT_EQ(121, result);
}

int main(int argc, char ** argv)
{
  // INFO_STEAM(BenchmarkPath);
  ::testing::InitGoogleTest(&argc, argv);
  LOGGER_MAIN_INSTANCE("CodeTest");
  return RUN_ALL_TESTS();
}
