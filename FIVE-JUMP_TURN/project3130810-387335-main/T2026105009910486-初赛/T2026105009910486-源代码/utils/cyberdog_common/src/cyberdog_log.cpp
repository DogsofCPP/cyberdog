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
#include "cyberdog_common/cyberdog_log.hpp"


using cyberdog::common::CyberdogLogger;
using cyberdog::common::CyberdogLoggerFactory;

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::main_logger = nullptr;
rclcpp::Clock global_steady_clock = rclcpp::Clock();

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::Get_Logger()
{
  return main_logger ==
         nullptr ? std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME)) :
         main_logger;
}

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::Get_Logger(const char * sz_name)
{
  if (!main_logger) {
    CyberdogLogger cyberdog_logger(sz_name);
    main_logger = cyberdog_logger.Get_Logger();
  }
  return main_logger;
}
