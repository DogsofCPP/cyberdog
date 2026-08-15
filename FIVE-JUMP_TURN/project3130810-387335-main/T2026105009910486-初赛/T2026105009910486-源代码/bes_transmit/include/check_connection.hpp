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
#ifndef CHECK_CONNECTION_HPP_
#define CHECK_CONNECTION_HPP_

#include <cstdlib>
#include <iostream>
#include <string>

struct CheckConnection
{
  /**
   * @brief Activate ping command to check connection
   * @param url The address that you want to check
   * @return Result of ping command
   */
  static int check(const std::string & url)
  {
    std::string cmd("ping -c 1 -W 2 ");
    size_t found_prefix = url.find("://");
    std::string ip;
    if (found_prefix != std::string::npos) {
      ip = url.substr(found_prefix + 3);
    } else {
      ip = url;
    }
    size_t port_position = ip.find(":");
    if (port_position != std::string::npos) {
      ip = ip.substr(0, port_position);
    }
    cmd += ip;
    int cmd_result = system(cmd.c_str());
    if (cmd_result != 0) {
      std::cout << "Not able to connect to " << ip << std::endl;
    }
    return cmd_result;
  }
};

#endif  // CHECK_CONNECTION_HPP_
