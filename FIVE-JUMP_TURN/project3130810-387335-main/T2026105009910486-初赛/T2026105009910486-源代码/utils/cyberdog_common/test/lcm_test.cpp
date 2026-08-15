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
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "lcmtestcs/lcmtestcs.hpp"
#include "structor/data.hpp"
// #include "backtrace.h"

void client_thread_function()
{
  lcm_request_data request_data;
  lcm_response_data response_data;

  request_data.name = std::string("client");
  request_data.id = 0;

  lcmtestcs::lcmtestcs request;
  lcmtestcs::lcmtestcs response;

  request.data = xpack::json::encode(request_data);
  request.cmd = std::string("play");

  std::string client_name("lcm_cs_test");
  auto client =
    cyberdog::common::LcmClient<lcmtestcs::lcmtestcs, lcmtestcs::lcmtestcs>(client_name);
  while (true) {
    INFO("client call once");
    auto result = client.Request(request, response, 1000);
    if (!result) {
      ERROR("client thread failed once");
    } else {
      // INFO("%s", response.cmd.c_str());
      // INFO("%s", response.data.c_str());
      // INFO("length: %d", (int)response.data.length());
      xpack::json::decode(response.data, response_data);
      INFO(
        "client get response cmd: %s\n\tname: %s, result: %d",
        response.cmd.c_str(), response_data.name.c_str(), (int8_t)response_data.result);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // client.SpinOnce(1000);
  }
}

void server_function(const lcmtestcs::lcmtestcs & req, lcmtestcs::lcmtestcs & res)
{
  lcm_request_data request_data;
  lcm_response_data response_data;
  xpack::json::decode(req.data, request_data);
  // INFO("server~~ %s", req.data.c_str());
  INFO(
    "server receive once\n\tname: %s, id: %d",
    request_data.name.c_str(), request_data.id);
  response_data.name = std::string("server");
  response_data.result = true;
  res.data = xpack::json::encode(response_data);
  res.cmd = std::string("playresponse");
  fflush(stdout);
}

void server_thread_function()
{
  std::string server_name("lcm_cs_test");
  // std::function<void(const lcmtest::lcmtest &, lcmtest::lcmtest &)> foo = server_function;
  auto server = cyberdog::common::LcmServer<lcmtestcs::lcmtestcs, lcmtestcs::lcmtestcs>(
    server_name,
    server_function);
  server.Spin();
}

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  LOGGER_MAIN_INSTANCE("LcmTest");

  // signal(SIGSEGV, signal_handler);
  // signal(SIGABRT, signal_handler);

  // std::thread server_thread = std::thread(server_thread_function);
  // std::thread client_thread = std::thread(client_thread_function);

  // if (client_thread.joinable()) {
  //   client_thread.join();
  // }
  // if (server_thread.joinable()) {
  //   server_thread.join();
  // }
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
