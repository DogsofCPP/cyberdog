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
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include "gtest/gtest.h"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"

#if 0
TEST(basic, build_and_print)
{
  auto Uninitialized_V = std::string("Uninitialized");
  auto SetUp_V = std::string("SetUp");
  auto SelfCheck_V = std::string("SelfCheck");
  auto Actuator_1 = std::string("test1");
  auto Actuator_2 = std::string("test2");
  INFO("FS Machine running");
  rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("FSController");
  if (node_ptr == nullptr) {
    INFO("node ptr create failed!");
  }
  std::shared_ptr<cyberdog::machine::MachineController> machine_ptr =
    std::make_shared<cyberdog::machine::MachineController>();
  if (!machine_ptr->MachineControllerInit(
      std::string(BenchmarkPath) +
      "/fs_machine_test_config.toml", node_ptr))
  {
    ERROR("Test FS Machine Init failed!");
    return;
  }
  std::map<std::string, std::string> state_map;

  machine_ptr->GetStateMap(state_map);
  EXPECT_EQ(Uninitialized_V, state_map["test1"]);

  auto result = machine_ptr->WaitActuatorsSetUp();  // 如果不启动test1或test2程序， 这里会gtest会fail，符合预期
  EXPECT_TRUE(result);
  std::string tmp_state;
  machine_ptr->GetState(Actuator_1, tmp_state);
  EXPECT_EQ(SetUp_V, tmp_state);

  result = machine_ptr->SetState(Uninitialized_V);
  machine_ptr->GetState(Actuator_1, tmp_state);
  EXPECT_NE(Uninitialized_V, tmp_state);
  EXPECT_FALSE(result);

  result = machine_ptr->SetState(SelfCheck_V);
  EXPECT_TRUE(result);
  machine_ptr->GetState(Actuator_1, tmp_state);
  EXPECT_EQ(SelfCheck_V, tmp_state);
}
#endif

/**
 * @brief Demo code for using MachineController
 *
 */
class ControllerDemo final
{
  std::string Uninitialized_V = std::string("Uninitialized");
  std::string SetUp_V = std::string("SetUp");
  std::string SelfCheck_V = std::string("SelfCheck");
  std::string Active_V = std::string("Active");
  std::string DeActive_V = std::string("DeActive");
  std::string Protected_V = std::string("Protected");
  std::string LowPower_V = std::string("LowPower");
  std::string OTA_V = std::string("OTA");
  std::string Error_V = std::string("Error");
  std::string Actuator_1 = std::string("test1");
  std::string Actuator_2 = std::string("test2");

public:
  ControllerDemo(const std::string & name)
  : name_(name)
  {

  }
  ~ControllerDemo() {}
  bool Init()
  {
    node_ptr_ = rclcpp::Node::make_shared(name_);
    machine_controller_ptr_ = std::make_shared<cyberdog::machine::MachineController>();
    if (!machine_controller_ptr_->MachineControllerInit(
        std::string(BenchmarkPath) +
        "/fs_machine_test_config.toml", node_ptr_))
    {
      ERROR("Test FS Machine Init failed!");
      return false;
    }
    if (!machine_controller_ptr_->WaitActuatorsSetUp()) {
      ERROR("Test FS Machine Init failed, actuators setup failed!");
      return false;
    } else {
      INFO("FS Machine Init OK.");
      return true;
    }
  }

  void Run()
  {
    if (machine_controller_ptr_->SetState(SelfCheck_V, map_result) != 0) {
      ERROR("selfcheck failed, cannot running!");
      return;
    }
    std::vector<std::string> state_vec{Active_V, DeActive_V, OTA_V, Protected_V, LowPower_V};
    int8_t counter = 0;
    while (true) {
      // 模拟真实执行场景
      counter = counter % 5;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      machine_controller_ptr_->SetState(state_vec[counter], map_result);
      counter++;
    }
  }

  void Spin()
  {
    rclcpp::spin(this->node_ptr_);
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  std::shared_ptr<cyberdog::machine::MachineController> machine_controller_ptr_ {nullptr};
  std::map<std::string, int32_t>  map_result;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("FSController");
  auto demo_ptr = std::make_shared<ControllerDemo>("controller_demo");
  if (!demo_ptr->Init()) {
    ERROR("init failed, program will exit with error!");
    return -1;
  }
  std::thread t([&](){
    demo_ptr->Spin();
  });
  demo_ptr->Run();
  t.join();
  return 0;
}
