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
#include <thread>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
enum class DemoCode : int32_t
{
  kDemoError1 = 21,
  kDemoError2 = 22,
  kDemoError3 = 23
};


class ActuatorDemo : public cyberdog::machine::MachineActuator
{
  std::string Uninitialized_V = std::string("Uninitialized");
  std::string SetUp_V = std::string("SetUp");
  std::string TearDown_V = std::string("TearDown");
  std::string SelfCheck_V = std::string("SelfCheck");
  std::string Active_V = std::string("Active");
  std::string DeActive_V = std::string("DeActive");
  std::string Protected_V = std::string("Protected");
  std::string LowPower_V = std::string("LowPower");
  std::string OTA_V = std::string("OTA");
  std::string Error_V = std::string("Error");

public:
  explicit ActuatorDemo(const std::string & name)
  : MachineActuator(name),
    name_(name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name_);
    code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<DemoCode>>(
      cyberdog::system::ModuleCode::kRobot);
  }
  ~ActuatorDemo() {}

  bool Init()
  {
    if (!this->MachineActuatorInit(
        std::string(BenchmarkPath) + "/fs_machine_test_config.toml",
        node_ptr_))
    {
      ERROR("Init failed, actuator init error.");
      return false;
    }
    this->RegisterStateCallback(SetUp_V, std::bind(&ActuatorDemo::OnSetUp, this));
    this->RegisterStateCallback(TearDown_V, std::bind(&ActuatorDemo::ONTearDown, this));
    this->RegisterStateCallback(SelfCheck_V, std::bind(&ActuatorDemo::OnSelfCheck, this));
    this->RegisterStateCallback(Active_V, std::bind(&ActuatorDemo::OnActive, this));
    this->RegisterStateCallback(DeActive_V, std::bind(&ActuatorDemo::OnDeActive, this));
    this->RegisterStateCallback(Protected_V, std::bind(&ActuatorDemo::OnProtected, this));
    this->RegisterStateCallback(LowPower_V, std::bind(&ActuatorDemo::OnLowPower, this));
    this->RegisterStateCallback(OTA_V, std::bind(&ActuatorDemo::OnOTA, this));
    this->RegisterStateCallback(Error_V, std::bind(&ActuatorDemo::OnError, this));
    return this->ActuatorStart();
  }

  void Spin()
  {
    INFO("ActuatorDemo: %s spin.", name_.c_str());
    rclcpp::spin(this->node_ptr_);
  }

private:
  /* 实现状态机虚函数，满足系统要求 */
  int32_t OnSetUp()
  {
    INFO("ActuatorDemo on setup.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t ONTearDown()
  {
    INFO("ActuatorDemo on teardown.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnSelfCheck()
  {
    INFO("ActuatorDemo on selfcheck.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnActive()
  {
    INFO("ActuatorDemo on active.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnDeActive()
  {
    INFO("ActuatorDemo on deactive.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnProtected()
  {
    INFO("ActuatorDemo on protected.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnLowPower()
  {
    INFO("ActuatorDemo on lowpower.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnOTA()
  {
    INFO("ActuatorDemo on OTA.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnError()
  {
    INFO("ActuatorDemo on OTA.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::shared_ptr<cyberdog::system::CyberdogCode<DemoCode>> code_ptr_ {nullptr};
};  // class ActuactorDemo


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("ActuatorTest2");
  INFO("Running");
  std::shared_ptr<ActuatorDemo> ptr = std::make_shared<ActuatorDemo>("test2");
  std::thread t([&](){
    ptr->Spin();
  });
  if (!ptr->Init()) {
    ERROR("init failed!");
    return -1;
  }
  t.join();
  return 0;
}
