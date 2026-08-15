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
/**
 * @file heartbeats_test.cpp
 * @author DuKun (dkun1@xiaomi.com)
 * @brief Demo & Test code for heartbeats.
 *        This code cannot pass CI.
 * @version 0.1
 * @date 2022-05-06
 *
 * @copyright Copyright (c) 2022, MiRoboticsLab.Rop.
 *
 */
#include <iostream>
#include <string>
#include <vector>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"
#include "protocol/msg/heartbeats.hpp"

class HeartbeatsTest : public cyberdog::machine::HeartBeats
{
public:
  HeartbeatsTest(const std::string & name, int32_t duration, int8_t limit)
  : HeartBeats(duration, limit), name_(name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name_);
    pub_ptr_ = node_ptr_->create_publisher<protocol::msg::Heartbeats>("heartbeats_test_topic", 10);

    // 心跳信息可以在任意位置任意时刻更新，保证publish callback的正确使用即可
    beats_msg_.name = name_;
    beats_msg_.state = 0;

    // 注册ros消息回调，用于刷新心跳监听输入
    sub_ptr_ = node_ptr_->create_subscription<protocol::msg::Heartbeats>(
      "heartbeats_test_topic",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&HeartbeatsTest::HeartTopicCallback, this, std::placeholders::_1));

    // 构造监听对象的容器
    std::vector<std::string> target_vec;
    target_vec.push_back(name_);

    // 配置心跳管理
    HeartConfig(target_vec, std::bind(&HeartbeatsTest::HeartbeatsNotify, this));

    // 注册监听回调和发送回调
    RegisterLostCallback(
      std::bind(
        &HeartbeatsTest::LostCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    RegisterKeepCallback(std::bind(&HeartbeatsTest::PublishCallback, this));

    // 启动心跳
    HeartRun();
  }
  ~HeartbeatsTest() {}

  void Spin()
  {
    rclcpp::spin(this->node_ptr_);
    rclcpp::shutdown();
  }

private:
  void LostCallback(const std::string & name, bool lost)
  {
    std::cout << name_ << " lost heartbeats with " << name <<
      " is " << (lost == true ? "lost" : "alive") << std::endl;
    std::fflush(stdout);
  }

  void PublishCallback()
  {
    printf(
      "Publisher %s pub once: name: %s state: %d\n", name_.c_str(),
      beats_msg_.name.c_str(), beats_msg_.state);
    std::fflush(stdout);
    pub_ptr_->publish(beats_msg_);


    // 心跳信息可以在任意位置任意时刻更新，保证publish callback的正确使用即可
    beats_msg_.state++;
  }

  void HeartTopicCallback(const protocol::msg::Heartbeats::SharedPtr msg)
  {
    std::cout << "Subscriber: " << name_ << " recv once: name: " << msg->name << " state: " <<
    (int32_t)msg->state << std::endl;
    std::fflush(stdout);
    HeartUpdate(msg->name);
  }

  void HeartbeatsNotify()
  {
  }

private:
  std::string name_;
  protocol::msg::Heartbeats beats_msg_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Publisher<protocol::msg::Heartbeats>::SharedPtr pub_ptr_ {nullptr};
  rclcpp::Subscription<protocol::msg::Heartbeats>::SharedPtr sub_ptr_ {nullptr};
};  // class HeartbeatsTest

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  HeartbeatsTest heartbeats_test("hearttest", 500, 5);
  // heartbeats_test.HeartRun();
  heartbeats_test.Spin();
  return 0;
}
