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
#ifndef BES_TRANSMIT_CENTER_HPP_
#define BES_TRANSMIT_CENTER_HPP_

#include <string>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/bes_http.hpp"
#include "protocol/srv/bes_http_send_file.hpp"
#include "protocol/srv/device_info.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "back_end_publisher.hpp"
#include "back_end_subscriber.hpp"
#include "back_end_http.hpp"


namespace cyberdog
{
namespace bridge
{
/**
 * @brief Manager of HTTP and MQTT objects
 */
class TransmitCenter final
{
public:
  /**
   * @brief Construct a new Transmit Center object
   */
  TransmitCenter();
  /**
   * @brief Destroy the Transmit Center object
   */
  ~TransmitCenter();
  void Run();

private:
  /**
   * @brief Callback of ROS subscriber to send message via MQTT
   * @param msg ROS Msg
   */
  void MqttPubCallback(const std_msgs::msg::String::SharedPtr msg);
  void MqttSubCallback(const std::string & msg);
  /**
   * @brief Callback of ROS server to call HTTP methods
   * @param request ROS service request
   * @param respose ROS service respose
   */
  void BesHttpCallback(
    const protocol::srv::BesHttp::Request::SharedPtr request,
    protocol::srv::BesHttp::Response::SharedPtr respose);
  /**
   * @brief Callback of ROS server to call HTTP methods sending files
   * @param request ROS service request
   * @param respose ROS service respose
   */
  void BesHttpSendFileCallback(
    const protocol::srv::BesHttpSendFile::Request::SharedPtr request,
    protocol::srv::BesHttpSendFile::Response::SharedPtr respose);
  void UploadSyslog(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr respose);

private:
  /**
   * @brief Get SN of the device and user ID
   * @param sn SN of the device
   * @param uid User ID
   * @return true Successfully called service for getting info
   * @return false Failed to call service for getting info
   */
  bool getDevInf(std::string & sn, std::string & uid);
  bool getEnv(uint8_t mode = 0);
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr tpub_node_ptr_ {nullptr};
  rclcpp::Node::SharedPtr tsub_node_ptr_ {nullptr};
  rclcpp::Node::SharedPtr http_node_ptr_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr http_node_cb_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr be_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr be_sub_;
  rclcpp::Service<protocol::srv::BesHttp>::SharedPtr http_srv_;
  rclcpp::Service<protocol::srv::BesHttpSendFile>::SharedPtr http_send_file_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr upload_syslog_srv_;
  std::unique_ptr<Backend_Publisher> bpub_ptr_ {nullptr};
  std::unique_ptr<Backend_Subscriber> bsub_ptr_ {nullptr};
  std::unique_ptr<Backend_Http> bhttp_ptr_ {nullptr};
  rclcpp::Client<protocol::srv::DeviceInfo>::SharedPtr device_info_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr env_client_ {nullptr};
  std::pair<std::string, std::string> env_mqtt_pub_url_[2] {
    {"10.38.205.52", "1883"}, {"10.108.134.85", "80"}
  };
  std::string env_http_url_[2] {
    "https://test-server.cyberdog.xiaomi.com", "https://server.cyberdog.xiaomi.com"
  };
  std::pair<std::string, std::string> env_mqtt_user_password[2] {
    {"cyberdog@mqtt", "cyberdog1*0o$"}, {"admin", "public"}
  };
  bool current_env_[2] {false, false};  // false for test, true for product
  bool bpub_is_ready_ {false};
  const std::string log_path_ {"/var/log/"};
  const std::string upload_url_ {"device/system/log"};
  LOGGER_MINOR_INSTANCE("TransmitCenter");
};  // TransmitCenter
}  // namespace bridge
}  // namespace cyberdog

#endif  // BES_TRANSMIT_CENTER_HPP_
