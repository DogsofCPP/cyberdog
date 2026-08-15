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
#include <vector>
#include <memory>
#include <algorithm>
#include <stdexcept>
#include <array>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include "cyberdog_common/cyberdog_json.hpp"
#include "bes_transmit_center.hpp"

#define   BTW_PUB_NODE_NAME   "bes_transmit_pub_waiter"
#define   BTW_SUB_NODE_NAME   "bes_transmit_sub_waiter"
#define   BT_HTTP_NODE_NAME   "bes_transmit_http_srv"

namespace cyberdog
{
namespace bridge
{

void reconnect_client(struct mqtt_client * client, void ** reconnect_state_vptr)
{
  struct reconnect_state_t * reconnect_state =
    *((struct reconnect_state_t **) reconnect_state_vptr);
  if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
    close(client->socketfd);
  }
  if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
    WARN(
      "reconnect_client: called while client was in error state \"%s\"\n",
      mqtt_error_str(client->error));
  }
  int sockfd = PosixSocket::open_nb_socket(reconnect_state->hostname, reconnect_state->port);
  if (sockfd == -1) {
    ERROR("Failed to open socket: ");
    close(sockfd);
    std::this_thread::sleep_for(std::chrono::microseconds(100000U));
    reconnect_client(client, reconnect_state_vptr);
    return;
  }
  mqtt_reinit(
    client, sockfd,
    reconnect_state->sendbuf, reconnect_state->sendbufsz,
    reconnect_state->recvbuf, reconnect_state->recvbufsz
  );
  const char * client_id = NULL;
  uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
  mqtt_connect(client, client_id, NULL, NULL, 0, NULL, NULL, connect_flags, 400);
  mqtt_subscribe(client, reconnect_state->topic, 0);
}

void subcribe_callback(void **, struct mqtt_response_publish * published)
{
  char * topic_name = reinterpret_cast<char *>(malloc(published->topic_name_size + 1));
  memcpy(topic_name, published->topic_name, published->topic_name_size);
  topic_name[published->topic_name_size] = '\0';
  INFO("Received publish('%s'): %s\n", topic_name, (const char *) published->application_message);
  free(topic_name);
}

void publish_callback(void **, struct mqtt_response_publish *)
{
}

}  // namespace bridge
}  // namespace cyberdog

cyberdog::bridge::TransmitCenter::TransmitCenter()
{
  tpub_node_ptr_ = rclcpp::Node::make_shared(BTW_PUB_NODE_NAME);
  tsub_node_ptr_ = rclcpp::Node::make_shared(BTW_SUB_NODE_NAME);
  http_node_ptr_ = rclcpp::Node::make_shared(BT_HTTP_NODE_NAME);
  executor_.add_node(tpub_node_ptr_);
  executor_.add_node(tsub_node_ptr_);
  executor_.add_node(http_node_ptr_);

  http_node_cb_group_ = http_node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  device_info_client_ =
    http_node_ptr_->create_client<protocol::srv::DeviceInfo>(
    "query_divice_info", rmw_qos_profile_services_default, http_node_cb_group_);

  bpub_ptr_ = std::make_unique<Backend_Publisher>(std::string("cyberdog/base_info/submit"));
  be_sub_ =
    tsub_node_ptr_->create_subscription<std_msgs::msg::String>(
    "cyberdog/base_info/submit", rclcpp::SystemDefaultsQoS(),
    std::bind(&TransmitCenter::MqttPubCallback, this, std::placeholders::_1));
  be_pub_ =
    tpub_node_ptr_->create_publisher<std_msgs::msg::String>(
    "bes_to_dog",
    rclcpp::SystemDefaultsQoS());
  bsub_ptr_ = std::make_unique<Backend_Subscriber>();

  bhttp_ptr_ = std::make_unique<Backend_Http>(
    "https://test-server.cyberdog.xiaomi.com", "/toml_config/manager/settings.json");
  INFO("http client is ready");

  http_srv_ =
    http_node_ptr_->create_service<protocol::srv::BesHttp>(
    "bes_http_srv",
    std::bind(
      &TransmitCenter::BesHttpCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, http_node_cb_group_);
  http_send_file_srv_ =
    http_node_ptr_->create_service<protocol::srv::BesHttpSendFile>(
    "bes_http_send_file_srv",
    std::bind(
      &TransmitCenter::BesHttpSendFileCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, http_node_cb_group_);
  upload_syslog_srv_ =
    http_node_ptr_->create_service<std_srvs::srv::Trigger>(
    "upload_syslog",
    std::bind(
      &TransmitCenter::UploadSyslog, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, http_node_cb_group_);
  env_client_ =
    http_node_ptr_->create_client<std_srvs::srv::Trigger>(
    "get_nx_environment", rmw_qos_profile_services_default, http_node_cb_group_);
}

cyberdog::bridge::TransmitCenter::~TransmitCenter()
{
}

void cyberdog::bridge::TransmitCenter::Run()
{
  INFO("start spin");
  executor_.spin();
}

void cyberdog::bridge::TransmitCenter::MqttPubCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!getEnv(1)) {
    return;
  }
  rapidjson::Document json_msg(kObjectType);
  std::string sn, uid, str_to_be_sent;
  if (!getDevInf(sn, uid)) {
    return;
  }
  if (!json_msg.Parse<0>(msg->data.c_str()).HasParseError()) {
    common::CyberdogJson::Add(json_msg, "account", uid);
    common::CyberdogJson::Add(json_msg, "number", sn);
  } else {
    WARN("Parse json error!");
    return;
  }
  if (!CyberdogJson::Document2String(json_msg, str_to_be_sent)) {
    ERROR("Error while msg json converting to string!");
    return;
  }
  bpub_ptr_->Publish(str_to_be_sent.c_str());
}

void cyberdog::bridge::TransmitCenter::MqttSubCallback(const std::string & msg)
{
  std_msgs::msg::String msg_data;
  msg_data.data = msg;
  be_pub_->publish(msg_data);
}

void cyberdog::bridge::TransmitCenter::BesHttpCallback(
  const protocol::srv::BesHttp::Request::SharedPtr request,
  protocol::srv::BesHttp::Response::SharedPtr response)
{
  getEnv(2);
  if (request->url.empty() || request->url == "/") {
    response->data = Backend_Http::GetDefaultResponse("Empty url");
    response->code = Backend_Http::ErrorCode::EMPTY_URL;
    ERROR("Empty url");
    return;
  }
  std::string params("");
  if (!request->params.empty()) {
    params = request->params;
  }
  int mill_seconds = std::min(static_cast<int>(request->milsecs), 6000);
  mill_seconds = std::max(0, mill_seconds);
  mill_seconds = (mill_seconds == 0) ? 3000 : mill_seconds;
  std::string sn, uid;
  response->code = Backend_Http::ErrorCode::OK;
  if (getDevInf(sn, uid)) {
    if (sn.empty()) {
      response->data = Backend_Http::GetDefaultResponse("SN is invalid");
      response->code = Backend_Http::ErrorCode::INVALID_SN;
      return;
    }
    int error_code = Backend_Http::ErrorCode::OK;
    bhttp_ptr_->SetInfo(sn, uid);
    if (request->method == protocol::srv::BesHttp::Request::HTTP_METHOD_GET) {
      response->data = bhttp_ptr_->get(request->url, request->params, mill_seconds, error_code);
    } else if (request->method == protocol::srv::BesHttp::Request::HTTP_METHOD_POST) {
      response->data = bhttp_ptr_->post(request->url, request->params, mill_seconds, error_code);
    }
    response->code = error_code;
  } else {
    response->data = Backend_Http::GetDefaultResponse("DeviceInfo service not available");
    response->code = Backend_Http::ErrorCode::INFO_SERVICE_ERROR;
  }
}

void cyberdog::bridge::TransmitCenter::BesHttpSendFileCallback(
  const protocol::srv::BesHttpSendFile::Request::SharedPtr request,
  protocol::srv::BesHttpSendFile::Response::SharedPtr response)
{
  getEnv(2);
  if (request->url.empty() || request->url == "/") {
    response->data = Backend_Http::GetDefaultResponse("Empty url");
    response->code = Backend_Http::ErrorCode::EMPTY_URL;
    ERROR("Empty url");
    return;
  }
  int mill_seconds = std::min(static_cast<int>(request->milsecs), 6000);
  mill_seconds = std::max(0, mill_seconds);
  mill_seconds = (mill_seconds == 0) ? 3000 : mill_seconds;
  std::string sn, uid;
  response->code = Backend_Http::ErrorCode::OK;
  if (getDevInf(sn, uid)) {
    if (sn.empty()) {
      response->data = Backend_Http::GetDefaultResponse("SN is invalid");
      response->code = Backend_Http::ErrorCode::INVALID_SN;
      return;
    }
    bhttp_ptr_->SetInfo(sn, uid);
    int error_code = Backend_Http::ErrorCode::OK;
    response->data = bhttp_ptr_->SendFile(
      request->method, request->url, request->file_name,
      request->content_type, request->info, request->milsecs, error_code);
    response->code = error_code;
  } else {
    response->data = Backend_Http::GetDefaultResponse("DeviceInfo service not available");
    response->code = Backend_Http::ErrorCode::INFO_SERVICE_ERROR;
  }
}

std::string getTimeStamp()
{
  std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream time_ss;
  time_ss << std::put_time(std::localtime(&tt), "%Y%m%d%H%M%S");
  return time_ss.str();
}

std::vector<std::string> getShellEcho(const std::string & cmd)
{
  std::array<char, 128> buffer;
  std::vector<std::string> result;
  std::unique_ptr<FILE, decltype(& pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    return result;
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    std::string segment_str(buffer.data());
    size_t carriage_return_position = segment_str.find("\n");
    if (carriage_return_position != std::string::npos) {
      segment_str = segment_str.substr(0, carriage_return_position);
    }
    result.push_back(segment_str);
  }
  return result;
}

bool compressLogFiles(std::string & copressed_file_name, const std::string & log_path)
{
  std::string cmd("tar -zcvf ");
  copressed_file_name = "/home/mi/syslog" + getTimeStamp() + ".tgz";
  cmd += copressed_file_name;
  std::vector<std::string> file_list(getShellEcho("ls " + log_path + " | grep syslog"));
  if (file_list.empty()) {
    return false;
  }
  for (auto & each_file : file_list) {
    cmd += " " + each_file;
  }
  cmd = "cd " + log_path + " && " + cmd;
  system(cmd.c_str());
  return true;
}

void cyberdog::bridge::TransmitCenter::UploadSyslog(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  std::string compressed_file_name;
  if (compressLogFiles(compressed_file_name, log_path_)) {
    INFO("System log files have been compressed as %s.", compressed_file_name.c_str());
    auto req = std::make_shared<protocol::srv::BesHttpSendFile::Request>();
    req->method = protocol::srv::BesHttpSendFile::Request::HTTP_METHOD_POST;
    req->url = upload_url_;
    req->file_name = compressed_file_name;
    req->content_type = "application/x-tar";
    req->info = "app触发主动上报";
    req->milsecs = 60000;  // 60s
    auto res = std::make_shared<protocol::srv::BesHttpSendFile::Response>();
    BesHttpSendFileCallback(req, res);
    if (res->code == Backend_Http::ErrorCode::OK) {
      response->success = true;
    } else {
      response->success = false;
    }
  } else {
    ERROR("Failed to compress log files!");
    response->success = false;
  }
}

bool cyberdog::bridge::TransmitCenter::getDevInf(std::string & sn, std::string & uid)
{
  static std::string sn_;
  if (!device_info_client_->wait_for_service(std::chrono::seconds(3))) {
    WARN("query_divice_info server not avalible!");
    return false;
  }
  auto req = std::make_shared<protocol::srv::DeviceInfo::Request>();
  std::vector<bool> req_v {true, false, true};
  bool sn_acquired = !sn_.empty();
  if (sn_acquired) {
    req_v[0] = false;
  }
  req->enables = req_v;
  auto future_result = device_info_client_->async_send_request(req);
  rapidjson::Document json_dev_inf_doc(kObjectType);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  if (status == std::future_status::ready) {
    std::string info = future_result.get()->info;
    if (!json_dev_inf_doc.Parse<0>(info.c_str()).HasParseError()) {
      if (sn_acquired) {
        sn = sn_;
        return common::CyberdogJson::Get(json_dev_inf_doc, "uid", uid);
      } else {
        return common::CyberdogJson::Get(json_dev_inf_doc, "sn", sn) &&
               common::CyberdogJson::Get(json_dev_inf_doc, "uid", uid);
      }
    } else {
      WARN("Parse json error!");
    }
  } else {
    WARN("query_divice_info service timeout!");
  }
  return false;
}

bool cyberdog::bridge::TransmitCenter::getEnv(uint8_t mode)
{
  if (!env_client_->wait_for_service(std::chrono::seconds(3))) {
    WARN("get_nx_environment server not avalible!");
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = env_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  if (status == std::future_status::ready) {
    std::string env_str = future_result.get()->message;
    if (env_str == "test" && (!bpub_is_ready_ || current_env_[0] || current_env_[1])) {
      if ((mode == 0 || mode == 1) && (!bpub_is_ready_ || current_env_[0])) {
        INFO("Switch to test mode for mqtt pub");
        if (!bpub_ptr_->SetAddr(
            env_mqtt_pub_url_[0].first, env_mqtt_pub_url_[0].second,
            env_mqtt_user_password[0].first, env_mqtt_user_password[0].second))
        {
          ERROR("Failed to init mqtt");
          return false;
        }
        current_env_[0] = false;
        bpub_is_ready_ = true;
      }
      if ((mode == 0 || mode == 2) && current_env_[1]) {
        INFO("Switch to test mode for http");
        bhttp_ptr_->SetBaseUrl(env_http_url_[0]);
        current_env_[1] = false;
      }
      INFO("Test mode is ready");
    } else if (env_str == "pro" && (!bpub_is_ready_ || !current_env_[0] || !current_env_[1])) {
      if ((mode == 0 || mode == 1) && (!bpub_is_ready_ || !current_env_[0])) {
        INFO("Switch to product mode for mqtt pub");
        if (!bpub_ptr_->SetAddr(
            env_mqtt_pub_url_[1].first, env_mqtt_pub_url_[1].second,
            env_mqtt_user_password[1].first, env_mqtt_user_password[1].second))
        {
          ERROR("Failed to init mqtt");
          return false;
        }
        current_env_[0] = true;
        bpub_is_ready_ = true;
      }
      if ((mode == 0 || mode == 2) && !current_env_[1]) {
        INFO("Switch to product mode for http");
        bhttp_ptr_->SetBaseUrl(env_http_url_[1]);
        current_env_[1] = true;
      }
      INFO("Product mode is ready");
    }
  } else {
    WARN("get_nx_environment service timeout!");
  }
  return true;
}
