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
#include <cstring>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>

#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "file_uploading/lcm_log_uploader.hpp"

using cyberdog::common::CyberdogJson;

namespace cyberdog
{
int runCommand(const std::string & cmd, std::string & std_output)
{
  FILE * fp;
  char buf[128] {'\0'};
  if ((fp = popen(cmd.c_str(), "r")) == NULL) {
    printf("failed to popen");
    return -1;
  }
  while (fgets(buf, sizeof(buf), fp) != NULL) {
    std_output += std::string(buf);
    memset(buf, '\0', sizeof(buf));
  }
  return pclose(fp);
}

LcmExceptionProcesserBase::LcmExceptionProcesserBase(rclcpp::Node::SharedPtr node)
{
  http_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  device_info_client_ =
    node->create_client<protocol::srv::DeviceInfo>(
    "query_divice_info", rmw_qos_profile_services_default, http_cb_group_);
  upload_url_ = "device/system/log";
}

bool LcmExceptionProcesserBase::getDevInf()
{
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
  rapidjson::Document json_dev_inf_doc(rapidjson::kObjectType);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  if (status == std::future_status::ready) {
    std::string info = future_result.get()->info;
    if (!json_dev_inf_doc.Parse<0>(info.c_str()).HasParseError()) {
      if (sn_acquired) {
        return common::CyberdogJson::Get(json_dev_inf_doc, "uid", uid_);
      } else {
        return common::CyberdogJson::Get(json_dev_inf_doc, "sn", sn_) &&
               common::CyberdogJson::Get(json_dev_inf_doc, "uid", uid_);
      }
    } else {
      WARN("Parse json error!");
    }
  } else {
    WARN("query_divice_info service timeout!");
  }
  return false;
}

LcmEventUploader::LcmEventUploader(rclcpp::Node::SharedPtr node)
: LcmExceptionProcesserBase(node)
{
  http_info_client_ =
    node->create_client<protocol::srv::BesHttp>(
    "bes_http_srv", rmw_qos_profile_services_default, http_cb_group_);
  save_log_cmd_ = "";
}

int LcmEventUploader::recordEvent(int motion_id, int code, int64_t time)
{
  auto code_itr = code_time_map_.find(code);
  if (code_itr != code_time_map_.end()) {
    if (std::abs(time - code_itr->second) <= 10000) {
      code_itr->second = time;
      return 0;
    }
  } else {
    code_time_map_[code] = time;
  }
  int result = 0;
  result = uploadEvent(motion_id, code, time);
  if (result != 0) {
    ERROR("Not albe to upload event to backend server. error code: %d", result);
    return result;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  std::string output;
  result = runCommand(save_log_cmd_, output);
  if (result != 0) {
    ERROR("Not albe to save lcm log. error code: %d", result);
  }
  return result;
}

int LcmEventUploader::uploadEvent(int motion_id, int code, int64_t time)
{
  if (!http_info_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("bes_http_srv not available");
    return 2;
  }
  rapidjson::Document json_event(rapidjson::kObjectType);
  CyberdogJson::Add(json_event, "motion_id", motion_id);
  CyberdogJson::Add(json_event, "code", code);
  CyberdogJson::Add(json_event, "time", uint64_t(time));
  std::string json_str;
  if (!CyberdogJson::Document2String(json_event, json_str)) {
    ERROR("error while encoding to json");
    return 2;
  }
  auto req = std::make_shared<protocol::srv::BesHttp::Request>();
  req->method = protocol::srv::BesHttp::Request::HTTP_METHOD_POST;
  req->url = upload_url_;
  req->params = json_str;
  req->milsecs = 10000;  // 10s
  auto future_result = http_info_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(11));
  if (status == std::future_status::ready) {
    INFO("Success to call bes_http_srv services.");
  } else {
    WARN(
      "Failed to call bes_http_srv services.");
    return 2;
  }
  if (future_result.get()->code != 200) {
    ERROR("Fail to post https request. code: %d", future_result.get()->code);
    return 3;
  }
  return 0;
}

LcmLogUploader::LcmLogUploader(rclcpp::Node::SharedPtr node)
: LcmExceptionProcesserBase(node)
{
  http_file_client_ =
    node->create_client<protocol::srv::BesHttpSendFile>(
    "bes_http_send_file_srv", rmw_qos_profile_services_default, http_cb_group_);
  get_log_path_cmd_ = "";
  delete_log_cmd_ = "";
}

int LcmLogUploader::checkAndUploadLcmLog()
{
  std::string file_path;
  int result = runCommand(get_log_path_cmd_, file_path);
  if (result != 0 || file_path.empty()) {
    WARN("No lcm log is recorded");
    return 1;
  }
  result = uploadLog(file_path);
  if (result != 0) {
    ERROR("Not albe to upload lcm log to backend server. error code: %d", result);
    return 3;
  }
  result = runCommand(delete_log_cmd_, file_path);
  if (result != 0) {
    ERROR("Not albe to delete lcm log. error code: %d", result);
    return 4;
  }
  return 0;
}

int LcmLogUploader::uploadLog(const std::string & file_path)
{
  if (!http_file_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("bes_http_send_file_srv not available");
    return 1;
  }
  auto req = std::make_shared<protocol::srv::BesHttpSendFile::Request>();
  req->method = protocol::srv::BesHttpSendFile::Request::HTTP_METHOD_POST;
  req->url = upload_url_;
  req->file_name = file_path;
  req->content_type = "application/x-tar";
  req->info = "lcm异常日志";
  req->milsecs = 60000;  // 60s
  auto future_result = http_file_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(61));
  if (status == std::future_status::ready) {
    INFO("Success to call bes_http_send_file_srv services.");
  } else {
    WARN(
      "Failed to call bes_http_send_file_srv services.");
    return 3;
  }
  if (future_result.get()->code != 200) {
    ERROR("Fail to post https request. code: %d", future_result.get()->code);
    return 3;
  }
  return 0;
}
}  // namespace cyberdog
