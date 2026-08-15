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

#ifndef FILE_UPLOADING__LCM_LOG_UPLOADER_HPP_
#define FILE_UPLOADING__LCM_LOG_UPLOADER_HPP_

#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/bes_http.hpp"
#include "protocol/srv/bes_http_send_file.hpp"
#include "protocol/srv/device_info.hpp"

namespace cyberdog
{
class LcmExceptionProcesserBase
{
public:
  explicit LcmExceptionProcesserBase(rclcpp::Node::SharedPtr node);

protected:
  bool getDevInf();
  std::string sn_, uid_;
  std::string upload_url_;
  rclcpp::CallbackGroup::SharedPtr http_cb_group_;
  rclcpp::Client<protocol::srv::DeviceInfo>::SharedPtr device_info_client_ {nullptr};
};

class LcmEventUploader : public LcmExceptionProcesserBase
{
public:
  explicit LcmEventUploader(rclcpp::Node::SharedPtr node);
  int recordEvent(int motion_id, int code, int64_t time);

private:
  int uploadEvent(int motion_id, int code, int64_t time);
  std::string save_log_cmd_;
  rclcpp::Client<protocol::srv::BesHttp>::SharedPtr http_info_client_ {nullptr};
  std::map<int, int64_t> code_time_map_;
};

class LcmLogUploader : public LcmExceptionProcesserBase
{
public:
  explicit LcmLogUploader(rclcpp::Node::SharedPtr node);
  int checkAndUploadLcmLog();

private:
  int uploadLog(const std::string & file_path);
  std::string get_log_path_cmd_, delete_log_cmd_;
  rclcpp::Client<protocol::srv::BesHttpSendFile>::SharedPtr http_file_client_ {nullptr};
};
}  // namespace cyberdog

#endif  // FILE_UPLOADING__LCM_LOG_UPLOADER_HPP_
