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

#ifndef CYBERDOG_COMMON__CYBERDOG_LOG_HPP_
#define CYBERDOG_COMMON__CYBERDOG_LOG_HPP_

#include <string>
#include <memory>
#include <map>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace cyberdog
{
namespace common
{
#define UNINITIALIZED_NAME "uninitialized"

class CyberdogLogger final
{
public:
  explicit CyberdogLogger(const char * name)
  : logger_name(name)
  {
    logger = logger_name.empty() ?
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME))
      :
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(logger_name));
  }
  CyberdogLogger(const CyberdogLogger &) = delete;
  CyberdogLogger & operator=(const CyberdogLogger &) = delete;

  ~CyberdogLogger() {}

  std::shared_ptr<rclcpp::Logger> Get_Logger()
  {
    return logger;
  }

private:
  std::string logger_name;
  std::shared_ptr<rclcpp::Logger> logger;
};  // class CyberdogLogger

class CyberdogLoggerFactory final
{
public:
  static std::shared_ptr<rclcpp::Logger> Get_Logger();
  static std::shared_ptr<rclcpp::Logger> Get_Logger(const char * sz_name);

private:
  static std::shared_ptr<rclcpp::Logger> main_logger;
};  // class CyberdogLoggerFactory

}  // namespace common
}  // namespace cyberdog

inline rclcpp::Logger get_logger()
{
  return *(cyberdog::common::CyberdogLoggerFactory::Get_Logger());
}

#define LOGGER_MINOR_INSTANCE(instance_name) \
  std::shared_ptr<rclcpp::Logger> logger; \
  std::mutex logger_mutex; \
  rclcpp::Logger get_logger() { \
    if (!logger) { \
      std::unique_lock<std::mutex> lock(logger_mutex); \
      if (!logger) { \
        std::shared_ptr<rclcpp::Logger> out_logger = \
          cyberdog::common::CyberdogLoggerFactory::Get_Logger(); \
        std::string str_name = std::string(out_logger->get_name()) + " " + instance_name; \
        cyberdog::common::CyberdogLogger cyberdog_logger(str_name.c_str()); \
        logger = cyberdog_logger.Get_Logger(); \
      } \
    } \
    return *logger; \
  }


#define LOGGER_MAIN_INSTANCE(instance_name) \
  std::shared_ptr<rclcpp::Logger> out_logger = \
    cyberdog::common::CyberdogLoggerFactory::Get_Logger( \
    instance_name);

#define DEBUG(...) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define INFO(...) RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define WARN(...) RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR(get_logger(), __VA_ARGS__)
#define FATAL(...) RCLCPP_FATAL(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(get_logger(), __VA_ARGS__)
#define INFO_STREAM(...) RCLCPP_INFO_STREAM(get_logger(), __VA_ARGS__)
#define WARN_STREAM(...) RCLCPP_WARN_STREAM(get_logger(), __VA_ARGS__)
#define ERROR_STREAM(...) RCLCPP_ERROR_STREAM(get_logger(), __VA_ARGS__)
#define FATAL_STREAM(...) RCLCPP_FATAL_STREAM(get_logger(), __VA_ARGS__)

#define DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define INFO_ONCE(...) RCLCPP_INFO_ONCE(get_logger(), __VA_ARGS__)
#define WARN_ONCE(...) RCLCPP_WARN_ONCE(get_logger(), __VA_ARGS__)
#define ERROR_ONCE(...) RCLCPP_ERROR_ONCE(get_logger(), __VA_ARGS__)
#define FATAL_ONCE(...) RCLCPP_FATAL_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_ONCE(...) RCLCPP_DEBUG_STREAM_ONCE(get_logger(), __VA_ARGS__)
#define INFO_STREAM_ONCE(...) RCLCPP_INFO_STREAM_ONCE(get_logger(), __VA_ARGS__)
#define WARN_STREAM_ONCE(...) RCLCPP_WARN_STREAM_ONCE(get_logger(), __VA_ARGS__)
#define ERROR_STREAM_ONCE(...) RCLCPP_ERROR_STREAM_ONCE(get_logger(), __VA_ARGS__)
#define FATAL_STREAM_ONCE(...) RCLCPP_FATAL_STREAM_ONCE(get_logger(), __VA_ARGS__)

#define DEBUG_FUNCTION(function, ...) RCLCPP_DEBUG_FUNCTION(get_logger(), function, __VA_ARGS__)
#define INFO_FUNCTION(function, ...) RCLCPP_INFO_FUNCTION(get_logger(), function, __VA_ARGS__)
#define WARN_FUNCTION(function, ...) RCLCPP_WARN_FUNCTION(get_logger(), function, __VA_ARGS__)
#define ERROR_FUNCTION(function, ...) RCLCPP_ERROR_FUNCTION(get_logger(), function, __VA_ARGS__)
#define FATAL_FUNCTION(function, ...) RCLCPP_FATAL_FUNCTION(get_logger(), function, __VA_ARGS__)
#define DEBUG_STREAM_FUNCTION(function, ...) RCLCPP_DEBUG_STREAM_FUNCTION( \
    get_logger(), function, __VA_ARGS__)
#define INFO_STREAM_FUNCTION(function, ...) RCLCPP_INFO_STREAM_FUNCTION( \
    get_logger(), function, __VA_ARGS__)
#define WARN_STREAM_FUNCTION(function, ...) RCLCPP_WARN_STREAM_FUNCTION( \
    get_logger(), function, __VA_ARGS__)
#define ERROR_STREAM_FUNCTION(function, ...) RCLCPP_ERROR_STREAM_FUNCTION( \
    get_logger(), function, __VA_ARGS__)
#define FATAL_STREAM_FUNCTION(function, ...) RCLCPP_FATAL_STREAM_FUNCTION( \
    get_logger(), function, __VA_ARGS__)

#define DEBUG_EXPRESSION(expression, ...) RCLCPP_DEBUG_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define INFO_EXPRESSION(expression, ...) RCLCPP_INFO_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define WARN_EXPRESSION(expression, ...) RCLCPP_WARN_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define ERROR_EXPRESSION(expression, ...) RCLCPP_ERROR_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define FATAL_EXPRESSION(expression, ...) RCLCPP_FATAL_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define DEBUG_STREAM_EXPRESSION(expression, ...) RCLCPP_DEBUG_STREAM_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define INFO_STREAM_EXPRESSION(expression, ...) RCLCPP_INFO_STREAM_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define WARN_STREAM_EXPRESSION(expression, ...) RCLCPP_WARN_STREAM_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define ERROR_STREAM_EXPRESSION(expression, ...) RCLCPP_ERROR_STREAM_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define FATAL_STREAM_EXPRESSION(expression, ...) RCLCPP_FATAL_STREAM_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)

extern rclcpp::Clock global_steady_clock;
#define DEBUG_MILLSECONDS(millseconds, ...) RCLCPP_DEBUG_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define INFO_MILLSECONDS(millseconds, ...) RCLCPP_INFO_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define WARN_MILLSECONDS(millseconds, ...) RCLCPP_WARN_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define ERROR_MILLSECONDS(millseconds, ...) RCLCPP_ERROR_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define FATAL_MILLSECONDS(millseconds, ...) RCLCPP_FATAL_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define DEBUG_STREAM_MILLSECONDS(millseconds, ...) RCLCPP_DEBUG_STREAM_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define INFO_STREAM_MILLSECONDS(millseconds, ...) RCLCPP_INFO_STREAM_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define WARN_STREAM_MILLSECONDS(millseconds, ...) RCLCPP_WARN_STREAM_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define ERROR_STREAM_MILLSECONDS(millseconds, ...) RCLCPP_ERROR_STREAM_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)
#define FATAL_STREAM_MILLSECONDS(millseconds, ...) RCLCPP_FATAL_STREAM_THROTTLE( \
    get_logger(), global_steady_clock, millseconds, __VA_ARGS__)

class CyberdogLoggerLever final
{
public:
  explicit CyberdogLoggerLever(const rclcpp::Node::SharedPtr & nptr)
  : node_ptr_(nptr)
  {
    if (node_ptr_) {
      std::string node_full_name = std::string(node_ptr_->get_namespace()) +
        node_ptr_->get_name();
      srv_ = node_ptr_->create_service<std_srvs::srv::SetBool>(
        node_full_name + "/set_severity", std::bind(
          &CyberdogLoggerLever::handle_logger_config_request,
          this, std::placeholders::_1, std::placeholders::_2));
    } else {
      ERROR("Invalide node handle.");
    }
  }

private:
  void handle_logger_config_request(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::map<uint8_t, const char *> severity_map = {
      {0, "debug"},
      {1, "info"},
      {2, "warn"},
      {3, "error"},
      {4, "fatal"}
    };
    uint8_t request_level = request->data == true ? 1 : 0;
    const char * severity_string = severity_map[request_level];
    int severity;
    response->message = "unkown";
    rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
      severity_string, rcl_get_default_allocator(), &severity);
    if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
      ERROR(
        "Unknown severity '%s'", severity_string);
      response->success = false;
      return;
    }
    if (RCUTILS_RET_OK != ret) {
      ERROR(
        "Error %d getting severity level from request: %s", ret,
        rcl_get_error_string().str);
      rcl_reset_error();
      response->success = false;
      return;
    }
    ret = rcutils_logging_set_logger_level(node_ptr_->get_name(), severity);
    if (ret != RCUTILS_RET_OK) {
      ERROR(
        "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
      response->success = false;
    }
    response->success = true;
    response->message = severity_string;
  }

private:
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
};  // class CyberdogLoggerLever

/*
#define LOGGER_INSTANCE(instance_name) \
        cyberdog::common::CyberdogLogger cyberdog_logger(instance_name); \
        std::shared_ptr<rclcpp::Logger> out_logger = cyberdog_logger.Get_Logger();
*/

#endif  // CYBERDOG_COMMON__CYBERDOG_LOG_HPP_
