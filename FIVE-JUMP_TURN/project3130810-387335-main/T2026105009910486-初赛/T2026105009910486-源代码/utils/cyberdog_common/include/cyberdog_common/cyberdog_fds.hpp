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
#ifndef CYBERDOG_COMMON__CYBERDOG_FDS_HPP_
#define CYBERDOG_COMMON__CYBERDOG_FDS_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <utility>
#include <functional>

#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace common
{
class FdsInterface
{
public:
  virtual ~FdsInterface() {}
  virtual std::vector<std::string> ListBuckets() = 0;
  virtual std::vector<std::string> ListObjects(
    const std::string & bucket_name, const std::string & prefix) = 0;
  virtual bool GetObject(
    const std::string & bucket_name, const std::string & object_name,
    const std::string & download_path, std::function<void(double)> progress) = 0;
  virtual std::map<std::string, std::string> GetObjectMetadata(
    const std::string & bucket_name,
    const std::string & object_name) = 0;
  virtual size_t GetObjectSize(
    const std::string & bucket_name,
    const std::string & object_name) = 0;
  virtual void StopDownloading() = 0;
};

class CyberdogFDS
{
public:
  CyberdogFDS();
  CyberdogFDS(
    const std::string & a_key, const std::string & s_key, const std::string & end_point);
  void CreateFdsClient(
    const std::string & a_key, const std::string & s_key, const std::string & end_point);
  inline bool DoesClientExist()
  {
    return fds_ != nullptr;
  }
  std::vector<std::string> ListBuckets();
  std::vector<std::string> ListObjects(
    const std::string & bucket_name, const std::string & prefix = "");
  bool GetObject(
    const std::string & bucket_name,
    const std::string & object_name, const std::string & download_path,
    std::function<void(double)> progress = [](double) {return;});
  bool GetObject(
    const std::string & bucket_name, const std::string & prefix,
    const std::string & object_name, const std::string & download_path,
    std::function<void(double)> progress = [](double) {return;});
  std::map<std::string, std::string> GetObjectMetadata(
    const std::string & bucket_name,
    const std::string & object_name);
  std::map<std::string, std::string> GetObjectMetadata(
    const std::string & bucket_name,
    const std::string & prefix, const std::string & object_name);
  template<typename ... T>
  std::string GetObjectMD5(T && ... args)
  {
    return GetObjectMetadata(std::forward<T>(args) ...)["content-md5"];
  }
  template<typename ... T>
  std::string GetObjectType(T && ... args)
  {
    return GetObjectMetadata(std::forward<T>(args) ...)["content-type"];
  }
  size_t GetObjectSize(
    const std::string & bucket_name,
    const std::string & object_name);
  size_t GetObjectSize(
    const std::string & bucket_name,
    const std::string & prefix, const std::string & object_name);
  void StopDownloading()
  {
    fds_->StopDownloading();
  }

private:
  std::unique_ptr<FdsInterface> fds_;

  LOGGER_MINOR_INSTANCE("CyberdogFDS")
};
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_FDS_HPP_
