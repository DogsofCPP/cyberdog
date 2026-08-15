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

#ifndef CYBERDOG_COMMON__CYBERDOG_FDS_IMPL_HPP_
#define CYBERDOG_COMMON__CYBERDOG_FDS_IMPL_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <atomic>

#include "/usr/local/include/galaxy_fds_client.h"
#include "cyberdog_common/cyberdog_fds.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace common
{
class FdsImpl : public FdsInterface
{
public:
  FdsImpl(const std::string & a_key, const std::string & s_key, const std::string & end_point);
  std::vector<std::string> ListBuckets() override;
  std::vector<std::string> ListObjects(
    const std::string & bucket_name, const std::string & prefix) override;
  bool GetObject(
    const std::string & bucket_name, const std::string & object_name,
    const std::string & download_path, std::function<void(double)> progress) override;
  std::map<std::string, std::string> GetObjectMetadata(
    const std::string & bucket_name,
    const std::string & object_name) override;
  size_t GetObjectSize(
    const std::string & bucket_name,
    const std::string & object_name) override;
  void StopDownloading() override
  {
    downloading_ = false;
  }

private:
  std::unique_ptr<galaxy::fds::GalaxyFDSClient> fds_client_;
  std::shared_ptr<galaxy::fds::FDSObject> getObjectPtr(
    const std::string & bucket_name,
    const std::string & object_name);
  std::atomic_bool downloading_ {false};

  LOGGER_MINOR_INSTANCE("FdsImpl");
};
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_FDS_IMPL_HPP_
