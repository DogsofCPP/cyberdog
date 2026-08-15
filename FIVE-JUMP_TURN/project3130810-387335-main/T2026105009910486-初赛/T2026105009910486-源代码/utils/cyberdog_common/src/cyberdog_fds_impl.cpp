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

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_common/cyberdog_fds_impl.hpp"
#include "/usr/local/include/fds_client_configuration.h"
#include "/usr/local/include/galaxy_fds_client_exception.h"
#include "/usr/local/include/model/fds_bucket.h"
#include "/usr/local/include/model/fds_object.h"
#include "/usr/local/include/model/fds_object_listing.h"
#include "/usr/local/include/model/fds_object_summary.h"

namespace cyberdog
{
namespace common
{
FdsImpl::FdsImpl(
  const std::string & a_key, const std::string & s_key, const std::string & end_point)
{
  galaxy::fds::FDSClientConfiguration cfg;
  std::string region;
  size_t find_ = end_point.find("fds.api.xiaomi");
  if (find_ != std::string::npos) {
    region = end_point.substr(0, find_ - 1);
    cfg.setRegionName(region);
  }
  cfg.setEndpoint(end_point);
  cfg.enableHttps(true);
  cfg.enableCdnForDownload(true);
  fds_client_ = std::make_unique<galaxy::fds::GalaxyFDSClient>(a_key, s_key, cfg);
}

std::vector<std::string> FdsImpl::ListBuckets()
{
  try {
    auto buckets = fds_client_->listBuckets();
    std::vector<std::string> buckets_name;
    for (auto & bucket : buckets) {
      buckets_name.emplace_back(bucket->name());
    }
    return buckets_name;
  } catch (std::exception & e) {
    ERROR_STREAM(e.what());
  }
  return std::vector<std::string>();
}

std::string SplitFilename(const std::string & str)
{
  std::size_t found = str.find_last_of('/');
  if (found == std::string::npos) {
    return str;
  }
  return str.substr(found + 1);
}

std::vector<std::string> FdsImpl::ListObjects(
  const std::string & bucket_name, const std::string & prefix)
{
  try {
    if (!fds_client_->doesBucketExist(bucket_name)) {
      ERROR_STREAM("There is no bucket " << bucket_name);
      return std::vector<std::string>();
    }
    auto objects = fds_client_->listObjects(bucket_name, prefix);
    std::vector<std::string> object_names;
    for (auto & prefix : objects->commonPrefixes()) {
      object_names.emplace_back(prefix.substr(prefix.find_last_of('/', prefix.length() - 2) + 1));
    }
    for (auto & obj_sum : objects->objectSummaries()) {
      if (obj_sum.size()) {
        object_names.emplace_back(SplitFilename(obj_sum.objectName()));
      }
    }
    return object_names;
  } catch (std::exception & e) {
    ERROR_STREAM(e.what());
  }
  return std::vector<std::string>();
}

size_t copyStream(
  std::istream & is, std::ostream & os,
  size_t full_size = 1, std::function<void(double)> progress = [](double) {return;},
  std::atomic_bool * status_mark = nullptr)
{
  if (status_mark) {
    *status_mark = true;
  }
  size_t bufferSize = 1024;
  double full_size_double = static_cast<double>(full_size);
  char * buffer = new char[bufferSize];
  std::streamsize len = 0;
  is.read(buffer, bufferSize);
  std::streamsize n = is.gcount();
  while (n > 0 && (!status_mark || *status_mark)) {
    len += n;
    os.write(buffer, n);
    progress(len / full_size_double);
    if (is) {
      is.read(buffer, bufferSize);
      n = is.gcount();
    } else {
      n = 0;
    }
  }
  delete[] buffer;
  return len;
}

bool FdsImpl::GetObject(
  const std::string & bucket_name, const std::string & object_name,
  const std::string & download_path, std::function<void(double)> progress)
{
  std::shared_ptr<galaxy::fds::FDSObject> object = getObjectPtr(bucket_name, object_name);
  if (!object) {
    return false;
  }
  std::string prefix_path(download_path);
  if (download_path.back() != '/') {
    prefix_path += "/";
  }
  std::ofstream outfile;
  outfile.open(
    prefix_path + SplitFilename(object_name),
    std::ofstream::out | std::ofstream::trunc | std::ofstream::binary);
  if (!outfile.is_open()) {
    ERROR_STREAM("Not able to create file: " << prefix_path + SplitFilename(object_name));
    return false;
  }
  std::istream & is = object->objectContent();
  size_t object_size = object->objectSummary().size();
  if (!object_size) {
    object_size = 1;
  }
  size_t downloaded_size = copyStream(is, outfile, object_size, progress, &downloading_);
  if (!downloading_ || downloaded_size < object_size) {
    outfile.close();
    WARN_STREAM("Downloading of object " << SplitFilename(object_name) << " is interrupted.");
    remove(std::string(prefix_path + SplitFilename(object_name)).c_str());
    downloading_ = false;
    return false;
  }
  outfile.close();
  return true;
}

std::map<std::string, std::string> FdsImpl::GetObjectMetadata(
  const std::string & bucket_name,
  const std::string & object_name)
{
  try {
    if (!fds_client_->doesBucketExist(bucket_name)) {
      ERROR_STREAM("There is no buckect " << bucket_name);
      return std::map<std::string, std::string>();
    }
    if (!fds_client_->doesObjectExist(bucket_name, object_name)) {
      ERROR_STREAM("There is no object " << object_name << " in " << bucket_name);
      return std::map<std::string, std::string>();
    }
    auto meta_data_ptr = fds_client_->getObjectMetadata(bucket_name, object_name);
    if (meta_data_ptr && !meta_data_ptr->metadata().empty()) {
      return meta_data_ptr->metadata();
    }
    WARN("Not able to get Metadata");
    return std::map<std::string, std::string>();
  } catch (std::exception & e) {
    ERROR_STREAM(e.what());
  }
  return std::map<std::string, std::string>();
}

size_t FdsImpl::GetObjectSize(
  const std::string & bucket_name,
  const std::string & object_name)
{
  std::shared_ptr<galaxy::fds::FDSObject> object = getObjectPtr(bucket_name, object_name);
  if (!object) {
    return 0;
  }
  return object->objectSummary().size();
}

std::shared_ptr<galaxy::fds::FDSObject> FdsImpl::getObjectPtr(
  const std::string & bucket_name,
  const std::string & object_name)
{
  try {
    if (!fds_client_->doesBucketExist(bucket_name)) {
      ERROR_STREAM("There is no buckect " << bucket_name);
      return nullptr;
    }
    if (!fds_client_->doesObjectExist(bucket_name, object_name)) {
      ERROR_STREAM("There is no object " << object_name << " in " << bucket_name);
      return nullptr;
    }
    return fds_client_->getObject(bucket_name, object_name);
  } catch (std::exception & e) {
    ERROR_STREAM(e.what());
  }
  return nullptr;
}

CyberdogFDS::CyberdogFDS()
: fds_(nullptr)
{}

CyberdogFDS::CyberdogFDS(
  const std::string & a_key, const std::string & s_key, const std::string & end_point)
: fds_(std::make_unique<FdsImpl>(a_key, s_key, end_point))
{}

void CyberdogFDS::CreateFdsClient(
  const std::string & a_key, const std::string & s_key, const std::string & end_point)
{
  fds_ = std::make_unique<FdsImpl>(a_key, s_key, end_point);
}

std::vector<std::string> CyberdogFDS::ListBuckets()
{
  std::vector<std::string> bucket_names;
  if (!fds_) {
    return bucket_names;
  }
  return fds_->ListBuckets();
}

std::vector<std::string> CyberdogFDS::ListObjects(
  const std::string & bucket_name, const std::string & prefix)
{
  if (!fds_) {
    ERROR("Please create fds client first");
    return std::vector<std::string>();
  }
  return fds_->ListObjects(bucket_name, prefix);
}

bool CyberdogFDS::GetObject(
  const std::string & bucket_name,
  const std::string & object_name, const std::string & download_path,
  std::function<void(double)> progress)
{
  if (!fds_) {
    ERROR("Please create fds client first");
    return false;
  }
  return fds_->GetObject(bucket_name, object_name, download_path, progress);
}

bool CyberdogFDS::GetObject(
  const std::string & bucket_name, const std::string & prefix,
  const std::string & object_name, const std::string & download_path,
  std::function<void(double)> progress)
{
  if (!fds_) {
    ERROR("Please create fds client first");
    return false;
  }
  std::string full_name;
  if (prefix.find_last_of('/') == std::string::npos) {
    full_name = prefix + "/" + object_name;
  } else {
    full_name = prefix + object_name;
  }
  return fds_->GetObject(bucket_name, full_name, download_path, progress);
}

std::map<std::string, std::string> CyberdogFDS::GetObjectMetadata(
  const std::string & bucket_name,
  const std::string & object_name)
{
  if (!fds_) {
    ERROR("Please create fds client first");
    return std::map<std::string, std::string>();
  }
  return fds_->GetObjectMetadata(bucket_name, object_name);
}

std::map<std::string, std::string> CyberdogFDS::GetObjectMetadata(
  const std::string & bucket_name,
  const std::string & prefix, const std::string & object_name)
{
  std::string full_name;
  if (prefix.find_last_of('/') == std::string::npos) {
    full_name = prefix + "/" + object_name;
  } else {
    full_name = prefix + object_name;
  }
  return GetObjectMetadata(bucket_name, full_name);
}

size_t CyberdogFDS::GetObjectSize(
  const std::string & bucket_name,
  const std::string & object_name)
{
  return fds_->GetObjectSize(bucket_name, object_name);
}

size_t CyberdogFDS::GetObjectSize(
  const std::string & bucket_name,
  const std::string & prefix, const std::string & object_name)
{
  std::string full_name;
  if (prefix.find_last_of('/') == std::string::npos) {
    full_name = prefix + "/" + object_name;
  } else {
    full_name = prefix + object_name;
  }
  return fds_->GetObjectSize(bucket_name, full_name);
}
}  // namespace common
}  // namespace cyberdog
