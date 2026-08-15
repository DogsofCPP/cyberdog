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
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include "cyberdog_common/cyberdog_fds.hpp"

double persentage;

void callback(double per)
{
  persentage = per;
  std::cout << "progress: " << persentage * 100 << "%" << std::endl;
}

void timeout(cyberdog::common::CyberdogFDS* fds)
{
  sleep(3);
  fds->StopDownloading();
  std::cout << "stop downloading!" << std::endl;
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cout << "account and secret" << std::endl;
    return -1;
  }
  std::string account(argv[1]), secret(argv[2]);
  cyberdog::common::CyberdogFDS fds_test(account, secret, "cnbj2m.fds.api.xiaomi.com");
  // or cyberdog::common::CyberdogFDS fds_test; fds_test.CreateFdsClient(account, secret, "cnbj2m.fds.api.xiaomi.com");
  if (!fds_test.DoesClientExist())
  {
    std::cout << "client creation failed" << std::endl;
  }
  /*
  auto bucket_list = fds_test.ListBuckets();
  for (auto & bucket : bucket_list)
  {
    std::cout << bucket << std::endl;
  }
  */
  std::vector<std::string> object_list = fds_test.ListObjects("platform-module", "algo/test-inner/");
  std::cout << "got object list. list size is " << object_list.size() <<std::endl;
  for (auto & obj : object_list)
  {
    std::cout << obj << std::endl;
  }
  object_list = fds_test.ListObjects("platform-module", "algo/");
  std::cout << "got object list. list size is " << object_list.size() <<std::endl;
  for (auto & obj : object_list)
  {
    std::cout << obj << std::endl;
  }
  std::thread t1(&timeout, &fds_test);
  if (fds_test.GetObject("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt", "/home/mi/", callback))  // or fds_test.GetObject("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt", "/home/mi/", callback)
  {
    std::cout << "Successfully downloaded file!" << std::endl;
    std::cout << "file md5 is " << fds_test.GetObjectMD5("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt") << std::endl;
    // or fds_test.GetObjectMD5("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt")
    std::cout << "file size is " << fds_test.GetObjectSize("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt") << std::endl;
    // or fds_test.GetObjectSize("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt")
    std::cout << "file type is " << fds_test.GetObjectType("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt") << std::endl;
    // or fds_test.GetObjectType("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt")
  } else {
    std::cout << "failed to download file" << std::endl;
  }
  t1.join();
  return 0;
}