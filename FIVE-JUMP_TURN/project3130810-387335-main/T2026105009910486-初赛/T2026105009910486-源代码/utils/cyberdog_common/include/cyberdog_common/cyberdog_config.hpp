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
#ifndef CYBERDOG_COMMON__CYBERDOG_CONFIG_HPP_
#define CYBERDOG_COMMON__CYBERDOG_CONFIG_HPP_
#include <string>
#include <vector>
#include "cyberdog_common/cyberdog_toml.hpp"

namespace cyberdog
{
namespace common
{
class CyberdogConfig final
{
public:
  explicit CyberdogConfig(const std::string & name)
  : name_(name) {}
  ~CyberdogConfig() {}

public:
  /**
   * @brief 根据K获取V
   *          1. 若动态配置中存在，则赋值为动态V值
   *          2. 若动态配置不存在，则赋值为静态V值
   *          3. 都不存在，返回false
   *
   * @tparam V
   * @param key
   * @param value
   * @return true
   * @return false
   */
  template<typename V>
  bool Get(const std::string & key, V & value)
  {
    return true;
  }

  template<typename T>
  bool Get(const std::string & key, std::vector<T> & value)
  {
    return true;
  }

  template<typename V>
  bool Get(const std::string & key_first, const std::string & key_second, V & value)
  {
    return true;
  }

  template<typename V>
  bool Get(const std::string & key_first, const int & key_second, V & value)
  {
    return true;
  }

  /**
   * @brief 根据K获取V
   *          1. 若存在动态配置，则存入K-V键值对
   *          2. 若动态配置不存在，则返回false
   *
   * @tparam K
   * @tparam V
   * @param k
   * @param v
   * @return true
   * @return false
   */
  template<typename K, typename V>
  bool Set(const K & k, const V & v)
  {
    return true;
  }

private:
  std::string name_;
  toml::value data_;
  toml::value data_default_;
};  // class CyberdogConfig
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_CONFIG_HPP_
