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
#ifndef CYBERDOG_COMMON__CYBERDOG_TOML_HPP_
#define CYBERDOG_COMMON__CYBERDOG_TOML_HPP_
#include <iostream>
#include <string>
#include <vector>
#include "toml/toml.hpp"

namespace cyberdog
{
namespace common
{
/**
 * @brief 封装toml作为配置文件场景的API
 *        均为static类型接口.
 *        实例化该类的行为是未定义的.
 */
class CyberdogToml final
{
public:
  CyberdogToml() {}
  ~CyberdogToml() {}

public:
  /* Trans data between toml and file. */
  /**
   * @brief 从文件中读取数据，并翻译成toml数据结构
   *
   * @return  是否执行成功.
   *          注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *                若无视错误返回，则可能引发未定义行为或程序崩溃，如获取数据Get等.
   */
  static bool ParseFile(const std::string & file_name, toml::value & v)
  {
    try {
      v = toml::parse(file_name);
      return true;
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
      return false;
    }
  }

  /**
   * @brief 将toml数据写入文件
   *
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的文件是不可读状态，对其操作的行为未定义.
   */
  static bool WriteFile(const std::string & file_name, const toml::value & v)
  {
    try {
      std::ofstream ofs(file_name, std::ofstream::out);
      ofs << v;
      ofs.close();
      return true;
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
      return false;
    }
  }

  /**
   * @brief 从toml表格数据中，依据键k读取一个值
   *
   * @tparam T 值类型，为toml支持的全部类型
   * @param v 要求已经初始化且为toml value_t::table类型
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的返回形参是不可使用状态，对其操作的行为未定义.
   */
  template<typename T>
  static bool Get(const toml::value & v, const std::string & k, T & m)
  {
    if ((v.is_uninitialized()) || (!v.is_table()) || (v.count(k) == 0)) {
      return false;
    } else {
      m = toml::get<T>(v.at(k));
      return true;
    }
  }

  /**
 * @brief 从toml数组数据中，依据角标序号读取一个值
 *
 * @tparam T 值类型，为toml支持的全部类型
 * @param v 要求已经初始化且为toml value_t::array类型
 * @return 执行是否成功
 *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
 *               此时的返回形参是不可使用状态，对其操作的行为未定义.
 */
  template<typename T>
  static bool Get(const toml::value & v, size_t k, T & m)
  {
    if ((v.is_uninitialized()) || (!v.is_array()) || (!(v.size() > k))) {
      return false;
    } else {
      m = toml::get<T>(v.at(k));
      return true;
    }
  }

  /**
   * @brief 为toml表格数据设置一个值
   *          1. 若该键已经存在，则会覆盖，包括不同类型
   *          2. 若该键不存在，则会添加一个新的键值对
   *
   * @tparam T 值类型，为toml支持的全部类型
   * @param v 要求未初始化， 或者已经初始化为toml value_t::table类型
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的输入形参v是不可使用状态，对其操作的行为未定义.
   */
  template<typename T>
  static bool Set(toml::value & v, const std::string & k, const T & m)
  {
    if ((!v.is_uninitialized()) && (!v.is_table())) {
      return false;
    } else {
      v[k] = m;
      return true;
    }
  }

  /**
   * @brief 为toml数组数据设置一个值
   *          1. 该值会被追加在数组末尾
   *
   * @tparam T 值类型，为toml支持的全部类型
   * @param v 要求未初始化， 或者已经初始化为toml value_t::array类型
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的输入形参v是不可使用状态，对其操作的行为未定义.
   */
  template<typename T>
  static bool Set(toml::value & v, const T & m)
  {
    if (v.is_uninitialized()) {
      std::vector<int> t;
      v = t;
      v.as_array().push_back(m);
    } else if (!v.is_array()) {
      return false;
    }
    return true;
  }

  /**
   * @brief 为toml数组数据设置一个值，依据角标序号
   *          1. 若序号已经存在，则会覆盖，包括不同类型
   *          2. 若值序号不存在，则不会添加，且返回错误
   *
   * @tparam T 值类型，为toml支持的全部类型
   * @param v 要求未初始化， 或者已经初始化为toml value_t::array类型
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的输入形参v是不可使用状态，对其操作的行为未定义.
   */
  template<typename T>
  static bool Set(toml::value & v, size_t k, const T & m)
  {
    if (v.is_uninitialized() || (!v.is_array()) || (!(v.size() > k))) {
      return false;
    } else {
      v[k] = m;
      return true;
    }
  }
};  // class CyberdotToml
}  // namespace common
}  // namespace cyberdog

#endif  // CYBERDOG_COMMON__CYBERDOG_TOML_HPP_
