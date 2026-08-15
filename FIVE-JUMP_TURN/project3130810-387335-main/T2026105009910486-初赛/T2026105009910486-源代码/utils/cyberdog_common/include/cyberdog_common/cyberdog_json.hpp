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
#ifndef CYBERDOG_COMMON__CYBERDOG_JSON_HPP_
#define CYBERDOG_COMMON__CYBERDOG_JSON_HPP_
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <mutex>
#include <vector>
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "cyberdog_common/cyberdog_log.hpp"

#define    CYBERDOGJSON     "CyberdogJson"

namespace cyberdog
{
namespace common
{
/* 要求使用json代替rapidjson命名空间 */
namespace json = rapidjson;

/**
 * @brief 封装json在序列化场景的API
 *        均为static类型接口.
 *        实例化该类的行为是未定义的.
 */
class CyberdogJson final
{
  // LOGGER_MINOR_INSTANCE("CyberdogJson");

public:
  CyberdogJson() {}
  ~CyberdogJson() {}

public:
  /**
   * @brief 为已创建好的Document添加一个变量
   *          1. key已存在则覆盖为新的值
   *          2. key不存在则增加一个键值对
   *          3. 增加时采用传入Document的分配器，即所赋值的成员具有与Document相同的生命周期
   * @param doc 要求具有rapidjson::kObjectType类型
   * @param value 1. 与rapidjson所支持类型一致，API已经进行了相应重载，直接调用即可
   *              2. 特别地，即使value为复杂类型，如char* 或rapidjson::value，API内部已经进行了拷贝处理，同样具有声明周期的安全性
   * @return  是否执行成功.
   *          注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *                若无视错误返回，则可能引发未定义行为或程序崩溃，如判断HasMember(keyName)或直接读值doc[keyName]等.
   */
  static bool Add(json::Document & doc, const std::string & keyName, const int value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetInt(value);
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      json::Value key(keyName.c_str(), allocator);
      json::Value val(value);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(json::Document & doc, const std::string & keyName, const uint64_t value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetUint64(value);
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      json::Value key(keyName.c_str(), allocator);
      json::Value val(value);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(json::Document & doc, const std::string & keyName, const double value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetDouble(value);
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      json::Value key(keyName.c_str(), allocator);
      json::Value val(value);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(json::Document & doc, const std::string & keyName, const float value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetFloat(value);
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      json::Value key(keyName.c_str(), allocator);
      json::Value val(value);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(
    json::Document & doc, const std::string & keyName,
    const std::string value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    json::Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetString(value.c_str(), value.length(), allocator);
    } else {
      json::Value key(keyName.c_str(), allocator);
      json::Value val;
      val.SetString(value.c_str(), value.length(), allocator);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(json::Document & doc, const std::string & keyName, const char * value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    json::Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetString(value, strlen(value), allocator);
    } else {
      json::Value key(keyName.c_str(), allocator);
      json::Value val;
      val.SetString(value, strlen(value), allocator);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(json::Document & doc, const std::string & keyName, const bool value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetBool(value);
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      json::Value key(keyName.c_str(), allocator);
      json::Value val(value);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  static bool Add(
    json::Document & doc, const std::string & keyName,
    json::Value & value)
  {
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Input doc should be json::kObjectType.",
        CYBERDOGJSON, __func__);
      return false;
    }
    json::Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].CopyFrom(value, allocator);
    } else {
      json::Value val;
      val.CopyFrom(value, allocator);
      json::Value key(keyName.c_str(), allocator);
      doc.AddMember(key, val, allocator);
    }
    return true;
  }

  /**
   * @brief 与上述Add API功能一致
   *
   * @param doc 要求具有rapidjson::kArrayType类型
   */
  static bool Add(json::Document & doc, json::Document & value)
  {
    bool ret = true;
    if (!doc.IsArray()) {
      ERROR("[%s] %s: failed! Doc shouled be json::kArrayType.", CYBERDOGJSON, __func__);
      ret = false;
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      static json::Document val(json::kObjectType);
      val.CopyFrom(value, allocator);
      doc.PushBack(val, allocator);
    }
    return ret;
  }

  static bool Add(json::Document & doc, json::Value & value)
  {
    bool ret = true;
    if (!doc.IsArray()) {
      ERROR("[%s] %s: failed! Doc shouled be json::kArrayType.", CYBERDOGJSON, __func__);
      ret = false;
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      static json::Value val(json::kObjectType);
      val.CopyFrom(value, allocator);
      doc.PushBack(val, allocator);
    }
    return ret;
  }

  static bool Add(
    json::Document & doc, const std::string & keyName,
    const std::vector<signed char> & vec)
  {
    bool ret = true;
    if (!doc.IsObject()) {
      ERROR(
        "[%s] %s: failed! Doc shouled be json::kArrayType.", CYBERDOGJSON,
        __func__);
      ret = false;
    } else {
      json::Document::AllocatorType & allocator = doc.GetAllocator();
      rapidjson::Value IntArray(rapidjson::kArrayType);
      json::Value key(keyName.c_str(), allocator);
      int size = vec.size();
      for (int i = 0; i < size; i++) {
        IntArray.PushBack(vec[i], allocator);
      }
      doc.AddMember(key, IntArray, allocator);
    }
    return ret;
  }

public:
  /* convert between string and json data */
  /**
   * @brief 反序列化
   *        将字符串转化成json数据结构
   *
   * @param str 要求具有json数据组织形式，形如： "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}"
   * @param doc 注意： 原有数据会被覆盖
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               若无视错误返回，则可能引发未定义行为或程序崩溃，如判断HasMember(keyName)或直接读值doc[keyName]等.
   */
  static bool String2Document(const std::string & str, json::Document & doc)
  {
    bool ret = true;
    if (str.empty()) {
      ERROR("[%s] %s: failed! Input string shouled not be empty.", CYBERDOGJSON, __func__);
      ret = false;
    } else {
      doc.Parse<0>(str.c_str());
      if (doc.HasParseError()) {
        ERROR(
          "[%s] %s: failed! Doc parse error with input:\n\t%s",
          CYBERDOGJSON, __func__, str.c_str());
        ret = false;
      }
    }
    return ret;
  }

  /**
   * @brief 序列化
   *        将一个Document转化成string，用于传输.
   *
   * @param doc 要求具有json::kObjectType或json::kArrayType.
   * @param str 可用于传输的字符串.
   *            对于接收方的反序列化工具没有要求，支持RFC7159(https://www.rfc-editor.org/info/rfc7159)及以上即可。
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的入参str是不可被反序列化的，否则其行为未定义.
   */
  static bool Document2String(const json::Document & doc, std::string & str)
  {
    if (!doc.IsObject() && !doc.IsArray()) {
      ERROR(
        "[%s] %s: failed! Input document shouled kObjectType or kArrayType.",
        CYBERDOGJSON, __func__);
      return false;
    }

    json::StringBuffer s;
    json::Writer<json::StringBuffer> writer(s);
    doc.Accept(writer);
    str = std::string(s.GetString());
    return true;
  }

  static void Value2String(const json::Value & val, std::string & valStr)
  {
    json::StringBuffer buffer;
    json::Writer<json::StringBuffer> writer(buffer);
    val.Accept(writer);
    valStr = buffer.GetString();
  }

  /**
   * @brief 将一个json::value 转换为 json::Document
   *        Document具有完备的内存资源，如分配器等.
   *        进而可以不依赖其它资源进行数据结构再处理，如Add等操作.
   *
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的doc是不可用状态，对其操作的行为未定义.
   */
  static bool Value2Document(const json::Value & val, json::Document & doc)
  {
    std::string valStr;
    json::StringBuffer buffer;
    json::Writer<json::StringBuffer> writer(buffer);
    val.Accept(writer);
    valStr = buffer.GetString();
    doc.Parse<0>(valStr.c_str());
    if (doc.HasParseError()) {
      INFO("[%s] %s: HasParseError!", CYBERDOGJSON, __func__);
      return false;
    } else {
      return true;
    }
  }

public:
  /* Trans data between json and file. */
  /**
   * @brief 从文件读取数据，并存储到json内存结构中
   *
   * @param doc 若不为空，原有数据会被覆盖
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的doc是不可用状态，对其操作的行为未定义.
   */
  static bool ReadJsonFromFile(const std::string jsonFileName, json::Document & doc)
  {
    std::string jsonStr;
    doc.SetObject();
    if (ReadFile(jsonFileName, jsonStr)) {
      doc.Parse<0>(jsonStr.c_str());
      if (doc.HasParseError()) {
        ERROR("[%s] %s: Failed! HasParseError!", CYBERDOGJSON, __func__);
        return false;
      } else {
        return true;
      }
    } else {
      ERROR(
        "[%s] %s: Failed! Cannot read file!\n%s", CYBERDOGJSON, __func__,
        jsonFileName.c_str());
      return false;
    }
  }

  /**
   * @brief 将json数据写入文件
   *        默认使用pretty格式，即保持相对美观的缩进.
   * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的文件是不可读状态，对其操作的行为未定义.
   */
  static bool WriteJsonToFile(const std::string jsonFileName, const json::Document & doc)
  {
    bool ret = true;
    int fd = open(jsonFileName.c_str(), O_WRONLY | O_CREAT | O_SYNC | O_TRUNC, 0660);
    if (fd >= 0) {
      json::StringBuffer buffer;
      json::PrettyWriter<json::StringBuffer> writer(buffer);
      writer.SetFormatOptions(json::kFormatSingleLineArray);
      doc.Accept(writer);
      std::string msg = buffer.GetString();
      int msg_size = msg.size();
      int len = 0;
      int n;
      while (1) {
        int wr_len = msg_size - len;
        n = write(fd, msg.c_str() + len, wr_len);
        if (n < 0) {
          INFO("[%s] %s: Write OK!", CYBERDOGJSON, __func__);
          break;
        } else if (n == wr_len) {
          ret = true;
          INFO(
            "[%s] %s: Msg_size: %d, write %d bytes!", CYBERDOGJSON, __func__, msg_size,
            n);
          break;
        }
        len += n;
      }
      close(fd);
    } else {
      ERROR("[%s] %s: Failed! Create file failed!", CYBERDOGJSON, __func__);
      ret = false;
    }
    return ret;
  }

private:
  static bool ReadFile(const std::string & jsonFileName, std::string & jsonStr)
  {
    bool ret = false;
    std::ifstream in_file;
    in_file.open(jsonFileName.c_str(), std::ios::in);
    if (in_file.is_open()) {
      std::ostringstream sin;
      sin << in_file.rdbuf();
      jsonStr = sin.str();
      in_file.close();
      ret = true;
    }
    return ret;
  }

public:
  /* parse json data */
  /**
   * @brief 从json::Document中读取一个值
   *
   * @param doc 要求具有json::kObjectType类型
   * @param value 1. 与rapidjson所支持类型一致，API已经进行了相应重载，直接调用即可
   *              2. 特别地，如果value为复杂类型，如char* 或rapidjson::value，其生命周期与入参Document相同.
  * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的引用形参返回值是不可读状态，对其操作的行为未定义.
   */
  static bool Get(
    const json::Document & doc, const char * key,
    std::string & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !doc[key].IsString()) {
      return false;
    }

    value = doc[key].GetString();
    return true;
  }

  static bool Get(const json::Document & doc, const char * key, std::vector<float> & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key)) {
      return false;
    }

    auto questions = doc[key].GetArray();
    uint questions_size = questions.Size();
    for (uint i = 0; i < questions_size; i++) {
      if (!questions[i].IsFloat()) {
        return false;
      }
      value.push_back(questions[i].GetFloat());
    }
    return true;
  }

  static bool Get(const json::Document & doc, const char * key, int32_t & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key)) {
      return false;
    }

    if (!doc[key].IsInt()) {
      return false;
    }
    value = doc[key].GetInt();
    return true;
  }

  static bool Get(const json::Document & doc, const char * key, uint32_t & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !doc[key].IsUint()) {
      return false;
    }
    value = doc[key].GetUint();
    return true;
  }

  static bool Get(const json::Document & doc, const char * key, int64_t & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !doc[key].IsInt64()) {
      return false;
    }
    value = doc[key].GetInt64();
    return true;
  }

  static bool Get(const json::Document & doc, const char * key, uint64_t & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !doc[key].IsUint64()) {
      return false;
    }
    value = doc[key].GetUint64();
    return true;
  }

  static bool Get(json::Document & doc, const char * key, json::Value & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key)) {
      return false;
    }

    value.CopyFrom(doc[key], doc.GetAllocator());
    return true;
  }

  static bool Get(const json::Document & doc, const char * key, float & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !(doc[key].IsFloat() || doc[key].IsInt())) {
      return false;
    }

    if (doc[key].IsFloat()) {
      value = doc[key].GetFloat();
    } else if (doc[key].IsInt()) {
      value = doc[key].GetInt();
    }

    return true;
  }

  static bool Get(const json::Document & doc, const char * key, double & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !(doc[key].IsFloat() || doc[key].IsDouble() || doc[key].IsInt())) {
      return false;
    }

    if (doc[key].IsDouble()) {
      value = doc[key].GetDouble();
    } else if (doc[key].IsFloat()) {
      value = doc[key].GetFloat();
    } else if (doc[key].IsInt()) {
      value = doc[key].GetInt();
    }

    return true;
  }

  static bool Get(const json::Document & doc, const char * key, bool & value)
  {
    if (!doc.IsObject()) {
      ERROR("[%s] %s: Failed! Input doc should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!doc.HasMember(key) || !doc[key].IsBool()) {
      return false;
    }

    value = doc[key].GetBool();
    return true;
  }

  /**
   * @brief 从json::Value中读取一个值
   *
   * @param doc 要求具有json::kObjectType类型
   * @param value 1. 与rapidjson所支持类型一致，API已经进行了相应重载，直接调用即可
   *              2. 特别地，如果value为复杂类型，如char* 或rapidjson::value，其生命周期与入参Value相同.
  * @return 执行是否成功
   *         注意： 如果返回失败，需要调用代码自行处理后续业务逻辑.
   *               此时的引用形参返回值是不可读状态，对其操作的行为未定义.
   */
  static bool Get(const json::Value & val, const char * key, std::string & value)
  {
    if (!val.IsObject()) {
      ERROR("[%s] %s: Failed! Input val should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!val.HasMember(key) || !val[key].IsString()) {
      return false;
    }

    value = val[key].GetString();
    return true;
  }

  static bool Get(const json::Value & val, const char * key, bool & value)
  {
    if (!val.IsObject()) {
      ERROR("[%s] %s Failed! Input val should be kObejectType!", CYBERDOGJSON, __func__);
      return false;
    }
    if (!val.HasMember(key) || !val[key].IsBool()) {
      return false;
    }

    value = val[key].GetBool();
    return true;
  }
};  // class CyberdogJson
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_JSON_HPP_
