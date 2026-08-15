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

#include "cyberdog_parameter/cyberdog_parameter.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <limits>
// #include <filesystem>

#include "Python.h"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace cyberdog
{
namespace parameter
{

ParameterParser::ParameterParser(const std::string & configuration_filename)
{
  // const std::string filename = configuration_directory + "/" + configuration_filename;
  // if (!std::filesystem::exists(filename)) {
  //   ERROR("[ParameterParser] : %s is not exist.", filename.c_str());
  //   return;
  // }

  std::string suffix = configuration_filename.substr(configuration_filename.find_last_of('.') + 1);
  if (suffix == "so") {
    std::string filename = configuration_filename.substr(0, configuration_filename.rfind("."));
    bool success = LoadDefaultParameters(filename);
    if (!success) {
      ERROR("[TomlParameterConfig] : Load dynamic library format configuration file : %s error.", filename.c_str());
      return;
    }
  }
  else {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("cyberdog_parameter");
    auto filename = local_share_dir + std::string("/config/") + configuration_filename;
    toml_config_filename_ = filename;
    
    // if (!std::filesystem::exists(filename)) {
    //   ERROR("[ParameterParser] : %s is not exist.", filename.c_str());
    //   return;
    // }

    bool success = common::CyberdogToml::ParseFile(filename, file_solver_handler_);
    if (!success) {
      ERROR("[TomlParameterConfig] : Load toml format configuration filename : %s error.", filename.c_str());
      return;
    }
  } 

  INFO("[TomlParameterConfig] : Load configuration parameters success.");
}

ParameterParser::ParameterParser(const std::string& directory, const std::string & configuration_filename)
{
  std::string suffix = configuration_filename.substr(configuration_filename.find_last_of('.') + 1);
  if (suffix == "so") {
    std::string filename = configuration_filename.substr(0, configuration_filename.rfind("."));
    bool success = LoadDefaultParameters(directory, filename);
    if (!success) {
      ERROR("[TomlParameterConfig] : Load dynamic library format configuration file : %s error.", filename.c_str());
      return;
    }
  }
  else {
    auto filename = directory + configuration_filename;
    toml_config_filename_ = filename;
    
    // if (!std::filesystem::exists(filename)) {
    //   ERROR("[ParameterParser] : %s is not exist.", filename.c_str());
    //   return;
    // }

    bool success = common::CyberdogToml::ParseFile(filename, file_solver_handler_);
    if (!success) {
      ERROR("[TomlParameterConfig] : Load toml format configuration filename : %s error.", filename.c_str());
      return;
    }
  } 
}

ParameterParser::~ParameterParser()
{
  Py_Finalize();
}

bool ParameterParser::LoadDefaultParameters(const std::string & toml_dynamic_library_name)
{
  return ReadParmtersFromSharedLibrary(toml_dynamic_library_name);
}

bool ParameterParser::LoadDefaultParameters(
  const std::string& directory, const std::string& filename)
{
  return ReadParmtersFromSharedLibrary(directory, filename);
}

bool ParameterParser::ParameterParser::HasKey(const std::string& key)
{
  (void)key;
  return true;
}

std::string ParameterParser::GetString(const std::string& key)
{
  auto keys = Tokenize(key, ".");
  auto data_parser = ParseToml(file_solver_handler_, keys, 0);

  std::string result;
  bool success = common::CyberdogToml::Get(std::get<0>(data_parser), std::get<1>(data_parser), result);
  if (!success) {
    ERROR("[ParameterParser] : GetString function key =  %s error.", key.c_str());
    return "";
  }
  return result;
}

double ParameterParser::GetDouble(const std::string& key)
{
  auto keys = Tokenize(key, ".");
  auto data_parser = ParseToml(file_solver_handler_, keys, 0);

  double result = std::numeric_limits<double>::max();
  bool success = common::CyberdogToml::Get(std::get<0>(data_parser), std::get<1>(data_parser), result);
  if (!success) {
    ERROR("[ParameterParser] : GetString function key = %s error.", key.c_str());
    return result;
  }
  return result;
}

int ParameterParser::GetInt(const std::string& key)
{
  auto keys = Tokenize(key, ".");
  auto data_parser = ParseToml(file_solver_handler_, keys, 0);

  int result = std::numeric_limits<int>::max();
  bool success = common::CyberdogToml::Get(std::get<0>(data_parser), std::get<1>(data_parser), result);
  if (!success) {
    ERROR("[ParameterParser] : GetInt() function key = %s error.", std::get<1>(data_parser).c_str());
    return result;
  }
  return result;
}

bool ParameterParser::GetBool(const std::string& key)
{
  auto keys = Tokenize(key, ".");
  auto data_parser = ParseToml(file_solver_handler_, keys, 0);

  bool result = false;
  bool success = common::CyberdogToml::Get(std::get<0>(data_parser), std::get<1>(data_parser), result);
  if (!success) {
    ERROR("[ParameterParser] : GetString function key =  %s error.", key.c_str());
    return result;
  }
  return result;
}

bool ParameterParser::SetString(const std::string& key, const std::string & value)
{
  bool success = common::CyberdogToml::Set(file_solver_handler_, key, value);
  if (!success) {
    ERROR("[ParameterParser] : SetString function key =  %s error.", key.c_str());
    return success;
  }
  return success;
}

bool ParameterParser::SetDouble(const std::string& key, const double & value)
{
  bool success = common::CyberdogToml::Set(file_solver_handler_, key, value);
  if (!success) {
    ERROR("[ParameterParser] : SetDouble function key =  %s error.", key.c_str());
    return success;
  }
  return success;
}

bool ParameterParser::SetInt(const std::string& key, const int & value)
{
  bool success = common::CyberdogToml::Set(file_solver_handler_, key, value);
  if (!success) {
    ERROR("[ParameterParser] : SetInt function key =  %s error.", key.c_str());
    return success;
  }
  return success;
}

bool ParameterParser::SetBool(const std::string& key, const bool & value)
{
  bool success = common::CyberdogToml::Set(file_solver_handler_, key, value);
  if (!success) {
    ERROR("[ParameterParser] : SetBool function key =  %s error.", key.c_str());
    return success;
  }
  return success;
}

std::vector<double> ParameterParser::GetArrayValuesAsDoubles(const std::string& key)
{
  const auto values = toml::find(file_solver_handler_, key);
  std::vector<double> result;
  for (int i = 0; i < static_cast<int>(values.size()); i++) {
    result.push_back(toml::find<double>(values, i));
  }
  return result;
}

std::vector<int> ParameterParser::GetArrayValuesAsIntegers(const std::string& key)
{
  const auto values = toml::find(file_solver_handler_, key);
  std::vector<int> result;
  for (int i = 0; i < static_cast<int>(values.size()); i++) {
    result.push_back(toml::find<int>(values, i));
  }
  return result;
}

std::vector<std::string> ParameterParser::GetArrayValuesAsStrings(const std::string& key)
{
  const auto values = toml::find(file_solver_handler_, key);
  std::vector<std::string> result;
  for (int i = 0; i < static_cast<int>(values.size()); i++) {
    result.push_back(toml::find<std::string>(values, i));
  }
  
  return result;
}

int ParameterParser::GetArrayTableOfInteger(const std::string& key, int index)
{
  auto keys = Tokenize(key, ".");
  auto tables = toml::find<std::vector<toml::table>>(file_solver_handler_, keys[0]);

  int result = std::numeric_limits<int>::max();
  bool success = common::CyberdogToml::Get(tables.at(index), keys[1], result);
  if (!success) {
    ERROR("[ParameterParser] : GetArrayTableOfInteger() function key = %s error.", keys.back().c_str());
    return result;
  }
  return result;
}

bool ParameterParser::GetArrayTableOfBool(const std::string& key, int index)
{
  auto keys = Tokenize(key, ".");
  auto tables = toml::find<std::vector<toml::table>>(file_solver_handler_, keys[0]);

  bool result = false;
  bool success = common::CyberdogToml::Get(tables.at(index), keys[1], result);
  if (!success) {
    ERROR("[ParameterParser] : GetArrayTableOfBool() function key = %s error.", keys.back().c_str());
    return result;
  }
  return result;
}

double ParameterParser::GetArrayTableOfDouble(const std::string& key, int index)
{
  auto keys = Tokenize(key, ".");
  auto tables = toml::find<std::vector<toml::table>>(file_solver_handler_, keys[0]);

  double result = std::numeric_limits<double>::max();
  bool success = common::CyberdogToml::Get(tables.at(index), keys[1], result);
  if (!success) {
    ERROR("[ParameterParser] : GetArrayTableOfDouble() function key = %s error.", keys.back().c_str());
    return result;
  }
  return result;
}

std::string ParameterParser::GetArrayTableOfString(const std::string& key, int index)
{
  auto keys = Tokenize(key, ".");
  auto tables = toml::find<std::vector<toml::table>>(file_solver_handler_, keys[0]);

  std::string result = std::numeric_limits<std::string>::max();
  bool success = common::CyberdogToml::Get(tables.at(index), keys[1], result);
  if (!success) {
    ERROR("[ParameterParser] : GetArrayTableOfString() function key = %s error.", keys.back().c_str());
    return result;
  }
  return result;
}

bool ParameterParser::ReadParmtersFromSharedLibrary(const std::string & shared_name)
{
  std::string import = "import " + shared_name + "\n";
  Py_Initialize();
  if (!Py_IsInitialized()) {
    return false;
  }

  std::string config_path =  ament_index_cpp::get_package_prefix("cyberdog_parameter") + 
    "/lib/cyberdog_parameter";

  std::string py_cmd = "sys.path.append('" + config_path + "')";
  PyRun_SimpleString("import sys; import toml;");
  PyRun_SimpleString(py_cmd.c_str());
  PyRun_SimpleString(import.c_str());

  PyObject* pName  = PyUnicode_FromString(shared_name.c_str());
  PyObject* pModule = PyImport_Import(pName);
  if( pModule == nullptr){
    std::cout <<"module not found" << std::endl;
    return false;
  }

  // get function
  std::string python_function = "get_" + shared_name + "_data";
  PyObject* pFunc = PyObject_GetAttrString(pModule, python_function.c_str());
  if( !pFunc || !PyCallable_Check(pFunc)) {
    std::cout << "Not found function : " << python_function << std::endl;
    return false;
  }

  // call function
  PyObject* function_ret = PyObject_CallObject(pFunc, NULL);
  if (!function_ret) {
      std::cout << "Call function error.!" << std::endl;
  }

  // istream parse toml
  char* result;
  PyArg_Parse(function_ret, "s", &result);//转换返回类型
  std::string toml_data = std::string(result);
  std::istringstream iss(toml_data);
  file_solver_handler_ = toml::parse(iss );

  // auto name = toml::find<std::string>(file_solver_handler_, "A", "f");
  // std::cout << "name = " << name << std::endl;

  Py_DECREF(pName);
  Py_DECREF(pModule);
  Py_DECREF(pFunc);
  Py_DECREF(function_ret);
  Py_Finalize();
  return true;
}

bool ParameterParser::ReadParmtersFromSharedLibrary(const std::string& directory, const std::string & filename)
{
  std::string import = "import " + filename + "\n";
  Py_Initialize();
  if (!Py_IsInitialized()) {
    return false;
  }

  std::string config_path =  directory;

  std::string py_cmd = "sys.path.append('" + config_path + "')";
  PyRun_SimpleString("import sys; import toml;");
  PyRun_SimpleString(py_cmd.c_str());
  PyRun_SimpleString(import.c_str());

  PyObject* pName  = PyUnicode_FromString(filename.c_str());
  PyObject* pModule = PyImport_Import(pName);
  if( pModule == nullptr){
    std::cout <<"module not found" << std::endl;
    return false;
  }

  // get function
  std::string python_function = "get_" + filename + "_data";
  PyObject* pFunc = PyObject_GetAttrString(pModule, python_function.c_str());
  if( !pFunc || !PyCallable_Check(pFunc)) {
    std::cout << "Not found function : " << python_function << std::endl;
    return false;
  }

  // call function
  PyObject* function_ret = PyObject_CallObject(pFunc, NULL);
  if (!function_ret) {
      std::cout << "Call function error.!" << std::endl;
  }

  // istream parse toml
  char* result;
  PyArg_Parse(function_ret, "s", &result);//转换返回类型
  std::string toml_data = std::string(result);
  std::istringstream iss(toml_data);
  file_solver_handler_ = toml::parse(iss );

  // auto name = toml::find<std::string>(file_solver_handler_, "A", "f");
  // std::cout << "name = " << name << std::endl;

  Py_DECREF(pName);
  Py_DECREF(pModule);
  Py_DECREF(pFunc);
  Py_DECREF(function_ret);
  Py_Finalize();
  return true;
}

std::vector<std::string> ParameterParser::Tokenize(
  const std::string & str,
  const std::string & delimiters)
{
  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

  // Find first "non-delimiter".
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  std::vector<std::string> tokens;
  while (std::string::npos != pos || std::string::npos != lastPos) {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);
    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }
  return tokens;
}


void ParameterParser::DeleteSpaces(std::string & str)
{
  std::string::iterator end_pos = std::remove(str.begin(), str.end(), ' ');
  str.erase(end_pos, str.end());
}

std::tuple<toml::value, std::string> ParameterParser::ParseToml(
  const toml::value toml_parser, 
  const std::vector<std::string> & keys, 
  int index)
{
  std::tuple<toml::value, std::string> data;  
  if (index < static_cast<int>(keys.size() - 1)) {
    std::get<0>(data) = toml::find(toml_parser, keys[index]);;
    std::get<1>(data) = keys[index];
    return ParseToml(std::get<0>(data), keys, index + 1); 
  }

  std::get<0>(data) = toml_parser;
  std::get<1>(data) = keys[index];
  return data;
}

}  //  namespace parameter
}  //  namespace cyberdog
