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

#ifndef CYBERDOG_PARAMETER__CYBERDOG_PARAMTER_HPP_
#define CYBERDOG_PARAMETER__CYBERDOG_PARAMTER_HPP_

#include <string>
#include <vector>

#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace parameter
{

class ParameterParser
{
public:
  using ParameterHandler = toml::value;

  // Toml format or dynamic library format configuration
  ParameterParser(const std::string & configuration_filename);
  ParameterParser(const std::string& directory, const std::string & configuration_filename);
  ~ParameterParser();

  ParameterParser(const ParameterParser &) = delete;
  ParameterParser& operator=(const ParameterParser &) = delete;

  // Load toml configuration from dynamic library
  bool LoadDefaultParameters(const std::string & toml_dynamic_library_name);
  bool LoadDefaultParameters(const std::string& directory, const std::string& filename);

  // Returns true if the key is in this config filename.
  bool HasKey(const std::string& key);

  // These methods CHECK() that the 'key' exists.
  std::string GetString(const std::string& key);
  double GetDouble(const std::string& key);
  int GetInt(const std::string& key);
  bool GetBool(const std::string& key);

  // Set key
  bool SetString(const std::string& key, const std::string & value);
  bool SetDouble(const std::string& key, const double & value);
  bool SetInt(const std::string& key, const int & value);
  bool SetBool(const std::string& key, const bool & value);

  // Returns the values of the keys '1', '2', '3' as the given types.
  virtual std::vector<double> GetArrayValuesAsDoubles(const std::string& key);
  virtual std::vector<int> GetArrayValuesAsIntegers(const std::string& key);
  virtual std::vector<std::string> GetArrayValuesAsStrings(const std::string& key);

  // Toml array table format
  // b.a 0 ==> b[0].a
  int GetArrayTableOfInteger(const std::string& key, int index);
  bool GetArrayTableOfBool(const std::string& key, int index);
  double GetArrayTableOfDouble(const std::string& key, int index);
  std::string GetArrayTableOfString(const std::string& key, int index);

  // In order to handle the saving of toml files
  ParameterHandler& GetParameterHandler() { return file_solver_handler_; }

  // Dynamic library handler
  ParameterHandler& GetDynamicLibraryParameterHandler() { return file_solver_handler_; }

private:
  // Reads dynamic library so default configuration file
  bool ReadParmtersFromSharedLibrary(const std::string & shared_name);

  // Reads dynamic library so default configuration file
  bool ReadParmtersFromSharedLibrary(const std::string& directory, const std::string & filename);

  // String split
  std::vector<std::string> Tokenize(const std::string & str, const std::string & delimiters);

  // Remove extra spaces from a string
  void DeleteSpaces(std::string & str);

  // Recursively parse toml files
  std::tuple<toml::value, std::string> ParseToml(
    const toml::value toml_parser, 
    const std::vector<std::string> & keys, int index);

  // toml file parse
  ParameterHandler file_solver_handler_;  
  std::string toml_config_filename_;

};  // class ParameterParser

}  //  namespace parameter
}  //  namespace cyberdog

#endif  //  CYBERDOG_PARAMETER__CYBERDOG_PARAMTER_HPP_
