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
#ifndef STRUCTOR__DATA_HPP_
#define STRUCTOR__DATA_HPP_

#include <string>
#include "xpack/json.h"


struct lcm_data_1
{
  int8_t a;
  int8_t b;
  double c;
  float d[20];
  float e[10];
  XPACK(O(a, b, c, d, e))
};


struct lcm_data_2
{
  int8_t a;
  int8_t b;
  double c;
  float d[10];
  float e[20];
  XPACK(O(a, b, c, e))
};
struct lcm_request_data
{
  std::string name;
  int8_t id;
  XPACK(O(name, id))
};

struct lcm_response_data
{
  std::string name;
  bool result;
  XPACK(O(name, result))
};

#endif  // STRUCTOR__DATA_HPP_
