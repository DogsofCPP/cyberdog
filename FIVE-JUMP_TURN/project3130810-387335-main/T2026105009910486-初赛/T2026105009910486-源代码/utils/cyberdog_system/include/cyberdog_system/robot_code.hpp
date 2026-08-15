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
#ifndef CYBERDOG_SYSTEM__ROBOT_CODE_HPP_
#define CYBERDOG_SYSTEM__ROBOT_CODE_HPP_
#include <string>
#include <memory>

namespace cyberdog
{
namespace system
{
/**
 * @brief Cyberdog global code for module.
 *        Regulations are tough on hundreds place.
 */
enum class ModuleCode : int32_t
{
  kUnknown = -1,

//  manager code
  kRobot = 100,      // cyberdog global manager
  kServer = 200,     //  200, 400, 500
  kBridgeGrpc = 300,
  kBridgeNet = 600,
  kAPP = 700,  // 700 - 899

//  device code
  kDeviceManager = 1000,
  kLED = 1100,
  kMiniLED = 1200,
  kWifi = 1300,
  kBms = 1400,
  kTouch = 1500,
  kUWB = 1600,

// sensor code
  kSensorManager = 2000,
  kLidar = 2100,
  kToF = 2200,
  kUltrasonic = 2300,
  kGPS = 2400,

// motion code
  kMotionManager = 3000,
  kMotionBoard = 3100,
  kMotionUtils = 3200,

// camera modules
  kFollow = 4100,
  kAI = 4200,
  kMapNav = 4300,
  kCameraServer = 4400,

//  core code
  kAudioNX = 5000,
  kAudioBoard = 5100,
  kConnector = 5200,
  kNavigation = 5300,
  kTransmission = 5400,
  kFence = 5500,
  kAFT = 5600,
  kOTA = 5700,
  kVisualProgram = 5800,
  kFace = 5900
};  // enum class ModuleCode

/**
 * @brief Cyberdog system reserved code,
 *        (int) 0 - 20;
 *        All modules should set self-code between 21 - 99.
 */
enum class KeyCode : int32_t
{
  kOK = 0,                      // 特别指令，结果正确 / 调用正常
  kFailed = 1,                  // 无特别定义的失败，统一使用
  kUninitialized = 2,           // 未初始化
  kStateInvalid = 3,            // 状态机不允许
  kStatusError = 4,             // 模块状态错误
  kNetError = 5,                // 网络错误
  kPermissionDenied = 6,        // 无操作权限
  kTimeout = 7,                 // 超时
  kUnSupport = 8,               // 指令不支持
  kSelfCheckFailed = 9,         // 自检失败
  kParametersInvalid = 10,      // 参数不合法
  kTargetBusy = 11,             // 状态忙碌，用于独占性调用目标
  kDeviceError = 12,            // 硬件错误， 外设、传感器类使用
  kProtectedError = 13          // 低电量保护模式
};  // enum class KeyCode

/**
 * @brief 全局结果码封装类
 *        使用方法可参考test/robot_code_test.cpp
 *
 * @tparam Code_T 模块自定义的编码class
 */
template<typename Code_T>
class CyberdogCode final
{
public:
  explicit CyberdogCode(ModuleCode module_code)
  {
    ModuleImpl_ = module_code;
  }

  ~CyberdogCode() {}

  /**
   * @brief Get the Key Code object
   *
   * @param code 预留关键码
   * @return int32_t 转为int值的代码
   */
  int32_t GetKeyCode(KeyCode code)
  {
    return code ==
           KeyCode::kOK ? static_cast<int32_t>(KeyCode::kOK) : static_cast<int32_t>(ModuleImpl_) +
           static_cast<int32_t>(code);
  }

  /**
   * @brief 获取模块自定义结果码
   *
   * @param code 自定义结果码
   * @return int32_t 转为int值的代码
   */
  int32_t GetCode(Code_T code)
  {
    return static_cast<int32_t>(ModuleImpl_) + static_cast<int32_t>(code);
  }

private:
  ModuleCode ModuleImpl_{ModuleCode::kUnknown};
};  // calss CyberdogCode
}  // namespace system
}  // namespace cyberdog

#endif  // CYBERDOG_SYSTEM__ROBOT_CODE_HPP_
