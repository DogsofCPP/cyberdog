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
/**
 * @brief Finite state machine, which is specially designed for cyberdog rop.
 *
 */
#ifndef CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
#define CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <utility>
#include "rclcpp/node.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "protocol/srv/fs_machine.hpp"

#define kConfigFile  "./fs_machine_config.toml"

namespace cyberdog
{
namespace machine
{

/**
 * @brief 状态机指令，用来区分查询与设置
 *
 */
enum class StateCmd : uint8_t
{
  kQuery = 0,  // 查询
  kSet = 1     // 设置
};  // enum class StateCmd

enum class MachineState : uint8_t
{
  MS_Uninitialized = 0,
  MS_SetUp = 1,
  MS_TearDown = 2,
  MS_SelfCheck = 3,
  MS_Active = 4,
  MS_DeActive = 5,
  MS_Protected = 6,
  MS_LowPower = 7,
  MS_OTA = 8,
  MS_Error = 9,
  MS_Unkown = 10
};  // enum class MachineState

class MachineContext
{
public:
  std::string Context(MachineState ms)
  {
    if (state_map_.find(ms) == state_map_.end()) {
      ERROR("set state error! machine state value not exsit!");
      return Unkown_V;
    }
    return state_map_.at(ms);
  }

private:
  const std::string Uninitialized_V{"Uninitialized"};
  const std::string SetUp_V{"SetUp"};
  const std::string TearDown_V{"TearDown"};
  const std::string SelfCheck_V{"SelfCheck"};
  const std::string Active_V{"Active"};
  const std::string DeActive_V{"DeActive"};
  const std::string Protected_V{"Protected"};
  const std::string LowPower_V{"LowPower"};
  const std::string OTA_V{"OTA"};
  const std::string Error_V{"Error"};
  const std::string Unkown_V{"Error"};
  const std::map<MachineState, std::string> state_map_ = {
    {MachineState::MS_Uninitialized, Uninitialized_V},
    {MachineState::MS_SetUp, SetUp_V},
    {MachineState::MS_TearDown, TearDown_V},
    {MachineState::MS_SelfCheck, SelfCheck_V},
    {MachineState::MS_Active, Active_V},
    {MachineState::MS_DeActive, DeActive_V},
    {MachineState::MS_Protected, Protected_V},
    {MachineState::MS_LowPower, LowPower_V},
    {MachineState::MS_OTA, OTA_V},
    {MachineState::MS_Error, Error_V},
    {MachineState::MS_Unkown, Unkown_V},
  };
};

/**
 * @brief 控制器配置参数管理类
 *
 */
class ControllerParams
{
public:
  bool Build(const toml::value & controller)
  {
    toml::array actuators;
    if (!common::CyberdogToml::Get(controller, "actuators", actuators_vec_)) {
      ERROR("Build ControllerParams failed, cannot get actuators!");
      return false;
    }
    toml::array states;
    if (!common::CyberdogToml::Get(controller, "states", states_vec_)) {
      ERROR("Build ControllerParams failed, cannot get states!");
      return false;
    }
    if (!common::CyberdogToml::Get(controller, "default_time", default_time_)) {
      ERROR("Build ControllerParams failed, cannot get default time!");
      return false;
    }
    if (!common::CyberdogToml::Get(controller, "default_state", default_state_)) {
      ERROR("Build ControllerParams failed, cannot get default state!");
      return false;
    }
    return true;
  }

  bool CheckActuator(const std::string & actuator)
  {
    return std::find(
      actuators_vec_.begin(), actuators_vec_.end(),
      actuator) == actuators_vec_.end() ? false : true;
  }

  bool CheckState(const std::string & state)
  {
    return std::find(
      states_vec_.begin(), states_vec_.end(),
      state) == states_vec_.end() ? false : true;
  }

  int32_t GetDefaultTime()
  {
    return default_time_;
  }

  const std::string & GetDefaultState()
  {
    return default_state_;
  }

  const std::vector<std::string> & GetAllStates()
  {
    return states_vec_;
  }

private:
  std::vector<std::string> actuators_vec_;
  std::vector<std::string> states_vec_;
  int32_t default_time_;
  std::string default_state_;
};  // class ControllerParams

/**
 * @brief 执行器配置参数管理类
 *
 */
class ActuatorParams final
{
public:
  bool Build(const toml::array & states, const toml::array & times)
  {
    if (states.size() != times.size()) {
      ERROR("Build ActuatorParams failed, states size is not equal with times!");
      return false;
    }
    for (size_t i = 0; i < states.size(); i++) {
      states_map_.insert(std::make_pair(states[i].as_string(), times[i].as_integer()));
    }
    return true;
  }

  /**
   * @brief 获取状态机切换时间开销
   *
   * @param state 目标状态机
   * @return int32_t 单位毫秒
   */
  int32_t GetTime(const std::string & state) const
  {
    auto iter = states_map_.find(state);
    if (iter != states_map_.end()) {
      return iter->second;
    } else {
      ERROR("GetTime failed, state invalid!");
      return kError_state_time;
    }
  }

  /**
   * @brief 检测执行器是否注册了该状态
   *
   * @param state 目标状态
   * @return true 检测成功
   * @return false 检测失败，此时不应该切换该目标状态，否则执行器不会有对应动作
   */
  bool CheckState(const std::string & state) const
  {
    return states_map_.find(state) == states_map_.end() ? false : true;
  }

  size_t GetStatesNum() const
  {
    return states_map_.size();
  }

private:
  std::map<std::string, int32_t> states_map_;
  std::string current_state_;
  const int32_t kError_state_time = -1;
};  // class ActuatorParams

/**
 * @brief 状态机控制器类
 *
 */
class MachineController
{
  using FSMACHINE_SRV_T = protocol::srv::FsMachine;
  using FS_CLINET_T = rclcpp::Client<FSMACHINE_SRV_T>::SharedPtr;

public:
  MachineController()
  {
    controller_params_ptr_ = std::make_shared<ControllerParams>();
  }
  ~MachineController()
  {
    // future: clear all container
  }

  /* 状态机不允许通过转移、赋值、拷贝来构造 */
  MachineController(const MachineController &) = delete;
  MachineController & operator=(const MachineController &) = delete;
  MachineController(MachineController &&) = delete;
  MachineController & operator=(MachineController &&) = delete;

public:
  /**
   * @brief 初始化控制器
   *          1. 检测配置文件合法性，合法性规则详见状态机设计文档
   *          2. 构造控制器的一系列管理容器
   *          3. 初始化工作结束后，运行时不再依赖toml文件
   *
   * @param node_ptr ros指针，用于内置与执行器的切换通信
   * @return true 只有返回初始化成功，才可以使用状态机
   * @return false 若初始化失败，状态机不可用，否则在切换、查询中其行为未定义
   */
  bool MachineControllerInit(
    const std::string & config_file = kConfigFile,
    rclcpp::Node::SharedPtr node_ptr = nullptr)
  {
    INFO("Init FS Machine, config file: %s", config_file.c_str());

    if (node_ptr == nullptr) {
      ERROR("FS Machine init failed, ros node pointer is invalid!");
      return false;
    } else {
      node_ptr_ = node_ptr;
    }

    toml::value config;
    if (!common::CyberdogToml::ParseFile(config_file, config)) {
      ERROR("Parse FS Machine config file failed, toml file is invalid!");
      return false;
    }
    toml::value controller;
    if (!common::CyberdogToml::Get(config, "controller", controller)) {
      ERROR("FS Machine init failed, parse controller config failed!");
      return false;
    } else if (!BuildControllerParams(controller)) {
      ERROR("FS Machine init failed, build controller failed!");
      return false;
    }

    deserve_state_vec_ = controller_params_ptr_->GetAllStates();

    // toml::array actuator_array;
    if (!common::CyberdogToml::Get(controller, "actuators", target_vec_)) {
      ERROR("FS Machine init failed, parse actuator array failed!");
      return false;
    }

    BuildClientMap();
    BuildStateMap();

    toml::value actuator;
    if (!common::CyberdogToml::Get(config, "actuator", actuator)) {
      ERROR("FS Machine init failed, parse actuators config failed!");
      return false;
    } else if (!BuildActuatorMap(actuator)) {
      ERROR("FS Machine init failed, build actuators params failed!");
      return false;
    }

    /* controller中的actuator与单独声明的actuator数量要求一致，检测失败则返回错误 */
    if (actuator.size() != target_vec_.size()) {
      ERROR("FS Machine init failed, actuator size invalid!");
      return false;
    }

    return true;
  }

  /**
   * @brief 阻塞函数，等待所有执行器接入状态机
   *
   * @return true 全部接入成功
   * @return false 超时
   */
  bool WaitActuatorsSetUp()
  {
    INFO("MachineController wait for all actuators setup, size: %ld.", actuator_map_.size());
    auto result = std::all_of(
      actuator_map_.cbegin(), actuator_map_.cend(), [this](const auto & iter) {
        INFO("Waiting for service: %s", iter.first.c_str());
        auto client_iter = this->client_map_.find(iter.first);
        if (client_iter == client_map_.end()) {
          ERROR(
            "MachineController wait actuator: %s setup failed, maybe has an error while building.",
            iter.first.c_str());
          return false;
        }

        auto time_cost = iter.second.GetTime("SetUp");
        if (time_cost == kError_state_time) {
          ERROR(
            "MachineController wait actuator: %s setup failed, cannot get time!",
            iter.first.c_str());
          return false;
        }
        bool wait_result = false;
        std::chrono::time_point<std::chrono::system_clock> p = std::chrono::system_clock::now();
        int64_t ticks_cur =
        std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
        int64_t ticks_until = ticks_cur + time_cost;
        while (ticks_until >= ticks_cur) {
          if (client_iter->second->wait_for_service(std::chrono::milliseconds(200))) {
            wait_result = true;
            break;
          }
          std::chrono::time_point<std::chrono::system_clock> p = std::chrono::system_clock::now();
          ticks_cur = std::chrono::duration_cast<std::chrono::milliseconds>(
            p.time_since_epoch()).count();
        }
        if (!wait_result) {
          ERROR("MachineController wait actuator: %s setup failed, timeout!", iter.first.c_str());
        } else {
          INFO("MachineController wait actuator: %s setup OK.", iter.first.c_str());
        }
        return wait_result;
      });
    if (!result) {
      ERROR("MachineController wait all actuators setup failed, state machine cannot work!");
    } else {
      INFO("MachineController wait all actuators setup OK.");
    }
    return result;
  }

  /**
   * @brief 获取状态机管理字典，预留功能
   *
   * @param state_map
   */
  void GetStateMap(std::map<std::string, std::string> & state_map)
  {
    state_map = state_map_;
  }

  /**
   * @brief 查询特定执行器的状态
   *          1. 该状态由controller的map维护，不需要动态进程间查询
   *          2. 执行器名称合法性由控制器配置管理，状态机字典此时在初始化检测时已经保证了安全性
   *
   * @param target_actuator 查询目标
   * @param state 查询结果
   * @return true 参数是否合法
   * @return false 返回失败时，引用状态可能为空，其使用后程序行为未定义
   */
  bool GetState(const std::string & target_actuator, std::string & state)
  {
    if (!controller_params_ptr_->CheckActuator(target_actuator)) {
      ERROR("GetState failed, target: %s is invalid!", target_actuator.c_str());
      return false;
    }
    state = state_map_.find(target_actuator)->second;
    return true;
  }

  /**
   * @brief 设置特定Actuator的状态
   *
   * @param target_actuator 目标name
   * @param target_state 目标状态
   * @return true 略。
   * @return false 如果返回失败，则目标状态为前一个状态，不受影响
   */
  int32_t SetState(
    const std::string & target_actuator, const std::string & target_state,
    std::map<std::string, int32_t> & state_map)
  {
    if (!controller_params_ptr_->CheckActuator(target_actuator)) {
      ERROR("SetState failed, target actuator is invalid!");
      return -1;
    }
    if (!controller_params_ptr_->CheckState(target_state)) {
      ERROR("SetState failed, target state is invalid!");
      return -1;
    }
    auto iter = client_map_.find(target_actuator);
    if (iter == client_map_.end()) {
      ERROR(
        "MachineController set state failed, get target actuator: %s client failed!",
        target_actuator.c_str());
      return -1;
    }
    auto request = std::make_shared<FSMACHINE_SRV_T::Request>();
    request->target_state = target_state;
    auto response = std::make_shared<FSMACHINE_SRV_T::Response>();
    auto result = iter->second->async_send_request(request);
    auto time_cost = actuator_map_.find(target_actuator)->second.GetTime(target_state);
    std::future_status status = result.wait_for(std::chrono::milliseconds(time_cost));
    if (status == std::future_status::ready) {
      INFO(
        "MachineController wait service actuator:%s state:%s ok.",
        target_actuator.c_str(), target_state.c_str());
    } else {
      ERROR("MachineController set state failed, get service result failed!");
      return -1;
    }
    // if (rclcpp::spin_until_future_complete(
    //     node_ptr_, result,
    //     std::chrono::seconds(time_cost)) != rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   ERROR("MachineController set state failed, get service result failed!");
    //   return false;
    // }
    if (result.get()->code != static_cast<int32_t>(cyberdog::system::KeyCode::kOK)) {
      ERROR(
        "MachineController set state failed, result code is not OK, expect 0 but actual is: %d.",
        result.get()->code);
      state_map.insert(std::pair<std::string, int32_t>(target_actuator, result.get()->code));
      return result.get()->code;
    }
    state_map.insert(std::pair<std::string, int32_t>(target_actuator, 0));
    UpdateState(target_actuator, target_state);
    return 0;
  }

  /**
   * @brief 设置全局状态机
   *
   * @param target_state 目标状态
   * @return true
   * @return false
   */
  int32_t SetState(const std::string & target_state, std::map<std::string, int32_t> & stmap)
  {
    int32_t code = 0;
    INFO("MachineController set global state: %s", target_state.c_str());
    auto set_result = std::all_of(
      target_vec_.cbegin(), target_vec_.cend(), [this, target_state, &code, &stmap](
        const std::string & name) {
        INFO("all of name: %s, target: %s", name.c_str(), target_state.c_str());
        code = SetState(name, target_state, stmap);

        if (code != 0 && target_state != "SelfCheck") {
          ERROR(
            "MachineController set state faild, target: %s, state: %s", name.c_str(),
            target_state.c_str());
          return false;
        }
        return true;
      });
    INFO("MachineController set state result: %s", set_result == true ? "true" : "false");
    return code;
  }

  const std::vector<std::string> & GetNeedAchieveStates()
  {
    return deserve_state_vec_;
  }

private:
  /* Internal API */
  /**
   * @brief 构建状态记录字典
   *
   */
  void BuildStateMap()
  {
    INFO("BuildStateMap on call");
    for (size_t i = 0; i < target_vec_.size(); i++) {
      state_map_.insert(std::make_pair(target_vec_[i], controller_params_ptr_->GetDefaultState()));
    }
  }

  /**
   * @brief 创建ros客户端字典
   *
   */
  void BuildClientMap()
  {
    INFO("BuildClientMap on call");
    for (size_t i = 0; i < target_vec_.size(); i++) {
      std::string client_name = target_vec_[i] + std::string(kMachineServiceName);
      auto client = node_ptr_->create_client<protocol::srv::FsMachine>(client_name);
      client_map_.insert(std::make_pair(target_vec_[i], std::move(client)));
    }
  }

  /**
   * @brief 构造执行器管理字典，用于后续通过key快速查询并操作
   *
   * @param actuator_params 配置文件中的执行器参数，数据格式为toml:table
   * @return true 构造成功
   * @return false 失败时，后续状态机操作不可用
   */
  bool BuildActuatorMap(const toml::value & actuator_params)
  {
    INFO("BuildActuatorMap on call.");
    return std::all_of(
      target_vec_.cbegin(), target_vec_.cend(), [this, &actuator_params](const std::string & name) {
        toml::value params;
        if (!common::CyberdogToml::Get(actuator_params, name, params)) {
          ERROR("BuildActuatorMap failed, cannot parse params!");
          return false;
        }
        if (!BuildActuatorParams(name, params)) {
          ERROR("BuildActuatorMap failed, cannot build params!");
          return false;
        }
        return true;
      });
  }

  /**
   * @brief 构造执行器字典的一条数据
   *
   * @param name 执行器名称
   * @param params 执行器参数，来自配置文件
   * @return true 构造成功
   * @return false 失败时，后续状态机操作不可用
   */
  bool BuildActuatorParams(const std::string name, const toml::value & params)
  {
    INFO("BuildActuatorParams on call.");
    toml::array states;
    if (!common::CyberdogToml::Get(params, "states", states)) {
      ERROR("BuildActuatorParams failed, cannot parse param states!");
      return false;
    }
    toml::array times;
    if (!common::CyberdogToml::Get(params, "times", times)) {
      ERROR("BuildActuatorParams failed, cannot parse param times!");
      return false;
    }
    ActuatorParams actuator_params;
    if (!actuator_params.Build(states, times)) {
      ERROR("BuildActuatorPrams failed!");
      return false;
    } else {
      actuator_map_.insert(std::make_pair(name, actuator_params));
    }
    return true;
  }

  /**
   * @brief 构造控制器参数管理类
   *
   * @param controller_params
   * @return true 构造成功
   * @return false 失败时，后续状态机操作不可用
   */
  bool BuildControllerParams(const toml::value & controller_params)
  {
    return controller_params_ptr_->Build(controller_params);
  }

  /**
   * @brief 更新状态机记录字典
   *
   * @param target_actuator
   * @param target_state
   */
  void UpdateState(const std::string & target_actuator, const std::string & target_state)
  {
    state_map_.find(target_actuator)->second = target_state;
  }

private:
  std::vector<std::string> target_vec_;               // 执行器容器
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};        // 节点指针，用于内置进程间Ros通信
  std::vector<std::string> deserve_state_vec_;        // 待实现的状态机数组
  std::map<std::string, FS_CLINET_T> client_map_;     // 执行器与Ros服务客户端反射
  std::map<std::string, std::string> state_map_;          // 状态机记录字典
  std::map<std::string, ActuatorParams> actuator_map_;
  std::shared_ptr<ControllerParams> controller_params_ptr_;
  static std::string config_file_;
  const int32_t kError_state_time = -1;
  const char * kMachineServiceName = "machine_service";
};  // class MachineController

/**
 * @brief 状态机执行器类
 *
 */
class MachineActuator
{
  using FSMACHINE_SRV_T = protocol::srv::FsMachine;

public:
  explicit MachineActuator(const std::string & name)
  : name_(name) {}
  ~MachineActuator() {}

  /* 状态机不允许通过转移、赋值、拷贝来构造 */
  MachineActuator(const MachineActuator &) = delete;
  MachineActuator & operator=(const MachineActuator &) = delete;
  MachineActuator(MachineActuator &&) = delete;
  MachineActuator & operator=(MachineActuator &&) = delete;

public:
  /**
   * @brief 初始化状态机
   *        若该操作失败，则后续状态机行为是未定义的
   *
   * @param node_ptr
   * @return true
   * @return false
   */
  bool MachineActuatorInit(
    const std::string & config_file = kConfigFile,
    rclcpp::Node::SharedPtr node_ptr = nullptr)
  {
    INFO("MachineActuator Init, config file: %s", config_file.c_str());
    toml::value config;
    if (!common::CyberdogToml::ParseFile(config_file, config)) {
      ERROR("MachineActuator Init failed, toml file is invalid!");
      return false;
    }
    toml::value actuator;
    if (!common::CyberdogToml::Get(config, "actuator", actuator)) {
      ERROR("MachineActuator Init failed, parse actuators config failed!");
      return false;
    }
    toml::value params;
    if (!common::CyberdogToml::Get(actuator, name_, params)) {
      ERROR("MachineActuator Init failed, parse %s's params config failed!", name_.c_str());
      return false;
    }
    toml::array states;
    if (!common::CyberdogToml::Get(params, "states", states)) {
      ERROR("MachineActuator Init failed, parse params states failed!");
      return false;
    }
    toml::array times;
    if (!common::CyberdogToml::Get(params, "times", times)) {
      ERROR("MachineActuator Init failed, parse params times failed!");
      return false;
    }
    params_ptr_ = std::make_shared<ActuatorParams>();
    if (!params_ptr_->Build(states, times)) {
      ERROR("MachineActuator Init failed, build actuator params failed!");
      return false;
    }

    if (node_ptr == nullptr) {
      ERROR("MachineActuator Init failed, node pointer is invalid!");
      return false;
    } else {
      node_ptr_ = node_ptr;
    }
    return true;
  }
  /**
   * @brief 注册回调函数,需要将配置中支持的状态机全部注册
   *
   * @param state 目标状态
   * @param callback 回调函数
   */
  void RegisterStateCallback(const std::string & state, std::function<int32_t(void)> callback)
  {
    if (!params_ptr_->CheckState(state)) {
      ERROR("MachineActuator regitster callback failed, state: %s is invalid!", state.c_str());
    } else {
      state_callback_map_.insert(std::make_pair(state, callback));
    }
  }

  /**
   * @brief 启动状态机执行器
   *
   * @return true 成功则状态机正常运行
   * @return false 返回失败，状态机不可用
   */
  bool ActuatorStart()
  {
    INFO("MachineActuator start on call.");
    if (state_callback_map_.size() != params_ptr_->GetStatesNum()) {
      ERROR("Actuator start failed, register callback error!");
      return false;
    } else if (state_callback_map_.find("SetUp") == state_callback_map_.end()) {
      ERROR("Actuator start failed, cannot setup!");
      return false;
    } else {
      /* SetUp为启动时内置状态，service初始化前即转为该状态 */
      CheckoutState("SetUp");
      ServiceSetUp();
    }
    return true;
  }

private:
  void ServiceSetUp()
  {
    INFO("MachineActuator service setup on call.");
    std::string service_name = name_ + std::string(kMachineServiceName);
    machine_callback_group_ =
      node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    machine_service_ptr_ =
      node_ptr_->create_service<FSMACHINE_SRV_T>(
      service_name,
      std::bind(
        &MachineActuator::ServiceCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, machine_callback_group_);
  }

  void ServiceCallback(
    const FSMACHINE_SRV_T::Request::SharedPtr request,
    FSMACHINE_SRV_T::Response::SharedPtr response)
  {
    INFO(
      "MachineActuator service callback on call.\n\t ===> target_state: %s",
      request->target_state.c_str());
    response->code = CheckoutState(request->target_state);
  }

  /**
   * @brief 检出状态机回调
   *
   * @param state 目标状态
   * @return int32_t 返回值，遵循系统返回码设计
   */
  int32_t CheckoutState(const std::string & state)
  {
    INFO("MachineActuator checkout state: %s.", state.c_str());
    if (!params_ptr_->CheckState(state)) {
      INFO("MachineActuator has no state: %s, skip once.", state.c_str());
      return static_cast<int32_t>(system::KeyCode::kUnSupport);
    }
    auto iter = state_callback_map_.find(state);
    if (iter == state_callback_map_.end()) {
      ERROR("MachineActuator checkout state: %s failed, has no such callback.", state.c_str());
      return static_cast<int32_t>(system::KeyCode::kUnSupport);
    } else {
      // iter->second();
      // return static_cast<int32_t>(system::KeyCode::kOK);
      return iter->second();
    }
  }

private:
  std::string name_;
  std::map<std::string, std::function<int32_t(void)>> state_callback_map_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr machine_callback_group_;
  rclcpp::Service<FSMACHINE_SRV_T>::SharedPtr machine_service_ptr_{nullptr};
  std::shared_ptr<ActuatorParams> params_ptr_{nullptr};
  const char * kMachineServiceName = "machine_service";
};  // class MachineActuator
}  // namespace machine
}  // namespace cyberdog
#endif  // CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
