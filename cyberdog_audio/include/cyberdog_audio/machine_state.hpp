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
#ifndef CYBERDOG_AUDIO__MACHINE_STATE_HPP_
#define CYBERDOG_AUDIO__MACHINE_STATE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <utility>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"

namespace cyberdog
{
namespace interaction
{
enum class StateCode : int32_t
{
  scUninitializedError = 2,
  scSelfCheckError = 9,
};
class MachineState : public cyberdog::machine::MachineActuator
{
  using MACHINE_STATE_FUNC = std::function<int32_t(void)>;

public:
  explicit MachineState(const std::string & name)
  : cyberdog::machine::MachineActuator(name),
    name_(name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name_ + "_machine_state");
    code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<StateCode>>(
      cyberdog::system::ModuleCode::kRobot);
    machine_context_ptr_ = std::make_unique<cyberdog::machine::MachineContext>();
    std::thread(
      [this]() {
        rclcpp::spin(node_ptr_);
      }).detach();
  }
  ~MachineState() {}
  void Init()
  {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
    if (!this->MachineActuatorInit(
        path,
        node_ptr_))
    {
      ERROR("Init failed, actuator init error.");
      return;
    }
    std::vector<MACHINE_STATE_FUNC> set_up_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_SetUp, std::move(set_up_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_SetUp),
      std::bind(&MachineState::OnSetUp, this));
    std::vector<MACHINE_STATE_FUNC> tear_down_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_TearDown, std::move(tear_down_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_TearDown),
      std::bind(&MachineState::ONTearDown, this));
    std::vector<MACHINE_STATE_FUNC> self_check_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_SelfCheck, std::move(self_check_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_SelfCheck),
      std::bind(&MachineState::OnSelfCheck, this));
    std::vector<MACHINE_STATE_FUNC> active_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_Active, std::move(active_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_Active),
      std::bind(&MachineState::OnActive, this));
    std::vector<MACHINE_STATE_FUNC> deactive_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_DeActive, std::move(deactive_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_DeActive),
      std::bind(&MachineState::OnDeActive, this));
    std::vector<MACHINE_STATE_FUNC> protected_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_Protected, std::move(protected_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_Protected),
      std::bind(&MachineState::OnProtected, this));
    std::vector<MACHINE_STATE_FUNC> low_power_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_LowPower, std::move(low_power_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_LowPower),
      std::bind(&MachineState::OnLowPower, this));
    std::vector<MACHINE_STATE_FUNC> ota_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_OTA, std::move(ota_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_OTA),
      std::bind(&MachineState::OnOTA, this));
    std::vector<MACHINE_STATE_FUNC> error_vec;
    machine_func_map.insert(
      std::map<cyberdog::machine::MachineState,
      std::vector<MACHINE_STATE_FUNC>>::value_type(
        cyberdog::machine::MachineState::MS_Error, std::move(error_vec)));
    this->RegisterStateCallback(
      machine_context_ptr_->Context(cyberdog::machine::MachineState::MS_Error),
      std::bind(&MachineState::OnError, this));
  }

  bool Start()
  {
    return this->ActuatorStart();
  }

public:
  void RegisterCallback(cyberdog::machine::MachineState ms, MACHINE_STATE_FUNC func)
  {
    if (machine_func_map.find(ms) == machine_func_map.end()) {
      WARN("unkown machine state:%d", (int)ms);
      return;
    }
    machine_func_map[ms].push_back(func);
  }

private:
  int32_t OnSetUp()
  {
    INFO("MachineState on setup.");
    return ExecuteState(cyberdog::machine::MachineState::MS_SetUp);
  }
  int32_t ONTearDown()
  {
    INFO("MachineState on teardown.");
    return ExecuteState(cyberdog::machine::MachineState::MS_TearDown);
  }
  int32_t OnSelfCheck()
  {
    INFO("MachineState on selfcheck.");
    return ExecuteState(cyberdog::machine::MachineState::MS_SelfCheck);
  }
  int32_t OnActive()
  {
    INFO("MachineState on active.");
    return ExecuteState(cyberdog::machine::MachineState::MS_Active);
  }
  int32_t OnDeActive()
  {
    INFO("MachineState on deactive.");
    return ExecuteState(cyberdog::machine::MachineState::MS_DeActive);
  }
  int32_t OnProtected()
  {
    INFO("MachineState on protected.");
    return ExecuteState(cyberdog::machine::MachineState::MS_Protected);
  }
  int32_t OnLowPower()
  {
    INFO("MachineState on lowpower.");
    return ExecuteState(cyberdog::machine::MachineState::MS_LowPower);
  }
  int32_t OnOTA()
  {
    INFO("MachineState on OTA.");
    return ExecuteState(cyberdog::machine::MachineState::MS_OTA);
  }
  int32_t OnError()
  {
    INFO("MachineState on OTA.");
    return ExecuteState(cyberdog::machine::MachineState::MS_Error);
  }

  int32_t ExecuteState(cyberdog::machine::MachineState ms)
  {
    int32_t code = 0;
    for (auto iter = machine_func_map[ms].cbegin();
      iter != machine_func_map[ms].cend(); iter++)
    {
      code = (*iter)();
      if (code != 0) {
        return code;
      }
    }
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::shared_ptr<cyberdog::system::CyberdogCode<StateCode>> code_ptr_ {nullptr};
  std::unique_ptr<cyberdog::machine::MachineContext> machine_context_ptr_ {nullptr};
  std::map<cyberdog::machine::MachineState, std::vector<MACHINE_STATE_FUNC>> machine_func_map;
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__MACHINE_STATE_HPP_
