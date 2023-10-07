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
#include <string>
#include <memory>
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Base::Init(
  const std::string _task_id,
  const rclcpp::Node::SharedPtr _node_immortal_ptr_,
  const rclcpp::Node::SharedPtr _node_mortal_ptr_,
  const std::shared_ptr<State> _transient_state_ptr_,
  const toml::value & _params_toml)
{
  try {
    this->log_ptr_ = Log::GetInstance(_node_mortal_ptr_, _task_id, _params_toml);
    if (!this->log_ptr_->Ok()) {
      this->SetState(StateCode::fail);
      return false;
    }
    this->log_ptr_->On();
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->task_id_ = _task_id;
    this->node_immortal_ptr_ = _node_immortal_ptr_;
    this->node_mortal_ptr_ = _node_mortal_ptr_;
    this->transient_state_ptr_ = _transient_state_ptr_;
    if (this->heartbeat_) {
      this->heartbeat_cb_group_ =
        this->node_immortal_ptr_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      int heartbeat_hz = 1;
      this->heartbeat_ptr_ = this->node_immortal_ptr_->create_wall_timer(
        std::chrono::seconds(static_cast<int64_t>(1 / heartbeat_hz)),
        std::bind(&Base::Heartbeat, this),
        this->heartbeat_cb_group_);
    }

    this->timeout_wait_for_service_ = toml::find<uint16_t>(
      _params_toml, "vp", "init", "params", "timeout_wait_for_service");
    this->timeout_wait_for_ = toml::find<uint16_t>(
      _params_toml, "vp", "init", "params", "timeout_wait_for");

    this->SetState(
      StateCode(
        (this->SetData(_params_toml) && this->SetMechanism(_params_toml)) ?
        StateCode::success : StateCode::fail));
  } catch (const std::exception & e) {
    Error("%s Init failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return static_cast<bool>(this->state_.code == StateCode::success);
}

void Base::SetLog(const bool _log)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s)", std::string(_log ? "true" : "false").c_str());
  INFO("%s %s ...", this->logger_.c_str(), funs.c_str());
  this->log_ = _log;
  if (_log) {
    this->log_ptr_->On();
  }
}

void Base::Heartbeat()
{
  if (Timeout(this->timens_)) {
    this->SetState(StateCode::no_data_update);
  } else {
    this->SetState(StateCode::success);
  }
}

void Base::SetState(const StateCode _code)
{
  if (_code == this->state_.code) {
    return;
  }
  this->state_.code = _code;
  this->state_.describe = this->logger_ + " >> " + StateDescribe_[_code];
  if (_code != StateCode::success) {
    Warn(
      "The module status(%d) is abnormal.\n%s",
      static_cast<int>(this->state_.code),
      this->state_.describe.c_str());
  }
}

State Base::GetState(const std::string _funs, const StateCode _code)
{
  State ret;
  ret.code = _code;
  ret.describe = this->logger_ + "." + _funs + " >> " + StateDescribe_[_code];
  if (_code != StateCode::success) {
    Warn(
      "The module status(%d) is abnormal, the request failed.\n%s",
      static_cast<int>(_code),
      ret.describe.c_str());
  }
  return ret;
}

std::string Base::GetDescribe(const std::string _funs, const StateCode _code)
{
  return std::string(this->logger_ + "." + _funs + " >> " + StateDescribe_[_code]);
}

bool Base::GetToml(const std::string & _toml_file, toml::value & _toml)
{
  if (!cyberdog::common::CyberdogToml::ParseFile(
      _toml_file.c_str(), _toml))
  {
    Error(
      "%s Config file is not in toml format, config file dir:\n%s",
      this->logger_.c_str(), _toml_file.c_str());
    return false;
  }
  return true;
}
}   // namespace cyberdog_visual_programming_abilityset
