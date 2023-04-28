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
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/personnel.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Personnel::SetData(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    if (!this->face_.Init(
        this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml))
    {
      Error("%s Init face is failed.", this->logger_.c_str());
      return false;
    }
    if (!this->voiceprint_.Init(
        this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml))
    {
      Error("%s Init voiceprint is failed.", this->logger_.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Personnel::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->personnel_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvPersonnel>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "personnel", "all_user_search"),
      ClientQos.get_rmw_qos_profile(),
      this->cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

SrvPersonnel::Response Personnel::RequestPersonnelSrv(
  int _service_start_timeout)
{
  SrvPersonnel::Response ret;
  ret.code = -1;
  try {
    if (rclcpp::ok()) {
      if (this->personnel_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout)))
      {
        Debug(
          "[%s] Requesting personnel service.",
          this->logger_.c_str());
        std::shared_ptr<SrvPersonnel::Request> request_ptr =
          std::make_shared<SrvPersonnel::Request>();
        request_ptr->command = "";
        auto result = this->personnel_cli_ptr_->async_send_request(request_ptr);
        // result.wait();
        std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
        if (status == std::future_status::ready) {
          auto result_ptr = result.get();
          ret = *result_ptr;
          INFO(
            "[%s] Requesting personnel service response:\n\tcode: %d\n\tresult: \n%s",
            this->logger_.c_str(),
            result_ptr->code,
            msgPersonnelVector(result_ptr->result, "\t").c_str());
        } else {
          this->transient_state_.code = StateCode::service_request_timeout;
          Warn(
            "[%s] Waiting for personnel service to response timeout.",
            this->logger_.c_str());
        }
      } else {
        this->transient_state_.code = StateCode::service_appear_timeout;
        Warn(
          "[%s] Waiting for personnel service to appear(start) timeout.",
          this->logger_.c_str());
      }
    } else {
      this->transient_state_.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for personnel service to appear.",
        this->logger_.c_str());
    }
  } catch (...) {
    this->transient_state_.code = StateCode::fail;
    Error("[%s] RequestPersonnelSrv() is failed.", this->logger_.c_str());
  }
  return ret;
}

SrvPersonnel::Response Personnel::GetData(const int _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)",
    _timeout);
  Info("%s", funs.c_str());
  return this->RequestPersonnelSrv(static_cast<int>((_timeout > 0) ? _timeout : 5));
}

StateCode Personnel::IdToUsername(
  const std::vector<std::string> & _target,
  std::vector<std::string> & target)
{
  try {
    Info(
      "%s() : _target = %s.",
      std::string(__FUNCTION__).c_str(),
      stringVector(_target).c_str());
    SrvPersonnel::Response personnels = this->GetData();
    if (personnels.code != 0) {
      Error("Get data is failed.");
      return StateCode::fail;
    }
    bool is_find;
    for (const auto & id : _target) {
      is_find = false;
      for (auto personnel : personnels.result) {
        if (std::to_string(personnel.id) == id) {
          is_find = true;
          target.push_back(personnel.username);
        }
      }
      if (!is_find) {
        Warn("People with no target id(%s).", id.c_str());
        return StateCode::parameter_is_invalid;
      }
    }
  } catch (const std::exception & e) {
    Error("[%s] IdToUsername() is failed.", this->logger_.c_str());
    return StateCode::fail;
  }
  Info(
    "%s() : target = %s.",
    std::string(__FUNCTION__).c_str(),
    stringVector(target).c_str());
  return StateCode::success;
}

FaceRecognizedSeviceResponse Personnel::FaceRecognized(
  const std::vector<std::string> & _target,
  const bool _and_operation,
  const int _duration)
{
  FaceRecognizedSeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %s, %d)",
    stringVector(_target).c_str(),
    std::string(_and_operation ? "True" : "False").c_str(),
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->transient_state_.code = StateCode::success;
  std::vector<std::string> target;
  StateCode username_state = this->IdToUsername(_target, target);
  if (username_state != StateCode::success) {
    ret.state = this->GetState(funs, username_state);
    Error("%s %s", funs.c_str(), ret.state.describe.c_str());
    return ret;
  }
  return this->face_.Recognized(target, _and_operation, _duration);
}

VoiceprintRecognizedResponse Personnel::VoiceprintRecognized(
  const std::vector<std::string> & _target,
  bool _and_operation,
  const int _duration,
  const int _sensitivity)
{
  VoiceprintRecognizedResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %s, %d, %d)",
    stringVector(_target).c_str(),
    std::string(_and_operation ? "True" : "False").c_str(),
    _duration,
    _sensitivity);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->transient_state_.code = StateCode::success;
  std::vector<std::string> target;
  StateCode username_state = this->IdToUsername(_target, target);
  if (username_state != StateCode::success) {
    ret.state = this->GetState(funs, username_state);
    Error("%s %s", funs.c_str(), ret.state.describe.c_str());
    return ret;
  }
  ret = this->voiceprint_.Recognized(_duration, _sensitivity);
  if (ret.data) {
    for (const auto & voiceprint_name : target) {
      bool is_recognized =
        (std::find(ret.list.begin(), ret.list.end(), voiceprint_name) != ret.list.end());
      if (_and_operation) {
        if (!is_recognized) {
          ret.data = false;
          break;
        }
      } else {
        if (is_recognized) {
          break;
        }
      }
    }
  }
  return ret;
}
}   // namespace cyberdog_visual_programming_abilityset
