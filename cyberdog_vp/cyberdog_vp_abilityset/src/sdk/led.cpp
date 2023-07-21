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

#include "cyberdog_vp_abilityset/led.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Led::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Led::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->cli_ptr_ = this->node_mortal_ptr_->create_client<SrvLedExecute>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "led", "led_execute"),
      ClientQos.get_rmw_qos_profile(),
      this->cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

std::shared_ptr<SrvLedExecute::Request> Led::GetRequest()
{
  std::shared_ptr<SrvLedExecute::Request> request_ptr = std::make_shared<SrvLedExecute::Request>();
  request_ptr->occupation = true;
  request_ptr->client = SrvLedExecute::Request::VP;
  request_ptr->target = SrvLedExecute::Request::HEAD_LED;
  request_ptr->mode = SrvLedExecute::Request::SYSTEM_PREDEFINED;
  request_ptr->effect = SrvLedExecute::Request::RED_ON;
  request_ptr->r_value = 0;
  request_ptr->g_value = 0;
  request_ptr->b_value = 0;
  return request_ptr;
}

bool Led::RequestSrv(
  LedSeviceResponse & _response,
  std::string _interface_name,
  std::shared_ptr<SrvLedExecute::Request> _request_ptr,
  int _service_timeout)
{
  try {
    if (!rclcpp::ok()) {
      Warn(
        "[%s] Client interrupted while requesting for service to appear.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_request_interrupted;
      return false;
    }
    if (!this->cli_ptr_->wait_for_service(std::chrono::seconds(_service_timeout))) {
      Warn(
        "[%s] Waiting for service to appear(start) timeout.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_appear_timeout;
      return false;
    }
    Info(
      "The %s interface is requesting led module info:\n\t+ request:"
      "\n\t  - client = %s"
      "\n\t  - occupation = %s"
      "\n\t  - target = %d"
      "\n\t  - mode = %d"
      "\n\t  - effect = %d"
      "\n\t  - r_value = %d"
      "\n\t  - g_value = %d"
      "\n\t  - b_value = %d",
      _interface_name.c_str(),
      _request_ptr->client.c_str(),
      std::string(_request_ptr->occupation ? "true" : "false").c_str(),
      _request_ptr->target,
      _request_ptr->mode,
      _request_ptr->effect,
      _request_ptr->r_value,
      _request_ptr->g_value,
      _request_ptr->b_value);
    auto result = this->cli_ptr_->async_send_request(_request_ptr);
    // result.wait();
    std::future_status status = result.wait_for(std::chrono::seconds(_service_timeout));
    if (status != std::future_status::ready) {
      _response.state.code = StateCode::service_request_timeout;
      Warn(
        "The %s interface waiting for led service response to response timeout.",
        _interface_name.c_str());
      return false;
    }
    auto result_ptr = result.get();
    _response.state.code = StateCode::success;
    _response.response = *result_ptr;
    if (result_ptr->code != SrvLedExecute::Response::SUCCEED) {
      Warn(
        "The %s interface is requesting led module failed, response code = %d",
        _interface_name.c_str(),
        static_cast<int>(result_ptr->code));
    } else {
      Info(
        "The %s interface is requesting led module succeeded, response code = %d",
        _interface_name.c_str(),
        static_cast<int>(result_ptr->code));
    }
    return true;
  } catch (...) {
    Warn("[%s] RequestSrv() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

LedSeviceResponse Led::Play(
  const int & _target,
  const int & _effect)
{
  transient_state_.code = StateCode::success;
  LedSeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _target, _effect);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  auto request_ptr = this->GetRequest();
  request_ptr->target = _target;
  request_ptr->mode = SrvLedExecute::Request::SYSTEM_PREDEFINED;
  request_ptr->effect = _effect;
  if (!this->RequestSrv(ret, std::string(__FUNCTION__), request_ptr)) {
    Warn(
      "[%s] Request led service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

LedSeviceResponse Led::PlayRgb(
  const int & _target,
  const int & _effect,
  const int & _r,
  const int & _g,
  const int & _b)
{
  transient_state_.code = StateCode::success;
  LedSeviceResponse ret;
  ret.response.code = -1;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _target, _effect);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  auto request_ptr = this->GetRequest();
  request_ptr->target = _target;
  request_ptr->mode = SrvLedExecute::Request::USER_DEFINED;
  request_ptr->effect = _effect;
  request_ptr->r_value = _r;
  request_ptr->g_value = _g;
  request_ptr->b_value = _b;
  if (!this->RequestSrv(ret, std::string(__FUNCTION__), request_ptr)) {
    Warn(
      "[%s] Request led service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

LedSeviceResponse Led::Freed(
  const int & _target)
{
  transient_state_.code = StateCode::success;
  LedSeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)",
    _target);
  Debug("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  auto request_ptr = this->GetRequest();
  request_ptr->occupation = false;
  request_ptr->target = _target;
  if (!this->RequestSrv(ret, std::string(__FUNCTION__), request_ptr)) {
    Warn(
      "[%s] Request led service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

}   // namespace cyberdog_visual_programming_abilityset
