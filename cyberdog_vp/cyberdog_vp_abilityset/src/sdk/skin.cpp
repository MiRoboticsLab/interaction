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

#include <embed_protocol/embed_protocol.hpp>

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/skin.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::shared_ptr<cyberdog::embed::Protocol<CanData>> can0_ptr_ = {nullptr};
bool Skin::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    INFO("%s Use can port control skin.", this->logger_.c_str());
    std::string params_pkg_dir = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    std::string skin_config = params_pkg_dir + "/config/skin.toml";
    can0_ptr_ = std::make_shared<cyberdog::embed::Protocol<CanData>>(skin_config, false);
    can0_ptr_->SetDataCallback(
      std::bind(&Skin::Can0CB, this, std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Skin::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->skin_enable_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->skin_set_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->skin_enable_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvSetBool>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "skin_enable", "skin_enable"),
      ClientQos.get_rmw_qos_profile(),
      this->skin_enable_cli_cb_group_);
    this->skin_set_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvElecSkin>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "skin_set", "skin_set"),
      ClientQos.get_rmw_qos_profile(),
      this->skin_set_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Skin::Can0CB(std::string & _name, std::shared_ptr<CanData> _data_ptr)
{
  Info("%s(%s)", std::string(__FUNCTION__).c_str(), _name.c_str());
  this->response_.name = _name;
  this->response_.data.data0 = _data_ptr->data0;
  this->response_.data.data1 = _data_ptr->data1;
  this->response_.data.data2 = _data_ptr->data2;
  this->response_.data.data3 = _data_ptr->data3;
}

SkinElectrochromicResponse Skin::Electrochromic(
  const int _model,
  const int _position,
  const int _rendering,
  const int _outset,
  const int _duration_ms)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d, %d, %d, %d)",
    _model, _position, _rendering, _outset, _duration_ms);
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      this->response_.state = this->GetState(funs, this->state_.code);
      return this->response_;
    }
    if (_model > SkinConstraint::model_max) {
      int duration_ms = 0;
      if (_rendering == SkinConstraint::rendering_fade_out) {
        duration_ms = 0;
      } else {  // SkinConstraint::rendering_fade_in
        duration_ms = 1;
      }
      return this->Discolored(6, duration_ms);
    }
    this->response_.state = this->GetState(funs, StateCode::success);
    auto legalization = [&](std::string name, const int target, const int target_max) -> int {
        int ret = target;
        if (target < 0) {
          Warn(
            "The current %s parameter (%d) "
            "is less than 0, and 0 will be used as the final parameter.",
            name.c_str(), target);
          ret = 0;
        }

        if (target > target_max) {
          Warn(
            "The current %s parameter (%d) "
            "is greater than %d, and 10 will be used as the final parameter.",
            name.c_str(), target, target_max);
          ret = target_max;
        }
        return ret;
      };

    int model = legalization(
      "model", _model,
      static_cast<int>(SkinConstraint::model_max));
    int rendering = legalization(
      "rendering", _rendering,
      static_cast<int>(SkinConstraint::rendering_max));
    int outset = legalization(
      "outset", _outset,
      static_cast<int>(SkinConstraint::outset_max));
    int duration_ms = legalization(
      "duration_ms", _duration_ms,
      static_cast<int>(5000));

    uint8_t m = static_cast<uint8_t>(model);
    uint8_t d_high = *(reinterpret_cast<uint8_t *>(&duration_ms));
    uint8_t d_low = *(reinterpret_cast<uint8_t *>(&duration_ms) + 1);
    can0_ptr_->Operate(
      this->model_map_.at(model),
      std::vector<uint8_t>{m, d_low, d_high});
    if (model == static_cast<int>(SkinConstraint::model_control)) {
      uint8_t r = static_cast<uint8_t>(rendering);
      uint8_t o = static_cast<uint8_t>(outset);
      uint8_t d_high = *(reinterpret_cast<uint8_t *>(&duration_ms));
      uint8_t d_low = *(reinterpret_cast<uint8_t *>(&duration_ms) + 1);
      if (_position > SkinConstraint::position_max) {
        for (size_t i = 0; i <= SkinConstraint::position_max; i++) {
          can0_ptr_->Operate(
            this->position_map_.at(i),
            std::vector<uint8_t>{r, o, d_low, d_high});
          Info(
            "%s Operate(%s, %d, %d, %d, %d) ok.",
            funs.c_str(),
            this->position_map_.at(i).c_str(),
            r, o, d_low, d_high);
        }
      } else {
        can0_ptr_->Operate(
          this->position_map_.at(_position),
          std::vector<uint8_t>{r, o, d_low, d_high});
        Info(
          "%s Operate(%s, %d, %d, %d, %d) ok.",
          funs.c_str(),
          this->position_map_.at(_position).c_str(),
          r, o, d_low, d_high);
      }
    }
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] Skin Recognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    this->response_.state = this->GetState(funs, StateCode::fail);
  }
  return this->response_;
}

State Skin::RequestEnableSrv(
  SrvSetBool::Response & _response,
  std::shared_ptr<SrvSetBool::Request> _request_ptr,
  const int _service_start_timeout)
{
  State ret;
  try {
    if (!rclcpp::ok()) {
      ret.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for skin enable service to appear.",
        this->logger_.c_str());
      return ret;
    }
    if (!this->skin_enable_cli_ptr_->wait_for_service(
        std::chrono::seconds(
          _service_start_timeout)))
    {
      ret.code = StateCode::service_appear_timeout;
      Warn(
        "[%s] Waiting for skin enable service to appear(start) timeout.",
        this->logger_.c_str());
      return ret;
    }
    Debug("The interface is requesting skin enable service.");
    auto result = this->skin_enable_cli_ptr_->async_send_request(_request_ptr);
    std::future_status status = result.wait_for(
      std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      ret.code = StateCode::service_request_timeout;
      Warn(
        "[%s] Waiting for skin enable service to response timeout.",
        this->logger_.c_str());
      return ret;
    }
    auto result_ptr = result.get();
    ret.code = (result_ptr->success) ? StateCode::success : StateCode::fail;
    _response = *result_ptr;
    return ret;
  } catch (...) {
    ret.code = StateCode::fail;
    Warn(
      "[%s] RequestSkeletonRecognizedSrv() is failed.",
      this->logger_.c_str());
  }
  return ret;
}

State Skin::RequestSetSrv(
  SrvElecSkin::Response & _response,
  std::shared_ptr<SrvElecSkin::Request> _request_ptr,
  const int _service_start_timeout)
{
  State ret;
  try {
    if (!rclcpp::ok()) {
      ret.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for skin set service to appear.",
        this->logger_.c_str());
      return ret;
    }
    if (!this->skin_set_cli_ptr_->wait_for_service(
        std::chrono::seconds(
          _service_start_timeout)))
    {
      ret.code = StateCode::service_appear_timeout;
      Warn(
        "[%s] Waiting for skin set service to appear(start) timeout.",
        this->logger_.c_str());
      return ret;
    }
    Debug("The interface is requesting skin set service.");
    auto result = this->skin_set_cli_ptr_->async_send_request(_request_ptr);
    std::future_status status = result.wait_for(
      std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      ret.code = StateCode::service_request_timeout;
      Warn(
        "[%s] Waiting for skin set service to response timeout.",
        this->logger_.c_str());
      return ret;
    }
    auto result_ptr = result.get();
    ret.code = (result_ptr->success) ? StateCode::success : StateCode::fail;
    _response = *result_ptr;
    return ret;
  } catch (...) {
    ret.code = StateCode::fail;
    Warn(
      "[%s] RequestSkeletonRecognizedSrv() is failed.",
      this->logger_.c_str());
  }
  return ret;
}

SkinElectrochromicResponse Skin::Discolored(
  const int _model,
  const int _duration_ms)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _model, _duration_ms);
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      this->response_.state = this->GetState(funs, this->state_.code);
      return this->response_;
    }
    this->response_.state = this->GetState(funs, StateCode::success);
    {
      std::shared_ptr<SrvSetBool::Request> request_ptr =
        std::make_shared<SrvSetBool::Request>();
      request_ptr->data = true;
      SrvSetBool::Response response;
      this->response_.state = this->RequestEnableSrv(response, request_ptr);
      if (this->response_.state.code != StateCode::success) {
        return this->response_;
      }
    }
    {
      std::shared_ptr<SrvElecSkin::Request> request_ptr =
        std::make_shared<SrvElecSkin::Request>();
      request_ptr->mode = _model;
      request_ptr->wave_cycle_time = _duration_ms;
      SrvElecSkin::Response response;
      this->response_.state = this->RequestSetSrv(response, request_ptr);
      if (this->response_.state.code != StateCode::success) {
        return this->response_;
      }
    }

    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] Skin Recognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    this->response_.state = this->GetState(funs, StateCode::fail);
  }
  return this->response_;
}
}   // namespace cyberdog_visual_programming_abilityset
