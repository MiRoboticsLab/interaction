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

#include "cyberdog_vp_abilityset/skin.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Skin::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    std::string params_pkg_dir = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    std::string skin_config_dir = params_pkg_dir + "/config/skin.toml";
    this->can0_ptr_ = std::make_shared<cyberdog::embed::Protocol<CanData>>(skin_config_dir, false);
    this->can0_ptr_->SetDataCallback(
      std::bind(&Skin::Can0CB, this, std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Skin::SetMechanism(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Skin::Can0CB(std::string & _name, std::shared_ptr<CanData> _data_ptr)
{
  Info("%s(%s)", std::string(__FUNCTION__).c_str(), _name.c_str());
  this->can0_response_.name = _name;
  this->can0_response_.data.data0 = _data_ptr->data0;
  this->can0_response_.data.data1 = _data_ptr->data1;
  this->can0_response_.data.data2 = _data_ptr->data2;
  this->can0_response_.data.data3 = _data_ptr->data3;
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
      this->can0_response_.state = this->GetState(funs, this->state_.code);
      return this->can0_response_;
    }
    this->can0_response_.state = this->GetState(funs, StateCode::success);
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
    int position = legalization(
      "position", _position,
      static_cast<int>(SkinConstraint::position_max));
    int rendering = legalization(
      "rendering", _rendering,
      static_cast<int>(SkinConstraint::rendering_max));
    int outset = legalization(
      "outset", _outset,
      static_cast<int>(SkinConstraint::outset_max));
    int duration_ms = legalization(
      "duration_ms", _duration_ms,
      static_cast<int>(1000 * 60 * 60 * 24 * 7));
    if (this->now_model_ != model) {
      this->now_model_ = model;
      uint8_t m = static_cast<uint8_t>(this->now_model_);
      uint8_t d_high = *(reinterpret_cast<uint8_t *>(&duration_ms));
      uint8_t d_low = *(reinterpret_cast<uint8_t *>(&duration_ms) + 1);
      this->can0_ptr_->Operate(
        this->constraint_map_.at(this->now_model_),
        std::vector<uint8_t>{m, d_low, d_high});
    }
    if (this->now_model_ == static_cast<int>(SkinConstraint::model_control)) {
      uint8_t r = static_cast<uint8_t>(rendering);
      uint8_t o = static_cast<uint8_t>(outset);
      uint8_t d_high = *(reinterpret_cast<uint8_t *>(&duration_ms));
      uint8_t d_low = *(reinterpret_cast<uint8_t *>(&duration_ms) + 1);
      this->can0_ptr_->Operate(
        this->constraint_map_.at(position),
        std::vector<uint8_t>{r, o, d_low, d_high});
    }
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] Skin Recognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    this->can0_response_.state = this->GetState(funs, StateCode::fail);
  }
  return this->can0_response_;
}
}   // namespace cyberdog_visual_programming_abilityset
