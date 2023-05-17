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

#include "cyberdog_vp_abilityset/bms.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex bms_data_cvm_;                         /*!< BMS 数据 cvm */
std::condition_variable bms_data_cv_;             /*!< BMS 数据 cv */

bool Bms::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("Set data failed: %s", e.what());
    return false;
  }
  return true;
}

bool Bms::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->sub_cb_group_;
    this->sub_ptr_ = this->node_mortal_ptr_->create_subscription<MsgBmsStatus>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "bms", "bms_status"),
      SubscriptionSensorQos,
      std::bind(&Bms::SubCB, this, std::placeholders::_1),
      sub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Bms::SubCB(const MsgBmsStatus::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(bms_data_cvm_);
    std::scoped_lock lk(bms_data_cvm_);
    this->data_ = *_msg_ptr;
    this->state_.code = StateCode::success;
  }
  bms_data_cv_.notify_all();
  this->timens_ = GetTimeNs();
}

MsgBmsStatus Bms::GetData(const int _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)", _timeout);
  try {
    Debug("%s %s", this->logger_.c_str(), funs.c_str());
    this->SetState(StateCode::no_data_update);
    std::unique_lock<std::mutex> lk(bms_data_cvm_);
    bms_data_cv_.wait_for(
      lk, std::chrono::seconds(_timeout), [&] {
        return this->state_.code == StateCode::success;
      });
  } catch (const std::exception & e) {
    Error("%s %s is failed: %s", this->logger_.c_str(), funs.c_str(), e.what());
  }
  Info("%s Bms battery power is %d.", funs.c_str(), this->data_.batt_soc);
  return this->data_;
}
}   // namespace cyberdog_visual_programming_abilityset
