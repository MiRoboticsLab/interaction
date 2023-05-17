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

#include "cyberdog_vp_abilityset/odometer.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex odometer_data_cvm_;                    /*!< Odometer 数据 cvm */
std::condition_variable odometer_data_cv_;        /*!< Odometer 数据 cv */

bool Odometer::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    return true;
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Odometer::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->sub_cb_group_;
    this->sub_ptr_ = this->node_mortal_ptr_->create_subscription<MsgOdometry>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "odometer", "odom_out"),
      SubscriptionSensorQos,
      std::bind(&Odometer::SubCB, this, std::placeholders::_1),
      sub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Odometer::SubCB(const MsgOdometry::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(odometer_data_cvm_);
    std::scoped_lock lk(odometer_data_cvm_);
    this->data_ = *_msg_ptr;
    this->state_.code = StateCode::success;
  }
  odometer_data_cv_.notify_all();
  this->timens_ = GetTimeNs();
}

MsgOdometry Odometer::GetData(const int _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)", _timeout);
  try {
    Debug("%s %s", this->logger_.c_str(), funs.c_str());
    this->SetState(StateCode::no_data_update);
    std::unique_lock<std::mutex> lk(odometer_data_cvm_);
    odometer_data_cv_.wait_for(
      lk, std::chrono::seconds(_timeout), [&] {
        return this->state_.code == StateCode::success;
      });
  } catch (const std::exception & e) {
    Error("%s %s is failed: %s", this->logger_.c_str(), funs.c_str(), e.what());
  }
  return this->data_;
}

bool Odometer::IfCumulativeDistance(
  MsgOdometry & _odom, double & _cumulative,
  const double _distance)
{
  auto delta_square = [&](const double & a, const double & b) -> double {
      double delta = a - b;
      return static_cast<double>(delta * delta);
    };
  if (_cumulative < 0.0) {
    _cumulative = 0.0;
  } else {
    _cumulative += std::sqrt(
      delta_square(this->data_.pose.pose.position.x, _odom.pose.pose.position.x) +
      delta_square(this->data_.pose.pose.position.y, _odom.pose.pose.position.y));
  }
  _odom = this->data_;
  return static_cast<bool>(_cumulative > _distance);
}
}   // namespace cyberdog_visual_programming_abilityset
