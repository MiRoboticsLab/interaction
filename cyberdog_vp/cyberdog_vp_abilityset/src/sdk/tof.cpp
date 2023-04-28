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

#include "cyberdog_vp_abilityset/tof.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex tof_data_cvm_;                         /*!< Tof 数据 cvm */
std::condition_variable tof_data_cv_;             /*!< Tof 数据 cv */

bool Tof::SetData(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->obstacle_distance_ = toml::find_or(
      _params_toml, "vp", "init", "detection", "tof", "obstacle_distance", 0.000001);

    this->obstacle_percentage_ = toml::find_or(
      _params_toml, "vp", "init", "detection", "tof", "obstacle_percentage", 80.0);
    if (this->obstacle_percentage_ > 100) {
      this->obstacle_percentage_ = 100;
    } else if (this->obstacle_percentage_ < 0) {
      this->obstacle_percentage_ = 1;
    }
    this->obstacle_percentage_ /= 100;

    this->sensitivity_s_ = toml::find_or(
      _params_toml, "vp", "init", "detection", "tof", "sensitivity_s", 1.0);
    if (this->sensitivity_s_ < 0) {
      this->sensitivity_s_ = 0.1;  // 10 Hz : 100 毫秒
    }
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Tof::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->sub_cb_group_;
    this->sub_head_ptr_ = this->node_mortal_ptr_->create_subscription<MsgHeadTofPayload>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "tof_head", "head_tof_payload"),
      SubscriptionSensorQos,
      std::bind(&Tof::SubHeadCB, this, std::placeholders::_1),
      sub_option);
    this->sub_rear_ptr_ = this->node_mortal_ptr_->create_subscription<MsgRearTofPayload>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "tof_rear", "rear_tof_payload"),
      SubscriptionSensorQos,
      std::bind(&Tof::SubRearCB, this, std::placeholders::_1),
      sub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Tof::SubHeadCB(const MsgHeadTofPayload::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(tof_data_cvm_);
    std::scoped_lock lk(tof_data_cvm_);
    this->data_.head = *_msg_ptr;
  }
  tof_data_cv_.notify_all();
  this->timens_ = GetTimeNs();
  this->Detection(true);
}

void Tof::SubRearCB(const MsgRearTofPayload::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(tof_data_cvm_);
    std::scoped_lock lk(tof_data_cvm_);
    this->data_.rear = *_msg_ptr;
  }
  tof_data_cv_.notify_all();
  this->timens_ = GetTimeNs();
  this->Detection(false);
}

TofPayload Tof::GetData(const int _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)", _timeout);
  try {
    Debug("%s %s", this->logger_.c_str(), funs.c_str());
    this->SetState(StateCode::no_data_update);
    std::unique_lock<std::mutex> lk(tof_data_cvm_);
    tof_data_cv_.wait_for(
      lk, std::chrono::seconds(_timeout), [&] {
        return this->state_.code == StateCode::success;
      });
  } catch (const std::exception & e) {
    Error("%s %s is failed: %s", this->logger_.c_str(), funs.c_str(), e.what());
  }
  return this->data_;
}

bool Tof::Detection(bool is_head)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    auto detection_obstacle = [&](MsgSingleTofPayload & payload, ObstacleMeta & obstacle) {
        if (!payload.data_available) {
          obstacle.detected = false;
          obstacle.validity_period_time = std::chrono::system_clock::now();
          return;
        }
        if ((obstacle.detected) &&
          (std::chrono::system_clock::now() < obstacle.validity_period_time))
        {
          return;
        }
        size_t obstacle_point = 0;
        for (auto ranging : payload.data) {
          if (ranging < this->obstacle_distance_) {
            ++obstacle_point;
          }
        }
        if (static_cast<float>(
            static_cast<float>(obstacle_point) /
            static_cast<float>(payload.data.size())) <
          this->obstacle_percentage_)
        {
          obstacle.detected = false;
          obstacle.validity_period_time = std::chrono::system_clock::now();
        } else {
          obstacle.detected = true;
          obstacle.validity_period_time = std::chrono::system_clock::now() +
            std::chrono::milliseconds(static_cast<int64_t>(1000 * this->sensitivity_s_));
        }
      };
    if (is_head) {
      detection_obstacle(this->data_.head.left_head, this->obstacle_.head_left);
      detection_obstacle(this->data_.head.right_head, this->obstacle_.head_right);
    } else {
      detection_obstacle(this->data_.rear.left_rear, this->obstacle_.rear_left);
      detection_obstacle(this->data_.rear.right_rear, this->obstacle_.rear_right);
    }
  } catch (const std::exception & e) {
    Error("%s Detection failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}
// obstacle
}   // namespace cyberdog_visual_programming_abilityset
