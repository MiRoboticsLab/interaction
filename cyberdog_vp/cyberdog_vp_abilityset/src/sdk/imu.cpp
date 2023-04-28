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

#include "cyberdog_vp_abilityset/imu.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex imu_data_cvm_;                         /*!< Imu 数据 cvm */
std::condition_variable imu_data_cv_;             /*!< Imu 数据 cv */

bool Imu::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Imu::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->sub_cb_group_;
    this->sub_ptr_ = this->node_mortal_ptr_->create_subscription<MsgImu>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "imu"),
      SubscriptionSensorQos,
      std::bind(&Imu::SubCB, this, std::placeholders::_1),
      sub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Imu::SubCB(const MsgImu::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(imu_data_cvm_);
    std::scoped_lock lk(imu_data_cvm_);
    this->data_ = *_msg_ptr;
  }
  imu_data_cv_.notify_all();
  this->timens_ = GetTimeNs();
}

MsgImu Imu::GetData(const int _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)", _timeout);
  try {
    Debug("%s %s", this->logger_.c_str(), funs.c_str());
    this->SetState(StateCode::no_data_update);
    std::unique_lock<std::mutex> lk(imu_data_cvm_);
    imu_data_cv_.wait_for(
      lk, std::chrono::seconds(_timeout), [&] {
        return this->state_.code == StateCode::success;
      });
  } catch (const std::exception & e) {
    Error("%s %s is failed: %s", this->logger_.c_str(), funs.c_str(), e.what());
  }
  return this->data_;
}

bool Imu::IfDeltaAngular(const MsgImu & _imu, const double _angular, RPYType _type)
{
  double ab_delta = 0.0;
  auto sign = [](const double data) -> int {
      return static_cast<int>((data > 0) ? 1 : -1);
    };
  auto delta = [&](const double a, const double b) -> double {
      int a_sign = sign(a);
      int b_sign = sign(b);
      if (a_sign == b_sign) {
        ab_delta = fabs(a - b);
      } else {
        ab_delta = fabs(a_sign * 180 - a) + fabs(b_sign * 180 - b);
      }
      return ab_delta;
    };
  RPY a, b;
  a = this->Quaternion2RPY(_imu);
  b = this->Quaternion2RPY(this->data_);
  switch (_type) {
    case RPYType::ROLL:
      ab_delta = delta(a.roll, b.roll);
      break;
    case RPYType::PITCH:
      ab_delta = delta(a.pitch, b.pitch);
      break;
    case RPYType::YAW:
      ab_delta = delta(a.yaw, b.yaw);
      break;
    default:
      ab_delta = 0.0;
      break;
  }
  return static_cast<bool>(ab_delta >= _angular);
}

RPY Imu::Quaternion2RPY(const MsgImu & imu)
{
  tf2::Matrix3x3 matri(tf2::Quaternion(
      imu.orientation.x,
      imu.orientation.y,
      imu.orientation.z,
      imu.orientation.w));
  RPY rpy;
  matri.getRPY(rpy.roll, rpy.pitch, rpy.yaw);
  return rpy;
}

RPY Imu::GetRPY()
{
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return RPY();
  }
  return this->Quaternion2RPY(this->data_);
  // //欧拉角转四元数
  //   tf2::Quaternion orientation;
  //   float yaw=1;
  //   orientation.setRPY(0.0, 0.0, yaw);
  // Quaternion to Euler Angles
}

}   // namespace cyberdog_visual_programming_abilityset
