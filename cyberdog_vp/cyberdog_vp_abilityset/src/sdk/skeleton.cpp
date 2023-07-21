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

#include "cyberdog_vp_abilityset/skeleton.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex skeleton_recognition_state_cvm_;       /*!< 识别状态cvm */
std::condition_variable
  skeleton_recognition_state_cv_;                 /*!< 识别状态cv */

bool Skeleton::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Skeleton::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->skeleton_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->skeleton_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->skeleton_sub_cb_group_;
    this->skeleton_recognition_sub_ptr_ = this->node_immortal_ptr_->create_subscription<MsgSport>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "ai_skeleton", "sport_counts_msg"),
      SubscriptionQos,
      std::bind(&Skeleton::SubSkeletonRecognitionCB, this, std::placeholders::_1),
      sub_option);

    this->skeleton_recognition_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvSport>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "ai_skeleton", "sport_manager"),
      ClientQos.get_rmw_qos_profile(),
      this->skeleton_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Skeleton::InitDependent(
  const std::function<AudioPlaySeviceResponse(const std::string, const int8_t)> & _Play,
  const std::function<State(const std::string, const int8_t)> & _InstantlyPlay)
{
  this->FPlay = _Play;
  this->FInstantlyPlay = _InstantlyPlay;
}

void Skeleton::SubSkeletonRecognitionCB(const MsgSport::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(skeleton_recognition_state_cvm_);
    std::scoped_lock lk(skeleton_recognition_state_cvm_);
    if (_msg_ptr->algo_switch == MsgSport::ALGO_OPEN) {
      this->recognition_ = *_msg_ptr;
    } else {
      this->recognition_.algo_switch = _msg_ptr->algo_switch;
    }
    this->skeleton_update_ = true;
  }
  skeleton_recognition_state_cv_.notify_all();
  if (this->interact_) {
    if (this->recognition_.algo_switch == MsgSport::ALGO_OPEN) {
      if (this->recognition_.sport_type == MsgSport::SPORT_PLANK) {
        if ((this->recognition_.duration % 5) == 0) {
          if (this->instantly_) {
            this->FInstantlyPlay(
              std::string(FORMAT("%d秒", this->recognition_.duration)),
              this->volume_);
          } else {
            this->FPlay(
              std::string(FORMAT("%d秒", this->recognition_.duration)),
              this->volume_);
          }
        }
      } else {
        if (this->instantly_) {
          this->FInstantlyPlay(
            std::string(FORMAT("%d", this->recognition_.counts)),
            this->volume_);
        } else {
          this->FPlay(
            std::string(FORMAT("%d", this->recognition_.counts)),
            this->volume_);
        }
      }
    } else {
      if (this->recognition_.sport_type == MsgSport::SPORT_PLANK) {
        if (this->recognition_.algo_switch == MsgSport::COUNT_COMPLETE_CLOSE) {
          this->FPlay(
            std::string(FORMAT("运动时长%d秒, 运动已达标。", this->recognition_.duration)),
            this->volume_);
        } else {
          this->FPlay(
            std::string(FORMAT("运动时长%d秒, 运动未达标。", this->recognition_.duration)),
            this->volume_);
        }
      } else {
        if (this->recognition_.algo_switch == MsgSport::COUNT_COMPLETE_CLOSE) {
          this->FPlay(
            std::string(FORMAT("运动计数%d个, 运动已达标。", this->recognition_.counts)),
            this->volume_);
        } else {
          this->FPlay(
            std::string(FORMAT("运动计数%d个, 运动未达标。", this->recognition_.counts)),
            this->volume_);
        }
      }
    }
  }
}

std::shared_ptr<SrvSport::Request> Skeleton::GetSkeletonRecognitionRequest()
{
  std::shared_ptr<SrvSport::Request> request_ptr =
    std::make_shared<SrvSport::Request>();
  request_ptr->id = this->task_id_;     // string   # 消息id
  request_ptr->command = false;         // bool     false关闭算法，true启动算法
//   SrvSport::Request::SPORT_SQUAT;    // 1 深蹲
//   SrvSport::Request::SPORT_HIGHKNEES;// 2 高抬腿
//   SrvSport::Request::SPORT_SITUP;    // 3 仰卧起坐
//   SrvSport::Request::SPORT_PRESSUP;  // 4 俯卧撑
//   SrvSport::Request::SPORT_PLANK;    // 5 平板支撑
//   SrvSport::Request::SPORT_JUMPJACK; // 6 开合跳
  request_ptr->sport_type = 0;          // uint8    # 运动类型
  request_ptr->counts = 0;              // int32    # 申请做动作的个数，从1开始
  request_ptr->timeout = 0;             // int32    # 有效时间1～300，default = 60
  return request_ptr;
}

State Skeleton::RequestSkeletonRecognizedSrv(
  SrvSport::Response & _response,
  std::shared_ptr<SrvSport::Request> _request_ptr,
  const int _service_start_timeout)
{
  State ret;
  try {
    if (!rclcpp::ok()) {
      ret.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for skeleton recognized service to appear.",
        this->logger_.c_str());
      return ret;
    }
    if (!this->skeleton_recognition_cli_ptr_->wait_for_service(
        std::chrono::seconds(
          _service_start_timeout)))
    {
      ret.code = StateCode::service_appear_timeout;
      Warn(
        "[%s] Waiting for skeleton recognized service to appear(start) timeout.",
        this->logger_.c_str());
      return ret;
    }
    Debug("The interface is requesting skeleton recognized service.");
    auto result = this->skeleton_recognition_cli_ptr_->async_send_request(_request_ptr);
    std::future_status status = result.wait_for(
      std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      ret.code = StateCode::service_request_timeout;
      Warn(
        "[%s] Waiting for skeleton recognized service to response timeout.",
        this->logger_.c_str());
      return ret;
    }
    auto result_ptr = result.get();
    ret.code = (result_ptr->result == SrvSport::Response::ENABLE_SUCCESS) ?
      StateCode::success : StateCode::fail;
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

SkeletonRecognizedSeviceResponse Skeleton::TurnOnRecognition(
  const uint _sport_type,
  const int _counts,
  const int _timeout)
{
  transient_state_.code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d, %d)", _sport_type, _counts, _timeout);
  SkeletonRecognizedSeviceResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      transient_state_ = ret.state;
      return ret;
    }
    SrvSport::Response response;
    std::shared_ptr<SrvSport::Request> request_ptr = this->GetSkeletonRecognitionRequest();
    request_ptr->command = true;
    request_ptr->sport_type = _sport_type;
    request_ptr->counts = _counts;
    request_ptr->sport_type = _sport_type;
    request_ptr->timeout = _timeout;
    ret.state = this->RequestSkeletonRecognizedSrv(
      response, request_ptr,
      SrvSport::Request::ALGORITHM_LOAD_DURATION);
    ret.response = response;
  } catch (const std::exception & e) {
    Warn(
      "[%s] SkeletonRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

SkeletonRecognizedSeviceResponse Skeleton::TurnOffRecognition()
{
  transient_state_.code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + "";
  SkeletonRecognizedSeviceResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      transient_state_ = ret.state;
      return ret;
    }
    SrvSport::Response response;
    std::shared_ptr<SrvSport::Request> request_ptr = this->GetSkeletonRecognitionRequest();
    ret.state = this->RequestSkeletonRecognizedSrv(response, request_ptr);
    ret.response = response;
  } catch (const std::exception & e) {
    Warn(
      "[%s] SkeletonRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

SkeletonRecognizedMessageResponse Skeleton::BlockingRecognized(
  const int _timeout)
{
  transient_state_.code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + "";
  SkeletonRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      transient_state_ = ret.state;
      return ret;
    }
    int timeout = 0;
    if (_timeout == -1) {
      timeout = SrvSport::Request::DEFAULT_TIMEOUT;
    } else if (_timeout > SrvSport::Request::MAX_TIMEOUT) {
      timeout = SrvSport::Request::MAX_TIMEOUT;
    } else if (_timeout < SrvSport::Request::MIN_TIMEOUT) {
      timeout = SrvSport::Request::MIN_TIMEOUT;
    } else {
      timeout = _timeout;
    }
    {
      std::unique_lock<std::mutex> lk(skeleton_recognition_state_cvm_);
      this->skeleton_update_ = false;
      skeleton_recognition_state_cv_.wait_for(
        lk, std::chrono::seconds(timeout), [&] {
          return this->skeleton_update_;
        });
      ret.response = this->recognition_;
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] SkeletonRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

SkeletonRecognizedMessageResponse Skeleton::InstantRecognized()
{
  transient_state_.code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + "";
  SkeletonRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      transient_state_ = ret.state;
      return ret;
    }
    ret.response = this->recognition_;
  } catch (const std::exception & e) {
    Warn(
      "[%s] SkeletonRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

SkeletonRecognizedMessageResponse Skeleton::SportsRecognition(
  const uint _sport_type,
  const int _counts,
  const int _timeout,
  const bool _interact,
  const bool _instantly,
  const int _volume)
{
  transient_state_.code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d, %d, %d, %d, %d)", _sport_type, _counts, _timeout, _interact, _instantly, _volume);
  SkeletonRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      transient_state_ = ret.state;
      return ret;
    }
    int timeout = 0;
    if (_timeout == -1) {
      timeout = SrvSport::Request::DEFAULT_TIMEOUT;
    } else if (_timeout > SrvSport::Request::MAX_TIMEOUT) {
      timeout = SrvSport::Request::MAX_TIMEOUT;
    } else if (_timeout < SrvSport::Request::MIN_TIMEOUT) {
      timeout = SrvSport::Request::MIN_TIMEOUT;
    } else {
      timeout = _timeout;
    }
    if (this->TurnOnRecognition(
        _sport_type, _counts,
        timeout).response.result ==
      SrvSport::Response::ENABLE_SUCCESS)
    {
      this->interact_ = _interact;
      this->volume_ = _volume;
      this->instantly_ = _instantly;
      {
        std::unique_lock<std::mutex> lk(skeleton_recognition_state_cvm_);
        this->skeleton_update_ = false;
        skeleton_recognition_state_cv_.wait_for(
          lk, std::chrono::seconds(
            timeout), [&] {
            return this->skeleton_update_ &&
            (this->recognition_.algo_switch != MsgSport::ALGO_OPEN);
          });
        ret.response = this->recognition_;
      }
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] SkeletonRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}
}   // namespace cyberdog_visual_programming_abilityset
