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

#include "cyberdog_vp_abilityset/gesture.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex gesture_recognition_state_cvm_;        /*!< 识别状态cvm */
std::condition_variable
  gesture_recognition_state_cv_;                  /*!< 识别状态cv */

bool Gesture::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Gesture::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->gesture_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->gesture_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->gesture_sub_cb_group_;
    this->gesture_recognition_sub_ptr_ = this->node_immortal_ptr_->create_subscription<MsgGesture>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "ai_gesture", "gesture_action_result"),
      SubscriptionQos,
      std::bind(&Gesture::SubGestureRecognitionCB, this, std::placeholders::_1),
      sub_option);

    this->gesture_recognition_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvGesture>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "ai_gesture", "gesture_action_control"),
      ClientQos.get_rmw_qos_profile(),
      this->gesture_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Gesture::SubGestureRecognitionCB(const MsgGesture::SharedPtr _msg_ptr)
{
  Info("%s(%d)", std::string(__FUNCTION__).c_str(), _msg_ptr->id);
  {
    // std::lock_guard<std::mutex> lk(gesture_recognition_state_cvm_);
    std::scoped_lock lk(gesture_recognition_state_cvm_);
    auto search = this->gesture_target_time_.find(_msg_ptr->id);
    if (search != this->gesture_target_time_.end()) {
      search->second = std::chrono::system_clock::now();
    } else {
      this->gesture_target_time_.insert(
        std::map<int, std::chrono::time_point<std::chrono::system_clock>>::value_type(
          _msg_ptr->id,
          std::chrono::system_clock::now()));
    }
  }
  gesture_recognition_state_cv_.notify_all();
}

std::shared_ptr<SrvGesture::Request> Gesture::GetGestureRecognitionRequest()
{
  std::shared_ptr<SrvGesture::Request> request_ptr =
    std::make_shared<SrvGesture::Request>();
  request_ptr->timeout = SrvGesture::Request::DEFAUT_TIMEOUT;   // int32
  request_ptr->command = SrvGesture::Request::START_ALGO;       // int8
  // SrvGesture::Request::STOP_ALGO;   // 关闭识别
  return request_ptr;
}

bool Gesture::RequestGestureRecognizedSrv(
  SrvGesture::Response & _response,
  std::shared_ptr<SrvGesture::Request> _request_ptr,
  const int _service_start_timeout)
{
  try {
    if (!rclcpp::ok()) {
      this->transient_state_ptr_->code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for gesture recognized service to appear.",
        this->logger_.c_str());
      return false;
    }
    if (!this->gesture_recognition_cli_ptr_->wait_for_service(
        std::chrono::seconds(
          _service_start_timeout)))
    {
      this->transient_state_ptr_->code = StateCode::service_appear_timeout;
      Warn(
        "[%s] Waiting for gesture recognized service to appear(start) timeout.",
        this->logger_.c_str());
      return false;
    }
    Info("The interface is requesting gesture recognized service.");
    auto result = this->gesture_recognition_cli_ptr_->async_send_request(_request_ptr);
    std::future_status status = result.wait_for(
      std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      this->transient_state_ptr_->code = StateCode::service_request_timeout;
      Warn(
        "[%s] Waiting for gesture recognized service to response timeout.",
        this->logger_.c_str());
      return false;
    }
    auto result_ptr = result.get();
    this->transient_state_ptr_->code = StateCode::success;
    _response = *result_ptr;
    return true;
  } catch (...) {
    this->transient_state_ptr_->code = StateCode::fail;
    Warn(
      "[%s] RequestGestureRecognizedSrv() is failed.",
      this->logger_.c_str());
  }
  return false;
}

GestureRecognizedMessageResponse Gesture::Recognized(
  const int _duration,
  const int _sensitivity)
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _duration,
    _sensitivity);
  GestureRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    if (this->TurnOnRecognition(_duration).response.code == SrvGesture::Response::RESULT_SUCCESS) {
      ret = this->RecognizedAnyGesture(_duration, _sensitivity);
    }
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] GestureRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

GestureRecognizedSeviceResponse Gesture::TurnOnRecognition(
  const int _duration)
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)",
    _duration);
  GestureRecognizedSeviceResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    SrvGesture::Response response;
    std::shared_ptr<SrvGesture::Request> request_ptr = this->GetGestureRecognitionRequest();
    request_ptr->timeout = (_duration > 0) ? _duration : SrvGesture::Request::DEFAUT_TIMEOUT;
    request_ptr->command = SrvGesture::Request::START_ALGO;
    if (this->RequestGestureRecognizedSrv(response, request_ptr)) {
      ret.response = response;
    }
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] %s is failed. %s",
      this->logger_.c_str(),
      funs.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

GestureRecognizedSeviceResponse Gesture::TurnOffRecognition()
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + "()";
  GestureRecognizedSeviceResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    SrvGesture::Response response;
    std::shared_ptr<SrvGesture::Request> request_ptr = this->GetGestureRecognitionRequest();
    request_ptr->command = SrvGesture::Request::STOP_ALGO;
    if (this->RequestGestureRecognizedSrv(response, request_ptr)) {
      ret.response = response;
    }
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] %s is failed. %s",
      this->logger_.c_str(),
      funs.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

GestureRecognizedMessageResponse Gesture::RecognizedDesignatedGesture(
  const int _timeout,
  const int _gesture_id)
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _timeout,
    _gesture_id);
  GestureRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    int sensitivity = 1;
    std::unique_lock<std::mutex> lk(gesture_recognition_state_cvm_);
    gesture_recognition_state_cv_.wait_for(
      lk, std::chrono::seconds(_timeout),
      [&] {
        bool type_state = false;
        for (const auto & [gesture_id, gesture_time] : this->gesture_target_time_) {
          bool is_target = (_gesture_id == gesture_id);
          switch (gesture_id) {
            case GestureType::no_gesture:
              break;
            case GestureType::pulling_hand_or_two_fingers_in:
              ret.data.pulling_hand_or_two_fingers_in =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.pulling_hand_or_two_fingers_in : false;
              break;
            case GestureType::pushing_hand_or_two_fingers_away:
              ret.data.pushing_hand_or_two_fingers_away =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.pushing_hand_or_two_fingers_away : false;
              break;
            case GestureType::sliding_hand_or_two_fingers_up:
              ret.data.sliding_hand_or_two_fingers_up =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.sliding_hand_or_two_fingers_up : false;
              break;
            case GestureType::sliding_hand_or_two_fingers_down:
              ret.data.sliding_hand_or_two_fingers_down =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.sliding_hand_or_two_fingers_down : false;
              break;
            case GestureType::sliding_hand_or_two_fingers_left:
              ret.data.sliding_hand_or_two_fingers_left =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.sliding_hand_or_two_fingers_left : false;
              break;
            case GestureType::sliding_hand_or_two_fingers_right:
              ret.data.sliding_hand_or_two_fingers_right =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.sliding_hand_or_two_fingers_right : false;
              break;
            case GestureType::stop_sign:
              ret.data.stop_sign =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              type_state = is_target ? ret.data.stop_sign : false;
              break;
            case GestureType::thumb_up:
              ret.data.thumb_up =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              type_state = is_target ? ret.data.thumb_up : false;
              break;
            case GestureType::zooming_in_with_hand_or_two_fingers:
              ret.data.zooming_in_with_hand_or_two_fingers =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.zooming_in_with_hand_or_two_fingers : false;
              break;
            case GestureType::zooming_out_with_hand_or_two_fingers:
              ret.data.zooming_out_with_hand_or_two_fingers =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(
                sensitivity));
              type_state = is_target ? ret.data.zooming_out_with_hand_or_two_fingers : false;
              break;
            case GestureType::thumb_down:
              ret.data.thumb_down =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              type_state = is_target ? ret.data.thumb_down : false;
              break;

            default:
              Warn(
                "[%s] %s illegal gesture %d.",
                this->logger_.c_str(),
                funs.c_str(),
                gesture_id);
              break;
          }
        }
        return type_state;
      });
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] GestureRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

GestureRecognizedMessageResponse Gesture::RecognizedAnyGesture(
  const int _timeout,
  const int _sensitivity)
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _timeout,
    _sensitivity);
  GestureRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    int sensitivity = (_sensitivity > 0) ? _sensitivity : 1;
    std::unique_lock<std::mutex> lk(gesture_recognition_state_cvm_);
    gesture_recognition_state_cv_.wait_for(
      lk, std::chrono::seconds(_timeout),
      [&] {
        for (const auto & [gesture_id, gesture_time] : this->gesture_target_time_) {
          switch (gesture_id) {
            case GestureType::no_gesture:
              break;
            case GestureType::pulling_hand_or_two_fingers_in:
              ret.data.pulling_hand_or_two_fingers_in =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::pushing_hand_or_two_fingers_away:
              ret.data.pushing_hand_or_two_fingers_away =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::sliding_hand_or_two_fingers_up:
              ret.data.sliding_hand_or_two_fingers_up =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::sliding_hand_or_two_fingers_down:
              ret.data.sliding_hand_or_two_fingers_down =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::sliding_hand_or_two_fingers_left:
              ret.data.sliding_hand_or_two_fingers_left =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::sliding_hand_or_two_fingers_right:
              ret.data.sliding_hand_or_two_fingers_right =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::stop_sign:
              ret.data.stop_sign =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::thumb_up:
              ret.data.thumb_up =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::zooming_in_with_hand_or_two_fingers:
              ret.data.zooming_in_with_hand_or_two_fingers =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::zooming_out_with_hand_or_two_fingers:
              ret.data.zooming_out_with_hand_or_two_fingers =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;
            case GestureType::thumb_down:
              ret.data.thumb_down =
              std::chrono::system_clock::now() <
              (gesture_time + std::chrono::seconds(sensitivity));
              break;

            default:
              Warn(
                "[%s] %s illegal gesture %d.",
                this->logger_.c_str(),
                funs.c_str(),
                gesture_id);
              break;
          }
        }
        return ret.data.pulling_hand_or_two_fingers_in ||
        ret.data.pushing_hand_or_two_fingers_away ||
        ret.data.sliding_hand_or_two_fingers_up ||
        ret.data.sliding_hand_or_two_fingers_down ||
        ret.data.sliding_hand_or_two_fingers_left ||
        ret.data.sliding_hand_or_two_fingers_right ||
        ret.data.stop_sign ||
        ret.data.thumb_down ||
        ret.data.thumb_up ||
        ret.data.zooming_in_with_hand_or_two_fingers ||
        ret.data.zooming_out_with_hand_or_two_fingers;
      });
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] GestureRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    ret.state.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}
}   // namespace cyberdog_visual_programming_abilityset
