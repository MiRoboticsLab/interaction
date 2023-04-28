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

#include "cyberdog_vp_abilityset/face.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Face::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Face::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->face_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->face_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->face_sub_cb_group_;
    this->face_recognition_sub_ptr_ = this->node_immortal_ptr_->create_subscription<MsgFaceRes>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "ai_face", "face_rec_msg"),
      SubscriptionQos,
      std::bind(&Face::SubFaceRecognitionCB, this, std::placeholders::_1),
      sub_option);

    this->face_recognition_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvFaceRec>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "ai_face", "cyberdog_face_recognition_srv"),
      ClientQos.get_rmw_qos_profile(),
      this->face_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Face::SubFaceRecognitionCB(const MsgFaceRes::SharedPtr _msg_ptr)
{
  if (!this->face_recognition_life_cycle_) {
    return;
  }
  if (this->face_target_id_.empty()) {
    this->face_recognition_data_.push_back(*_msg_ptr);
  } else {
    auto target_ptr = this->face_target_id_.find(_msg_ptr->username);
    std::string target_request_id =
      static_cast<bool>(target_ptr != this->face_target_id_.end()) ?
      target_ptr->second.c_str() : "";
    if (target_request_id == _msg_ptr->id) {
      this->face_recognition_data_.push_back(*_msg_ptr);
    }
  }
}

std::shared_ptr<SrvFaceRec::Request> Face::GetFaceRecognitionRequest()
{
  std::shared_ptr<SrvFaceRec::Request> request_ptr =
    std::make_shared<SrvFaceRec::Request>();
  request_ptr->id = this->task_id_;     // string   # 消息id
  request_ptr->command =                // int32
    SrvFaceRec::Request::COMMAND_RECOGNITION_ALL;   // 识别数据库中存在的人脸
  // SrvFaceRec::Request::COMMAND_RECOGNITION_SINGLE;  // 识别指定昵称人脸
  // SrvFaceRec::Request::COMMAND_RECOGNITION_CANCEL;  // 取消识别指定昵称人脸
  request_ptr->username = "";           // string   # 识别目标名称
  request_ptr->timeout = 0;             // int32    # 有效时间1～300，default = 60
  return request_ptr;
}

bool Face::RequestFaceRecognizedSrv(
  SrvFaceRec::Response & _response,
  std::shared_ptr<SrvFaceRec::Request> _request_ptr,
  const int _service_start_timeout)
{
  try {
    if (!rclcpp::ok()) {
      this->transient_state_.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for face recognized service to appear.",
        this->logger_.c_str());
      return false;
    }
    if (!this->face_recognition_cli_ptr_->wait_for_service(
        std::chrono::seconds(
          _service_start_timeout)))
    {
      this->transient_state_.code = StateCode::service_appear_timeout;
      Warn(
        "[%s] Waiting for face recognized service to appear(start) timeout.",
        this->logger_.c_str());
      return false;
    }
    Debug("The interface is requesting face recognized service.");
    auto result = this->face_recognition_cli_ptr_->async_send_request(_request_ptr);
    std::future_status status = result.wait_for(
      std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      this->transient_state_.code = StateCode::service_request_timeout;
      Warn(
        "[%s] Waiting for face recognized service to response timeout.",
        this->logger_.c_str());
      return false;
    }
    auto result_ptr = result.get();
    _response = *result_ptr;
    if (_response.result == SrvFaceRec::Response::ENABLE_SUCCESS) {
      this->transient_state_.code = StateCode::success;
    } else {
      this->transient_state_.code = StateCode::service_request_rejected;
    }
    return static_cast<bool>(_response.result == SrvFaceRec::Response::ENABLE_SUCCESS);
  } catch (...) {
    this->transient_state_.code = StateCode::fail;
    Warn(
      "[%s] RequestFaceRecognizedSrv() is failed.",
      this->logger_.c_str());
  }
  return false;
}

FaceRecognizedSeviceResponse Face::Recognized(
  const std::vector<std::string> & _voiceprint_target,
  const bool _and_operation,
  const int _duration)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %s, %d)",
    stringVector(_voiceprint_target).c_str(),
    std::string(_and_operation ? "True" : "False").c_str(),
    _duration);
  FaceRecognizedSeviceResponse ret;
  this->transient_state_.code = StateCode::success;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      return ret;
    }
    this->face_target_id_.clear();
    this->face_recognition_data_.clear();
    SrvFaceRec::Response response;
    std::shared_ptr<SrvFaceRec::Request> request_ptr = this->GetFaceRecognitionRequest();
    if (_duration == -1) {
      request_ptr->timeout = SrvFaceRec::Request::DEFAULT_TIMEOUT;
    } else if (_duration > SrvFaceRec::Request::MAX_TIMEOUT) {
      request_ptr->timeout = SrvFaceRec::Request::MAX_TIMEOUT;
    } else if (_duration < SrvFaceRec::Request::MIN_TIMEOUT) {
      request_ptr->timeout = SrvFaceRec::Request::MIN_TIMEOUT;
    } else {
      request_ptr->timeout = _duration;
    }
    auto get_response_msg = [&]() {
        this->face_recognition_life_cycle_ = true;
        bool recognized = false;
        std::chrono::time_point<std::chrono::system_clock> timeout =
          std::chrono::system_clock::now() +
          std::chrono::seconds(
          static_cast<int>(request_ptr->timeout +
          SrvFaceRec::Request::ALGORITHM_LOAD_DURATION));
        while (std::chrono::system_clock::now() < timeout) {
          if (this->face_target_id_.empty()) {
            if (!this->face_recognition_data_.empty()) {
              recognized = true;
              break;
            }
          } else {
            if (_and_operation) {
              if (this->face_recognition_data_.size() == this->face_target_id_.size()) {
                recognized = true;
                break;
              }
            } else {
              if (!this->face_recognition_data_.empty()) {
                recognized = true;
                break;
              }
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 50hz
        }
        this->face_recognition_life_cycle_ = false;
        if (!recognized) {
          this->transient_state_.code = StateCode::fail;
        }
      };
    std::string now_time = GetTime(TimeMode::_Y_M_D_H_M_S);
    if (_voiceprint_target.empty()) {
      request_ptr->id = this->task_id_ + "_" + now_time;
      request_ptr->command = SrvFaceRec::Request::COMMAND_RECOGNITION_ALL;
      if (this->RequestFaceRecognizedSrv(response, request_ptr)) {
        get_response_msg();
      }
    } else {
      request_ptr->command = SrvFaceRec::Request::COMMAND_RECOGNITION_SINGLE;
      int personnel_id = 0;
      for (const auto meta : _voiceprint_target) {
        request_ptr->id = this->task_id_ + "_" + now_time + "_" + std::to_string(personnel_id++);
        request_ptr->username = meta;
        if (this->RequestFaceRecognizedSrv(response, request_ptr)) {
          this->face_target_id_.insert(
            std::map<std::string, std::string>::value_type(
              meta,
              request_ptr->id));
        } else {
          if (_and_operation) {
            break;
          }
        }
      }
      if ((_voiceprint_target.empty()) ||
        (!_and_operation && !this->face_target_id_.empty()) ||
        (_and_operation && (this->face_target_id_.size() == _voiceprint_target.size())))
      {
        get_response_msg();
      }
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] FaceRecognized() is failed. %s",
      this->logger_.c_str(),
      e.what());
    this->transient_state_.code = StateCode::fail;
  }
  ret.state = this->GetState(funs, this->transient_state_.code);
  ret.list = this->face_recognition_data_;
  for (const MsgFaceRes & meta : ret.list) {
    ret.dictionary.insert(
      std::map<std::string, MsgFaceRes>::value_type(
        meta.username, meta));
  }
  return ret;
}

FaceSeviceResponse Face::CancelRecognize(
  const int _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)",
    _timeout);
  FaceSeviceResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      return ret;
    }
    std::shared_ptr<SrvFaceRec::Request> request_ptr = this->GetFaceRecognitionRequest();
    request_ptr->timeout = SrvFaceRec::Request::DEFAULT_TIMEOUT;
    request_ptr->command = SrvFaceRec::Request::COMMAND_RECOGNITION_CANCEL;
    if (!this->RequestFaceRecognizedSrv(ret.response, request_ptr)) {
      Warn("[%s] %s is error.", this->logger_.c_str(), funs.c_str());
      ret.state = this->GetState(funs, StateCode::fail);
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] %s is failed. %s",
      this->logger_.c_str(),
      funs.c_str(),
      e.what());
    ret.state = this->GetState(funs, StateCode::fail);
  }
  ret.state = this->GetState(funs, StateCode::success);
  return ret;
}
}   // namespace cyberdog_visual_programming_abilityset
