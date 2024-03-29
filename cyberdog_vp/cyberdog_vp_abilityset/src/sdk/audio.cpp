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

#include "cyberdog_vp_abilityset/audio.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex user_dialogue_data_cvm_;               /*!< 用户对话 数据 cvm */
std::condition_variable user_dialogue_data_cv_;   /*!< 用户对话 数据 cv */

bool Audio::SetData(const toml::value &)
{
  try {
    Debug("%s() ...", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Audio::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s() ...", std::string(__FUNCTION__).c_str());
    this->play_pub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->control_dialogue_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->user_dialogue_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->play_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->get_volume_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->set_volume_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->play_pub_cb_group_;
    this->play_message_pub_ = this->node_mortal_ptr_->create_publisher<MsgAudioPlayExtend>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "audio", "speech_play_extend"),
      PublisherQos,
      pub_option);

    pub_option.callback_group = this->control_dialogue_cb_group_;
    this->control_dialogue_message_pub_ = this->node_mortal_ptr_->create_publisher<MsgBool>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "dialogue", "continue_dialog"),
      PublisherQos,
      pub_option);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->user_dialogue_cb_group_;
    this->user_dialogue_message_sub_ = this->node_mortal_ptr_->create_subscription<MsgString>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "user_dialogue", "asr_text"),
      SubscriptionSensorQos,
      std::bind(&Audio::UserDialogueCB, this, std::placeholders::_1),
      sub_option);

    this->play_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvAudioTextPlay>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "audio_play", "speech_text_play"),
      ClientQos.get_rmw_qos_profile(),
      this->play_cli_cb_group_);
    this->get_volume_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvAudioGetVolume>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "audio_get_volume", "audio_volume_get"),
      ClientQos.get_rmw_qos_profile(),
      this->get_volume_cli_cb_group_);
    this->set_volume_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvAudioSetVolume>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "audio_set_volume", "audio_volume_set"),
      ClientQos.get_rmw_qos_profile(),
      this->set_volume_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Audio::UserDialogueCB(const MsgString::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(user_dialogue_data_cvm_);
    std::scoped_lock lk(user_dialogue_data_cvm_);
    DialogueResponse now_msg;
    now_msg.data = _msg_ptr->data;
    now_msg.time_ns = GetTimeNs();
    this->user_dialogues_.push_back(now_msg);
    this->user_dialogue_state_.code = StateCode::success;
  }
  user_dialogue_data_cv_.notify_all();
}

std::shared_ptr<SrvAudioTextPlay::Request> Audio::GetPlayRequest()
{
  std::shared_ptr<SrvAudioTextPlay::Request> request_ptr =
    std::make_shared<SrvAudioTextPlay::Request>();
  request_ptr->module_name = "cyberdog_vp";
  request_ptr->is_online = true;
  // request_ptr->speech;
  request_ptr->text = "";
  return request_ptr;
}

std::shared_ptr<SrvAudioSetVolume::Request> Audio::GetSetVolumeRequest()
{
  std::shared_ptr<SrvAudioSetVolume::Request> request_ptr =
    std::make_shared<SrvAudioSetVolume::Request>();
  return request_ptr;
}

bool Audio::RequestPlaySrv(
  AudioPlaySeviceResponse & _response,
  std::string _interface_name,
  std::shared_ptr<SrvAudioTextPlay::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    if (!rclcpp::ok()) {
      Warn(
        "[%s] Client interrupted while requesting for service to appear.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_request_interrupted;
      return false;
    }
    if (!this->play_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout))) {
      Warn(
        "[%s] Waiting for service to appear(start) timeout.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_appear_timeout;
      return false;
    }
    Info(
      "The %s interface is requesting audio service.",
      _interface_name.c_str());
    auto result = this->play_cli_ptr_->async_send_request(_request_ptr);
    // result.wait();
    std::future_status status = result.wait_for(
      std::chrono::seconds(
        _request_ptr->text.length() * 2));  // 默认最慢速度为两秒一字
    if (status != std::future_status::ready) {
      _response.state.code = StateCode::service_request_timeout;
      Warn(
        "The %s interface waiting for audio service response play to response timeout.",
        _interface_name.c_str());
      return false;
    }
    auto result_ptr = result.get();
    _response.state.code = StateCode::success;
    _response.response = *result_ptr;
    Info(
      "The %s interface requesting audio service response play is ready, code = %d.",
      _interface_name.c_str(),
      result_ptr->code);
    return true;
  } catch (...) {
    Warn("[%s] RequestPlaySrv() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

bool Audio::RequestGetVolumeSrv(
  AudioGetVolumeSeviceResponse & _response,
  std::string _interface_name,
  int _service_start_timeout)
{
  try {
    if (!rclcpp::ok()) {
      Warn(
        "[%s] Client interrupted while requesting for service to appear.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_request_interrupted;
      return false;
    }
    if (!this->get_volume_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout)))
    {
      Warn(
        "[%s] Waiting for service to appear(start) timeout.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_appear_timeout;
      return false;
    }
    Info(
      "The %s interface is requesting audio service.",
      _interface_name.c_str());
    auto result = this->get_volume_cli_ptr_->async_send_request(
      std::make_shared<SrvAudioGetVolume::Request>());
    // result.wait();
    std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      _response.state.code = StateCode::service_request_timeout;
      Warn(
        "The %s interface waiting for audio service response get volume to response timeout.",
        _interface_name.c_str());
      return false;
    }
    auto result_ptr = result.get();
    _response.state.code = StateCode::success;
    _response.response = *result_ptr;
    Info(
      "The %s interface requesting audio service response get volume is ready, code = %d.",
      _interface_name.c_str(),
      result_ptr->code);
    return true;
  } catch (...) {
    Warn("[%s] RequestGetVolumeSrv() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

bool Audio::RequestSetVolumeSrv(
  AudioSetVolumeSeviceResponse & _response,
  std::string _interface_name,
  std::shared_ptr<SrvAudioSetVolume::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    if (!rclcpp::ok()) {
      Warn(
        "[%s] Client interrupted while requesting for service to appear.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_request_interrupted;
      return false;
    }
    if (!this->set_volume_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout)))
    {
      Warn(
        "[%s] Waiting for service to appear(start) timeout.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_appear_timeout;
      return false;
    }
    Info(
      "The %s interface is requesting audio service.",
      _interface_name.c_str());
    auto result = this->set_volume_cli_ptr_->async_send_request(_request_ptr);
    // result.wait();
    std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      _response.state.code = StateCode::service_request_timeout;
      Warn(
        "The %s interface waiting for audio service response set volume to response timeout.",
        _interface_name.c_str());
      return false;
    }
    auto result_ptr = result.get();
    _response.state.code = StateCode::success;
    _response.response = *result_ptr;
    Info(
      "The %s interface requesting audio service response set volume is ready, code = %d.",
      _interface_name.c_str(),
      result_ptr->code);
    return true;
  } catch (...) {
    Warn("[%s] RequestSetVolumeSrv() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

AudioPlaySeviceResponse Audio::OnlinePlay(
  const std::string _message,
  const int8_t _volume)
{
  this->transient_state_ptr_->code = StateCode::success;
  AudioPlaySeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %d) ...", _message.c_str(), _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
    return ret;
  }
  if (_volume >= 0) {
    this->SetVolume(_volume);
  }
  auto request_ptr = this->GetPlayRequest();
  request_ptr->text = _message;
  if (!this->RequestPlaySrv(ret, std::string(__FUNCTION__), request_ptr)) {
    Warn(
      "[%s] Request play service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

State Audio::OnlineInstantlyPlay(
  const std::string _message,
  const int8_t _volume)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %d) ...", _message.c_str(), _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  if (_volume >= 0) {
    this->SetVolume(_volume);
  }
  MsgAudioPlayExtend audio_msg;
  audio_msg.module_name = "cyberdog_vp";
  audio_msg.is_online = true;
  audio_msg.text = _message;
  // audio_msg.speech.xxx
  this->play_message_pub_->publish(audio_msg);
  return this->GetState(funs, StateCode::success);
}

AudioPlaySeviceResponse Audio::OfflinePlay(
  const uint16_t _audio_id,
  const int8_t _volume)
{
  this->transient_state_ptr_->code = StateCode::success;
  AudioPlaySeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d) ...", _audio_id, _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
    return ret;
  }
  if (_volume >= 0) {
    this->SetVolume(_volume);
  }
  auto request_ptr = this->GetPlayRequest();
  request_ptr->is_online = false;
  request_ptr->speech.module_name = "cyberdog_vp";
  request_ptr->speech.play_id = _audio_id;
  if (!this->RequestPlaySrv(ret, std::string(__FUNCTION__), request_ptr)) {
    Warn(
      "[%s] Request play service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

State Audio::OfflineInstantlyPlay(
  const uint16_t _audio_id,
  const int8_t _volume)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d) ...", _audio_id, _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  if (_volume >= 0) {
    this->SetVolume(_volume);
  }
  MsgAudioPlayExtend audio_msg;
  audio_msg.module_name = "cyberdog_vp";
  audio_msg.is_online = false;
  audio_msg.speech.module_name = "cyberdog_vp";
  audio_msg.speech.play_id = _audio_id;
  this->play_message_pub_->publish(audio_msg);
  return this->GetState(funs, StateCode::success);
}

AudioGetVolumeSeviceResponse Audio::GetVolume()
{
  this->transient_state_ptr_->code = StateCode::success;
  AudioGetVolumeSeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + "() ...";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
    return ret;
  }
  if (!this->RequestGetVolumeSrv(ret, std::string(__FUNCTION__))) {
    Warn(
      "[%s] Request play service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

AudioSetVolumeSeviceResponse Audio::SetVolume(const uint8_t _volume)
{
  this->transient_state_ptr_->code = StateCode::success;
  AudioSetVolumeSeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT("(%d) ...", _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
    return ret;
  }
  auto request_ptr = this->GetSetVolumeRequest();
  request_ptr->volume = (_volume > 100) ? 100 : _volume;
  if (!this->RequestSetVolumeSrv(ret, std::string(__FUNCTION__), request_ptr)) {
    Warn(
      "[%s] Request play service is error.",
      funs.c_str());
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}

State Audio::SetDialogue(const bool _turn_on)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT("(%d) ...", _turn_on);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  MsgBool control_msg;
  control_msg.data = _turn_on;
  this->control_dialogue_message_pub_->publish(control_msg);
  return this->GetState(funs, StateCode::success);
}

State Audio::ResetUserDialogue()
{
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  {
    // std::lock_guard<std::mutex> lk(user_dialogue_data_cvm_);
    std::scoped_lock lk(user_dialogue_data_cvm_);
    this->user_dialogues_.clear();
    this->user_dialogue_state_.code = StateCode::invalid;
  }
  user_dialogue_data_cv_.notify_all();
  return this->GetState(funs, StateCode::success);
}

AudioGetUserDialogueResponse Audio::GetUserDialogue(
  const uint16_t _timeout)
{
  this->transient_state_ptr_->code = StateCode::success;
  AudioGetUserDialogueResponse ret;
  ret.state.code = StateCode::timeout;
  std::string funs = std::string(__FUNCTION__) + FORMAT("(%d) ...", _timeout);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
    return ret;
  }
  this->user_dialogue_state_.code = StateCode::timeout;
  std::unique_lock<std::mutex> lk(user_dialogue_data_cvm_);
  user_dialogue_data_cv_.wait_for(
    lk, std::chrono::seconds(_timeout), [&] {
      if ((!this->user_dialogues_.empty()) ||
      (this->user_dialogue_state_.code == StateCode::success))
      {
        // ret.response.assign(this->user_dialogues_.begin(), this->user_dialogues_.end());
        // this->user_dialogues_.clear();
        ret.response.swap(this->user_dialogues_);
        ret.state.code = StateCode::success;
        return true;
      }
      return false;
    });
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  if (ret.state.code != StateCode::success) {
    this->transient_state_ptr_->code = ret.state.code;
    this->transient_state_ptr_->describe = ret.state.describe;
  }
  return ret;
}
}   // namespace cyberdog_visual_programming_abilityset
