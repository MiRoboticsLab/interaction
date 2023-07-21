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

#include "cyberdog_vp_abilityset/train.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex training_words_state_cvm_;             /*!< 任务状态cvm */
std::condition_variable
  training_words_state_cv_;                       /*!< 任务状态cv */

bool Train::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Train::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->training_words_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->user_dialogue_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->training_words_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->training_words_sub_cb_group_;
    this->training_words_recognition_sub_ptr_ =
      this->node_immortal_ptr_->create_subscription<MsgTrainingWords>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "training_words", "train_plan_word"),
      SubscriptionQos,
      std::bind(&Train::SubTrainingWordsRecognitionCB, this, std::placeholders::_1),
      sub_option);

    sub_option.callback_group = this->user_dialogue_sub_cb_group_;
    this->user_dialogue_message_sub_ptr_ =
      this->node_immortal_ptr_->create_subscription<MsgString>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "user_dialogue", "asr_text"),
      SubscriptionSensorQos,
      std::bind(&Train::SubUserDialogueCB, this, std::placeholders::_1),
      sub_option);

    this->training_words_recognition_cli_ptr_ =
      this->node_mortal_ptr_->create_client<SrvTrainingWords>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "training_words", "query_all_train_plan"),
      ClientQos.get_rmw_qos_profile(),
      this->training_words_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Train::SubTrainingWordsRecognitionCB(const MsgTrainingWords::SharedPtr _msg_ptr)
{
  {
    // std::lock_guard<std::mutex> lk(training_words_state_cvm_);
    std::scoped_lock lk(training_words_state_cvm_);
    this->training_words_ = *_msg_ptr;
    this->training_words_update_ = true;
  }
  training_words_state_cv_.notify_all();
}

void Train::SubUserDialogueCB(const MsgString::SharedPtr _msg_ptr)
{
  for (const MsgTrainingWords & meta : this->training_words_set_.training_set) {
    if (_msg_ptr->data == meta.trigger) {
      {
        // std::lock_guard<std::mutex> lk(training_words_state_cvm_);
        std::scoped_lock lk(training_words_state_cvm_);
        this->training_words_ = meta;
        this->training_words_update_ = true;
      }
      training_words_state_cv_.notify_all();
      break;
    }
  }
}

State Train::RequestTrainingWordsRecognizedSrv(
  SrvTrainingWords::Response & _response,
  std::shared_ptr<SrvTrainingWords::Request> _request_ptr,
  const int _service_start_timeout)
{
  State ret;
  try {
    if (!rclcpp::ok()) {
      ret.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for training words recognized service to appear.",
        this->logger_.c_str());
      return ret;
    }
    if (!this->training_words_recognition_cli_ptr_->wait_for_service(
        std::chrono::seconds(
          _service_start_timeout)))
    {
      ret.code = StateCode::service_appear_timeout;
      Warn(
        "[%s] Waiting for training words recognized service to appear(start) timeout.",
        this->logger_.c_str());
      return ret;
    }
    Debug("The interface is requesting training words recognized service.");
    auto result = this->training_words_recognition_cli_ptr_->async_send_request(_request_ptr);
    std::future_status status = result.wait_for(
      std::chrono::seconds(_service_start_timeout));
    if (status != std::future_status::ready) {
      ret.code = StateCode::service_request_timeout;
      Warn(
        "[%s] Waiting for training words recognized service to response timeout.",
        this->logger_.c_str());
      return ret;
    }
    auto result_ptr = result.get();
    ret.code = StateCode::success;
    _response = *result_ptr;
    return ret;
  } catch (...) {
    ret.code = StateCode::fail;
    Warn(
      "[%s] RequestTrainRecognizedSrv() is failed.",
      this->logger_.c_str());
  }
  return ret;
}

TrainingWordsRecognizedSeviceResponse Train::GetTrainingWordsSet()
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + "()";
  TrainingWordsRecognizedSeviceResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    SrvTrainingWords::Response response;
    ret.state = this->RequestTrainingWordsRecognizedSrv(
      response,
      std::make_shared<SrvTrainingWords::Request>());
    if (ret.state.code == StateCode::success) {
      ret.response = response;
      ret.dictionary.clear();
      for (const MsgTrainingWords & meta : ret.response.training_set) {
        ret.dictionary.insert(
          std::map<std::string, MsgTrainingWords>::value_type(
            meta.trigger, meta));
      }
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] TrainRecognized() is failed. %s",
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

TrainingWordsRecognizedMessageResponse Train::TrainingWordsRecognized(
  const int _timeout)
{
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)", _timeout);
  TrainingWordsRecognizedMessageResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    this->training_words_set_ = this->GetTrainingWordsSet().response;
    int DEFAULT_TIMEOUT = 3;
    int MAX_TIMEOUT = 60 * 60;
    int MIN_TIMEOUT = 1;
    int timeout = 0;
    if (_timeout == -1) {
      timeout = DEFAULT_TIMEOUT;
    } else if (_timeout > MAX_TIMEOUT) {
      timeout = MAX_TIMEOUT;
    } else if (_timeout < MIN_TIMEOUT) {
      timeout = MIN_TIMEOUT;
    } else {
      timeout = _timeout;
    }
    {
      std::unique_lock<std::mutex> lk(training_words_state_cvm_);
      this->training_words_update_ = false;
      training_words_state_cv_.wait_for(
        lk, std::chrono::seconds(timeout), [&] {return this->training_words_update_;});
      ret.response = this->training_words_;
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] TrainRecognized() is failed. %s",
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
