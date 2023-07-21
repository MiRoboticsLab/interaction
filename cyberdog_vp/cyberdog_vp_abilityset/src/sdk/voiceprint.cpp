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

#include "cyberdog_vp_abilityset/voiceprint.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Voiceprint::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Voiceprint::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->voiceprint_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->voiceprint_sub_cb_group_;
    this->voiceprint_recognition_sub_ptr_ =
      this->node_immortal_ptr_->create_subscription<MsgString>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "ai_voiceprint", "voice_dlg_info"),
      SubscriptionQos,
      std::bind(&Voiceprint::SubVoiceprintRecognitionCB, this, std::placeholders::_1),
      sub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Voiceprint::SubVoiceprintRecognitionCB(const MsgString::SharedPtr _msg_ptr)
{
  Info("%s(%s)", std::string(__FUNCTION__).c_str(), _msg_ptr->data.c_str());
  auto search = this->voiceprint_target_time_.find(_msg_ptr->data);
  if (search != this->voiceprint_target_time_.end()) {
    search->second = std::chrono::system_clock::now();
  } else {
    this->voiceprint_target_time_.insert(
      std::map<std::string, std::chrono::time_point<std::chrono::system_clock>>::value_type(
        _msg_ptr->data,
        std::chrono::system_clock::now()));
  }
}

VoiceprintRecognizedResponse Voiceprint::Recognized(
  const int _duration,
  const int _sensitivity)
{   // 期望值
  this->transient_state_ptr_->code = StateCode::success;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d)",
    _duration,
    _sensitivity);
  VoiceprintRecognizedResponse ret;
  try {
    Info("%s", funs.c_str());
    if (this->state_.code != StateCode::success) {
      ret.state = this->GetState(funs, this->state_.code);
      this->transient_state_ptr_->code = ret.state.code;
      this->transient_state_ptr_->describe = ret.state.describe;
      return ret;
    }
    int duration = (_duration > 0) ? _duration : 3;
    int sensitivity = (_sensitivity > 0) ? _sensitivity : 1;
    auto updata_voiceprint = [&]() -> bool {
        for (const auto & [voiceprint_id, voiceprint_time] : this->voiceprint_target_time_) {
          if ((std::find(ret.list.begin(), ret.list.end(), voiceprint_id) == ret.list.end()) &&
            (std::chrono::system_clock::now() <
            (voiceprint_time + std::chrono::seconds(sensitivity))))
          {
            ret.list.push_back(voiceprint_id);
          }
        }
        ret.data = !ret.list.empty();
        return ret.data;
      };
    std::chrono::time_point<std::chrono::system_clock> timeout =
      std::chrono::system_clock::now() + std::chrono::seconds(duration);
    while (std::chrono::system_clock::now() < timeout) {
      if (updata_voiceprint()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));        // 100hz
    }
    Info("%s is ok.", funs.c_str());
  } catch (const std::exception & e) {
    Warn(
      "[%s] Voiceprint Recognized() is failed. %s",
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
