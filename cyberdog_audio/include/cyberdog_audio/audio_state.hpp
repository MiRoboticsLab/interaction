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
#ifndef CYBERDOG_AUDIO__AUDIO_STATE_HPP_
#define CYBERDOG_AUDIO__AUDIO_STATE_HPP_

#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <map>
#include "cyberdog_common/cyberdog_log.hpp"
#include "xpack/json.h"

namespace cyberdog
{
namespace interaction
{
struct check_status
{
  int8_t status;
  int32_t counter;
  XPACK(O(status, counter))
};
enum class SelfCheckState : int8_t
{
  SCSUnReady = -3,
  SCSUnkown = -2,
  SCSError = -1,
  SCSNormal = 0,
  SCSUnBindServer = 1,
  SCSUnInBindingServer = 2,
};  // enum class SelfCheckState

enum class AudioWorkState : uint8_t
{
  AWSNormal = 0,
  AWSOffMic = 1,
  AWSSilentSpeaker = 2,
  AWSUnkown = 255,
};  // enum class AudioWorkState

using SELF_CHECK_CALLBACK = std::function<void (SelfCheckState &)>;

class AudioState
{
public:
  AudioState(/* args */)
  : sound_volume(-1),
    self_check_status(SelfCheckState::SCSUnkown),
    audio_work_status(AudioWorkState::AWSUnkown)
  {
  }
  bool RegisterNotice(const SELF_CHECK_CALLBACK & register_function)
  {
    notify_callbacks_.push_back(register_function);
    return true;
  }
  bool SetSelfCheckState(const std::string & data)
  {
    check_status c_s;
    c_s.status = static_cast<int8_t>(SelfCheckState::SCSUnkown);
    xpack::json::decode(data, c_s);
    SelfCheckState asc = SelfCheckState(c_s.status);
    auto iter = self_check_state_code_map.find(asc);
    if (iter == self_check_state_code_map.end()) {
      asc = SelfCheckState::SCSUnkown;
      ERROR("unkown audio board self-check state:%d, set it unkown state", c_s.status);
    }
    if (asc != self_check_status) {
      self_check_status = asc;
      INFO(
        "audio board now in %s self-check state", self_check_state_code_map.at(
          self_check_status).c_str());
      for (auto & callback_ : notify_callbacks_) {
        callback_(self_check_status);
      }
    }
    return true;
  }
  std::string GetSelfCheckState()
  {
    return self_check_state_code_map.at(self_check_status);
  }
  bool GetActivateState()
  {
    return self_check_status == SelfCheckState::SCSNormal;
  }
  void SetAudioWorkState(const AudioWorkState & code)
  {
    AudioWorkState aws = code;
    auto iter = audio_work_state_code_map.find(aws);
    if (iter == audio_work_state_code_map.end()) {
      aws = AudioWorkState::AWSUnkown;
      ERROR("unkown audio board work state:%d, set it unkown work state", (int)code);
    }
    if (aws != audio_work_status) {
      audio_work_status = aws;
      INFO(
        "audio board now in %s work state",
        audio_work_state_code_map.at(audio_work_status).c_str());
      // for (auto & callback_ : notify_callbacks_) {
      //   callback_(self_check_status);
      // }
    }
  }
  void SetSoundVolume(int8_t val)
  {
    sound_volume = val;
  }

  void GetAudioWorkState(const AudioWorkState & code)
  {
    if (code != audio_work_status) {
      WARN(
        "audio board in %s work state, but record %s work state. sync it.",
        audio_work_state_code_map.at(code).c_str(),
        audio_work_state_code_map.at(audio_work_status).c_str());
      audio_work_status = code;
    }
  }

  bool GetAudioWorkState()
  {
    return audio_work_status == AudioWorkState::AWSNormal ? true : false;
  }

  int8_t GetSoundVolume()
  {
    return sound_volume;
  }

private:
  /* data */
  int8_t sound_volume;
  SelfCheckState self_check_status;
  AudioWorkState audio_work_status;
  std::vector<SELF_CHECK_CALLBACK> notify_callbacks_;
  const std::map<SelfCheckState, std::string> self_check_state_code_map = {
    {SelfCheckState::SCSError, "error"},
    {SelfCheckState::SCSUnReady, "unready"},
    {SelfCheckState::SCSUnkown, "unkown"},
    {SelfCheckState::SCSNormal, "normal"},
    {SelfCheckState::SCSUnBindServer, "unbind-server"},
    {SelfCheckState::SCSUnInBindingServer, "in-binding-server"}
  };
  const std::map<AudioWorkState, std::string> audio_work_state_code_map = {
    {AudioWorkState::AWSNormal, "normal-speaker"},
    // {AudioWorkState::AWSSilentSpeaker, "slient-speaker"},
    {AudioWorkState::AWSOffMic, "off-mic"},
    {AudioWorkState::AWSUnkown, "unkown"}
  };
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__AUDIO_STATE_HPP_
