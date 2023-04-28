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
#ifndef CYBERDOG_AUDIO__PARAMETERS_INFO_HPP_
#define CYBERDOG_AUDIO__PARAMETERS_INFO_HPP_

#include <mutex>
#include <atomic>
#include <string>
#include <memory>
#include <condition_variable>
#include "cyberdog_audio/instruction_def.hpp"
#include "cyberdog_audio/speech_handler.hpp"
#include "cyberdog_audio/audio_state.hpp"

namespace cyberdog
{
namespace interaction
{
class ParametersInfo
{
public:
  std::atomic_bool is_wifi_connected;
  TokenInfo auth_ti;
  std::string cyberdog_sn;
  bool is_authenticated;
  std::string triple_mac;
  std::string triple_did;
  std::string triple_key;
  DogInfo personal_info;
  std::mutex play_mtx_;
  bool is_play_;
  std::condition_variable play_cv_;
  std::shared_ptr<SpeechHandler> speech_handler_ptr_;
  bool token_invalid_;
  uint32_t recv_msg_cnt;
  SelfCheckState check_state;
  AudioState audio_state;
  uint32_t failed_times;

private:
  static std::mutex mutex_;
  static ParametersInfo * parameters_info_;

public:
  static ParametersInfo * & GetInstance();
  static void DeleteInstance();

private:
  ParametersInfo(/* args */);
  ~ParametersInfo();
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AUDIO__PARAMETERS_INFO_HPP_
