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
#ifndef CYBERDOG_AUDIO__AUDIO_PLAY_HPP_
#define CYBERDOG_AUDIO__AUDIO_PLAY_HPP_

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <condition_variable>
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_audio/speech_handler.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"

namespace cyberdog
{
namespace interaction
{
struct play_sound_info
{
  bool is_online {false};
  bool is_music {false};
  uint16_t play_id;
  std::string play_text;
  // std::string url;
  // std::vector<std::string> urls;
};
class AudioPlay
{
public:
  AudioPlay(
    std::function<void(const std::shared_ptr<SpeechActionServer::GoalHandleSpeech>)> goal_func)
  {
    if (access(SOUND_DIRECTORY.c_str(), F_OK) == 0) {
      std::string config_toml_file = SOUND_DIRECTORY + "/yaml/sound.toml";
      if (access(config_toml_file.c_str(), F_OK) == 0) {
        toml::value base_val;
        if (common::CyberdogToml::ParseFile(config_toml_file, base_val)) {
          INFO("Audio Play parse config file %s success.", config_toml_file.c_str());
          toml::value config;
          if (common::CyberdogToml::Get(base_val, "config", config)) {
            INFO("Audio Play parse config field success.");
            std::vector<uint16_t> tts_ids_vec;
            std::vector<std::string> tts_names_vec;
            if (common::CyberdogToml::Get(config, "tts_ids", tts_ids_vec) &&
              common::CyberdogToml::Get(config, "tts_names", tts_names_vec))
            {
              INFO("Audio Play parse tts ids and tts names array success.");
              if (tts_ids_vec.size() == tts_names_vec.size()) {
                size_t len = tts_ids_vec.size();
                for (size_t i = 0; i < len; i++) {
                  std::string file_path = SOUND_DIRECTORY + "/" + tts_names_vec[i];
                  std::string http_file_path = SOUND_HTTP_DIRECTORY + "/" + tts_names_vec[i];
                  if (access(file_path.c_str(), F_OK) == 0) {
                    http_play_map.insert(
                      std::map<uint16_t, std::string>::value_type(tts_ids_vec[i], http_file_path));
                  } else {
                    ERROR("%s does not exist!", file_path.c_str());
                  }
                }
              } else {
                WARN("Audio Play tts ids size and tts names size not equal, please check!");
              }
            }
            std::vector<uint16_t> music_ids_vec;
            std::vector<std::string> music_names_vec;
            if (common::CyberdogToml::Get(config, "music_ids", music_ids_vec) &&
              common::CyberdogToml::Get(config, "music_names", music_names_vec))
            {
              INFO("Audio Play parse music ids and music names array success.");
              if (music_ids_vec.size() == music_names_vec.size()) {
                size_t len = music_ids_vec.size();
                for (size_t i = 0; i < len; i++) {
                  std::string file_path = SOUND_DIRECTORY + "/" + music_names_vec[i];
                  std::string http_file_path = SOUND_HTTP_DIRECTORY + "/" +
                    music_names_vec[i];
                  if (access(file_path.c_str(), F_OK) == 0) {
                    http_music_map.insert(
                      std::map<uint16_t, std::string>::value_type(
                        music_ids_vec[i],
                        http_file_path));
                  } else {
                    ERROR("%s does not exist!", file_path.c_str());
                  }
                }
              } else {
                WARN("Audio Play music ids size and music names size not equal, please check!");
              }
            }
          }
        }
      }
    }
    speech_handler_ptr_ = std::make_shared<SpeechHandler>(goal_func);
    audio_notify_pub_ =
      get_nodify_node_->create_publisher<std_msgs::msg::Bool>(
      "audio_notification_report",
      rclcpp::SystemDefaultsQoS());
    std::thread([this]() {rclcpp::spin(get_nodify_node_);}).detach();
  }
  ~AudioPlay() = default;
  std::shared_ptr<audio_lcm::lcm_data> SoundPlay(play_sound_info & psi)
  {
    if (psi.is_online) {
      return speech_handler_ptr_->Play(psi.play_text);
    }
    if (psi.play_id >= 60000) {
      psi.is_music = true;
    }
    if (!psi.is_music) {
      auto play_iter = http_play_map.find(psi.play_id);
      if (play_iter != http_play_map.end()) {
        std::string url = play_iter->second;
        return speech_handler_ptr_->HttpPlay(url);
      } else {
        return SpeechPlay(psi.play_id);
      }
    }
    auto music_iter = http_music_map.find(psi.play_id);
    if (music_iter != http_music_map.end()) {
      std::string url = music_iter->second;
      std::vector<std::string> urls;
      urls.push_back(url);
      return speech_handler_ptr_->HttpPlay(urls);
    }
    return speech_handler_ptr_->Play("未找到音乐");
  }
  void StartPlay()
  {
    std::unique_lock<std::mutex> lck(play_mtx_);
    is_play_ = true;
    play_cv_.notify_all();
  }
  void StopPlay()
  {
    std::unique_lock<std::mutex> lck(play_mtx_);
    is_play_ = false;
    play_cv_.notify_all();
  }
  bool WaitPlay()
  {
    std::unique_lock<std::mutex> lck(play_mtx_);
    if (play_cv_.wait_for(
        lck, std::chrono::seconds(10), [this]() -> bool {
          return (is_play_ == false) || (!rclcpp::ok());
        }) == false)
    {
      return false;
    }
    return true;
  }
  bool IsSdcardHaveId(uint16_t & play_id)
  {
    auto play_iter = http_play_map.find(play_id);
    if (play_iter != http_play_map.end()) {
      return true;
    }
    return false;
  }

public:
  void PlayNotify(const std::string & data)
  {
    // 录音文件播放通知
    play_notify_info pni;
    xpack::json::decode(data, pni);
    INFO("play notify name:%s, status:%d", pni.name.c_str(), pni.status);
    UploadPlayNotify(pni.status);
    Notify(pni.status);
  }
  void HttpPlayNotify(const std::string & data)
  {
    http_play_notify_info hpni;
    xpack::json::decode(data, hpni);
    INFO("http play notify name:%s, status:%d", hpni.name.c_str(), hpni.status);
    Notify(hpni.status);
  }

private:
  std::shared_ptr<audio_lcm::lcm_data> SpeechPlay(uint16_t & play_id)
  {
    if (play_id == 9999) {
      return speech_handler_ptr_->PlayCancel();
    } else {
      return speech_handler_ptr_->Play(play_id);
    }
  }
  void Notify(uint8_t status)
  {
    if (status == 1) {
      {
        std::unique_lock<std::mutex> lck(play_mtx_);
        is_play_ = true;
        play_cv_.notify_all();
      }
      speech_handler_ptr_->SetState(SpeechRunCode::srcRunning);
    } else if (status == 0) {
      {
        std::unique_lock<std::mutex> lck(play_mtx_);
        is_play_ = false;
        play_cv_.notify_all();
      }
      speech_handler_ptr_->SetState(SpeechRunCode::srcEnded);
    } else {
      WARN("unimplement play notify state:%d", status);
    }
  }
  void UploadPlayNotify(uint8_t status)
  {
    std_msgs::msg::Bool msg;
    msg.data = (status == 1) ? true : false;
    audio_notify_pub_->publish(msg);
  }

private:
  rclcpp::Node::SharedPtr get_nodify_node_ =
    std::make_shared<rclcpp::Node>("get_notify_node");
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr audio_notify_pub_;
  std::shared_ptr<SpeechHandler> speech_handler_ptr_;
  std::mutex play_mtx_;
  bool is_play_{false};
  std::condition_variable play_cv_;
  std::map<uint16_t, std::string> http_play_map;
  std::map<uint16_t, std::string> http_music_map;
  const std::string SOUND_DIRECTORY = "/SDCARD/sound";
  const std::string SOUND_HTTP_DIRECTORY = "http://192.168.44.1:8001/";
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__AUDIO_PLAY_HPP_
