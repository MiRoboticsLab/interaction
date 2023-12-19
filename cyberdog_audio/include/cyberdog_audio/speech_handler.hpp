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
#ifndef CYBERDOG_AUDIO__SPEECH_HANDLER_HPP_
#define CYBERDOG_AUDIO__SPEECH_HANDLER_HPP_

// #include <map>
// #include <functional>
#include <vector>
#include <utility>
#include <memory>
#include <string>
#include <map>
#include <functional>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_audio/instruction_def.hpp"
#include "audio_lcm/lcm_data.hpp"
#include "xpack/json.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "protocol/action/speech.hpp"


namespace cyberdog
{
namespace interaction
{
struct speech_data
{
  std::string name;
  XPACK(O(name))
};
struct tts_text
{
  std::string text;
  XPACK(O(text))
};
struct http_tts_url
{
  std::string url;
  XPACK(O(url))
};
struct http_music_url
{
  std::vector<std::string> urls;
  XPACK(O(urls))
};

enum class SpeechRunCode : int8_t
{
  srcEnded = 0,
  srcStarted = 1,
  srcRunning = 2
};  // enum class SpeechRunCode

class SpeechActionServer : public rclcpp::Node
{
public:
  using SpeechAction = protocol::action::Speech;
  using GoalHandleSpeech = rclcpp_action::ServerGoalHandle<SpeechAction>;

  explicit SpeechActionServer(
    std::function<bool(void)> gaol_func,
    std::function<bool(void)> exec_func,
    std::function<void(SpeechRunCode)> set_func,
    std::function<void(const std::shared_ptr<GoalHandleSpeech>)> goal_func,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("speech_action_server", options),
    goal_condition_func_(gaol_func),
    exec_condition_func_(exec_func),
    set_condition_func_(set_func),
    goal_play_func_(goal_func)
  {
    this->action_server_ = rclcpp_action::create_server<SpeechAction>(
      this,
      "speech_text_action",
      std::bind(
        &SpeechActionServer::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&SpeechActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SpeechActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<SpeechAction>::SharedPtr action_server_;
  std::function<bool(void)> goal_condition_func_;
  std::function<bool(void)> exec_condition_func_;
  std::function<void(SpeechRunCode)> set_condition_func_;
  std::function<void(const std::shared_ptr<GoalHandleSpeech>)> goal_play_func_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SpeechAction::Goal> goal)
  {
    INFO(
      "Received goal request with module name %s, onlie:%s",
      goal->module_name.c_str(), (goal->is_online == true ? "true" : "false"));
    (void)uuid;
    if (goal_condition_func_()) {
      return rclcpp_action::GoalResponse::REJECT;
    } else {
      set_condition_func_(SpeechRunCode::srcStarted);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSpeech> goal_handle)
  {
    (void)goal_handle;
    if (goal_handle->is_executing()) {
      INFO("Received request to cancel goal, reject!");
      return rclcpp_action::CancelResponse::REJECT;
    } else {
      INFO("No goal can cancel.");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSpeech> goal_handle)
  {
    goal_play_func_(goal_handle);
    std::thread{std::bind(&SpeechActionServer::execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSpeech> goal_handle)
  {
    INFO("Executing goal");
    rclcpp::Rate loop_rate(50);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SpeechAction::Feedback>();
    feedback->talking = true;
    auto result = std::make_shared<SpeechAction::Result>();
    auto start = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok() ) {
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      if (elapsed.count() > 15000) {
        WARN("Executing goal exception:over time.");
        if (rclcpp::ok()) {
          result->status = SpeechAction::Result::SPEECH_RESULT_FAILED;
          goal_handle->succeed(result);
          INFO("Goal succeeded");
        }
        set_condition_func_(SpeechRunCode::srcEnded);
        return;
      }
      if (!exec_condition_func_()) {
        goal_handle->publish_feedback(feedback);
      } else {
        break;
      }
      loop_rate.sleep();
    }
    if (rclcpp::ok()) {
      result->status = SpeechAction::Result::SPEECH_RESULT_SUCCESS;
      goal_handle->succeed(result);
      INFO("Goal succeeded");
    }
  }
};

class SpeechHandler final
{
public:
  SpeechHandler(
    std::function<void(const std::shared_ptr<SpeechActionServer::GoalHandleSpeech>)> goal_func)
  : exec_src(SpeechRunCode::srcEnded)
  {
    action_server_ptr = std::make_shared<SpeechActionServer>(
      std::bind(&SpeechHandler::is_running, this),
      std::bind(&SpeechHandler::is_over, this),
      std::bind(&SpeechHandler::SetState, this, std::placeholders::_1),
      goal_func);
    t_spin = std::thread(
      [this]() {
        rclcpp::spin(action_server_ptr);
      });
    t_spin.detach();
  }
  // 录音文件播放
  std::shared_ptr<audio_lcm::lcm_data> Play(const uint16_t & id)
  {
    std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
    speech_data s_d;
    auto play_iter = play_map.find(id);
    if (play_iter == play_map.end()) {
      WARN("no play id %d file", id);
    } else {
      s_d.name = play_map.at(id);
      l_d->cmd = PLAY;
      l_d->data = xpack::json::encode(s_d);
    }
    return l_d;
  }

  std::shared_ptr<audio_lcm::lcm_data> Play(const std::string & text)
  {
    std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
    tts_text t_t;
    t_t.text = text;
    l_d->cmd = TTS_PLAY;
    l_d->data = xpack::json::encode(t_t);
    return l_d;
  }

  std::shared_ptr<audio_lcm::lcm_data> PlayCancel()
  {
    std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
    l_d->cmd = PLAY_CANCEL;
    return l_d;
  }

  std::shared_ptr<audio_lcm::lcm_data> HttpPlay(const std::string & url)
  {
    std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
    http_tts_url h_t_u;
    h_t_u.url = url;
    l_d->cmd = HTTP_TTS_PLAY;
    l_d->data = xpack::json::encode(h_t_u);
    return l_d;
  }

  std::shared_ptr<audio_lcm::lcm_data> HttpPlay(const std::vector<std::string> & urls)
  {
    std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
    http_music_url h_m_u;
    h_m_u.urls = std::move(urls);
    l_d->cmd = HTTP_MUSIC_PLAY;
    l_d->data = xpack::json::encode(h_m_u);
    return l_d;
  }

  void SetState(SpeechRunCode src)
  {
    std::unique_lock<std::mutex> lck(exec_mtx);
    exec_src = src;
  }

private:
  bool is_running()
  {
    std::unique_lock<std::mutex> lck(exec_mtx);
    return exec_src != SpeechRunCode::srcEnded;
  }

  bool is_over()
  {
    std::unique_lock<std::mutex> lck(exec_mtx);
    return exec_src == SpeechRunCode::srcEnded;
  }

private:
  /* data */
  // std::map<uint32_t, std::function<void(uint32_t)>> tts_map;
  const std::map<uint16_t, std::string> play_map = {
    {1, "wifi_enter_connect_mode"},
    {2, "wifi_enable_connect_mode"},
    {3, "wifi_wait_scan_code"},
    {4, "wifi_scan_code_succeed"},
    {5, "wifi_connection_succeed0"},
    {6, "wifi_connection_succeed1"},
    {7, "wifi_connection_failed0"},
    {8, "wifi_connection_failed1"},
    {9, "wifi_connection_failed2"},
    {10, "wifi_exit_connection_mode"},
    {11, "wifi_offline"},
    {12, "wifi_communication"},
    {13, "wifi_scan_code_ip_error"},
    {14, "wifi_scan_code_info_error"},
    // {15, "wifi_request_open_camera_success"},
    // {16, "wifi_request_open_camera_fail"},
    // {17, "wifi_request_close_camera_success"},
    {15, "is_connecting"},
    {16, "wait_connect_network"},
    {17, "connect_network_failed_retry"},
    {18, "wifi_request_close_camera_fail"},
    {21, "face_entry_add_face"},
    {22, "face_entry_cancel_add_face"},
    {23, "face_entry_confirm_last_face"},
    {24, "face_entry_update_face"},
    {25, "face_entry_delete_face"},
    {26, "face_entry_get_all_faces"},
    {27, "face_entry_fix_pose"},
    {28, "face_entry_fix_pose_left"},
    {29, "face_entry_fix_pose_right"},
    {30, "face_entry_fix_pose_up"},
    {31, "face_entry_fix_pose_down"},
    {32, "face_entry_fix_distance_close"},
    {33, "face_entry_fix_distance_near"},
    {34, "face_entry_fix_stable"},
    {35, "face_entry_mutiple_faces"},
    {36, "face_entry_none_faces"},
    {37, "face_entry_timeout"},
    {38, "face_entry_finish"},
    {39, "face_recognition_request"},
    {40, "face_degree_head_tilt"},
    {41, "face_recognition_finsish"},
    {42, "face_recogniton_timeout"},
    {43, "face_already_exist"},
    {50, "camera_start_pic_transfer"},
    {51, "camera_start_photos"},
    {52, "camera_take_videos"},
    {53, "camera_video_recordig"},
    {60, "self_check_success"},
    {61, "self_check_failed"},
    {124, "ai_please_enable"},
    {125, "ai_enable_success"},
    {127, "ai_service_expired"},
    {3000, "test_hardware_audio"},
    {3001, "test_stage_one"},
    {3003, "test_stage_three"},
    {4000, "dog_woof_7"},
    {4001, "dog_woof_100"},
    {6000, "barns_courtney"},
    {6001, "thats_what"},
    {9000, "bootup"},
  };
  std::shared_ptr<SpeechActionServer> action_server_ptr;
  std::thread t_spin;
  SpeechRunCode exec_src;
  std::mutex exec_mtx;
};

}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__SPEECH_HANDLER_HPP_
