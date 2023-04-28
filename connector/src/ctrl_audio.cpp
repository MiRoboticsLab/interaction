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
#include "connector/ctrl_audio.hpp"

namespace cyberdog
{
namespace interaction
{
CtrlAudio::CtrlAudio(const std::string name)
: Node(name)
{
  INFO("Creating [CtrlAudio] object(node)");
  if (!this->Init()) {exit(-1);}
}

CtrlAudio::~CtrlAudio()
{
  INFO("Destroy [CtrlAudio] object(node)");
}

bool CtrlAudio::Init()
{
  try {
    INFO("Initializing data ...");
    toml::value params_toml_;
    std::string params_pkg_dir_ = ament_index_cpp::get_package_share_directory("connector");
    std::string node_config_dir_ = params_pkg_dir_ + "/config/connector.toml";
    INFO("Params config file dir:<%s>", node_config_dir_.c_str());
    if (access(node_config_dir_.c_str(), F_OK)) {
      ERROR("Params config file does not exist");
      return false;
    }
    if (access(node_config_dir_.c_str(), R_OK)) {
      ERROR("Params config file does not have read permissions");
      return false;
    }
    if (access(node_config_dir_.c_str(), W_OK)) {
      ERROR("Params config file does not have write permissions");
      return false;
    }
    if (!cyberdog::common::CyberdogToml::ParseFile(
        node_config_dir_.c_str(), params_toml_))
    {
      ERROR("Params config file is not in toml format");
      return false;
    }
    this->timer_05hz_ = this->create_wall_timer(
      std::chrono::seconds(static_cast<int64_t>(5)),
      std::bind(&CtrlAudio::Timer_05hz, this));
    this->timer_05hz_->cancel();
    this->topic_pub_ = this->create_publisher<AudioMsg>(
      toml::find<std::string>(
        params_toml_, "connector", "initialization", "topic", "audio"),
      10);
    this->server_cli_ = this->create_client<AudioSrv>(
      toml::find<std::string>(
        params_toml_, "connector", "initialization", "service", "audio"));
    this->action_cli_ = rclcpp_action::create_client<AudioAct>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      toml::find<std::string>(
        params_toml_, "connector", "initialization", "action", "audio"));
    this->send_goal_options_.goal_response_callback =
      std::bind(&CtrlAudio::GoalResponseCallback, this, std::placeholders::_1);
    this->send_goal_options_.feedback_callback =
      std::bind(&CtrlAudio::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    this->send_goal_options_.result_callback =
      std::bind(&CtrlAudio::ResultCallback, this, std::placeholders::_1);
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

void CtrlAudio::Timer_05hz()
{
  this->RequestTopic(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
}

void CtrlAudio::GoalResponseCallback(GoalHandleAudioAct::SharedPtr goal_handle)
{
  if (!goal_handle) {
    ERROR("Goal was rejected by server");
  } else {
    INFO("Goal accepted by server, waiting for result");
  }
}

void CtrlAudio::FeedbackCallback(
  GoalHandleAudioAct::SharedPtr,
  const std::shared_ptr<const AudioAct::Feedback> feedback_ptr)
{
  INFO(
    "[%s] playing voice...",
    std::string(feedback_ptr->talking ? "True" : "False").c_str());
}

void CtrlAudio::ResultCallback(const GoalHandleAudioAct::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("Goal was canceled");
      return;
    default:
      ERROR("Unknown result code");
      return;
  }
  INFO(
    "Result received: %d (%s)", result.result->status,
    std::string(result.result->status ? "False" : "True").c_str());
}

uint CtrlAudio::RequestTopic(uint16_t _id)
{
  // std::unique_lock<std::mutex> unlk(topic_mutex_);
  AudioMsg audio_msg;
  audio_msg.module_name = "connector";
  audio_msg.play_id = _id;
  this->topic_pub_->publish(audio_msg);
  return 0;
}

uint CtrlAudio::RequestServer(uint16_t _id)
{
  auto request = std::make_shared<AudioSrv::Request>();
  request->module_name = "connector";
  request->is_online = false;
  request->speech.module_name = "connector";
  request->speech.play_id = _id;
  request->text = "";
  if (!rclcpp::ok()) {
    WARN("Client interrupted while requesting for service to appear.");
    return 1;
  }
  if (!this->server_cli_->wait_for_service(std::chrono::seconds(3))) {
    WARN("Waiting for service to appear(start) timeout.");
    return 2;
  }
  auto result = this->server_cli_->async_send_request(request);
  std::future_status status =
    result.wait_for(std::chrono::seconds(5));
  if (status != std::future_status::ready) {
    WARN("Request audio module timedout or deferred.");
    return 3;
  }
  auto response_ptr = result.get();
  if (response_ptr->status != 0) {
    WARN(
      "Control audio module failed, status = %d", static_cast<int>(response_ptr->status));
    return 4;
  }
  return 0;
}

uint CtrlAudio::RequestAction(uint16_t _id)
{
  // auto request = std::make_shared<LedSrv::Request>();
  // // request->client = this->name_;
  // // request->target = toml::find<std::string>(led_meta, "target");
  // // request->effect = toml::find<std::string>(led_meta, "effect");
  // // request->priority = toml::find<int>(led_meta, "priority");
  // // request->timeout = toml::find<int>(led_meta, "timeout");

  // if (!rclcpp::ok()) {
  //   WARN("Client interrupted while requesting for audio action service to appear.");
  //   return 1;
  // }
  // if (!this->action_cli_->wait_for_action_server(std::chrono::seconds(3))) {
  //   WARN("Waiting for audio action service to appear(start) timeout.");
  //   return 2;
  // }
  // auto result = this->action_cli_->async_send_request(request);
  // std::future_status status =
  //   result.wait_for(std::chrono::seconds(3));
  // if (status != std::future_status::ready) {
  //   WARN("Request audio module timedout or deferred.");
  //   return 3;
  // }
  // auto response_ptr = result.get();
  return _id;
}

uint CtrlAudio::ControlAudio(uint16_t _id)
{
  // PID_WIFI_ENTER_CONNECTION_MODE_0 开启配网功能
  // PID_WIFI_FAILED_PLEASE_RETRY     启动失败请重试
  // PID_WIFI_WAIT_FOR_SCAN_CODE_0    等待扫描二维码(1/5s)
  // PID_WIFI_SCAN_CODE_SUCCEEDED_0   扫码成功，网络连接中
  // PID_WIFI_CONNECTION_SUCCEEDED_0  连网成功
  // PID_WIFI_CONNECTED_UNKNOWN_NET   已连接无密码的网络
  // PID_WIFI_SCAN_CODE_IP_ERROR      二维码信息错误，请使用正确二维码
  // PID_WIFI_CONNECTION_FAILED_0     无线网络名称错误，请修改后重试
  // PID_WIFI_CONNECTION_FAILED_1     无线网络密码错误，请修改后重试
  // PID_WIFI_CONNECTION_FAILED_2     无法连接网络，请检查网络状况并重新尝试
  // PID_WIFI_SCAN_CODE_INFO_ERROR    二维码失效，请重新生成
  // PID_WIFI_EXIT_CONNECTION_MODE_0  关闭配网功能
  // /*
  //  * return:
  //  * 0: 控制 Audio 成功
  //  * 1: 客户端在请求服务出现时被打断
  //  * 2: 等待服务出现（启动）超时
  //  * 3: 请求相机模块超时或延迟
  //  * 4: 控制 Audio 失败
  //  */
  INFO("Control audio <%d>", _id);
  if (_id == AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0) {
    this->timer_05hz_->reset();
  } else if (!this->timer_05hz_->is_canceled()) {
    this->timer_05hz_->cancel();
  }
  // return this->RequestTopic(_id);
  return this->RequestServer(_id);
  // return this->RequestAction(_id);
}

}   // namespace interaction
}   // namespace cyberdog
