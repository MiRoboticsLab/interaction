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
#ifndef CONNECTOR__CTRL_AUDIO_HPP_
#define CONNECTOR__CTRL_AUDIO_HPP_

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>

#include <protocol/msg/audio_play.hpp>
#include <protocol/srv/audio_text_play.hpp>
#include <protocol/action/speech.hpp>

#include <string>
#include <memory>

namespace cyberdog
{
namespace interaction
{
/*! \file       ctrl_audio.hpp
    \brief      语音控制模块。
    \details    创建及初始化语音控制模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class CtrlAudio final : public rclcpp::Node
{
  using AudioMsg = protocol::msg::AudioPlay;            /*!< [topic 类型]语音消息 */
  using AudioSrv = protocol::srv::AudioTextPlay;        /*!< [topic 类型]语音服务 */
  using AudioAct = protocol::action::Speech;            /*!< [action 类型]语音动作 */
  using GoalHandleAudioAct = rclcpp_action::ClientGoalHandle<AudioAct>;

public:
  explicit CtrlAudio(const std::string name);
  ~CtrlAudio();
  uint ControlAudio(uint16_t _command);                 /*!< 控制语音 */

private:
  bool Init();                                          /*!< 初始化 */
  void GoalResponseCallback(
    GoalHandleAudioAct::SharedPtr);                     /*!< 请求回调 */
  void FeedbackCallback(
    GoalHandleAudioAct::SharedPtr,
    const std::shared_ptr<const AudioAct::Feedback>);   /*!< 反馈回调 */
  void ResultCallback(
    const GoalHandleAudioAct::WrappedResult &);         /*!< 结果回调 */
  void Timer_05hz();                                    /*!< 定时器 */
  uint RequestTopic(uint16_t);                          /*!< 请求 topic */
  uint RequestServer(uint16_t);                         /*!< 请求 server */
  uint RequestAction(uint16_t);                         /*!< 请求 action */

private:
  rclcpp::TimerBase::SharedPtr timer_05hz_ {nullptr};   /*!< [定时器] */
  std::mutex topic_mutex_;                              /*!< 语音互斥锁 */
  rclcpp::Publisher<AudioMsg>::SharedPtr
    topic_pub_ {nullptr};                               /*!< [发布器]语音请求 */
  rclcpp::Client<AudioSrv>::SharedPtr
    server_cli_ {nullptr};                              /*!< [客户端]语音 server 驱动 */
  rclcpp_action::Client<AudioAct>::SharedPtr
    action_cli_ {nullptr};                              /*!< [客户端]语音 action 驱动 */
  rclcpp_action::Client<AudioAct>::SendGoalOptions
    send_goal_options_;                                 /*!< 目标选项 */
};  // class CtrlAudio
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__CTRL_AUDIO_HPP_
