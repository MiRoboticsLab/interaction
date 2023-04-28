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

#ifndef CYBERDOG_VP_ABILITYSET__AUDIO_HPP_
#define CYBERDOG_VP_ABILITYSET__AUDIO_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       audio.hpp
    \brief      语音模块。
    \details    创建及初始化语音模块，以便任务调用语音功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Audio final : public Base
{
public:
  Audio()
  : Base(std::string(__FUNCTION__)) {}
  ~Audio() {}
  AudioPlaySeviceResponse OnlinePlay(
    const std::string message = "我是谁",
    const int8_t volume = -1);                    /*!< 在线播放音效 */
  State OnlineInstantlyPlay(
    const std::string message = "我是谁",
    const int8_t volume = -1);                    /*!< 在线立即播放音效 */
  AudioPlaySeviceResponse OfflinePlay(
    const uint16_t type = 4000,
    const int8_t volume = -1);                    /*!< 离线播放音效 */
  State OfflineInstantlyPlay(
    const uint16_t type = 4000,
    const int8_t volume = -1);                    /*!< 离线立即播放音效 */
  AudioGetVolumeSeviceResponse GetVolume();       /*!< 获取音量 */
  AudioSetVolumeSeviceResponse SetVolume(
    const uint8_t volume);                        /*!< 设置音量(0-100) */

private:
  rclcpp::CallbackGroup::SharedPtr
    play_pub_cb_group_ {nullptr};                 /*!< [回调组]请求播放 */
  rclcpp::CallbackGroup::SharedPtr
    play_cli_cb_group_ {nullptr};                 /*!< [回调组]请求播放 */
  rclcpp::CallbackGroup::SharedPtr
    get_volume_cli_cb_group_ {nullptr};           /*!< [回调组]音量获取 */
  rclcpp::CallbackGroup::SharedPtr
    set_volume_cli_cb_group_ {nullptr};           /*!< [回调组]音量设置 */

  rclcpp::Publisher<MsgAudioPlayExtend>::SharedPtr
    topic_pub_ {nullptr};                         /*!< [发布器]语音请求 */

  rclcpp::Client<SrvAudioTextPlay>::SharedPtr
    play_cli_ptr_ {nullptr};                      /*!< [客户端]数据 */
  rclcpp::Client<SrvAudioGetVolume>::SharedPtr
    get_volume_cli_ptr_ {nullptr};                /*!< [客户端]音量获取 */
  rclcpp::Client<SrvAudioSetVolume>::SharedPtr
    set_volume_cli_ptr_ {nullptr};                /*!< [客户端]音量设置 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  std::shared_ptr<SrvAudioTextPlay::Request>
  GetPlayRequest();                               /*!< 获取播放语音请求 */
  std::shared_ptr<SrvAudioSetVolume::Request>
  GetSetVolumeRequest();                          /*!< 获取设置音量请求 */
  bool RequestPlaySrv(
    AudioPlaySeviceResponse & _response,
    std::string _interface_name,
    std::shared_ptr<SrvAudioTextPlay::Request> _request_ptr,
    int _service_start_timeout = 3);              /*!< 请求播放语音服务 */
  bool RequestGetVolumeSrv(
    AudioGetVolumeSeviceResponse & _response,
    std::string _interface_name,
    int _service_start_timeout = 3);              /*!< 请求获取音量服务 */
  bool RequestSetVolumeSrv(
    AudioSetVolumeSeviceResponse & _response,
    std::string _interface_name,
    std::shared_ptr<SrvAudioSetVolume::Request> _request_ptr,
    int _service_start_timeout = 3);              /*!< 请求设置音量服务 */
};  // class Audio
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__AUDIO_HPP_
