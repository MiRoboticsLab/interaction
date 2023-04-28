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

#ifndef CYBERDOG_VP_ABILITYSET__VOICEPRINT_HPP_
#define CYBERDOG_VP_ABILITYSET__VOICEPRINT_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       gesture.hpp
    \brief      声纹识别模块。
    \details    创建及初始化声纹识别模块，以便任务调用声纹识别功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Voiceprint final : public Base
{
public:
  Voiceprint()
  : Base(std::string(__FUNCTION__)) {}
  ~Voiceprint() {}
  VoiceprintRecognizedResponse Recognized(
    const int _duration = -1,
    const int _sensitivity = 1);                  /*!< 识别到声纹 */

private:
  std::map<
    std::string /* name */,
    std::chrono::time_point<std::chrono::system_clock>/*时间*/>
  voiceprint_target_time_;                        /*!< 声纹识别目标时间 */

  rclcpp::CallbackGroup::SharedPtr
    voiceprint_sub_cb_group_ {nullptr};           /*!< [回调组]声纹识别响应 */

  rclcpp::Subscription<MsgString>::SharedPtr
    voiceprint_recognition_sub_ptr_ {nullptr};    /*!< [监听器]声纹识别响应 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubVoiceprintRecognitionCB(
    const MsgString::SharedPtr);                  /*!< 声纹识别响应数据回调 */
};  // class Voiceprint
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__VOICEPRINT_HPP_
