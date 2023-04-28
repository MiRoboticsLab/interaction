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

#ifndef CYBERDOG_VP_ABILITYSET__GESTURE_HPP_
#define CYBERDOG_VP_ABILITYSET__GESTURE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       gesture.hpp
    \brief      手势识别模块。
    \details    创建及初始化手势识别模块，以便任务调用手势识别功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Gesture final : public Base
{
public:
  Gesture()
  : Base(std::string(__FUNCTION__)) {}
  ~Gesture() {}

  GestureRecognizedMessageResponse Recognized(
    const int _duration = -1,
    const int _sensitivity = 1);                  /*!< 开始识别并识别到任意手势 */

  GestureRecognizedSeviceResponse TurnOnRecognition(
    const int _duration = -1);                    /*!< 打开识别手势功能 */
  GestureRecognizedSeviceResponse TurnOffRecognition(
  );                                              /*!< 关闭识别手势功能 */
  GestureRecognizedMessageResponse RecognizedDesignatedGesture(
    const int timeout = -1,
    const int gesture_type = 0);                  /*!< 识别到指定手势 */
  GestureRecognizedMessageResponse RecognizedAnyGesture(
    const int timeout = -1,
    const int sensitivity = 1);                   /*!< 识别到任意手势 */

private:
  bool gesture_recognition_life_cycle_ {false};   /*!< 手势识别生命周期内 */
  std::map<
    int /*id*/,
    std::chrono::time_point<std::chrono::system_clock>/*时间*/>
  gesture_target_time_;                           /*!< 手势识别目标时间 */

  rclcpp::CallbackGroup::SharedPtr
    gesture_sub_cb_group_ {nullptr};              /*!< [回调组]手势识别响应 */
  rclcpp::CallbackGroup::SharedPtr
    gesture_cli_cb_group_ {nullptr};              /*!< [回调组]手势识别服务 */

  rclcpp::Subscription<MsgGesture>::SharedPtr
    gesture_recognition_sub_ptr_ {nullptr};       /*!< [监听器]手势识别响应 */
  rclcpp::Client<SrvGesture>::SharedPtr
    gesture_recognition_cli_ptr_ {nullptr};       /*!< [客户端]手势识别服务 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubGestureRecognitionCB(
    const MsgGesture::SharedPtr);                 /*!< 手势识别响应数据回调 */
  std::shared_ptr<SrvGesture::Request>
  GetGestureRecognitionRequest();                 /*!< 获取手势识别服务请求 */
  bool RequestGestureRecognizedSrv(
    SrvGesture::Response &,
    std::shared_ptr<SrvGesture::Request>,
    const int _service_start_timeout = 10);       /*!< 请求手势识别服务 */
};  // class Gesture
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__GESTURE_HPP_
