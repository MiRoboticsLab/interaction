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
#ifndef CONNECTOR__CTRL_LED_HPP_
#define CONNECTOR__CTRL_LED_HPP_

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>

#include <protocol/msg/audio_play.hpp>
#include <protocol/srv/led_execute.hpp>

#include <string>

namespace cyberdog
{
namespace interaction
{
/*! \file       ctrl_led.hpp
    \brief      LED控制模块。
    \details    创建及初始化LED控制模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class CtrlLed final : public rclcpp::Node
{
  using AudioMsg = protocol::msg::AudioPlay;            /*!< [topic 类型]语音消息 */
  using LedSrv = protocol::srv::LedExecute;             /*!< [service 类型]LED 驱动服务 */

public:
  explicit CtrlLed(const std::string name);
  ~CtrlLed();
  uint ControlLed(const uint16_t & _id);                /*!< 控制led */

private:
  bool Init();                                          /*!< 初始化 */
  uint EnableLed(
    const bool & occupation = false,
    const uint8_t & target = LedSrv::Request::MINI_LED,
    const uint8_t & mode = LedSrv::Request::SYSTEM_PREDEFINED,
    const uint8_t & effect = LedSrv::Request::MINI_OFF,
    const uint8_t & r = 0,
    const uint8_t & g = 0,
    const uint8_t & b = 0);                             /*!< 使能 */

private:
  rclcpp::Client<LedSrv>::SharedPtr client_ {nullptr};  /*!< [客户端]LED 灯驱动 */
};  // class CtrlLed
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__CTRL_LED_HPP_
