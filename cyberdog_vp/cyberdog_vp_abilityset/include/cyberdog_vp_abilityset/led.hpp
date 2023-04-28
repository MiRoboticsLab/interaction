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

#ifndef CYBERDOG_VP_ABILITYSET__LED_HPP_
#define CYBERDOG_VP_ABILITYSET__LED_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       led.hpp
    \brief      led模块。
    \details    创建及初始化led模块，以便任务调用led功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Led final : public Base
{
public:
  Led()
  : Base(std::string(__FUNCTION__)) {}
  ~Led() {}
  LedSeviceResponse Play(
    const int & _target = SrvLedExecute::Request::HEAD_LED,
    const int & _effect = SrvLedExecute::Request::RED_ON
  );                                              /*!< 设置灯效 */
  LedSeviceResponse PlayRgb(
    const int &,
    const int &,
    const int &,
    const int &,
    const int &
  );                                              /*!< 设置灯效 */
  LedSeviceResponse Freed(const int &);           /*!< 释放灯控制权 */

private:
  rclcpp::CallbackGroup::SharedPtr
    cli_cb_group_ {nullptr};                      /*!< [回调组]请求播放 */

  rclcpp::Client<SrvLedExecute>::SharedPtr
    cli_ptr_ {nullptr};                           /*!< [客户端]数据 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  std::shared_ptr<SrvLedExecute::Request>
  GetRequest();                                   /*!< 获取请求 */
  bool RequestSrv(
    LedSeviceResponse & _response,
    std::string _interface_name,
    std::shared_ptr<SrvLedExecute::Request> _request_ptr,
    int _service_timeout = 5);                    /*!< 请求服务 */
};  // class Led
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__LED_HPP_
