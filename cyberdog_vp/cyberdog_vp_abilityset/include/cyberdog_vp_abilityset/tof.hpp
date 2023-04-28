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

#ifndef CYBERDOG_VP_ABILITYSET__TOF_HPP_
#define CYBERDOG_VP_ABILITYSET__TOF_HPP_

#include <string>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       tof.hpp
    \brief      激光测距模块。
    \details    创建及初始化激光测距模块，以便任务调用激光测距功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化传感器。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Tof final : public Base
{
public:
  Tof()
  : Base(std::string(__FUNCTION__), true) {}
  ~Tof() {}
  TofPayload data_;                               /*!< 数据 */
  TofPayload GetData(const int _timeout = 5);     /*!< 获取最新数据 */
  TofObstacle obstacle_;                          /*!< 障碍物 */

private:
  rclcpp::CallbackGroup::SharedPtr
    sub_cb_group_ {nullptr};                      /*!< [回调组] 设备数据 */

  rclcpp::Subscription<MsgHeadTofPayload>::SharedPtr
    sub_head_ptr_ {nullptr};                      /*!< [监听器]设备头部数据 */
  rclcpp::Subscription<MsgRearTofPayload>::SharedPtr
    sub_rear_ptr_ {nullptr};                      /*!< [监听器]设备尾部数据 */
  float obstacle_distance_ {0.6};                 /*!< 检测:障碍物测距 */
  float obstacle_percentage_ {80};                /*!< 检测:Tof:0值百分比:(>80%视为检测到障碍物) */
  float sensitivity_s_ {1};                       /*!< 检测:Tof:林敏度:(<1s视为检测到障碍物) */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  bool Detection(bool);                           /*!< 检测 */
  void SubHeadCB(
    const MsgHeadTofPayload::SharedPtr);          /*!< 数据回调 */
  void SubRearCB(
    const MsgRearTofPayload::SharedPtr);          /*!< 数据回调 */
};  // class Tof
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__TOF_HPP_
