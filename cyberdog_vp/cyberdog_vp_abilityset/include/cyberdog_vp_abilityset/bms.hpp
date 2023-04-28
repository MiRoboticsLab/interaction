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

#ifndef CYBERDOG_VP_ABILITYSET__BMS_HPP_
#define CYBERDOG_VP_ABILITYSET__BMS_HPP_

#include <string>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       bms.hpp
    \brief      电池管理系统模块。
    \details    创建及初始化电池管理系统模块，以便任务调用电池管理系统功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Bms final : public Base
{
public:
  Bms()
  : Base(std::string(__FUNCTION__), true) {}
  ~Bms() {}
  MsgBmsStatus data_;                             /*!< 数据 */
  MsgBmsStatus GetData(const int _timeout = 5);   /*!< 获取最新数据 */

private:
  rclcpp::CallbackGroup::SharedPtr
    sub_cb_group_ {nullptr};                      /*!< [回调组]定时器 */
  rclcpp::CallbackGroup::SharedPtr
    cli_cb_group_ {nullptr};                      /*!< [回调组]请求播放 */

  rclcpp::Subscription<MsgBmsStatus>::SharedPtr
    sub_ptr_ {nullptr};                           /*!< [监听器]设备数据 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubCB(const MsgBmsStatus::SharedPtr);      /*!< 数据回调 */
};  // class Bms
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__BMS_HPP_
