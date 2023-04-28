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

#ifndef CYBERDOG_VP_ABILITYSET__NETWORK_HPP_
#define CYBERDOG_VP_ABILITYSET__NETWORK_HPP_

#include <string>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       network.hpp
    \brief      网络模块。
    \details    创建及初始化机器人网络能力，以便任务调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Network final : public Base
{
public:
  Network()
  : Base(std::string(__FUNCTION__), true) {}
  ~Network() {}
  MsgConnectorStatus data_;                       /*!< 网络 */

private:
  rclcpp::CallbackGroup::SharedPtr
    sub_cb_group_ {nullptr};                      /*!< [回调组] 网络状态 */

  rclcpp::Subscription<MsgConnectorStatus>::SharedPtr
    sub_ptr_;                                     /*!< [监听器] 网络状态 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubCB(const MsgConnectorStatus::SharedPtr _msg_ptr)
  {
    this->timens_ = GetTimeNs();
    this->data_ = *_msg_ptr;
  }                                               /*!< 里程计回调 */
};  // class Odometer
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__NETWORK_HPP_
