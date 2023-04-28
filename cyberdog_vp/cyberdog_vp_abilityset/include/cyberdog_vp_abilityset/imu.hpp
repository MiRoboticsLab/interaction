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

#ifndef CYBERDOG_VP_ABILITYSET__IMU_HPP_
#define CYBERDOG_VP_ABILITYSET__IMU_HPP_

#include <string>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       imu.hpp
    \brief      imu模块。
    \details    创建及初始化机器人惯导imu能力，以便任务调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Imu final : public Base
{
public:
  Imu()
  : Base(std::string(__FUNCTION__), true) {}
  ~Imu() {}
  MsgImu data_;                                   /*!< IMU */
  MsgImu GetData(const int _timeout = 5);         /*!< 获取最新数据 */
  RPY GetRPY();                                   /*!< 获取欧拉角 */

private:
  rclcpp::CallbackGroup::SharedPtr
    sub_cb_group_ {nullptr};                      /*!< [回调组] IMU */

  rclcpp::Subscription<MsgImu>::SharedPtr
    sub_ptr_ {nullptr};                           /*!< [监听器] IMU */

protected:
  bool IfDeltaAngular(
    const MsgImu &,
    const double,
    RPYType _type = RPYType::YAW);                /*!< 判断角度 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubCB(const MsgImu::SharedPtr);            /*!< 数据回调 */
  RPY Quaternion2RPY(const MsgImu &);             /*!< 四元数转欧拉角 */
};  // class Imu
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__IMU_HPP_
