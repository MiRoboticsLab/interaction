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

#ifndef CYBERDOG_VP_ABILITYSET__CYBERDOG_HPP_
#define CYBERDOG_VP_ABILITYSET__CYBERDOG_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

#include "cyberdog_vp_abilityset/network.hpp"
#include "cyberdog_vp_abilityset/follow.hpp"
#include "cyberdog_vp_abilityset/motion.hpp"
#include "cyberdog_vp_abilityset/navigation.hpp"
#include "cyberdog_vp_abilityset/task.hpp"
#include "cyberdog_vp_abilityset/train.hpp"
#include "cyberdog_vp_abilityset/personnel.hpp"
#include "cyberdog_vp_abilityset/gesture.hpp"
#include "cyberdog_vp_abilityset/skeleton.hpp"

#include "cyberdog_vp_abilityset/audio.hpp"
#include "cyberdog_vp_abilityset/bms.hpp"
#include "cyberdog_vp_abilityset/led.hpp"
#include "cyberdog_vp_abilityset/touch.hpp"
#include "cyberdog_vp_abilityset/imu.hpp"
#include "cyberdog_vp_abilityset/odometer.hpp"
#include "cyberdog_vp_abilityset/gps.hpp"
#include "cyberdog_vp_abilityset/lidar.hpp"
#include "cyberdog_vp_abilityset/tof.hpp"
#include "cyberdog_vp_abilityset/ultrasonic.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       cyberdog.hpp
    \brief      铁蛋模块。
    \details    创建及初始化铁蛋模块，以便任务调用各接口。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        ROS2 环境。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Cyberdog : public Base
{
public:
  Cyberdog(
    std::string _task = "task",
    std::string _namespace = "namespace",
    bool _ros = false,
    std::string _parameters = "");
  ~Cyberdog() {}

  void SetLog(const bool);                        /*!< 设置日志 */
  void Shutdown(bool _exit = true);               /*!< 终止 */
  State Ready(const uint16_t timeout = 30);       /*!< 就绪 */

  Network network_;                               /*!< 网络模块句柄 */
  Follow follow_;                                 /*!< 跟随模块句柄 */
  Motion motion_;                                 /*!< 运动模块句柄 */
  Navigation navigation_;                         /*!< 导航模块句柄 */
  Task task_;                                     /*!< 任务模块句柄 */
  Train train_;                                   /*!< 训练模块句柄 */
  Personnel personnel_;                           /*!< 人员模块句柄 */
  Gesture gesture_;                               /*!< 手势识别模块句柄 */
  Skeleton skeleton_;                             /*!< 骨骼（点）模块句柄 */

  Audio audio_;                                   /*!< 语音模块句柄 */
  Bms bms_;                                       /*!< Bms模块句柄 */
  Led led_;                                       /*!< Led模块句柄 */
  Touch touch_;                                   /*!< 触摸板模块句柄 */

  Imu imu_;                                       /*!< Imu模块句柄 */
  Odometer odometer_;                             /*!< 里程计模块句柄 */
  Gps gps_;                                       /*!< Gps模块句柄 */
  Lidar lidar_;                                   /*!< 雷达模块句柄 */
  Tof tof_;                                       /*!< Tof模块句柄 */
  Ultrasonic ultrasonic_;                         /*!< 超声波模块句柄 */

private:
  bool ros_ {true};                               /*!< ros 环境 */

private:
  bool Start();                                   /*!< 初始化 */
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
};  // class Cyberdog
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__CYBERDOG_HPP_
