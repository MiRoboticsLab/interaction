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

#ifndef CYBERDOG_VP_TERMINAL__COMMON_HPP_
#define CYBERDOG_VP_TERMINAL__COMMON_HPP_

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include <cyberdog_common/cyberdog_json.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_msgs/msg/string.hpp>

#include <protocol/srv/led_execute.hpp>
#include <protocol/srv/task_processor.hpp>
#include <protocol/srv/motion_result_cmd.hpp>

#include <protocol/msg/audio_play.hpp>
#include <protocol/msg/connector_status.hpp>
#include <protocol/msg/motion_id.hpp>
#include <protocol/msg/bms_status.hpp>
#include <protocol/msg/touch_status.hpp>
#include <protocol/msg/motion_servo_cmd.hpp>
#include <protocol/msg/motion_servo_response.hpp>
#include <protocol/msg/visual_programming_operate.hpp>

#include <cyberdog_vp_abilityset/cyberdog.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include <math.h>

#include <ncurses.h>
#include <panel.h>
#include <menu.h>
#include <sys/ioctl.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <mutex>
#include <tuple>
#include <cctype>
#include <typeinfo>

namespace cyberdog_visual_programming_terminal
{
/*! \file       common.hpp
    \brief      通用模块。
    \details    创建及初始化机器人通用模块，以便测试调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保通用文档的正确性。
    \note       确保通用的持续补充及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/

namespace py = pybind11;
using CyberdogJson =
  cyberdog::common::CyberdogJson;                 /*!< json 解析类型 */
using CyberdogToml =
  cyberdog::common::CyberdogToml;                 /*!< Toml 解析及构建模块类型 */
using GRPCMsg = std_msgs::msg::String;            /*!< [topic 类型]GRPC 消息 */
using OperateMsg =
  protocol::msg::VisualProgrammingOperate;        /*!< [topic 类型]任务操作 */

/*! 约束获取时间接口入参的合法值 */
enum TimeMode
{
  Ms1970 = 0,                                     /*!< 1970年1月1日到现在的时间(毫秒) */
  _Y_M_D_H_M_S,                                   /*!< "*Y*M*D*H*M*S" */
  STANDARD,                                       /*!< "*Y.*M.*D-*H:*M:*S" */
};
/*!< {0:Ms1970, 1:_Y_M_D_H_M_S, 2:STANDARD} */
std::string GetTime(
  int nowModo = 0);                               /*!< 获取时间戳 */
bool JudgeConfileFile(std::string _file);         /*!< 判断配置文件权限 */
std::vector<std::string> GetVector(
  const std::string &, char,
  const std::string & _head = "");                /*!< 获取向量 */
void CoutJson(
  const std::string &,
  const std::string &);                           /*!< 输出Json消息 */
}  // namespace cyberdog_visual_programming_terminal
#endif  // CYBERDOG_VP_TERMINAL__COMMON_HPP_
