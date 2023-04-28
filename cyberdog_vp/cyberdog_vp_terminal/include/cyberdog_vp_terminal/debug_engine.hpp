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

#ifndef CYBERDOG_VP_TERMINAL__DEBUG_ENGINE_HPP_
#define CYBERDOG_VP_TERMINAL__DEBUG_ENGINE_HPP_

#include <cyberdog_vp_abilityset/cyberdog.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "cyberdog_vp_terminal/common.hpp"

namespace cyberdog_visual_programming_terminal
{
/*! \file       debug_engine.hpp
    \brief      引擎调试器模块。
    \details    创建及初始化机器人引擎调试器，以便调试。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class DebugEngine
{
public:
  DebugEngine();
  ~DebugEngine();
  bool Init(const std::string &);                 /*!< 初始化 */
  void MockAppRequest(const std::string &);       /*!< 模拟APP请求 */

private:
  void FrontendMsgPublisher(
    const OperateMsg &);                          /*!< 前端消息发布 */
  void RobotendMsgCallback(
    const GRPCMsg::SharedPtr);                    /*!< 机器人端消息回调 */

private:
  std::string logger_ {""};                       /*!< 日志名称 */
  std::string params_pkg_dir_ {""};               /*!< 参数所在包路径 */
  std::string node_config_dir_ {""};              /*!< 节点配置路径 */
  toml::value params_toml_;                       /*!< 配置文件数据 */
  rclcpp::Node::SharedPtr node_ptr_
  {nullptr};                                      /*!< 节点 */
  rclcpp::Publisher<GRPCMsg>::SharedPtr
    grpc_pub_ {nullptr};                          /*!< [发布器]GRPC消息：前端 */
  rclcpp::Subscription<GRPCMsg>::SharedPtr
    grpc_sub_ {nullptr};                          /*!< [监听器]GRPC消息：机器人端 */
};  // class DebugEngine
}  // namespace cyberdog_visual_programming_terminal
#endif  // CYBERDOG_VP_TERMINAL__DEBUG_ENGINE_HPP_
