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
#ifndef CYBERDOG_VP_ENGINE__SLAM_HPP_
#define CYBERDOG_VP_ENGINE__SLAM_HPP_

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       artificial_intelligence.hpp
    \brief      AI模块。
    \details    负责辅助SLAM功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        相互作用有效。
    \bug        模块操作尚待调试
    \warning    留意注册表的稳定性
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Slam
{
public:
  Slam();
  ~Slam();
  bool Init(
    const rclcpp::Node::SharedPtr &,
    const toml::value &);                         /*!< 初始化 */
  bool RespondToRequests(
    const OperateMsg &, GRPCMsg &);               /*!< 执行请求 */

private:
  std::string logger_ {""};                       /*!< 日志 */
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};    /*!< 节点 */
  toml::value params_toml_;                       /*!< 配置文件数据 */
  StateEnum state_;                               /*!< 状态 */
  std::string describe_ {""};                     /*!< 描述 */

  rclcpp::CallbackGroup::SharedPtr
    preset_cli_cb_group_ {nullptr};               /*!< [回调组]人员信息服务 */

  rclcpp::Client<PresetSrv>::SharedPtr
    preset_cli_ptr_ {nullptr};                    /*!< [客户端]人员信息服务 */

private:
  bool InitData();                                /*!< 初始化数据 */
  bool RequestPresetSrv(
    PresetSrv::Response &,
    std::shared_ptr<PresetSrv::Request>,
    int _service_start_timeout = 3);              /*!< 请求人员服务 */
  bool PresetsToJson(
    const OperateMsg &,
    const PresetSrv::Response &,
    GRPCMsg &);                                   /*!< 预置点转json */
  void getRobotendMsg(
    const OperateMsg &,
    GRPCMsg &);                                   /*!< 获取机器人端消息 */
};  // class Slam
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__SLAM_HPP_
