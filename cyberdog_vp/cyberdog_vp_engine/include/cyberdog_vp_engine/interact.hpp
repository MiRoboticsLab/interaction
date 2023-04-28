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
#ifndef CYBERDOG_VP_ENGINE__INTERACT_HPP_
#define CYBERDOG_VP_ENGINE__INTERACT_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_engine/artificial_intelligence.hpp"
#include "cyberdog_vp_engine/backend_message.hpp"
#include "cyberdog_vp_engine/common.hpp"
#include "cyberdog_vp_engine/frontend_message.hpp"
#include "cyberdog_vp_engine/fsm.hpp"
#include "cyberdog_vp_engine/module.hpp"
#include "cyberdog_vp_engine/python_interpreter.hpp"
#include "cyberdog_vp_engine/slam.hpp"
#include "cyberdog_vp_engine/task.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       interact.hpp
    \brief      可视化编程引擎相互作用模块。
    \details    可视化编程引擎的核心模块，负责消息收发及处理。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化ros2。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Interact
{
public:
  Interact();
  ~Interact();
  bool Init();                                    /*!< 初始化 */

private:
  std::string logger_ {""};                       /*!< 日志 */
  Task task_;                                     /*!< 任务 */
  Module module_;                                 /*!< 模块 */
  ArtificialIntelligence AI_;                     /*!< 人工智能模块 */
  Slam SLAM_;                                     /*!< SLAM 模块 */
  Fsm fsm_;                                       /*!< 状态机 */
  std::shared_ptr<PythonInterpreter>
  py_interpreter_ptr_ {nullptr};                  /*!< python 解释器 */

  bool use_backend_ {false};                      /*!< 是否使用后端 */
  bool use_voice_ctrl_ {false};                   /*!< 是否使用语音控制 */

  std::string params_pkg_dir_ {""};               /*!< params 包路径 */
  std::string node_config_dir_ {""};              /*!< node 配置文件路径 */
  const std::string judge_at_ {"at -V"};          /*!< 判断 at */
  const std::string judge_cron_
  {"service cron status"};                        /*!< 判断 cron */
  const std::string judge_pyflakes_
  {"pyflakes --version "};                        /*!< 判断 pyflakes */
  toml::value params_toml_;                       /*!< 配置文件数据 */

  rclcpp::Node::SharedPtr
    frontend_node_ptr_ {nullptr};                 /*!< 前端节点 */
  rclcpp::Node::SharedPtr
    backend_node_ptr_ {nullptr};                  /*!< 后端节点 */
  rclcpp::Node::SharedPtr
    robotend_node_ptr_ {nullptr};                 /*!< 机器人端节点 */
  rclcpp::Node::SharedPtr
    fsm_node_ptr_ {nullptr};                      /*!< 状态机节点 */

  rclcpp::CallbackGroup::SharedPtr
    frontend_msg_cb_group_ {nullptr};             /*!< [回调组]前端消息 */
  rclcpp::CallbackGroup::SharedPtr
    audio_msg_cb_group_ {nullptr};                /*!< [回调组]语音消息 */
  rclcpp::CallbackGroup::SharedPtr
    grpc_pub_cb_group_ {nullptr};                 /*!< [回调组]GRPC消息 */
  rclcpp::CallbackGroup::SharedPtr
    http_cli_cb_group_ {nullptr};                 /*!< [回调组]后端服务器 */
  rclcpp::CallbackGroup::SharedPtr
    operate_srv_cb_group_ {nullptr};              /*!< [回调组]操作服务 */

  rclcpp::Subscription<GRPCMsg>::SharedPtr
    grpc_sub_ {nullptr};                          /*!< [监听器]前端消息 */
  rclcpp::Subscription<ASRMsg>::SharedPtr
    asr_sub_ {nullptr};                           /*!< [监听器]语音消息 */
  rclcpp::Publisher<GRPCMsg>::SharedPtr
    grpc_pub_ {nullptr};                          /*!< [发布器]GRPC消息 */
  rclcpp::Client<HTPSrv>::SharedPtr
    http_cli_ {nullptr};                          /*!< [客户端]后端服务器 */
  rclcpp::Service<OperateSrv>::SharedPtr
    operate_srv_ {nullptr};                       /*!< [服务端]操作服务 */

  using CommonEnum = enum
  {
    frontend = 0,                                 /*!< 前端 */
    backend,                                      /*!< 后端 */
  };

private:
  bool InitWorkspace();                           /*!< 初始化工作空间 */
  bool InitData();                                /*!< 初始化数据 */
  std::shared_ptr<HTPSrv::Request>
  GetRequest(const std::string &);                /*!< 获取服务请求 */
  bool RequestsBackend(
    const std::shared_ptr<HTPSrv::Request> &,
    BackendMsg &);                                /*!< 请求后端 */
  void CoutJson(
    const std::string &,
    const std::string &);                         /*!< 输出消息 */
  void FrontendMsgCallback(
    const GRPCMsg::SharedPtr msg);                /*!< 前端消息回调 */
  void ASRMsgCallback(
    const ASRMsg::SharedPtr msg);                 /*!< 语音消息回调 */
  bool RespondToFrontAndBackEndRequests(
    const CommonEnum &,
    const std::string &, OperateMsg &);           /*!< 响应前后端请求 */
  bool OperateMsgToJson(
    const OperateMsg &,
    std::string &);                               /*!< 操作转json */
  void OperateResponse(
    const std::shared_ptr<OperateSrv::Request>,
    std::shared_ptr<OperateSrv::Response>);       /*!< 操作响应 */
};  // class Interact
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__INTERACT_HPP_
