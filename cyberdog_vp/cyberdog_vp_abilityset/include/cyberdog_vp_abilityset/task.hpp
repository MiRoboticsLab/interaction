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

#ifndef CYBERDOG_VP_ABILITYSET__TASK_HPP_
#define CYBERDOG_VP_ABILITYSET__TASK_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       task.hpp
    \brief      任务模块。
    \details    创建及初始化任务模块，以便控制任务执行状态。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \note       留意各种场景下控制是否有效
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Task final : public Base
{
public:
  Task()
  : Base(std::string(__FUNCTION__)) {}
  ~Task() {}
  bool GetProcessor(Processor);                   /*!< 获取处理器 */
  State Start();                                  /*!< 开始任务 */
  State Stop();                                   /*!< 结束任务 */
  State Recover();                                /*!< 继续任务 */
  State Block(const std::string id);              /*!< 设置块 */
  State BreakpointBlock(const std::string id);    /*!< 设置断点块 */

  void InitDependent(
    const std::function<void(bool)> &,
    const std::function<AudioPlaySeviceResponse(
      const std::string, const int8_t)> &);       /*!< 初始化依赖 */

private:
  std::string block_id_ {""};                     /*!< 块id */
  std::string task_state_ {""};                   /*!< 任务状态 */
  std::string processor_srv_ {""};                /*!< 处理器服务名称 */
  bool is_activation_ {false};                    /*!< 是否激活任务属性 */
  std::function<void(bool)> FShutdown;            /*!< 终止 */
  std::function<AudioPlaySeviceResponse(
      const std::string,
      const int8_t)> FAudioPlay;                  /*!< 获取处理器 */

  MsgVisualProgrammingOperate fsm_;               /*!< 状态机数据 */

  rclcpp::CallbackGroup::SharedPtr
    operate_cli_cb_group_ {nullptr};              /*!< [回调组]操作 */
  rclcpp::CallbackGroup::SharedPtr
    frontend_option_sub_cb_group_ {nullptr};      /*!< [回调组]任务操作 */
  rclcpp::CallbackGroup::SharedPtr
    robotend_grpc_pub_cb_group_ {nullptr};        /*!< [回调组]GRPC消息 */
  rclcpp::CallbackGroup::SharedPtr
    fsm_sub_cb_group_ {nullptr};                  /*!< [回调组]FSM消息 */

  rclcpp::Client<SrvVisualProgrammingOperate>::SharedPtr
    operate_cli_ptr_ {nullptr};                   /*!< [服务端]操作 */
  rclcpp::Subscription<MsgVisualProgrammingOperate>::SharedPtr
    frontend_option_sub_ptr_ {nullptr};           /*!< [监听器]任务操作 */
  rclcpp::Publisher<MsgString>::SharedPtr
    robotend_grpc_pub_ptr_ {nullptr};             /*!< [发布器]GRPC消息 */
  rclcpp::Subscription<MsgVisualProgrammingOperate>::SharedPtr
    fsm_sub_ {nullptr};                           /*!< [监听器]状态机 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  bool PubRobotendGrpc(rapidjson::Document &);    /*!< 发布GRPC消息 */
  void SetTaskState(const std::string);           /*!< 设置任务状态 */
  void FrontendOperateCB(
    const MsgVisualProgrammingOperate::SharedPtr
  );                                              /*!< 前端操作回调 */
  void FsmCB(
    const MsgVisualProgrammingOperate::SharedPtr
  );                                              /*!< 状态机消息回调 */
  bool JudgeFSM();                                /*!< 判断状态机 */
  void FSMReciprocalCompensation(StateCode &);    /*!< 状态机异常时候交互补偿 + 终止任务 */
  void GetBlockJson(
    rapidjson::Document &, std::string,
    std::string _describe = "",
    int _state = 0);                              /*!< 获取后端块json数据 */
  bool RequestEngine(const std::string &);         /*!< 请求引擎 */
};  // class Task
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__TASK_HPP_
