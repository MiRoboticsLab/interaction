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
#ifndef CYBERDOG_VP_ENGINE__FSM_HPP_
#define CYBERDOG_VP_ENGINE__FSM_HPP_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       fsm.hpp
    \brief      状态机模块。
    \details    负责辅助状态机功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        相互作用有效。
    \bug        模块操作尚待调试
    \warning    留意注册表的稳定性
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Fsm : public cyberdog::machine::MachineActuator
{
public:
  Fsm();
  ~Fsm();
  bool Init(
    const rclcpp::Node::SharedPtr &,
    const toml::value &);                         /*!< 初始化 */
  bool RespondToRequests(
    const OperateMsg &, GRPCMsg &);               /*!< 执行请求 */

  int32_t OnSetUp();                              /*!< SetUp */
  int32_t ONTearDown();                           /*!< TearDown */
  int32_t OnSelfCheck();                          /*!< SelfCheck */
  int32_t OnActive();                             /*!< Active */
  int32_t OnDeActive();                           /*!< DeActive */
  int32_t OnProtected();                          /*!< Protected */
  int32_t OnLowPower();                           /*!< LowPower */
  int32_t OnOTA();                                /*!< OTA */
  int32_t OnError();                              /*!< Error */

private:
  std::string logger_ {""};                       /*!< 日志 */
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};    /*!< 节点 */
  toml::value params_toml_;                       /*!< 配置文件数据 */
  StateEnum state_;                               /*!< 状态 */
  std::string describe_ {""};                     /*!< 描述 */
  OperateMsg info_;                               /*!< 状态机信息 */

  rclcpp::CallbackGroup::SharedPtr
    timer_cb_group_ {nullptr};                    /*!< [回调组] timer */
  rclcpp::CallbackGroup::SharedPtr
    pub_info_cb_group_ {nullptr};                 /*!< [回调组] info */
  rclcpp::CallbackGroup::SharedPtr
    pub_audio_cb_group_ {nullptr};                /*!< [回调组] audio */

  rclcpp::TimerBase::SharedPtr
    update_info_timer_ {nullptr};                 /*!< [定时器]更新数据 */
  rclcpp::Publisher<OperateMsg>::SharedPtr
    info_pub_ {nullptr};                          /*!< [发布器]连接状态 */
  rclcpp::Publisher<AudioPlayExtendMsg>::SharedPtr
    audio_pub_ {nullptr};                         /*!< [发布器]语音请求 */

private:
  const std::unordered_map<std::string,
    std::unordered_map<std::string, std::vector<std::string>>> LEGAL_OPERATE_CONDITION =
  {
    {OperateMsg::TYPE_TASK, {
        {OperateMsg::FSM_UNINITIALIZED, {
          }},
        {OperateMsg::FSM_SET_UP, {
          }},
        {OperateMsg::FSM_TEAR_DOWN, {
          }},
        {OperateMsg::FSM_SELF_CHECK, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SUSPEND,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_STOP
          }},
        {OperateMsg::FSM_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE,
            OperateMsg::OPERATE_DEBUG,
            OperateMsg::OPERATE_RUN,
            OperateMsg::OPERATE_SUSPEND,
            OperateMsg::OPERATE_RECOVER,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_START,
            OperateMsg::OPERATE_STOP
          }},
        {OperateMsg::FSM_DE_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE,
            OperateMsg::OPERATE_SUSPEND,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_STOP
          }},
        {OperateMsg::FSM_PROTECTED, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE,
            OperateMsg::OPERATE_DEBUG,
            OperateMsg::OPERATE_RUN,
            OperateMsg::OPERATE_SUSPEND,
            OperateMsg::OPERATE_RECOVER,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_START,
            OperateMsg::OPERATE_STOP
          }},
        {OperateMsg::FSM_LOW_POWER, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_STOP
          }},
        {OperateMsg::FSM_OTA, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SUSPEND,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_STOP
          }},
        {OperateMsg::FSM_ERROR, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SUSPEND,
            OperateMsg::OPERATE_SHUTDOWN,
            OperateMsg::OPERATE_STOP
          }}
      }},
    {OperateMsg::TYPE_MODULE, {
        {OperateMsg::FSM_UNINITIALIZED, {
          }},
        {OperateMsg::FSM_SET_UP, {
          }},
        {OperateMsg::FSM_TEAR_DOWN, {
          }},
        {OperateMsg::FSM_SELF_CHECK, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE
          }},
        {OperateMsg::FSM_DE_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE
          }},
        {OperateMsg::FSM_PROTECTED, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE
          }},
        {OperateMsg::FSM_LOW_POWER, {
            OperateMsg::OPERATE_INQUIRY,
            OperateMsg::OPERATE_SAVE,
            OperateMsg::OPERATE_DELETE
          }},
        {OperateMsg::FSM_OTA, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_ERROR, {
            OperateMsg::OPERATE_INQUIRY
          }}
      }},
    {OperateMsg::TYPE_AI, {
        {OperateMsg::FSM_UNINITIALIZED, {
          }},
        {OperateMsg::FSM_SET_UP, {
          }},
        {OperateMsg::FSM_TEAR_DOWN, {
          }},
        {OperateMsg::FSM_SELF_CHECK, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_DE_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_PROTECTED, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_LOW_POWER, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_OTA, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_ERROR, {
            OperateMsg::OPERATE_INQUIRY
          }}
      }},
    {OperateMsg::TYPE_SLAM, {
        {OperateMsg::FSM_UNINITIALIZED, {
          }},
        {OperateMsg::FSM_SET_UP, {
          }},
        {OperateMsg::FSM_TEAR_DOWN, {
          }},
        {OperateMsg::FSM_SELF_CHECK, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_DE_ACTIVE, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_PROTECTED, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_LOW_POWER, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_OTA, {
            OperateMsg::OPERATE_INQUIRY
          }},
        {OperateMsg::FSM_ERROR, {
            OperateMsg::OPERATE_INQUIRY
          }}
      }}
  };                                              /*!< 合法操作约束 */

private:
  bool InitData();                                /*!< 初始化数据 */
  void UpdateInfo();                              /*!< 更新信息 */
  void getRobotendMsg(
    const OperateMsg &,
    GRPCMsg &);                                   /*!< 获取机器人端消息 */
  void AudioPlay(const std::string &);            /*!< 语音播放 */
  bool JudgeOperateLegality(const OperateMsg &);  /*!< 判断操作合法性 */
};  // class Fsm
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__FSM_HPP_
