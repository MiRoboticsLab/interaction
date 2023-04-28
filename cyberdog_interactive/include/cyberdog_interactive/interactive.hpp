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
#ifndef CYBERDOG_INTERACTIVE__INTERACTIVE_HPP_
#define CYBERDOG_INTERACTIVE__INTERACTIVE_HPP_

#include <unistd.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>

#include <protocol/msg/motion_status.hpp>
#include <protocol/msg/single_tof_payload.hpp>
#include <protocol/msg/head_tof_payload.hpp>
#include <protocol/msg/audio_play_extend.hpp>

#include <protocol/srv/motion_result_cmd.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>

#include <string>
#include <thread>
#include <map>
#include <mutex>
#include <tuple>
#include <memory>
#include <functional>
#include <type_traits>
#include <chrono>
#include <condition_variable>

namespace cyberdog
{
namespace interaction
{
using SrvMotionResultCmd = protocol::srv::MotionResultCmd;    /*!< 结果指令 */
using MsgMotionStatus = protocol::msg::MotionStatus;          /*!< 运动状态 */
using MsgHeadTofPayload = protocol::msg::HeadTofPayload;      /*!< 头部tof数据 */
using MsgSingleTofPayload = protocol::msg::SingleTofPayload;  /*!< 单个tof数据 */
using MsgAudioPlayExtend = protocol::msg::AudioPlayExtend;    /*!< 语音消息:在线 */

/*! \file       interactive.hpp
    \brief      交互模块。
    \details    创建及初始化交互模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Interactive
{
public:
  explicit Interactive(const std::string & name);
  ~Interactive();
  bool Init();                                      /*!< 初始化 */

public:
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};      /*!< 节点 */

private:
  void BeingTouchedOnTheChinWhileSitting();         /*!< 当坐下时被抚摸下巴 */
  void RelativePosture();                           /*!< 相对姿态，状态插值 */
  void SitDown();                                   /*!< 坐下 */
  void ShakeAssLeft();                              /*!< 屁股左扭 */
  void ShakeAssRight();                             /*!< 屁股右扭 */
  void ShakeAssFromSideToSide();                    /*!< 屁股左右扭 */
  void WoofWoof(uint16_t);                          /*!< 汪汪 */
  void SubHeadCB(
    const MsgHeadTofPayload::SharedPtr);            /*!< 头部tof监听 */
  void MotionStatusResponse(
    const MsgMotionStatus::SharedPtr);              /*!< 运动状态监听 */
  bool RequestResultSrv(
    std::shared_ptr<SrvMotionResultCmd::Request>,
    int);                                           /*!< 请求结果指令服务 */
  std::shared_ptr<SrvMotionResultCmd::Request>
  GetResultRequest();                               /*!< 获取结果指令服务请求 */

private:
  struct ObstacleMeta
  {
    bool detected = false;                          /*!< 检测到 */
    std::chrono::time_point<std::chrono::system_clock>
    validity_period_time;                           /*!< 有效期时间 */
  };
  ObstacleMeta head_left;
  ObstacleMeta head_right;

  int old_motion_id;
  int motion_id;
  float obstacle_distance_ {0.6};   /*!< 检测:障碍物测距 */
  float obstacle_percentage_ {80};  /*!< 检测:Tof:0值百分比:(大于 80% 视为检测到障碍物) */
  float sensitivity_s_ {1};         /*!< 检测:Tof:林敏度:(小于 1s 视为检测到障碍物) */

  rclcpp::CallbackGroup::SharedPtr
    BTOTCWS_cb_group_ {nullptr};                    /*!< [回调组] 当坐下时被抚摸下巴 */
  rclcpp::TimerBase::SharedPtr
    BTOTCWS_timer_ {nullptr};                       /*!< [定时器]当坐下时被抚摸下巴 */

  rclcpp::CallbackGroup::SharedPtr
    tof_sub_cb_group_ {nullptr};                    /*!< [回调组] TOF数据 */
  rclcpp::Subscription<MsgHeadTofPayload>::SharedPtr
    tof_sub_head_ptr_ {nullptr};                    /*!< [监听器] TOF数据 */

  rclcpp::CallbackGroup::SharedPtr
    status_sub_cb_group_ {nullptr};                 /*!< [回调组] motion数据 */
  rclcpp::Subscription<MsgMotionStatus>::SharedPtr
    status_sub_ptr_ {nullptr};                      /*!< [监听器] motion数据 */

  rclcpp::CallbackGroup::SharedPtr
    result_cli_cb_group_ {nullptr};                 /*!< [回调组]结果指令服务 */
  rclcpp::Client<SrvMotionResultCmd>::SharedPtr
    result_cli_ptr_ {nullptr};                      /*!< [客户端]结果指令服务 */

  rclcpp::CallbackGroup::SharedPtr
    play_pub_cb_group_ {nullptr};                   /*!< [回调组]请求播放 */
  rclcpp::Publisher<MsgAudioPlayExtend>::SharedPtr
    topic_pub_ {nullptr};                           /*!< [发布器]语音请求 */
};  // class Interactive
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_INTERACTIVE__INTERACTIVE_HPP_
