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

#ifndef CYBERDOG_VP_ABILITYSET__MOTION_HPP_
#define CYBERDOG_VP_ABILITYSET__MOTION_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"
#include "cyberdog_vp_abilityset/odometer.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       motion.hpp
    \brief      运动模块。
    \details    创建及初始化机器人运动能力，以便任务调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Motion final : public Base
{
public:
  Motion()
  : Base(std::string(__FUNCTION__)) {}
  ~Motion() {}
  MotionParams params_;                           /*!< 参数 */
  int old_motion_id_;                             /*!< 上一个运动ID */
  int motion_id_;                                 /*!< 运动ID */

  MotionResultServiceResponse Request(
    const int32_t motion_id = 0,
    const int32_t duration = 0);                  /*!< [结果指令]请求接口 */

  MotionResultServiceResponse WalkTheDog(
    const double, const double);                  /*!< [结果指令]遛狗 */
  MotionResultServiceResponse AbsoluteForceControlAttitude(
    const double,
    const double, const double, const double,
    const double);                                /*!< [结果指令]绝对力控姿态 */
  MotionResultServiceResponse RelativelyForceControlAttitude(
    const double, const double, const double,
    const double, const double, const double,
    const double);                                /*!< [结果指令]相对力控姿态 */
  MotionResultServiceResponse AbsolutePositionControlAttitude(
    const double, const double);                  /*!< [结果指令]绝对位控姿态 */
  MotionResultServiceResponse RelativelyPositionControlAttitude(
    const double, const double, const double,
    const double, const double, const double,
    const double, const double, const double,
    const double);                                /*!< [结果指令]相对位控姿态 */

  MotionServoCmdResponse JumpBackAndForth(
    const double, const double, const double,
    const double, const double,
    const double _distance = 0,
    const double _duration = 0,
    const uint _compensation_frame_size = 0);     /*!< [伺服指令]前后跳 */
  MotionServoCmdResponse SmallJumpWalking(
    const double, const double, const double,
    const double, const double,
    const double _distance = 0,
    const double _duration = 0,
    const uint _compensation_frame_size = 0);     /*!< [伺服指令]小跳行走 */
  MotionServoCmdResponse AutomaticFrequencyConversionWalking(
    const double, const double, const double,
    const double, const double,
    const double _distance = 0,
    const double _duration = 0,
    const uint _compensation_frame_size = 0);     /*!< [伺服指令]自动变频行走 */
  MotionServoCmdResponse TrotWalking(
    const double, const double, const double,
    const double, const double,
    const double _distance = 0,
    const double _duration = 0,
    const uint _compensation_frame_size = 0);     /*!< [伺服指令]小跑行走 */
  MotionServoCmdResponse RunFastWalking(
    const double, const double, const double,
    const double, const double,
    const double _distance = 0,
    const double _duration = 0,
    const uint _compensation_frame_size = 0);     /*!< [伺服指令]快跑行走 */
  MotionServoCmdResponse Turn(
    const double,
    double);                                      /*!< 转向 */
  MotionServoCmdResponse GoStraight(
    const double, const double, const double);    /*!< 直行 */
  MotionServoCmdResponse LateralMovement(
    const double, const double, const double);    /*!< 横移 */

  MotionSequenceServiceResponse RunSequence(
    const MotionSequence &);                      /*!< 运行序列动作 */

  void InitDependent(
    const std::function<AudioPlaySeviceResponse(const std::string, const int8_t)> &,
    const std::function<AudioPlaySeviceResponse(const uint16_t, const int8_t)> &,
    const std::function<bool(MsgOdometry &, double &, const double)> &,
    const std::function<bool(Processor)> &);      /*!< 初始化依赖 */

private:
  uint compensation_frame_size_ {0};              /*!< 补偿帧数量 */
  MsgMotionServoCmd compensation_frame_;          /*!< 补偿帧 */
  MsgMotionServoResponse servo_response_msg_;     /*!< 伺服反馈消息 */
  std::function<AudioPlaySeviceResponse(const std::string, const int8_t)>
  FOnlinePlay;                                    /*!< 在线播放音效 */
  std::function<AudioPlaySeviceResponse(const uint16_t, const int8_t)>
  FOfflinePlay;                                   /*!< 离线播放音效 */
  std::function<bool(MsgOdometry &, double &, const double)>
  FIfCumulativeDistance;                          /*!< 判断累计变化距离 */
  std::function<bool(Processor)> FGetProcessor;   /*!< 获取处理器 */
  double servo_cmd_deviation_coefficient_ {1.0};  /*!< 伺服指令误差系数(实际=目标*系数) */
  double odom_deviation_coefficient_ {1.0};       /*!< 里程计误差系数(实际=计算*系数) */
  uint servo_cmd_end_frame_time_consuming_ {500}; /*!< 伺服指令end帧耗时(毫秒) */

  rclcpp::CallbackGroup::SharedPtr
    timer_cb_group_ {nullptr};                    /*!< [回调组]补偿帧 */
  rclcpp::CallbackGroup::SharedPtr
    result_cli_cb_group_ {nullptr};               /*!< [回调组]结果指令服务 */
  rclcpp::CallbackGroup::SharedPtr
    sequence_cli_cb_group_ {nullptr};             /*!< [回调组]序列动作服务 */
  rclcpp::CallbackGroup::SharedPtr
    sub_cb_group_ {nullptr};                      /*!< [回调组]伺服反馈 */
  rclcpp::CallbackGroup::SharedPtr
    pub_cb_group_ {nullptr};                      /*!< [回调组]伺服指令 */
  rclcpp::CallbackGroup::SharedPtr
    status_sub_cb_group_ {nullptr};               /*!< [回调组]伺服指令 */

  rclcpp::TimerBase::SharedPtr
    compensation_frame_timer_ {nullptr};          /*!< [定时器]补偿帧 */
  rclcpp::Client<SrvMotionResultCmd>::SharedPtr
    result_cli_ptr_ {nullptr};                    /*!< [客户端]结果指令服务 */
  rclcpp::Client<SrvMotionSequenceShow>::SharedPtr
    sequence_cli_ptr_ {nullptr};                  /*!< [客户端]序列动作服务 */
  rclcpp::Subscription<MsgMotionServoResponse>::SharedPtr
    sub_ptr_ {nullptr};                           /*!< [监听器]伺服反馈 */
  rclcpp::Subscription<MsgMotionStatus>::SharedPtr
    status_sub_ptr_ {nullptr};                    /*!< [监听器]运动状态 */
  rclcpp::Publisher<MsgMotionServoCmd>::SharedPtr
    pub_ptr_ {nullptr};                           /*!< [发布器]伺服指令 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  std::shared_ptr<SrvMotionResultCmd::Request>
  GetResultRequest();                             /*!< 获取结果指令服务请求 */
  std::shared_ptr<SrvMotionSequenceShow::Request>
  GetSequenceRequest(const MotionSequence &);     /*!< 获取序列动作服务请求 */
  std::shared_ptr<MsgMotionServoCmd> GetServoCmd(
    const int32_t _motion_id = 0,
    const int32_t _cmd_type = 1);                 /*!< 获取伺服指令 */
  bool RequestResultSrv(
    MotionResultServiceResponse &,
    std::string, std::shared_ptr<SrvMotionResultCmd::Request>,
    int _service_start_timeout = 3);              /*!< 请求结果指令服务 */
  bool RequestSequenceSrv(
    MotionSequenceServiceResponse &,
    std::string, std::shared_ptr<SrvMotionSequenceShow::Request>,
    int _service_start_timeout = 3);              /*!< 请求序列动作服务 */
  void ServiceBaseMotionAttitude(
    MotionResultServiceResponse &,
    const std::string,
    const int32_t,
    const MsgPose &,
    const MsgPoint &);                            /*!< [结果指令]基础运动 */
  void ServiceBaseMotionLh(
    MotionResultServiceResponse &,
    const std::string, const int32_t,
    const double, const double);                  /*!< [结果指令]基础运动 */
  void ServoBaseMotionLh(
    MotionServoCmdResponse &,
    const std::string, const int32_t,
    const double, const double,
    const uint &);                                /*!< [伺服指令]基础运动 */
  void ServoBaseMotionVxyzLh(
    MotionServoCmdResponse &,
    const std::string, const int32_t,
    const double, const double, const double,
    const double, const double,
    const double, const double,
    const uint &);                                /*!< [伺服指令]基础运动 */
  bool PublisherServoCmd(
    MotionServoCmdResponse &,
    std::string,
    std::shared_ptr<MsgMotionServoCmd>,
    const uint & _compensation_frame_size = 0);   /*!< 发布伺服指令并取回最近一次的反馈数据 */
  bool PublisherServoEnd(
    MotionServoCmdResponse &,
    const int32_t,
    const std::string);                           /*!< 发布伺服终止指令并取回最近一次的反馈数据 */
  void ServoResponse(
    const MsgMotionServoResponse::SharedPtr);     /*!< 伺服反馈 */
  void CompensationFrame();                       /*!< 补偿帧 */
  void MotionStatusResponse(
    const MsgMotionStatus::SharedPtr);            /*!< 运动状态反馈 */
};  // class Motion
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__MOTION_HPP_
