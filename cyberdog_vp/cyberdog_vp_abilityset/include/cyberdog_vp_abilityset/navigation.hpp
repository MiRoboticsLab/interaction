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

#ifndef CYBERDOG_VP_ABILITYSET__NAVIGATION_HPP_
#define CYBERDOG_VP_ABILITYSET__NAVIGATION_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       navigation.hpp
    \brief      导航模块。
    \details    创建及初始化机器人导航能力，以便任务调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Navigation final : public Base
{
public:
  Navigation()
  : Base(std::string(__FUNCTION__)) {}
  ~Navigation() {}
  MapPresetSeviceResponse GetPreset(
    const int _timeout = 3);                      /*!< 获取预置点 */
  NavigationActionResponse TurnOnNavigation(
    const bool outdoor = false,
    const bool assisted_relocation = false,
    const bool interact = false,
    const int volume = 50);                       /*!< 请求开启重定位 */
  NavigationActionResponse TurnOffNavigation();   /*!< 请求关闭重定位 */
  NavigationActionResponse NavigationToPreset(
    const std::string);                           /*!< 导航到预置点 */
  NavigationActionResponse NavigationToCoordinates(
    const bool is_outdoor = false,
    const double x = 0,
    const double y = 0,
    const double z = 0,
    const double roll = 0,
    const double pitch = 0,
    const double yaw = 0);                        /*!< 导航到坐标点 */
  NavigationActionResponse NavigationToPose(
    const MsgPose &);                             /*!< 导航到位姿 */
  NavigationActionResponse CancelNavigation(
    const int _timeout = 3);                      /*!< 取消导航 */
  NavigationActionResponse ToPreset(
    const std::string,
    const bool assisted_relocation = false,
    const bool interact = false,
    const int volume = 50);                       /*!< 去预置点 */

  State AddPreset(
    const std::string);                           /*!< 添加预置点 */
  State DeletePreset(
    const std::string);                           /*!< 删除预置点 */

  void InitDependent(
    const std::function<MotionServoCmdResponse(const double, double)> &,
    const std::function<AudioPlaySeviceResponse(const std::string, const int8_t)> &,
    const std::function<State(const std::string, const int8_t)> &
  );                                              /*!< 初始化依赖 */

private:
  std::function<MotionServoCmdResponse(const double, double)>
  FTurn;                                          /*!< 转向 */
  std::function<AudioPlaySeviceResponse(const std::string, const int8_t)>
  FPlay;                                          /*!< 播报语音 */
  std::function<State(const std::string, const int8_t)>
  FInstantlyPlay;                                 /*!< 立即播报语音 */

  rclcpp::CallbackGroup::SharedPtr
    algo_status_sub_cb_group_ {nullptr};          /*!< [回调组]算法状态 */
  rclcpp::CallbackGroup::SharedPtr
    assisted_relocation_timer_cb_group_
  {nullptr};                                      /*!< [回调组]辅助重定位定时器 */
  rclcpp::CallbackGroup::SharedPtr
    get_preset_cli_cb_group_ {nullptr};           /*!< [回调组]获取预置点服务 */
  rclcpp::CallbackGroup::SharedPtr
    cancel_navigation_cli_cb_group_ {nullptr};    /*!< [回调组]取消导航 */
  rclcpp::CallbackGroup::SharedPtr
    navigation_cli_cb_group_ {nullptr};           /*!< [回调组]导航 */

  rclcpp::Subscription<MsgAlgoStatus>::SharedPtr
    algo_status_sub_ptr_ {nullptr};               /*!< [监听器]算法状态 */
  rclcpp::TimerBase::SharedPtr
    assisted_relocation_timer_ {nullptr};         /*!< [定时器]辅助重定位 */
  rclcpp::Client<SrvGetPreset>::SharedPtr
    get_preset_cli_ptr_ {nullptr};                /*!< [客户端]获取预置点服务 */
  rclcpp::Client<SrvCancelNavigation>::SharedPtr
    cancel_navigation_cli_ptr_ {nullptr};         /*!< [客户端]取消导航 */
  rclcpp_action::Client<ActNavigation>::SharedPtr
    navigation_cli_ptr_ {nullptr};                /*!< [客户端]导航 */

  bool assisted_relocation_ {false};              /*!< 是否开启辅助重定位 */
  bool assisted_relocation_interact_ {false};     /*!< 是否开启辅助重定位交互 */
  int assisted_relocation_volume_ {50};           /*!< 辅助重定位交互音量 */
  MsgAlgoStatus algo_status_;                     /*!< 算法状态 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubAlgoStatusCB(
    const MsgAlgoStatus::SharedPtr);              /*!< 算法状态数据回调 */
  SrvGetPreset::Response RequestPresetSrv(int);   /*!< 请求预置点服务 */
  SrvCancelNavigation::Response RequestCancelSrv(
    const std::shared_ptr<SrvCancelNavigation::Request>,
    int);                                         /*!< 请求取消导航服务 */
  void NavigationFeedbackCB(
    rclcpp_action::Client<ActNavigation>::GoalHandle::SharedPtr,
    const std::shared_ptr<const ActNavigation::Feedback>
  );                                              /*!< 动作反馈 */
  ActNavigation::Result RequestNavigationAct(
    const ActNavigation::Goal &,
    int _timeout = 60 *10);                       /*!< 请求导航动作 */
  void AssistedRelocation();                      /*!< 辅助重定位 */
};  // class Navigation
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__NAVIGATION_HPP_
