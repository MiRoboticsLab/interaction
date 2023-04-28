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

#ifndef CYBERDOG_VP_ABILITYSET__SKELETON_HPP_
#define CYBERDOG_VP_ABILITYSET__SKELETON_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       skeleton.hpp
    \brief      骨骼（点）识别模块。
    \details    创建及初始化骨骼（点）识别模块，以便任务调用骨骼（点）识别功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Skeleton final : public Base
{
public:
  Skeleton()
  : Base(std::string(__FUNCTION__)) {}
  ~Skeleton() {}
  SkeletonRecognizedSeviceResponse TurnOnRecognition(
    const uint, const int, const int);            /*!< 打开识别骨骼（点）功能 */
  SkeletonRecognizedSeviceResponse TurnOffRecognition(
  );                                              /*!< 关闭识别骨骼（点）功能 */
  SkeletonRecognizedMessageResponse BlockingRecognized(
    const int timeout = -1);                      /*!< 阻塞式识别到：状态改变才返回 */
  SkeletonRecognizedMessageResponse InstantRecognized(
  );                                              /*!< 瞬时式识别到：返回最近的状态 */
  SkeletonRecognizedMessageResponse SportsRecognition(
    const uint, const int, const int,
    const bool interact = false,
    const bool instantly = false,
    const int volume = 50
  );                                              /*!< 运动识别 */

  void InitDependent(
    const std::function<AudioPlaySeviceResponse(const std::string, const int8_t)> &,
    const std::function<State(const std::string, const int8_t)> &
  );                                              /*!< 初始化依赖 */

private:
  rclcpp::CallbackGroup::SharedPtr
    skeleton_sub_cb_group_ {nullptr};             /*!< [回调组]骨骼（点）识别响应 */
  rclcpp::CallbackGroup::SharedPtr
    skeleton_cli_cb_group_ {nullptr};             /*!< [回调组]骨骼（点）识别服务 */

  rclcpp::Subscription<MsgSport>::SharedPtr
    skeleton_recognition_sub_ptr_ {nullptr};      /*!< [监听器]骨骼（点）识别响应 */
  rclcpp::Client<SrvSport>::SharedPtr
    skeleton_recognition_cli_ptr_ {nullptr};      /*!< [客户端]骨骼（点）识别服务 */

  MsgSport recognition_;                          /*!< [客户端]骨骼（点）识别目标 */
  bool interact_ {false};                         /*!< 运动识别过程交互开关 */
  bool instantly_ {false};                        /*!< 立即播报 */
  int volume_ {50};                               /*!< 运动识别过程交互音量 */

  std::function<AudioPlaySeviceResponse(const std::string, const int8_t)>
  FPlay;                                          /*!< 播报语音 */
  std::function<State(const std::string, const int8_t)>
  FInstantlyPlay;                                 /*!< 立即播报语音 */
  bool skeleton_update_ {false};                  /*!< 骨骼点更新 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubSkeletonRecognitionCB(
    const MsgSport::SharedPtr);                   /*!< 骨骼（点）识别响应数据回调 */
  std::shared_ptr<SrvSport::Request>
  GetSkeletonRecognitionRequest();                /*!< 获取骨骼（点）识别服务请求 */
  bool RequestSkeletonRecognizedSrv(
    SrvSport::Response &,
    std::shared_ptr<SrvSport::Request>,
    const int _service_start_timeout = 10);       /*!< 请求骨骼（点）识别服务 */
};  // class Skeleton
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__SKELETON_HPP_
