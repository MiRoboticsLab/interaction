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

#ifndef CYBERDOG_VP_ABILITYSET__PERSONNEL_HPP_
#define CYBERDOG_VP_ABILITYSET__PERSONNEL_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/face.hpp"
#include "cyberdog_vp_abilityset/voiceprint.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       personnel.hpp
    \brief      人员识别模块。
    \details    创建及初始化人脸识别模块，以便任务调用人脸识别功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Personnel final : public Base
{
public:
  Personnel()
  : Base(std::string(__FUNCTION__)) {}
  ~Personnel() {}
  SrvPersonnel::Response GetData(
    const int _timeout = 3);                      /*!< 获取人员信息 */

  Face face_;                                     /*!< 人脸识别句柄 */
  Voiceprint voiceprint_;                         /*!< 声纹识别句柄 */

  FaceRecognizedSeviceResponse FaceRecognized(
    const std::vector<std::string> &,
    const bool _and_operation = false,
    const int _duration = -1);                    /*!< 识别到目标人员人脸 */
  VoiceprintRecognizedResponse VoiceprintRecognized(
    const std::vector<std::string> &,
    bool _and_operation = false,
    const int _duration = -1,
    const int _sensitivity = 1);                  /*!< 识别到目标人员声纹 */

private:
  rclcpp::CallbackGroup::SharedPtr
    cli_cb_group_ {nullptr};                      /*!< [回调组]请求播放 */

  rclcpp::Client<SrvPersonnel>::SharedPtr
    personnel_cli_ptr_ {nullptr};                 /*!< [客户端]人员信息服务 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void UpdateData();                              /*!< 更新数据 */
  SrvPersonnel::Response RequestPersonnelSrv(
    int _service_start_timeout = 3);              /*!< 请求人员服务 */
  StateCode IdToUsername(
    const std::vector<std::string> &,
    std::vector<std::string> &);                  /*!< id 转用户昵称 */
};  // class Personnel
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__PERSONNEL_HPP_
