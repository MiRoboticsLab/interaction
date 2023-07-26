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

#ifndef CYBERDOG_VP_ABILITYSET__SKIN_HPP_
#define CYBERDOG_VP_ABILITYSET__SKIN_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       skin.hpp
    \brief      皮肤模块。
    \details    创建及初始化皮肤模块，以便任务调用皮肤功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Skin final : public Base
{
public:
  Skin()
  : Base(std::string(__FUNCTION__)) {}
  ~Skin() {}
  SkinElectrochromicResponse Electrochromic(
    const int _model = 0,                         /*!< 模式 */
    const int _position = 0,                      /*!< 部位 */
    const int _rendering = 0,                     /*!< 渲染 */
    const int _outset = 0,                        /*!< 起点 */
    const int _duration_ms = 1000                 /*!< 时长（毫秒） */
  );                                              /*!< 电致变色 */

  SkinElectrochromicResponse Discolored(
    const int _model = 0,                         /*!< 模式 */
    const int _duration_ms = 1000                 /*!< 时长（毫秒） */
  );                                              /*!< 电致变色 */

private:
  SkinElectrochromicResponse response_;           /*!< Can0 回值 */
  std::unordered_map<int, std::string> model_map_ = {
    {static_cast<int>(SkinConstraint::model_flash), "model_on"},
    {static_cast<int>(SkinConstraint::model_wavef), "model_on"},
    {static_cast<int>(SkinConstraint::model_random), "model_on"},
    {static_cast<int>(SkinConstraint::model_waveb), "model_on"},
    {static_cast<int>(SkinConstraint::model_control), "model_on"}
  };
  std::unordered_map<int, std::string> position_map_ = {
    {static_cast<int>(SkinConstraint::position_body_middle), "body_middle"},
    {static_cast<int>(SkinConstraint::position_left_back_leg), "left_back_leg"},
    {static_cast<int>(SkinConstraint::position_body_left), "body_left"},
    {static_cast<int>(SkinConstraint::position_left_front_leg), "left_front_leg"},
    {static_cast<int>(SkinConstraint::position_front_chest), "front_chest"},
    {static_cast<int>(SkinConstraint::position_right_front_leg), "right_front_leg"},
    {static_cast<int>(SkinConstraint::position_body_right), "body_right"},
    {static_cast<int>(SkinConstraint::position_right_back_leg), "right_back_leg"}
  };
  rclcpp::CallbackGroup::SharedPtr
    skin_enable_cli_cb_group_ {nullptr};          /*!< [回调组]皮肤使能 */
  rclcpp::CallbackGroup::SharedPtr
    skin_set_cli_cb_group_ {nullptr};             /*!< [回调组]皮肤设置 */

  rclcpp::Client<SrvSetBool>::SharedPtr
    skin_enable_cli_ptr_ {nullptr};               /*!< [客户端]皮肤使能 */
  rclcpp::Client<SrvElecSkin>::SharedPtr
    skin_set_cli_ptr_ {nullptr};                  /*!< [客户端]皮肤设置 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void Can0CB(
    std::string &,
    std::shared_ptr<CanData>);                    /*!< Can0 回调 */
  State RequestEnableSrv(
    SrvSetBool::Response &,
    std::shared_ptr<SrvSetBool::Request>,
    const int _service_start_timeout = 10);       /*!< 请求皮肤使能服务 */
  State RequestSetSrv(
    SrvElecSkin::Response &,
    std::shared_ptr<SrvElecSkin::Request>,
    const int _service_start_timeout = 10);       /*!< 请求皮肤设置服务 */
};  // class Skin
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__SKIN_HPP_
