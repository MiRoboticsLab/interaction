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

#ifndef CYBERDOG_VP_ABILITYSET__FOLLOW_HPP_
#define CYBERDOG_VP_ABILITYSET__FOLLOW_HPP_

#include <string>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       follow.hpp
    \brief      跟随模块。
    \details    创建及初始化机器人跟随能力，以便任务调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Follow final : public Base
{
public:
  Follow()
  : Base(std::string(__FUNCTION__)) {}
  ~Follow() {}
  State AddPersonnel(
    const std::string);                           /*!< 添加人员 */
  State DeletePersonnel(
    const std::string);                           /*!< 删除人员 */
  State CancelFollow();                           /*!< 取消跟随 */
  State FollowPersonnel(
    const std::string, const double);             /*!< 跟踪人员 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
};  // class Follow
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__FOLLOW_HPP_
