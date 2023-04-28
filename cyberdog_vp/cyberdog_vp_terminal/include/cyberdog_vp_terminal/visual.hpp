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

#ifndef CYBERDOG_VP_TERMINAL__VISUAL_HPP_
#define CYBERDOG_VP_TERMINAL__VISUAL_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_terminal/common.hpp"

#include "cyberdog_vp_terminal/interface.hpp"
#include "cyberdog_vp_terminal/debugger.hpp"

namespace cyberdog_visual_programming_terminal
{
namespace VPA = cyberdog_visual_programming_abilityset;
/*! \file       visual.hpp
    \brief      可视化模块。
    \details    创建及初始化可视化，以便用户编辑。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        ROS2 环境。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Visual
{
public:
  Visual(
    std::string _task = "visual",
    std::string _namespace = "",
    bool _ros = false,
    std::string _parameters = "");
  ~Visual();
  std::shared_ptr<VPA::Cyberdog>
  cyberdog_ptr_ {nullptr};                        /*!< 铁蛋 */

  Interface interface_;                           /*!< 接口模块 */
  Debugger debugger_;                             /*!< 调试模块 */
};  // class Visual
}  // namespace cyberdog_visual_programming_terminal
#endif  // CYBERDOG_VP_TERMINAL__VISUAL_HPP_
