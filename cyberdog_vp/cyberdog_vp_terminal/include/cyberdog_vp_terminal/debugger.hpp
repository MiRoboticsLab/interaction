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

#ifndef CYBERDOG_VP_TERMINAL__DEBUGGER_HPP_
#define CYBERDOG_VP_TERMINAL__DEBUGGER_HPP_

#include <cyberdog_vp_abilityset/cyberdog.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "cyberdog_vp_terminal/common.hpp"
#include "cyberdog_vp_terminal/debug_abilityset.hpp"
#include "cyberdog_vp_terminal/debug_engine.hpp"

namespace cyberdog_visual_programming_terminal
{
/*! \file       debugger.hpp
    \brief      调试器模块。
    \details    创建及初始化机器人调试器，以便调试。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
namespace VPA = cyberdog_visual_programming_abilityset;
class Debugger
{
public:
  Debugger();
  ~Debugger();

public:
  DebugAbilityset abilityset_;                    /*!< 能力集 */
  DebugEngine engine_;                            /*!< 引擎 */

private:
  std::string logger_ {""};                       /*!< 日志名称 */
};  // class Debugger
}  // namespace cyberdog_visual_programming_terminal
#endif  // CYBERDOG_VP_TERMINAL__DEBUGGER_HPP_
