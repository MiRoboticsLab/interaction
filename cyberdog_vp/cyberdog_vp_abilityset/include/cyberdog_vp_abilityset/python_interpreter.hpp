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
#ifndef CYBERDOG_VP_ABILITYSET__PYTHON_INTERPRETER_HPP_
#define CYBERDOG_VP_ABILITYSET__PYTHON_INTERPRETER_HPP_

#include <string>
#include <map>
#include <vector>

#include "cyberdog_vp_abilityset/common.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       python_interpreter.hpp
    \brief      python 解释器模块。
    \details    负责执行 python 。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        注册表有效。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class PythonInterpreter
{
public:
  PythonInterpreter() {}
  ~PythonInterpreter() {}
  bool Init();                                    /*!< 初始化 */
  bool Choreographer(
    const uint64_t,
    const std::string,
    const py::args);                              /*!< 编舞 */
  bool Choreographer(
    const uint64_t,
    const py::kwargs);                            /*!< 编舞 */

private:
};  // class PythonInterpreter
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__PYTHON_INTERPRETER_HPP_
