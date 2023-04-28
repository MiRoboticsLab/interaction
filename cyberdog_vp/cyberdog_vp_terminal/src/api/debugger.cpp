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

#include <string>

#include "vptpy.hpp"
#include "cyberdog_vp_terminal/debugger.hpp"

namespace cyberdog_visual_programming_terminal_py
{
namespace VPT = cyberdog_visual_programming_terminal;
void DefineDebugger(py::object m)
{
  py::class_<VPT::Debugger>(m, "Debugger", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Debugger object )pbdoc")
  .def_readwrite("abilityset", &VPT::Debugger::abilityset_, R"pbdoc( 能力集调试器句柄 )pbdoc")
  .def_readwrite("engine", &VPT::Debugger::engine_, R"pbdoc( 引擎调试器句柄 )pbdoc")

  .def(
    "__repr__", [](const VPT::Debugger &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Debugger"
          "\n└────────────────────────────────────────---"));
    })
  ;
}
}   // namespace cyberdog_visual_programming_terminal_py
