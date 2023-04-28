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
#include "cyberdog_vp_terminal/visual.hpp"

namespace cyberdog_visual_programming_terminal_py
{
namespace VPT = cyberdog_visual_programming_terminal;
void DefineVisual(py::object m)
{
  py::class_<VPT::Visual>(m, "Visual", py::dynamic_attr())
  .def(
    py::init<std::string, std::string, bool, std::string>(),
    R"pbdoc( create Visual object )pbdoc",
    py::arg("task") = "visual",
    py::arg("namespace") = "",
    py::arg("ros") = true,
    py::arg("parameters") = ""
  )

  .def_readwrite("interface", &VPT::Visual::interface_, R"pbdoc( 接口模块 )pbdoc")
  .def_readwrite("debugger", &VPT::Visual::debugger_, R"pbdoc( 调试模块 )pbdoc")

  .def(
    "__repr__", [](const VPT::Visual &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Visual"
          "\n└────────────────────────────────────────---"));
    })
  ;
}
}   // namespace cyberdog_visual_programming_terminal_py
