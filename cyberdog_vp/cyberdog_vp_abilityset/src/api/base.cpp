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

#include "vpapy.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineBase(py::object m)
{
  py::class_<VPA::Base>(m, "Base", py::dynamic_attr())
  .def(
    py::init<std::string>(),
    R"pbdoc( create Base object )pbdoc",
    py::arg("logger") = "base"
  )
  .def(
    "set_log", &VPA::Base::SetLog, R"pbdoc( 设置日志 )pbdoc",
    py::arg("log") = false
  )
  .def_readonly("state", &VPA::Base::state_, R"pbdoc( Base 状态 )pbdoc")
  .def(
    "__repr__", [](const VPA::Base & _base) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Base"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _base.state_.code, _base.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
