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
#include "cyberdog_vp_abilityset/skeleton.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineSkin(py::object m)
{
  py::class_<VPA::Skin, VPA::Base>(m, "Skin", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Skin object )pbdoc")
  .def(
    "electrochromic", &VPA::Skin::Electrochromic, R"pbdoc( 电致变色 )pbdoc",
    py::arg("model") = 0,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )
  .def(
    "__repr__", [](const VPA::Skin & _skin) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: State"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _skin.state_.code,
          _skin.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
