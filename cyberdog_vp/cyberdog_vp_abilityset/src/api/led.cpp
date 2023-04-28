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
#include "cyberdog_vp_abilityset/led.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineLed(py::object m)
{
  py::class_<VPA::Led, VPA::Base>(m, "LED", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create LED object )pbdoc")
  .def(
    "play", &VPA::Led::Play, R"pbdoc( 设置led系统灯效 )pbdoc",
    py::arg("target"),
    py::arg("effect")
  )
  .def(
    "play_rgb", &VPA::Led::PlayRgb, R"pbdoc( 设置led用户定义RGB灯效 )pbdoc",
    py::arg("target"),
    py::arg("effect"),
    py::arg("r"),
    py::arg("g"),
    py::arg("b")
  )
  .def(
    "freed", &VPA::Led::Freed, R"pbdoc( 释放led设备 )pbdoc",
    py::arg("target")
  )

  .def(
    "__repr__", [](const VPA::Led & _led) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: LED"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _led.state_.code, _led.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
