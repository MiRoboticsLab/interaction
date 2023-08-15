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
#include "cyberdog_vp_abilityset/skin.hpp"

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
    "discolored", &VPA::Skin::Discolored, R"pbdoc( 变色 )pbdoc",
    py::arg("model") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "flashing", &VPA::Skin::Electrochromic, R"pbdoc( 闪烁 )pbdoc",
    py::arg("model") = 0,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "gradient", &VPA::Skin::Electrochromic, R"pbdoc( 渐变（由前向后变深） )pbdoc",
    py::arg("model") = 1,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "random", &VPA::Skin::Electrochromic, R"pbdoc( 随机 )pbdoc",
    py::arg("model") = 2,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "keep_it_dark", &VPA::Skin::Electrochromic, R"pbdoc( 保持深色 )pbdoc",
    py::arg("model") = 4,
    py::arg("position") = 8,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "keep_it_light", &VPA::Skin::Electrochromic, R"pbdoc( 保持浅色 )pbdoc",
    py::arg("model") = 4,
    py::arg("position") = 8,
    py::arg("rendering") = 1,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "light_dynamic", &VPA::Skin::Electrochromic, R"pbdoc( 浅色动态 )pbdoc",
    py::arg("model") = 5,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "dark_dynamic", &VPA::Skin::Electrochromic, R"pbdoc( 深色动态 )pbdoc",
    py::arg("model") = 5,
    py::arg("position") = 0,
    py::arg("rendering") = 1,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 1000
  )

  .def(
    "flashing_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速闪烁 )pbdoc",
    py::arg("model") = 0,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "gradient_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速渐变（由前向后变深） )pbdoc",
    py::arg("model") = 1,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "random_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速随机 )pbdoc",
    py::arg("model") = 2,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "keep_it_dark_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速保持深色 )pbdoc",
    py::arg("model") = 4,
    py::arg("position") = 8,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "keep_it_light_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速保持浅色 )pbdoc",
    py::arg("model") = 4,
    py::arg("position") = 8,
    py::arg("rendering") = 1,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "light_dynamic_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速浅色动态 )pbdoc",
    py::arg("model") = 5,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "dark_dynamic_slow", &VPA::Skin::Electrochromic, R"pbdoc( 慢速深色动态 )pbdoc",
    py::arg("model") = 5,
    py::arg("position") = 0,
    py::arg("rendering") = 1,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 2000
  )

  .def(
    "flashing_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速闪烁 )pbdoc",
    py::arg("model") = 0,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
  )

  .def(
    "gradient_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速渐变（由前向后变深） )pbdoc",
    py::arg("model") = 1,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
  )

  .def(
    "random_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速随机 )pbdoc",
    py::arg("model") = 2,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
  )

  .def(
    "keep_it_dark_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速保持深色 )pbdoc",
    py::arg("model") = 4,
    py::arg("position") = 8,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
  )

  .def(
    "keep_it_light_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速保持浅色 )pbdoc",
    py::arg("model") = 4,
    py::arg("position") = 8,
    py::arg("rendering") = 1,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
  )

  .def(
    "light_dynamic_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速浅色动态 )pbdoc",
    py::arg("model") = 5,
    py::arg("position") = 0,
    py::arg("rendering") = 0,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
  )

  .def(
    "dark_dynamic_fast", &VPA::Skin::Electrochromic, R"pbdoc( 快速深色动态 )pbdoc",
    py::arg("model") = 5,
    py::arg("position") = 0,
    py::arg("rendering") = 1,
    py::arg("outset") = 0,
    py::arg("duration_ms") = 500
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
