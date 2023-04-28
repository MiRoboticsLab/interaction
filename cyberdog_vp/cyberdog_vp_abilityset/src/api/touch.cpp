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
#include "cyberdog_vp_abilityset/touch.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineTouch(py::object m)
{
  py::class_<VPA::Touch, VPA::Base>(m, "Touch", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Touch object )pbdoc")
  .def_readonly("data", &VPA::Touch::data_, R"pbdoc( 数据 )pbdoc")
  .def(
    "get_data", &VPA::Touch::GetData, R"pbdoc( 获取最新数据 )pbdoc",
    py::arg("timeout") = 5
  )
  .def(
    "__repr__", [](const VPA::Touch & _touch) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Touch"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - data:"
          "\n│    - header:"
          "\n│      - stamp:"
          "\n│        - sec = %d"
          "\n│        - nanosec = %d"
          "\n│      - frame_id = '%s'"
          "\n│    - touch_state = %d"
          "\n└────────────────────────────────────────---",
          _touch.state_.code, _touch.state_.describe.c_str(),
          _touch.data_.header.stamp.sec, _touch.data_.header.stamp.nanosec,
          _touch.data_.header.frame_id.c_str(),
          _touch.data_.touch_state));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
