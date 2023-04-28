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
#include "cyberdog_vp_abilityset/gps.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineGps(py::object m)
{
  py::class_<VPA::Gps, VPA::Base>(m, "GPS", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create GPS object )pbdoc")
  .def_readonly("data", &VPA::Gps::data_, R"pbdoc( data )pbdoc")
  .def(
    "get_data", &VPA::Gps::GetData, R"pbdoc( 获取最新数据 )pbdoc",
    py::arg("timeout") = 5
  )
  .def(
    "__repr__", [](const VPA::Gps & _gps) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: GPS"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _gps.state_.code, _gps.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
