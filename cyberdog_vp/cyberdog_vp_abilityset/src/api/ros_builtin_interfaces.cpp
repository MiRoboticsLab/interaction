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

#include <builtin_interfaces/msg/time.hpp>

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineBuiltinInterfaces(py::object m)
{
  DefineTimer(m);
}

void DefineTimer(py::object m)
{
  py::class_<builtin_interfaces::msg::Time>(m, "Time", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("sec", &builtin_interfaces::msg::Time::sec, R"pbdoc( 秒 )pbdoc")
  .def_readwrite("nanosec", &builtin_interfaces::msg::Time::nanosec, R"pbdoc( 纳秒 )pbdoc")
  .def(
    "__repr__", [](const builtin_interfaces::msg::Time & _time) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: builtin_interfaces::msg::Time"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - sec = %d"
          "\n│  - nanosec = '%d'"
          "\n└────────────────────────────────────────---",
          _time.sec, _time.nanosec));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
