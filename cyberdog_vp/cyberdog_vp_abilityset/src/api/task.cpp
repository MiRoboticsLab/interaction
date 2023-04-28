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
#include "cyberdog_vp_abilityset/task.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineTask(py::object m)
{
  py::class_<VPA::Task, VPA::Base>(m, "Task", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Task object )pbdoc")

  .def(
    "start",
    &VPA::Task::Start, R"pbdoc( 开始任务 )pbdoc")
  .def(
    "stop",
    &VPA::Task::Stop, R"pbdoc( 结束任务 )pbdoc")
  .def(
    "recover",
    &VPA::Task::Recover, R"pbdoc( 继续任务 )pbdoc")
  .def(
    "block",
    &VPA::Task::Block, R"pbdoc( 设置块 )pbdoc",
    py::arg("id") = ""
  )
  .def(
    "breakpoint_block",
    &VPA::Task::BreakpointBlock, R"pbdoc( 设置断点块 )pbdoc",
    py::arg("id") = ""
  )

  .def(
    "__repr__", [](const VPA::Task & _task) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Task"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _task.state_.code, _task.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
