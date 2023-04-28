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
#include "cyberdog_vp_abilityset/follow.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineFollow(py::object m)
{
  py::class_<VPA::Follow, VPA::Base>(m, "Follow", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Follow object )pbdoc")
  .def(
    "add_personnel", &VPA::Follow::AddPersonnel, R"pbdoc( 添加人员 )pbdoc",
    py::arg("preset_name") = ""
  )
  .def(
    "delete_personnel", &VPA::Follow::DeletePersonnel, R"pbdoc( 删除人员 )pbdoc",
    py::arg("preset_name") = ""
  )
  .def("cancel_follow", &VPA::Follow::CancelFollow, R"pbdoc( 取消跟随 )pbdoc")
  .def(
    "follow_personnel", &VPA::Follow::FollowPersonnel, R"pbdoc( 跟踪人员 )pbdoc",
    py::arg("preset_name") = "",
    py::arg("intimacy") = 0.0
  )

  .def(
    "__repr__", [](const VPA::Follow & _follow) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Follow"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _follow.state_.code, _follow.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
