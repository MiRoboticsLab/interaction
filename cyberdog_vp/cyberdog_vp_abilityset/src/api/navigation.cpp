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
#include "cyberdog_vp_abilityset/navigation.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineNavigation(py::object m)
{
  py::class_<VPA::Navigation, VPA::Base>(m, "Navigation", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Navigation object )pbdoc")

  .def(
    "get_preset",
    &VPA::Navigation::GetPreset,
    R"pbdoc( 获取预置点 )pbdoc",
    py::arg("timeout") = 3
  )
  .def(
    "turn_on_navigation",
    &VPA::Navigation::TurnOnNavigation,
    R"pbdoc( 开启（进入）导航模式 )pbdoc",
    py::arg("outdoor") = false,
    py::arg("assisted_relocation") = false,
    py::arg("interact") = false,
    py::arg("volume") = 50
  )
  .def(
    "turn_off_navigation",
    &VPA::Navigation::TurnOffNavigation,
    R"pbdoc( 关闭（退出）导航模式 )pbdoc"
  )
  .def(
    "cancel_navigation",
    &VPA::Navigation::CancelNavigation,
    R"pbdoc( 取消导航 )pbdoc",
    py::arg("timeout") = 3
  )
  .def(
    "navigation_to_preset",
    &VPA::Navigation::NavigationToPreset,
    R"pbdoc( 导航到预置点 )pbdoc",
    py::arg("preset_id") = ""
  )
  .def(
    "navigation_to_coordinates",
    &VPA::Navigation::NavigationToCoordinates,
    R"pbdoc( 导航到坐标点 )pbdoc",
    py::arg("is_outdoor") = false,
    py::arg("x") = 0,
    py::arg("y") = 0,
    py::arg("z") = 0,
    py::arg("roll") = 0,
    py::arg("pitch") = 0,
    py::arg("yaw") = 0
  )
  .def(
    "navigation_to_pose",
    &VPA::Navigation::NavigationToPose,
    R"pbdoc( 导航到目标位姿 )pbdoc",
    py::arg("pose")
  )
  .def(
    "to_preset",
    &VPA::Navigation::ToPreset,
    R"pbdoc( 去预置点 )pbdoc",
    py::arg("preset_id") = "",
    py::arg("assisted_relocation") = false,
    py::arg("interact") = false,
    py::arg("volume") = 50
  )

  .def(
    "__repr__", [](const VPA::Navigation & _navigation) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Navigation"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _navigation.state_.code, _navigation.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
