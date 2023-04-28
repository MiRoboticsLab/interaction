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

void DefineSkeleton(py::object m)
{
  py::class_<VPA::Skeleton, VPA::Base>(m, "Skeleton", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Skeleton object )pbdoc")
  .def(
    "turn_on_recognition", &VPA::Skeleton::TurnOnRecognition, R"pbdoc( 打开识别骨骼（点）功能 )pbdoc",
    py::arg("sport_type") = 0,
    py::arg("counts") = 0,
    py::arg("timeout") = -1
  )
  .def(
    "turn_off_recognition", &VPA::Skeleton::TurnOffRecognition, R"pbdoc( 关闭识别骨骼（点）功能 )pbdoc"
  )
  .def(
    "blocking_recognized", &VPA::Skeleton::BlockingRecognized, R"pbdoc( 阻塞式识别到 )pbdoc",
    py::arg("timeout") = -1
  )
  .def(
    "instant_recognized", &VPA::Skeleton::InstantRecognized, R"pbdoc( 瞬时式识别到 )pbdoc"
  )
  .def(
    "sports_recognition", &VPA::Skeleton::SportsRecognition, R"pbdoc( 运动识别 )pbdoc",
    py::arg("sport_type") = 0,
    py::arg("counts") = 0,
    py::arg("timeout") = -1,
    py::arg("interact") = true,
    py::arg("instantly") = true,
    py::arg("volume") = 50
  )
  .def(
    "__repr__", [](const VPA::Skeleton & _skeleton) {
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
          _skeleton.state_.code,
          _skeleton.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
