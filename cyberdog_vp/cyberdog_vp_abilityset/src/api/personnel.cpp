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
#include "cyberdog_vp_abilityset/personnel.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefinePersonnel(py::object m)
{
  py::class_<VPA::Personnel, VPA::Base>(m, "Personnel", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Personnel object )pbdoc")
  .def(
    "get_data", &VPA::Personnel::GetData, R"pbdoc( 获取人员数据 )pbdoc",
    py::arg("timeout") = 3
  )
  .def(
    "face_recognized", &VPA::Personnel::FaceRecognized, R"pbdoc( 识别到目标人脸 )pbdoc",
    py::arg("personnel_ids"),
    py::arg("and_operation") = false,
    py::arg("duration") = -1
  )
  .def(
    "voiceprint_recognized", &VPA::Personnel::VoiceprintRecognized, R"pbdoc( 识别到目标人员声纹 )pbdoc",
    py::arg("personnel_ids"),
    py::arg("and_operation") = false,
    py::arg("duration") = -1,
    py::arg("sensitivity") = 1
  )
  .def(
    "__repr__", [](const VPA::Personnel & _personnel) {
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
          _personnel.state_.code,
          _personnel.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
