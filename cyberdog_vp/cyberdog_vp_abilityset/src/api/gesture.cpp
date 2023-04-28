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
#include "cyberdog_vp_abilityset/gesture.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineGesture(py::object m)
{
  py::class_<VPA::Gesture, VPA::Base>(m, "Gesture", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Gesture object )pbdoc")

  .def(
    "turn_on_recognition", &VPA::Gesture::TurnOnRecognition, R"pbdoc( 打开识别手势功能 )pbdoc",
    py::arg("duration") = -1
  )
  .def(
    "turn_off_recognition", &VPA::Gesture::TurnOffRecognition, R"pbdoc( 关闭识别手势功能 )pbdoc"
  )
  .def(
    "recognized_designated_gesture", &VPA::Gesture::RecognizedDesignatedGesture,
    R"pbdoc( 识别到指定手势 )pbdoc",
    py::arg("timeout") = -1,
    py::arg("gesture_type") = 0
  )
  .def(
    "recognized_any_gesture", &VPA::Gesture::RecognizedAnyGesture, R"pbdoc( 识别到任意手势 )pbdoc",
    py::arg("timeout") = -1,
    py::arg("sensitivity") = 1
  )
  .def(
    "recognized", &VPA::Gesture::Recognized, R"pbdoc( 开始识别并识别到任意手势 )pbdoc",
    py::arg("duration") = -1,
    py::arg("sensitivity") = 1
  )
  .def(
    "__repr__", [](const VPA::Gesture & _gesture) {
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
          _gesture.state_.code,
          _gesture.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
