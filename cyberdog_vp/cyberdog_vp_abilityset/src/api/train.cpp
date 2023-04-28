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
#include <memory>

#include "vpapy.hpp"
#include "cyberdog_vp_abilityset/train.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineTrain(py::object m)
{
  py::class_<VPA::Train, VPA::Base>(m, "Train", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Train object )pbdoc")

  .def(
    "get_training_words_set", &VPA::Train::GetTrainingWordsSet, R"pbdoc( 获取训练词集合 )pbdoc"
  )
  .def(
    "training_words_recognized", &VPA::Train::TrainingWordsRecognized, R"pbdoc( 识别到训练词 )pbdoc",
    py::arg("timeout") = -1
  )
  .def(
    "__repr__", [](const VPA::Train & _train) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Train"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _train.state_.code, _train.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
