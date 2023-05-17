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
#include "cyberdog_vp_abilityset/audio.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineAudio(py::object m)
{
  py::class_<VPA::Audio, VPA::Base>(m, "Audio", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Audio object )pbdoc")
  .def(
    "play", &VPA::Audio::OnlinePlay, R"pbdoc( 阻塞式:离线:播放语音 )pbdoc",
    py::arg("message") = "我是谁",
    py::arg("volume") = -1
  )
  .def(
    "instantly_play", &VPA::Audio::OnlineInstantlyPlay, R"pbdoc( 立即式:离线:播放语音 )pbdoc",
    py::arg("message") = "我是谁",
    py::arg("volume") = -1
  )
  .def(
    "offline_play", &VPA::Audio::OfflinePlay, R"pbdoc( 阻塞式:离线:播放语音 )pbdoc",
    py::arg("audio_id") = 4000,
    py::arg("volume") = -1
  )
  .def(
    "offline_instantly_play", &VPA::Audio::OfflineInstantlyPlay, R"pbdoc( 立即式:离线:播放语音 )pbdoc",
    py::arg("audio_id") = 4000,
    py::arg("volume") = -1
  )
  .def(
    "get_volume", &VPA::Audio::GetVolume, R"pbdoc( 获取音量 )pbdoc"
  )
  .def(
    "set_volume", &VPA::Audio::SetVolume, R"pbdoc( 设置音量 )pbdoc",
    py::arg("volume")
  )
  .def(
    "turn_on_dialogue", &VPA::Audio::SetDialogue, R"pbdoc( 打开对话 )pbdoc",
    py::arg("volume") = true
  )
  .def(
    "turn_off_dialogue", &VPA::Audio::SetDialogue, R"pbdoc( 关闭对话 )pbdoc",
    py::arg("volume") = false
  )
  .def(
    "reset_user_dialogue", &VPA::Audio::ResetUserDialogue, R"pbdoc( 重置用户对话 )pbdoc"
  )
  .def(
    "get_user_dialogue", &VPA::Audio::GetUserDialogue, R"pbdoc( 获取用户对话 )pbdoc",
    py::arg("timeout") = 3
  )
  .def(
    "__repr__", [](const VPA::Audio & _audio) {
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
          _audio.state_.code, _audio.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
