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

#include <std_msgs/msg/header.hpp>

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineStdMsgs(py::object m)
{
  DefineHeader(m);
}

void DefineHeader(py::object m)
{
  py::class_<std_msgs::msg::Header>(m, "Header", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("stamp", &std_msgs::msg::Header::stamp, R"pbdoc( 时间戳 )pbdoc")
  .def_readwrite("frame_id", &std_msgs::msg::Header::frame_id, R"pbdoc( 帧id )pbdoc")
  .def(
    "__repr__", [](const std_msgs::msg::Header & _header) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: std_msgs::msg::Header"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - stamp:"
          "\n│    - sec = %d"
          "\n│    - nanosec = %d"
          "\n│  - frame_id = '%s'"
          "\n└────────────────────────────────────────---",
          _header.stamp.sec, _header.stamp.nanosec, _header.frame_id.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
