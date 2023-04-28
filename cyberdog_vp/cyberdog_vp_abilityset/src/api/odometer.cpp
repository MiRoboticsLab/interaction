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
#include "cyberdog_vp_abilityset/odometer.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineOdometer(py::object m)
{
  py::class_<VPA::Odometer, VPA::Base>(m, "Odometer", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Odometer object )pbdoc")
  .def_readonly("data", &VPA::Odometer::data_, R"pbdoc( data )pbdoc")
  .def(
    "get_data", &VPA::Odometer::GetData, R"pbdoc( 获取最新数据 )pbdoc",
    py::arg("timeout") = 5
  )
  .def(
    "__repr__", [](const VPA::Odometer & _odom) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Odometer"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - data:"
          "\n│    - header:"
          "\n│      - stamp:"
          "\n│        - sec = %d"
          "\n│        - nanosec = %d"
          "\n│      - frame_id = '%s'"
          "\n│    - child_frame_id = '%s'"
          "\n│    - pose:"
          "\n│      - pose:"
          "\n│        - position:"
          "\n│          - x = %lf"
          "\n│          - y = %lf"
          "\n│          - z = %lf"
          "\n│        - orientation:"
          "\n│          - x = %lf"
          "\n│          - y = %lf"
          "\n│          - z = %lf"
          "\n│          - w = %lf"
          "\n│      - covariance: %s"
          "\n│    - twist:"
          "\n│      - twist:"
          "\n│        - linear:"
          "\n│          - x = %lf"
          "\n│          - y = %lf"
          "\n│          - z = %lf"
          "\n│        - angular:"
          "\n│          - x = %lf"
          "\n│          - y = %lf"
          "\n│          - z = %lf"
          "\n│      - covariance: %s"
          "\n└────────────────────────────────────────---",
          _odom.state_.code, _odom.state_.describe.c_str(),
          _odom.data_.header.stamp.sec, _odom.data_.header.stamp.nanosec,
          _odom.data_.header.frame_id.c_str(),
          _odom.data_.child_frame_id.c_str(),
          _odom.data_.pose.pose.position.x, _odom.data_.pose.pose.position.y,
          _odom.data_.pose.pose.position.z,
          _odom.data_.pose.pose.orientation.x, _odom.data_.pose.pose.orientation.y,
          _odom.data_.pose.pose.orientation.z, _odom.data_.pose.pose.orientation.w,
          VPA::covariance36(_odom.data_.pose.covariance, "\n│                     ").c_str(),
          _odom.data_.twist.twist.linear.x, _odom.data_.twist.twist.linear.y,
          _odom.data_.twist.twist.linear.z,
          _odom.data_.twist.twist.angular.x, _odom.data_.twist.twist.angular.y,
          _odom.data_.twist.twist.angular.z,
          VPA::covariance36(_odom.data_.twist.covariance, "\n│                     ").c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
