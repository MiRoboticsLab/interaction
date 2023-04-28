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

#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineNavMsgs(py::object m)
{
  DefineMsgOdometry(m);
}

void DefineMsgOdometry(py::object m)
{
  py::class_<nav_msgs::msg::Odometry>(m, "Odometry", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("header", &nav_msgs::msg::Odometry::header, R"pbdoc( header )pbdoc")
  .def_readwrite(
    "child_frame_id", &nav_msgs::msg::Odometry::child_frame_id,
    R"pbdoc( child_frame_id )pbdoc")
  .def_readwrite("pose", &nav_msgs::msg::Odometry::pose, R"pbdoc( pose )pbdoc")
  .def_readwrite("twist", &nav_msgs::msg::Odometry::twist, R"pbdoc( header )pbdoc")
  .def(
    "__repr__", [](const nav_msgs::msg::Odometry & _odom) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: nav_msgs::msg::Odometry"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - child_frame_id = '%s'"
          "\n│  - pose:"
          "\n│    - pose:"
          "\n│      - position:"
          "\n│        - x = %lf"
          "\n│        - y = %lf"
          "\n│        - z = %lf"
          "\n│      - orientation:"
          "\n│        - x = %lf"
          "\n│        - y = %lf"
          "\n│        - z = %lf"
          "\n│        - w = %lf"
          "\n│    - covariance: %s"
          "\n│  - twist:"
          "\n│    - twist:"
          "\n│      - linear:"
          "\n│        - x = %lf"
          "\n│        - y = %lf"
          "\n│        - z = %lf"
          "\n│      - angular:"
          "\n│        - x = %lf"
          "\n│        - y = %lf"
          "\n│        - z = %lf"
          "\n│    - covariance: %s"
          "\n└────────────────────────────────────────---",
          _odom.header.stamp.sec, _odom.header.stamp.nanosec, _odom.header.frame_id.c_str(),
          _odom.child_frame_id.c_str(),
          _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z,
          _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y,
          _odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w,
          VPA::covariance36(_odom.pose.covariance, "\n│                   ").c_str(),
          _odom.twist.twist.linear.x, _odom.twist.twist.linear.y, _odom.twist.twist.linear.z,
          _odom.twist.twist.angular.x, _odom.twist.twist.angular.y, _odom.twist.twist.angular.z,
          VPA::covariance36(_odom.twist.covariance, "\n│                   ").c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
