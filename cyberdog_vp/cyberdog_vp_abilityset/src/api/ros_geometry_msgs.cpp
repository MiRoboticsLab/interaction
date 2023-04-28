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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineGeometryMsgs(py::object m)
{
  DefinePoint(m);
  DefineQuaternion(m);
  DefinePose(m);
  DefineVector3(m);
  DefineTwist(m);
  DefinePoseWithCovariance(m);
  DefineTwistWithCovariance(m);
}

void DefinePoint(py::object m)
{
  py::class_<geometry_msgs::msg::Point>(m, "Point", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("x", &geometry_msgs::msg::Point::x, R"pbdoc( x )pbdoc")
  .def_readwrite("y", &geometry_msgs::msg::Point::y, R"pbdoc( y )pbdoc")
  .def_readwrite("z", &geometry_msgs::msg::Point::z, R"pbdoc( z )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::Point & _point) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::Point"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - x = %lf"
          "\n│  - y = %lf"
          "\n│  - z = %lf"
          "\n└────────────────────────────────────────---",
          _point.x, _point.y, _point.z));
    })
  ;
}

void DefineQuaternion(py::object m)
{
  py::class_<geometry_msgs::msg::Quaternion>(m, "Quaternion", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("x", &geometry_msgs::msg::Quaternion::x, R"pbdoc( x )pbdoc")
  .def_readwrite("y", &geometry_msgs::msg::Quaternion::y, R"pbdoc( y )pbdoc")
  .def_readwrite("z", &geometry_msgs::msg::Quaternion::z, R"pbdoc( z )pbdoc")
  .def_readwrite("w", &geometry_msgs::msg::Quaternion::w, R"pbdoc( w )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::Quaternion & _point) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::Quaternion"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - x = %lf"
          "\n│  - y = %lf"
          "\n│  - z = %lf"
          "\n│  - w = %lf"
          "\n└────────────────────────────────────────---",
          _point.x, _point.y, _point.z, _point.w));
    })
  ;
}

void DefinePose(py::object m)
{
  py::class_<geometry_msgs::msg::Pose>(m, "Pose", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "position", &geometry_msgs::msg::Pose::position,
    R"pbdoc( position )pbdoc")
  .def_readwrite(
    "orientation", &geometry_msgs::msg::Pose::orientation,
    R"pbdoc( orientation )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::Pose & _pose) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::Pose"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - position:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│  - orientation:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│    - w = %lf"
          "\n└────────────────────────────────────────---",
          _pose.position.x, _pose.position.y, _pose.position.z,
          _pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w));
    })
  ;
}

void DefineVector3(py::object m)
{
  py::class_<geometry_msgs::msg::Vector3>(m, "Vector3", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("x", &geometry_msgs::msg::Vector3::x, R"pbdoc( x )pbdoc")
  .def_readwrite("y", &geometry_msgs::msg::Vector3::y, R"pbdoc( y )pbdoc")
  .def_readwrite("z", &geometry_msgs::msg::Vector3::z, R"pbdoc( z )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::Vector3 & _v3) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::Vector3"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - x = %lf"
          "\n│  - y = %lf"
          "\n│  - z = %lf"
          "\n└────────────────────────────────────────---",
          _v3.x, _v3.y, _v3.z));
    })
  ;
}

void DefineTwist(py::object m)
{
  py::class_<geometry_msgs::msg::Twist>(m, "Twist", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "linear", &geometry_msgs::msg::Twist::linear,
    R"pbdoc( linear )pbdoc")
  .def_readwrite(
    "angular", &geometry_msgs::msg::Twist::angular,
    R"pbdoc( angular )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::Twist & _twist) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::Twist"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - linear:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│  - angular:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n└────────────────────────────────────────---",
          _twist.linear.x, _twist.linear.y, _twist.linear.z,
          _twist.angular.x, _twist.angular.y, _twist.angular.z));
    })
  ;
}

void DefinePoseWithCovariance(py::object m)
{
  py::class_<geometry_msgs::msg::PoseWithCovariance>(m, "PoseWithCovariance", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "pose", &geometry_msgs::msg::PoseWithCovariance::pose,
    R"pbdoc( pose )pbdoc")
  .def_readwrite(
    "covariance", &geometry_msgs::msg::PoseWithCovariance::covariance,
    py::return_value_policy::reference_internal,
    R"pbdoc( covariance )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::PoseWithCovariance & _pose) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::PoseWithCovariance"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - pose:"
          "\n│    - position:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│    - orientation:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│      - w = %lf"
          "\n│  - covariance: %s"
          "\n└────────────────────────────────────────---",
          _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z,
          _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z,
          _pose.pose.orientation.w,
          VPA::covariance36(_pose.covariance, "\n│                 ").c_str()));
    })
  ;
}

void DefineTwistWithCovariance(py::object m)
{
  py::class_<geometry_msgs::msg::TwistWithCovariance>(m, "TwistWithCovariance", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "twist", &geometry_msgs::msg::TwistWithCovariance::twist,
    R"pbdoc( twist )pbdoc")
  .def_readwrite(
    "covariance", &geometry_msgs::msg::TwistWithCovariance::covariance,
    py::return_value_policy::reference_internal,
    R"pbdoc( covariance )pbdoc")
  .def(
    "__repr__", [](const geometry_msgs::msg::TwistWithCovariance & _twist) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: geometry_msgs::msg::TwistWithCovariance"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - twist:"
          "\n│    - linear:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│    - angular:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│  - covariance: %s"
          "\n└────────────────────────────────────────---",
          _twist.twist.linear.x, _twist.twist.linear.y, _twist.twist.linear.z,
          _twist.twist.angular.x, _twist.twist.angular.y, _twist.twist.angular.z,
          VPA::covariance36(_twist.covariance, "\n│                 ").c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
