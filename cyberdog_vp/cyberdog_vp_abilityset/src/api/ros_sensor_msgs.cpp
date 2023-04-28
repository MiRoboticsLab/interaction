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

#include <sensor_msgs/msg/range.hpp>

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineSensorMsgs(py::object m)
{
  DefineMsgRange(m);
  DefineMsgLaserScan(m);
  DefineMsgImu(m);
}

void DefineMsgRange(py::object m)
{
  py::class_<sensor_msgs::msg::Range>(m, "Range", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("header", &sensor_msgs::msg::Range::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite(
    "radiation_type", &sensor_msgs::msg::Range::radiation_type,
    R"pbdoc( 传感器使用的辐射类型 )pbdoc")
  .def_readwrite(
    "field_of_view", &sensor_msgs::msg::Range::field_of_view,
    R"pbdoc( 距离读数的圆弧大小 )pbdoc")
  .def_readwrite("min_range", &sensor_msgs::msg::Range::min_range, R"pbdoc( 最小范围值[m] )pbdoc")
  .def_readwrite("max_range", &sensor_msgs::msg::Range::max_range, R"pbdoc( 最大范围值[m] )pbdoc")
  .def_readwrite("range", &sensor_msgs::msg::Range::range, R"pbdoc( 范围数据[m] )pbdoc")
  .def(
    "__repr__", [](const sensor_msgs::msg::Range & _range) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: sensor_msgs::msg::Range"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - radiation_type = %d"
          "\n│  - field_of_view = %f"
          "\n│  - min_range = %f"
          "\n│  - max_range = %f"
          "\n│  - range = %f"
          "\n└────────────────────────────────────────---",
          _range.header.stamp.sec, _range.header.stamp.nanosec, _range.header.frame_id.c_str(),
          _range.radiation_type, _range.field_of_view, _range.min_range, _range.max_range,
          _range.range));
    })
  ;
}

void DefineMsgLaserScan(py::object m)
{
  py::class_<sensor_msgs::msg::LaserScan>(m, "LaserScan", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("header", &sensor_msgs::msg::LaserScan::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite(
    "angle_min", &sensor_msgs::msg::LaserScan::angle_min,
    R"pbdoc( 扫描的起始角度[rad] )pbdoc")
  .def_readwrite("angle_max", &sensor_msgs::msg::LaserScan::angle_max, R"pbdoc( 扫描结束角度[rad] )pbdoc")
  .def_readwrite(
    "angle_increment", &sensor_msgs::msg::LaserScan::angle_increment,
    R"pbdoc( 测量之间的角距离[rad] )pbdoc")
  .def_readwrite(
    "time_increment", &sensor_msgs::msg::LaserScan::time_increment,
    R"pbdoc( 测量之间的时间[s] )pbdoc")
  .def_readwrite(
    "scan_time", &sensor_msgs::msg::LaserScan::scan_time,
    R"pbdoc( 两次扫描之间的时间[秒] )pbdoc")
  .def_readwrite("range_min", &sensor_msgs::msg::LaserScan::range_min, R"pbdoc( 最小范围值[m] )pbdoc")
  .def_readwrite("range_max", &sensor_msgs::msg::LaserScan::range_max, R"pbdoc( 最大范围值[m] )pbdoc")
  .def_readwrite(
    "ranges", &sensor_msgs::msg::LaserScan::ranges,
    py::return_value_policy::reference_internal, R"pbdoc( 范围数据[m] )pbdoc")
  .def_readwrite(
    "intensities", &sensor_msgs::msg::LaserScan::intensities,
    py::return_value_policy::reference_internal, R"pbdoc( 强度数据[m] )pbdoc")
  .def(
    "__repr__", [](const sensor_msgs::msg::LaserScan & _scan) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: sensor_msgs::msg::LaserScan"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - angle_min = %f"
          "\n│  - angle_max = %f"
          "\n│  - angle_increment = %f"
          "\n│  - time_increment = %f"
          "\n│  - scan_time = %f"
          "\n│  - range_min = %f"
          "\n│  - range_max = %f"
          "\n│  - ranges = %ld"
          "\n│  - intensities = %ld"
          "\n└────────────────────────────────────────---",
          _scan.header.stamp.sec, _scan.header.stamp.nanosec, _scan.header.frame_id.c_str(),
          _scan.angle_min,
          _scan.angle_max,
          _scan.angle_increment,
          _scan.time_increment,
          _scan.scan_time,
          _scan.range_min,
          _scan.range_max,
          _scan.ranges.size(),
          _scan.intensities.size()));
    })
  ;
}

void DefineMsgImu(py::object m)
{
  py::class_<sensor_msgs::msg::Imu>(m, "Imu", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("header", &sensor_msgs::msg::Imu::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite("orientation", &sensor_msgs::msg::Imu::orientation, R"pbdoc( 方向[rad] )pbdoc")
  .def_readwrite(
    "orientation_covariance", &sensor_msgs::msg::Imu::orientation_covariance,
    py::return_value_policy::reference_internal, R"pbdoc( 方向协方差 )pbdoc")
  .def_readwrite(
    "angular_velocity", &sensor_msgs::msg::Imu::angular_velocity,
    R"pbdoc( 角速度[rad/s] )pbdoc")
  .def_readwrite(
    "angular_velocity_covariance", &sensor_msgs::msg::Imu::angular_velocity_covariance,
    py::return_value_policy::reference_internal, R"pbdoc( 角速度协方差 )pbdoc")
  .def_readwrite(
    "linear_acceleration", &sensor_msgs::msg::Imu::linear_acceleration,
    R"pbdoc( 线性加速度[m/s^2] )pbdoc")
  .def_readwrite(
    "linear_acceleration_covariance",
    &sensor_msgs::msg::Imu::linear_acceleration_covariance,
    py::return_value_policy::reference_internal, R"pbdoc( 线性加速度协方差 )pbdoc")
  .def(
    "__repr__", [](const sensor_msgs::msg::Imu & _imu) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: sensor_msgs::msg::Imu"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - orientation:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│    - w = %lf"
          "\n│  - orientation_covariance: %s"
          "\n│  - angular_velocity:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│  - angular_velocity_covariance: %s"
          "\n│  - linear_acceleration:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│  - linear_acceleration_covariance: %s"
          "\n└────────────────────────────────────────---",
          _imu.header.stamp.sec,
          _imu.header.stamp.nanosec,
          _imu.header.frame_id.c_str(),
          _imu.orientation.x,
          _imu.orientation.y,
          _imu.orientation.z,
          _imu.orientation.w,
          VPA::covariance9(_imu.orientation_covariance, "\n│                             ").c_str(),
          _imu.angular_velocity.x,
          _imu.angular_velocity.y,
          _imu.angular_velocity.z,
          VPA::covariance9(
            _imu.angular_velocity_covariance,
            "\n│                                  ").c_str(),
          _imu.linear_acceleration.x,
          _imu.linear_acceleration.y,
          _imu.linear_acceleration.z,
          VPA::covariance9(
            _imu.linear_acceleration_covariance,
            "\n│                                     ").c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
