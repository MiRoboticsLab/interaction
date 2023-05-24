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
#include "cyberdog_vp_abilityset/cyberdog.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineCyberdog(py::object m)
{
  py::class_<VPA::Cyberdog, VPA::Base>(m, "Cyberdog", py::dynamic_attr())
  .def(
    py::init<std::string, std::string, bool, std::string>(),
    R"pbdoc( create Cyberdog object )pbdoc",
    py::arg("task") = "terminal",
    py::arg("namespace") = "",
    py::arg("ros") = true,
    py::arg("parameters") = ""
    // "task"_a = "terminal", "namespace"_a = "", "ros"_a = true, "parameters"_a = ""
    // (error: assignBoolToPointer) Boolean value assigned to pointer.
  )
  .def(
    "shutdown", &VPA::Cyberdog::Shutdown, R"pbdoc( 终止 )pbdoc",
    py::arg("exit") = true)
  .def(
    "ready", &VPA::Cyberdog::Ready, R"pbdoc( 准备就绪 )pbdoc",
    py::arg("timeout") = 30
  )
  .def_readwrite("network", &VPA::Cyberdog::network_, R"pbdoc( 网络模块句柄 )pbdoc")
  .def_readwrite("follow", &VPA::Cyberdog::follow_, R"pbdoc( 跟随模块句柄 )pbdoc")
  .def_readwrite("motion", &VPA::Cyberdog::motion_, R"pbdoc( 运动模块句柄 )pbdoc")
  .def_readwrite("navigation", &VPA::Cyberdog::navigation_, R"pbdoc( 导航模块句柄 )pbdoc")
  .def_readwrite("task", &VPA::Cyberdog::task_, R"pbdoc( 任务模块句柄 )pbdoc")
  .def_readwrite("train", &VPA::Cyberdog::train_, R"pbdoc( 训练模块句柄 )pbdoc")
  .def_readwrite("personnel", &VPA::Cyberdog::personnel_, R"pbdoc( 人员模块句柄 )pbdoc")
  .def_readwrite("gesture", &VPA::Cyberdog::gesture_, R"pbdoc( 手势识别模块句柄 )pbdoc")
  .def_readwrite("skeleton", &VPA::Cyberdog::skeleton_, R"pbdoc( 骨骼（点）识别模块句柄 )pbdoc")

  .def_readwrite("audio", &VPA::Cyberdog::audio_, R"pbdoc( 语音模块 )pbdoc")
  .def_readwrite("led", &VPA::Cyberdog::led_, R"pbdoc( LED灯 )pbdoc")
  .def_readwrite("bms", &VPA::Cyberdog::bms_, R"pbdoc( 电池管理系统 )pbdoc")
  .def_readwrite("touch", &VPA::Cyberdog::touch_, R"pbdoc( 触摸板 )pbdoc")
  .def_readwrite("gps", &VPA::Cyberdog::gps_, R"pbdoc( 全球定位系统 )pbdoc")
  .def_readwrite("tof", &VPA::Cyberdog::tof_, R"pbdoc( 激光测距 )pbdoc")
  .def_readwrite("lidar", &VPA::Cyberdog::lidar_, R"pbdoc( 雷达 )pbdoc")
  .def_readwrite("ultrasonic", &VPA::Cyberdog::ultrasonic_, R"pbdoc( 超声波 )pbdoc")
  .def_readwrite("odometer", &VPA::Cyberdog::odometer_, R"pbdoc( 里程计 )pbdoc")
  .def_readwrite("imu", &VPA::Cyberdog::imu_, R"pbdoc( 惯导 )pbdoc")

  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
