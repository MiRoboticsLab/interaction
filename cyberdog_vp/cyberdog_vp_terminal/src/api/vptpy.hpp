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

#ifndef API__VPTPY_HPP_
#define API__VPTPY_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace cyberdog_visual_programming_terminal_py
{
namespace py = pybind11;

void DefineInterfaceNetwork(py::object);          /*!< 定义 Network */
void DefineInterfaceFollow(py::object);           /*!< 定义 Follow */
void DefineInterfaceMotion(py::object);           /*!< 定义 Motion */
void DefineInterfaceNavigation(py::object);       /*!< 定义 Navigation */
void DefineInterfaceTask(py::object);             /*!< 定义 Task */
void DefineInterfaceTrain(py::object);            /*!< 定义 Train */
void DefineInterfacePersonnel(py::object);        /*!< 定义 Personnel */
void DefineInterfaceGesture(py::object);          /*!< 定义 Gesture */
void DefineInterfaceSkeleton(py::object);         /*!< 定义 Skeleton */

void DefineInterfaceBms(py::object);              /*!< 定义 Bms */
void DefineInterfaceLed(py::object);              /*!< 定义 Led */
void DefineInterfaceAudio(py::object);            /*!< 定义 Audio */
void DefineInterfaceTouch(py::object);            /*!< 定义 Touch */
void DefineInterfaceGps(py::object);              /*!< 定义 Gps */
void DefineInterfaceTof(py::object);              /*!< 定义 Tof */
void DefineInterfaceLidar(py::object);            /*!< 定义 Lidar */
void DefineInterfaceUltrasonic(py::object);       /*!< 定义 Ultrasonic */
void DefineInterfaceOdometer(py::object);         /*!< 定义 Odometer */
void DefineInterfaceImu(py::object);              /*!< 定义 Imu */

void DefineInterfaceCyberdog(py::object);         /*!< 定义 InterfaceCyberdog */
void DefineInterfaceVisualInterface(py::object);  /*!< 定义 InterfaceVisualInterface */
void DefineInterfaceVisualDebugAbilityset(
  py::object);                                    /*!< 定义 InterfaceVisualDebugAbilityset */
void DefineInterfaceVisualDebuggerEngine(
  py::object);                                    /*!< 定义 InterfaceVisualDebuggerEngine */
void DefineInterfaceVisualDebugger(py::object);   /*!< 定义 InterfaceVisualDebugger */
void DefineInterfaceVisual(py::object);           /*!< 定义 InterfaceVisual */

void DefineInterfaceTypeEnum(py::object);         /*!< 定义 InterfaceTypeEnum */
void DefineInterfaceTypeClass(py::object);        /*!< 定义 InterfaceTypeClass */
void DefineInterfaceType(py::object);             /*!< 定义 InterfaceType */
void DefineInterface(py::object);                 /*!< 定义 Interface */
void DefineDebugger(py::object);                  /*!< 定义 Debugger */
void DefineDebugAbilityset(py::object);           /*!< 定义 DebugAbilityset */
void DefineDebugEngine(py::object);               /*!< 定义 DebugEngine */
void DefineVisual(py::object);                    /*!< 定义 Visual */
}  // namespace cyberdog_visual_programming_terminal_py

#endif  // API__VPTPY_HPP_
