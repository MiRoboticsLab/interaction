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

#include <pybind11/pybind11.h>
#include "vptpy.hpp"

namespace cyberdog_visual_programming_terminal_py
{
PYBIND11_MODULE(terminal, m)
{
  m.doc() =
    R"pbdoc(
        Cyberdog visual programming terminal ability set python. 
        --------------------------------------
    )pbdoc";

  DefineInterfaceNetwork(m);
  DefineInterfaceFollow(m);
  DefineInterfaceMotion(m);
  DefineInterfaceNavigation(m);
  DefineInterfaceTask(m);
  DefineInterfaceTrain(m);
  DefineInterfacePersonnel(m);
  DefineInterfaceGesture(m);
  DefineInterfaceSkeleton(m);

  DefineInterfaceBms(m);
  DefineInterfaceLed(m);
  DefineInterfaceAudio(m);
  DefineInterfaceTouch(m);
  DefineInterfaceGps(m);
  DefineInterfaceTof(m);
  DefineInterfaceLidar(m);
  DefineInterfaceUltrasonic(m);
  DefineInterfaceOdometer(m);
  DefineInterfaceImu(m);

  DefineInterfaceCyberdog(m);

  DefineInterfaceVisualInterface(m);

  DefineInterfaceVisualDebugAbilityset(m);
  DefineInterfaceVisualDebuggerEngine(m);
  DefineInterfaceVisualDebugger(m);
  DefineInterfaceVisual(m);

  DefineInterfaceTypeEnum(m);
  DefineInterfaceTypeClass(m);
  DefineInterfaceType(m);

  DefineInterface(m);
  DefineDebugger(m);
  DefineDebugAbilityset(m);
  DefineDebugEngine(m);
  DefineVisual(m);
}

}  // namespace cyberdog_visual_programming_terminal_py
