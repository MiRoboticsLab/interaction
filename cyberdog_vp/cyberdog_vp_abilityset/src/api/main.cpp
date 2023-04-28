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
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
PYBIND11_MODULE(abilityset, m)
{
  m.doc() =
    R"pbdoc(
      Cyberdog visual programming ability set python. 
      --------------------------------------
    )pbdoc";
  DefineCommonType(m);

  DefineBase(m);

  DefineNetwork(m);
  DefineFollow(m);
  DefineMotion(m);
  DefineNavigation(m);
  DefineTask(m);
  DefineTrain(m);
  DefinePersonnel(m);
  DefineGesture(m);
  DefineSkeleton(m);

  DefineAudio(m);
  DefineLed(m);
  DefineBms(m);
  DefineTouch(m);
  DefineGps(m);
  DefineTof(m);
  DefineLidar(m);
  DefineUltrasonic(m);
  DefineOdometer(m);
  DefineImu(m);

  DefineCyberdog(m);
}

}  // namespace cyberdog_visual_programming_abilityset_py
