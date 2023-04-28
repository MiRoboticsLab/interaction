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
#include "cyberdog_vp_abilityset/network.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineNetwork(py::object m)
{
  py::class_<VPA::Network, VPA::Base>(m, "Network", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Network object )pbdoc")
  .def_readonly("data", &VPA::Network::data_, R"pbdoc( data )pbdoc")
  .def(
    "__repr__", [](const VPA::Network & _net) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Network"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - data:"
          "\n│    - is_connected = %s"
          "\n│    - is_internet = %s"
          "\n│    - ssid = %s"
          "\n│    - robot_ip = %s"
          "\n│    - provider_ip = %s"
          "\n│    - strength = %d"
          "\n│    - code = %d"
          "\n└────────────────────────────────────────---",
          _net.state_.code, _net.state_.describe.c_str(),
          std::string(_net.data_.is_connected ? "True" : "False").c_str(),
          std::string(_net.data_.is_internet ? "True" : "False").c_str(),
          _net.data_.ssid.c_str(), _net.data_.robot_ip.c_str(), _net.data_.provider_ip.c_str(),
          _net.data_.strength, _net.data_.code));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
