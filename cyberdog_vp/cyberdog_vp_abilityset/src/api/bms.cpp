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
#include "cyberdog_vp_abilityset/bms.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineBms(py::object m)
{
  py::class_<VPA::Bms, VPA::Base>(m, "BMS", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create BMS object )pbdoc")
  .def_readonly("data", &VPA::Bms::data_, R"pbdoc( 数据 )pbdoc")
  .def(
    "get_data", &VPA::Bms::GetData, R"pbdoc( 获取最新数据 )pbdoc",
    py::arg("timeout") = 5
  )
  .def(
    "__repr__", [](const VPA::Bms & _bms) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: BMS"
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
          "\n│    - batt_volt(mV) = %d"
          "\n│    - batt_curr(mA) = %d"
          "\n│    - batt_soc = %d"
          "\n│    - batt_temp = %d"
          "\n│    - power_adapter_temp = %d"
          "\n│    - wireless_charging_temp = %d"
          "\n│    - batt_st = %d [%s]"
          "\n│    - bms_state_one = %d [%s]"
          "\n│    - bms_state_two = %d [%s]"
          "\n│    - batt_health = %d"
          "\n│    - batt_loop_number = %d"
          "\n│    - power_normal = %d"
          "\n│    - power_wired_charging = %d"
          "\n│    - power_finished_charging = %d"
          "\n│    - power_motor_shutdown = %d"
          "\n│    - power_soft_shutdown = %d"
          "\n│    - power_wp_place = %d"
          "\n│    - power_wp_charging = %d"
          "\n│    - power_expower_supply = %d"
          "\n└────────────────────────────────────────---",
          _bms.state_.code, _bms.state_.describe.c_str(),
          _bms.data_.header.stamp.sec, _bms.data_.header.stamp.nanosec,
          _bms.data_.header.frame_id.c_str(),
          _bms.data_.batt_volt,
          _bms.data_.batt_curr,
          _bms.data_.batt_soc,
          _bms.data_.batt_temp,
          _bms.data_.power_adapter_temp,
          _bms.data_.wireless_charging_temp,
          _bms.data_.batt_st, VPA::int2binary(_bms.data_.batt_st).c_str(),
          _bms.data_.bms_state_one, VPA::int2binary(_bms.data_.bms_state_one).c_str(),
          _bms.data_.bms_state_two, VPA::int2binary(_bms.data_.bms_state_two).c_str(),
          _bms.data_.batt_health,
          _bms.data_.batt_loop_number,
          _bms.data_.power_normal,
          _bms.data_.power_wired_charging,
          _bms.data_.power_finished_charging,
          _bms.data_.power_motor_shutdown,
          _bms.data_.power_soft_shutdown,
          _bms.data_.power_wp_place,
          _bms.data_.power_wp_charging,
          _bms.data_.power_expower_supply
      ));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
