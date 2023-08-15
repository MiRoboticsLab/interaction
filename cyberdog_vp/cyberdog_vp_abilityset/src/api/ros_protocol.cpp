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

#include <protocol/msg/bms_status.hpp>
#include <protocol/msg/touch_status.hpp>
#include <protocol/msg/connector_status.hpp>
#include <protocol/msg/motion_sequence_param.hpp>

#include <protocol/srv/audio_text_play.hpp>
#include <protocol/srv/led_execute.hpp>
#include <protocol/srv/motion_result_cmd.hpp>
#include <protocol/srv/motion_sequence.hpp>

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "vpapy.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineProtocol(py::object m)
{
  DefineMsgBmsStatus(m);
  DefineMsgTouchStatus(m);
  DefineMsgConnectorStatus(m);
  DefineMsgMotionSequenceGait(m);
  DefineMsgMotionSequencePace(m);
  DefineMsgGpsPayload(m);
  DefineMsgSingleTofPayload(m);
  DefineMsgHeadTofPayload(m);
  DefineMsgRearTofPayload(m);
  DefineMsgFaceRes(m);
  DefineMsgPreset(m);
  DefineMsgSport(m);
  DefineMsgTrainingWords(m);

  DefineSrvFaceRecResponse(m);
  DefineSrvSportResponse(m);
  DefineSrvTrainingWordsResponse(m);
  DefineSrvAudioTextPlayResponse(m);
  DefineSrvLedExecuteResponse(m);
  DefineSrvMotionResultCmdResponse(m);
  DefineSrvMotionSequenceShowResponse(m);

  DefineActNavigationResult(m);
}

void DefineMsgBmsStatus(py::object m)
{
  py::class_<protocol::msg::BmsStatus>(m, "BmsStatus", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create MsgBmsStatus object )pbdoc")
  .def_readwrite("header", &protocol::msg::BmsStatus::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite("batt_volt", &protocol::msg::BmsStatus::batt_volt, R"pbdoc( 电压 - mV )pbdoc")
  .def_readwrite("batt_curr", &protocol::msg::BmsStatus::batt_curr, R"pbdoc( 电流 - mA )pbdoc")
  .def_readwrite("batt_soc", &protocol::msg::BmsStatus::batt_soc, R"pbdoc( 剩余电量 )pbdoc")
  .def_readwrite("batt_temp", &protocol::msg::BmsStatus::batt_temp, R"pbdoc( 温度 - °C )pbdoc")
  .def_readwrite(
    "power_adapter_temp", &protocol::msg::BmsStatus::power_adapter_temp,
    R"pbdoc( 电源适配器温度 - °C )pbdoc")
  .def_readwrite(
    "wireless_charging_temp", &protocol::msg::BmsStatus::wireless_charging_temp,
    R"pbdoc( 无线充电线圈温度 - °C )pbdoc")
  .def_readwrite(
    "batt_st", &protocol::msg::BmsStatus::batt_st,
    R"pbdoc( 保留字 )pbdoc")
  .def_readwrite(
    "bms_state_one", &protocol::msg::BmsStatus::bms_state_one,
    R"pbdoc( 开机时电池状态 )pbdoc")
  .def_readwrite(
    "bms_state_two", &protocol::msg::BmsStatus::bms_state_two,
    R"pbdoc( 关机时电池状态 )pbdoc")
  .def_readwrite("batt_health", &protocol::msg::BmsStatus::batt_health, R"pbdoc( 电池健康 )pbdoc")
  .def_readwrite(
    "batt_loop_number", &protocol::msg::BmsStatus::batt_loop_number,
    R"pbdoc( 电池循环数 )pbdoc")
  .def_readwrite("power_normal", &protocol::msg::BmsStatus::power_normal, R"pbdoc( 正常模式 )pbdoc")
  .def_readwrite(
    "power_wired_charging", &protocol::msg::BmsStatus::power_wired_charging,
    R"pbdoc( 有线充电中 )pbdoc")
  .def_readwrite(
    "power_finished_charging", &protocol::msg::BmsStatus::power_finished_charging,
    R"pbdoc( 充电完成 )pbdoc")
  .def_readwrite(
    "power_motor_shutdown", &protocol::msg::BmsStatus::power_motor_shutdown,
    R"pbdoc( 电机掉电 )pbdoc")
  .def_readwrite(
    "power_soft_shutdown", &protocol::msg::BmsStatus::power_soft_shutdown,
    R"pbdoc( 软关机 )pbdoc")
  .def_readwrite(
    "power_wp_place", &protocol::msg::BmsStatus::power_wp_place,
    R"pbdoc( 无线充电在位 )pbdoc")
  .def_readwrite(
    "power_wp_charging", &protocol::msg::BmsStatus::power_wp_charging,
    R"pbdoc( 无线充电中 )pbdoc")
  .def_readwrite(
    "power_expower_supply", &protocol::msg::BmsStatus::power_expower_supply,
    R"pbdoc( 外部供电 )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::BmsStatus & _bms) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: BMS"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - batt_volt(mV) = %d"
          "\n│  - batt_curr(mA) = %d"
          "\n│  - batt_soc = %d"
          "\n│  - batt_temp = %d"
          "\n│  - power_adapter_temp = %d"
          "\n│  - wireless_charging_temp = %d"
          "\n│  - batt_st = %d [%s]"
          "\n│  - bms_state_one = %d [%s]"
          "\n│  - bms_state_two = %d [%s]"
          "\n│  - batt_health = %d"
          "\n│  - batt_loop_number = %d"
          "\n│  - power_normal = %d"
          "\n│  - power_wired_charging = %d"
          "\n│  - power_finished_charging = %d"
          "\n│  - power_motor_shutdown = %d"
          "\n│  - power_soft_shutdown = %d"
          "\n│  - power_wp_place = %d"
          "\n│  - power_wp_charging = %d"
          "\n│  - power_expower_supply = %d"
          "\n└────────────────────────────────────────---",
          _bms.header.stamp.sec, _bms.header.stamp.nanosec,
          _bms.header.frame_id.c_str(),
          _bms.batt_volt,
          _bms.batt_curr,
          _bms.batt_soc,
          _bms.batt_temp,
          _bms.power_adapter_temp,
          _bms.wireless_charging_temp,
          _bms.batt_st, VPA::int2binary(_bms.batt_st).c_str(),
          _bms.bms_state_one, VPA::int2binary(_bms.bms_state_one).c_str(),
          _bms.bms_state_two, VPA::int2binary(_bms.bms_state_two).c_str(),
          _bms.batt_health,
          _bms.batt_loop_number,
          _bms.power_normal,
          _bms.power_wired_charging,
          _bms.power_finished_charging,
          _bms.power_motor_shutdown,
          _bms.power_soft_shutdown,
          _bms.power_wp_place,
          _bms.power_wp_charging,
          _bms.power_expower_supply
      ));
    })
  ;
}

void DefineMsgTouchStatus(py::object m)
{
  py::class_<protocol::msg::TouchStatus>(m, "TouchStatus", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("header", &protocol::msg::TouchStatus::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite("touch_state", &protocol::msg::TouchStatus::touch_state, R"pbdoc( 触摸板状态 )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::TouchStatus & _touch) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: protocol::msg::TouchStatus"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - touch_state = %d"
          "\n└────────────────────────────────────────---",
          _touch.header.stamp.sec, _touch.header.stamp.nanosec, _touch.header.frame_id.c_str(),
          _touch.touch_state));
    })
  ;
}

void DefineMsgConnectorStatus(py::object m)
{
  py::class_<protocol::msg::ConnectorStatus>(m, "ConnectorStatus", py::dynamic_attr())
  .def(py::init<>())
  // .def_readwrite("header", &protocol::msg::ConnectorStatus::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite(
    "is_connected", &protocol::msg::ConnectorStatus::is_connected,
    R"pbdoc( 是否连接wifi )pbdoc")
  .def_readwrite(
    "is_internet", &protocol::msg::ConnectorStatus::is_internet,
    R"pbdoc( 是否可以访问外网 )pbdoc")
  .def_readwrite("ssid", &protocol::msg::ConnectorStatus::ssid, R"pbdoc( wifi名称 )pbdoc")
  .def_readwrite("robot_ip", &protocol::msg::ConnectorStatus::robot_ip, R"pbdoc( 机器人IP )pbdoc")
  .def_readwrite(
    "provider_ip", &protocol::msg::ConnectorStatus::provider_ip,
    R"pbdoc( wifi提供方/移动端 IP )pbdoc")
  .def_readwrite("strength", &protocol::msg::ConnectorStatus::strength, R"pbdoc( wifi信号强度 )pbdoc")
  .def_readwrite("code", &protocol::msg::ConnectorStatus::code, R"pbdoc( 标准错误码 )pbdoc")

  .def(
    "__repr__", [](const protocol::msg::ConnectorStatus & _connector) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: protocol::msg::ConnectorStatus"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          // "\n│  - header:"
          // "\n│    - stamp:"
          // "\n│      - sec = %d"
          // "\n│      - nanosec = %d"
          // "\n│    - frame_id = '%s'"
          "\n│  - is_connected = %s"
          "\n│  - is_internet = %s"
          "\n│  - ssid = %s"
          "\n│  - robot_ip = %s"
          "\n│  - provider_ip = %s"
          "\n│  - strength = %d"
          "\n│  - code = %d"
          "\n└────────────────────────────────────────---",
          // _connector.header.stamp.sec,
          // _connector.header.stamp.nanosec,
          // _connector.header.frame_id.c_str(),
          std::string(_connector.is_connected ? "True" : "False").c_str(),
          std::string(_connector.is_internet ? "True" : "False").c_str(),
          _connector.ssid.c_str(), _connector.robot_ip.c_str(), _connector.provider_ip.c_str(),
          _connector.strength, _connector.code));
    })
  ;
}

void DefineMsgMotionSequenceGait(py::object m)
{
  py::class_<protocol::msg::MotionSequenceGait>(m, "MotionSequenceGait", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "right_forefoot", &protocol::msg::MotionSequenceGait::right_forefoot,
    R"pbdoc( 右前足:是否接触地面？ )pbdoc")
  .def_readwrite(
    "left_forefoot", &protocol::msg::MotionSequenceGait::left_forefoot,
    R"pbdoc( 左前足:是否接触地面？ )pbdoc")
  .def_readwrite(
    "right_hindfoot", &protocol::msg::MotionSequenceGait::right_hindfoot,
    R"pbdoc( 右后足:是否接触地面？ )pbdoc")
  .def_readwrite(
    "left_hindfoot", &protocol::msg::MotionSequenceGait::left_hindfoot,
    R"pbdoc( 左后足:是否接触地面？ )pbdoc")
  .def_readwrite(
    "duration", &protocol::msg::MotionSequenceGait::duration,
    R"pbdoc( 持续时间（毫秒） )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::MotionSequenceGait & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MotionSequenceGait"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - right_forefoot = %s"
          "\n│  - left_forefoot = %s"
          "\n│  - right_hindfoot = %s"
          "\n│  - left_hindfoot = %s"
          "\n│  - duration = %ld"
          "\n└────────────────────────────────────────---",
          std::string(_data.right_forefoot ? "True" : "False").c_str(),
          std::string(_data.left_forefoot ? "True" : "False").c_str(),
          std::string(_data.right_hindfoot ? "True" : "False").c_str(),
          std::string(_data.left_hindfoot ? "True" : "False").c_str(),
          _data.duration));
    })
  ;
}

void DefineMsgMotionSequencePace(py::object m)
{
  py::class_<protocol::msg::MotionSequencePace>(m, "MotionSequencePace", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "twist", &protocol::msg::MotionSequencePace::twist,
    R"pbdoc( 速度 )pbdoc")
  .def_readwrite(
    "centroid", &protocol::msg::MotionSequencePace::centroid,
    R"pbdoc( 质心 )pbdoc")
  .def_readwrite(
    "weight", &protocol::msg::MotionSequencePace::weight,
    R"pbdoc( 权重 )pbdoc")
  .def_readwrite(
    "right_forefoot", &protocol::msg::MotionSequencePace::right_forefoot,
    R"pbdoc( 右前足:落脚点位置(x,y,z); 抬脚高度(w) )pbdoc")
  .def_readwrite(
    "left_forefoot", &protocol::msg::MotionSequencePace::left_forefoot,
    R"pbdoc( 左前足:落脚点位置(x,y,z); 抬脚高度(w)  )pbdoc")
  .def_readwrite(
    "right_hindfoot", &protocol::msg::MotionSequencePace::right_hindfoot,
    R"pbdoc( 右后足:落脚点位置(x,y,z); 抬脚高度(w)  )pbdoc")
  .def_readwrite(
    "left_hindfoot", &protocol::msg::MotionSequencePace::left_hindfoot,
    R"pbdoc( 左后足:落脚点位置(x,y,z); 抬脚高度(w)  )pbdoc")
  .def_readwrite(
    "friction_coefficient", &protocol::msg::MotionSequencePace::friction_coefficient,
    R"pbdoc( 摩擦系数[0.1 1.0] )pbdoc")
  .def_readwrite(
    "landing_gain", &protocol::msg::MotionSequencePace::landing_gain,
    R"pbdoc( 落地系数[0,1.0] )pbdoc")
  .def_readwrite(
    "use_mpc_track", &protocol::msg::MotionSequencePace::use_mpc_track,
    R"pbdoc( 是否使用 MPC 轨迹 )pbdoc")
  .def_readwrite(
    "duration", &protocol::msg::MotionSequencePace::duration,
    R"pbdoc( 持续时间（毫秒） )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::MotionSequencePace & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MotionSequencePace"
          "\n├───────────────────────────────────────────────────---"
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
          "\n│  - centroid:"
          "\n│    - position:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│    - orientation:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│      - w = %lf"
          "\n│  - weight:"
          "\n│    - linear:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│    - angular:"
          "\n│      - x = %lf"
          "\n│      - y = %lf"
          "\n│      - z = %lf"
          "\n│  - right_forefoot:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│    - w = %lf"
          "\n│  - left_forefoot:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│    - w = %lf"
          "\n│  - right_hindfoot:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│    - w = %lf"
          "\n│  - left_hindfoot:"
          "\n│    - x = %lf"
          "\n│    - y = %lf"
          "\n│    - z = %lf"
          "\n│    - w = %lf"
          "\n│  - friction_coefficient = %lf"
          "\n│  - landing_gain = %lf"
          "\n│  - use_mpc_track = %s"
          "\n│  - duration = %ld"
          "\n└────────────────────────────────────────---",
          _data.twist.linear.x,
          _data.twist.linear.y,
          _data.twist.linear.z,
          _data.twist.angular.x,
          _data.twist.angular.y,
          _data.twist.angular.z,
          _data.centroid.position.x,
          _data.centroid.position.y,
          _data.centroid.position.z,
          _data.centroid.orientation.x,
          _data.centroid.orientation.y,
          _data.centroid.orientation.z,
          _data.centroid.orientation.w,
          _data.weight.linear.x,
          _data.weight.linear.y,
          _data.weight.linear.z,
          _data.weight.angular.x,
          _data.weight.angular.y,
          _data.weight.angular.z,
          _data.right_forefoot.x,
          _data.right_forefoot.y,
          _data.right_forefoot.z,
          _data.right_forefoot.w,
          _data.left_forefoot.x,
          _data.left_forefoot.y,
          _data.left_forefoot.z,
          _data.left_forefoot.w,
          _data.right_hindfoot.x,
          _data.right_hindfoot.y,
          _data.right_hindfoot.z,
          _data.right_hindfoot.w,
          _data.left_hindfoot.x,
          _data.left_hindfoot.y,
          _data.left_hindfoot.z,
          _data.left_hindfoot.w,
          _data.friction_coefficient,
          _data.landing_gain,
          std::string(_data.use_mpc_track ? "True" : "False").c_str(),
          _data.duration));
    })
  ;
}

void DefineMsgGpsPayload(py::object m)
{
  py::class_<protocol::msg::GpsPayload>(m, "GpsPayload", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("sec", &protocol::msg::GpsPayload::sec, R"pbdoc( 秒 )pbdoc")
  .def_readwrite("nanosec", &protocol::msg::GpsPayload::nanosec, R"pbdoc( 纳秒 )pbdoc")
  .def_readwrite("itow", &protocol::msg::GpsPayload::itow, R"pbdoc( 时间戳 )pbdoc")
  .def_readwrite("fix_type", &protocol::msg::GpsPayload::fix_type, R"pbdoc( 优化类型 )pbdoc")
  .def_readwrite("num_sv", &protocol::msg::GpsPayload::num_sv, R"pbdoc( 搜星个数 )pbdoc")
  .def_readwrite("lon", &protocol::msg::GpsPayload::lon, R"pbdoc( 经度 )pbdoc")
  .def_readwrite("lat", &protocol::msg::GpsPayload::lat, R"pbdoc( 纬度 )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::GpsPayload & _gps) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: protocol::msg::GpsPayload"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - sec = %d"
          "\n│  - nanosec = %d"
          "\n│  - itow = %d"
          "\n│  - fix_type = %d"
          "\n│  - num_sv = %d"
          "\n│  - lon = %lf"
          "\n│  - lat = %lf"
          "\n└────────────────────────────────────────---",
          _gps.sec,
          _gps.nanosec,
          _gps.itow,
          _gps.fix_type,
          _gps.num_sv,
          _gps.lon,
          _gps.lat));
    })
  ;
}

void DefineMsgSingleTofPayload(py::object m)
{
  py::class_<protocol::msg::SingleTofPayload>(m, "SingleTofPayload", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("header", &protocol::msg::SingleTofPayload::header, R"pbdoc( 消息头 )pbdoc")
  .def_readwrite(
    "data_available", &protocol::msg::SingleTofPayload::data_available,
    R"pbdoc( 数据是否可用 )pbdoc")
  .def_readwrite(
    "tof_position", &protocol::msg::SingleTofPayload::tof_position,
    R"pbdoc( 传感器的位置(左前:0,右前:1,左后:2,右后:3) )pbdoc")
  .def_readwrite(
    "data", &protocol::msg::SingleTofPayload::data,
    py::return_value_policy::reference_internal, R"pbdoc( 传感器数据[m] )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::SingleTofPayload & _tof) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: protocol::msg::SingleTofPayload"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - header:"
          "\n│    - stamp:"
          "\n│      - sec = %d"
          "\n│      - nanosec = %d"
          "\n│    - frame_id = '%s'"
          "\n│  - data_available = %s"
          "\n│  - tof_position = %d"
          "\n│  - data = %ld"
          "\n└────────────────────────────────────────---",
          _tof.header.stamp.sec,
          _tof.header.stamp.nanosec,
          _tof.header.frame_id.c_str(),
          _tof.data_available ? "True" : "False",
          _tof.tof_position,
          _tof.data.size()));
    })
  ;
}

void DefineMsgHeadTofPayload(py::object m)
{
  py::class_<protocol::msg::HeadTofPayload>(m, "HeadTofPayload", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("left_head", &protocol::msg::HeadTofPayload::left_head, R"pbdoc( 左前 )pbdoc")
  .def_readwrite("right_head", &protocol::msg::HeadTofPayload::right_head, R"pbdoc( 右前 )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::HeadTofPayload & _tof) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: protocol::msg::HeadTofPayload"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - left_head:"
          "\n│    - header:"
          "\n│      - stamp:"
          "\n│        - sec = %d"
          "\n│        - nanosec = %d"
          "\n│      - frame_id = '%s'"
          "\n│    - data_available = %s"
          "\n│    - tof_position = %d"
          "\n│    - data = %ld"
          "\n│  - right_head:"
          "\n│    - header:"
          "\n│      - stamp:"
          "\n│        - sec = %d"
          "\n│        - nanosec = %d"
          "\n│      - frame_id = '%s'"
          "\n│    - data_available = %s"
          "\n│    - tof_position = %d"
          "\n│    - data = %ld"
          "\n└────────────────────────────────────────---",
          _tof.left_head.header.stamp.sec,
          _tof.left_head.header.stamp.nanosec,
          _tof.left_head.header.frame_id.c_str(),
          _tof.left_head.data_available ? "True" : "False",
          _tof.left_head.tof_position,
          _tof.left_head.data.size(),
          _tof.right_head.header.stamp.sec,
          _tof.right_head.header.stamp.nanosec,
          _tof.right_head.header.frame_id.c_str(),
          _tof.right_head.data_available ? "True" : "False",
          _tof.right_head.tof_position,
          _tof.right_head.data.size()));
    })
  ;
}

void DefineMsgRearTofPayload(py::object m)
{
  py::class_<protocol::msg::RearTofPayload>(m, "RearTofPayload", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("left_rear", &protocol::msg::RearTofPayload::left_rear, R"pbdoc( 左后 )pbdoc")
  .def_readwrite("right_rear", &protocol::msg::RearTofPayload::right_rear, R"pbdoc( 右后 )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::RearTofPayload & _tof) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: protocol::msg::RearTofPayload"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - left_rear:"
          "\n│    - header:"
          "\n│      - stamp:"
          "\n│        - sec = %d"
          "\n│        - nanosec = %d"
          "\n│      - frame_id = '%s'"
          "\n│    - data_available = %s"
          "\n│    - tof_position = %d"
          "\n│    - data = %ld"
          "\n│  - right_rear:"
          "\n│    - header:"
          "\n│      - stamp:"
          "\n│        - sec = %d"
          "\n│        - nanosec = %d"
          "\n│      - frame_id = '%s'"
          "\n│    - data_available = %s"
          "\n│    - tof_position = %d"
          "\n│    - data = %ld"
          "\n└────────────────────────────────────────---",
          _tof.left_rear.header.stamp.sec,
          _tof.left_rear.header.stamp.nanosec,
          _tof.left_rear.header.frame_id.c_str(),
          _tof.left_rear.data_available ? "True" : "False",
          _tof.left_rear.tof_position,
          _tof.left_rear.data.size(),
          _tof.right_rear.header.stamp.sec,
          _tof.right_rear.header.stamp.nanosec,
          _tof.right_rear.header.frame_id.c_str(),
          _tof.right_rear.data_available ? "True" : "False",
          _tof.right_rear.tof_position,
          _tof.right_rear.data.size()));
    })
  ;
}

void DefineMsgFaceRes(py::object m)
{
  py::class_<VPA::MsgFaceRes>(m, "MsgFaceRes", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("result", &VPA::MsgFaceRes::result, R"pbdoc( 人员识别结果 )pbdoc")
  .def_readwrite("username", &VPA::MsgFaceRes::username, R"pbdoc( 人员昵称 )pbdoc")
  .def_readwrite("age", &VPA::MsgFaceRes::age, R"pbdoc( 人员年龄 )pbdoc")
  .def_readwrite("emotion", &VPA::MsgFaceRes::emotion, R"pbdoc( 人员情绪 )pbdoc")

  .def(
    "__repr__", [](const VPA::MsgFaceRes & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgFaceRes"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - result: = %d"
          "\n│  - username: = %s"
          "\n│  - age: = %lf"
          "\n│  - emotion: = %lf"
          "\n└───────────────────────────────────────────────────---",
          _data.result,
          _data.username.c_str(),
          _data.age,
          _data.emotion));
    })
  ;
}

void DefineMsgPreset(py::object m)
{
  py::class_<protocol::msg::Label>(
    m, "MsgPreset",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "label_name", &protocol::msg::Label::label_name,
    R"pbdoc( 预置点名称 )pbdoc")
  .def_readwrite(
    "physic_x", &protocol::msg::Label::physic_x,
    R"pbdoc( 预置点 X 坐标 )pbdoc")
  .def_readwrite(
    "physic_y", &protocol::msg::Label::physic_y,
    R"pbdoc( 预置点 y 坐标 )pbdoc")
  .def(
    "__repr__", [](const protocol::msg::Label & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::msg::Label"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - label_name = %s"
          "\n│  - physic_x = %f"
          "\n│  - physic_y = %f"
          "\n└───────────────────────────────────────────────────---",
          _data.label_name.c_str(),
          _data.physic_x,
          _data.physic_y));
    })
  ;
}

void DefineMsgSport(py::object m)
{
  py::class_<protocol::msg::SportCountsResult>(m, "MsgSport", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "algo_switch", &protocol::msg::SportCountsResult::algo_switch,
    R"pbdoc( 算法开关 )pbdoc")
  .def_readwrite("sport_type", &protocol::msg::SportCountsResult::sport_type, R"pbdoc( 运动类型 )pbdoc")
  .def_readwrite("counts", &protocol::msg::SportCountsResult::counts, R"pbdoc( 运动个数 )pbdoc")
  .def_readwrite("duration", &protocol::msg::SportCountsResult::duration, R"pbdoc( 运动时长 )pbdoc")

  .def(
    "__repr__", [](const protocol::msg::SportCountsResult & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgSport"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - algo_switch: = %ds"
          "\n│  - sport_type: = %d"
          "\n│  - counts: = %d"
          "\n│  - duration: = %d"
          "\n└───────────────────────────────────────────────────---",
          _data.algo_switch,
          _data.sport_type,
          _data.counts,
          _data.duration));
    })
  ;
}

void DefineMsgTrainingWords(py::object m)
{
  py::class_<VPA::MsgTrainingWords>(m, "MsgTrainingWords", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("trigger", &VPA::MsgTrainingWords::trigger, R"pbdoc( 关键词 )pbdoc")
  .def_readwrite("type", &VPA::MsgTrainingWords::type, R"pbdoc( 类型 )pbdoc")
  .def_readwrite("value", &VPA::MsgTrainingWords::value, R"pbdoc( 值 )pbdoc")

  .def(
    "__repr__", [](const VPA::MsgTrainingWords & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgTrainingWords"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - trigger: = %s"
          "\n│  - type: = %s"
          "\n│  - value: = %s"
          "\n└───────────────────────────────────────────────────---",
          _data.trigger.c_str(),
          _data.type.c_str(),
          _data.value.c_str()));
    })
  ;
}

void DefineSrvFaceRecResponse(py::object m)
{
  py::class_<protocol::srv::FaceRec::Response>(
    m, "FaceRec_Response",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("result", &protocol::srv::FaceRec::Response::result, R"pbdoc( 结果 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::FaceRec::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::FaceRec::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - result: = %d"
          "\n└───────────────────────────────────────────────────---",
          _res.result));
    })
  ;
}

void DefineSrvSportResponse(py::object m)
{
  py::class_<protocol::srv::SportManager::Response>(
    m, "SportManager_Response",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("result", &protocol::srv::SportManager::Response::result, R"pbdoc( 结果 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::SportManager::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::SportManager::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - result: = %d"
          "\n└───────────────────────────────────────────────────---",
          _res.result));
    })
  ;
}

void DefineSrvTrainingWordsResponse(py::object m)
{
  py::class_<protocol::srv::TrainPlanAll::Response>(
    m, "TrainPlanAll_Response",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readonly(
    "training_set", &protocol::srv::TrainPlanAll::Response::training_set,
    R"pbdoc( 训练词能力集 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::TrainPlanAll::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::TrainPlanAll::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - training_set:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgTrainingWordsVector(_res.training_set, "│    ").c_str()));
    })
  ;
}

void DefineSrvAudioTextPlayResponse(py::object m)
{
  py::class_<protocol::srv::AudioTextPlay::Response>(
    m, "AudioTextPlay_Response",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("status", &protocol::srv::AudioTextPlay::Response::status, R"pbdoc( 状态码 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::AudioTextPlay::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::LedExecute::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - status: = %d"
          "\n└───────────────────────────────────────────────────---",
          _res.status));
    })
  ;
}

void DefineSrvLedExecuteResponse(py::object m)
{
  py::class_<protocol::srv::LedExecute::Response>(m, "LedExecute_Response", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("code", &protocol::srv::LedExecute::Response::code, R"pbdoc( 返回码 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::LedExecute::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::LedExecute::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - code: = %d"
          "\n└───────────────────────────────────────────────────---",
          _res.code));
    })
  ;
}

void DefineSrvMotionResultCmdResponse(py::object m)
{
  py::class_<protocol::srv::MotionResultCmd::Response>(
    m, "MotionResultCmd_Response",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "motion_id", &protocol::srv::MotionResultCmd::Response::motion_id,
    R"pbdoc( 机器人运控姿态ID )pbdoc")
  .def_readwrite("result", &protocol::srv::MotionResultCmd::Response::result, R"pbdoc( 执行结果 )pbdoc")
  .def_readwrite("code", &protocol::srv::MotionResultCmd::Response::code, R"pbdoc( 通用状态码 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::MotionResultCmd::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::MotionResultCmd::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - motion_id: = %d"
          "\n│  - result = %s"
          "\n│  - code = %d"
          "\n└───────────────────────────────────────────────────---",
          _res.motion_id, std::string(_res.result ? "True" : "False").c_str(), _res.code));
    })
  ;
}

void DefineSrvMotionSequenceShowResponse(py::object m)
{
  py::class_<protocol::srv::MotionSequenceShow::Response>(
    m, "MotionSequenceShow_Response",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "motion_id", &protocol::srv::MotionSequenceShow::Response::motion_id,
    R"pbdoc( 运动id )pbdoc")
  .def_readwrite(
    "result", &protocol::srv::MotionSequenceShow::Response::result,
    R"pbdoc( 执行结果 )pbdoc")
  .def_readwrite("code", &protocol::srv::MotionSequenceShow::Response::code, R"pbdoc( 通用状态码 )pbdoc")
  .def_readwrite(
    "describe", &protocol::srv::MotionSequenceShow::Response::describe,
    R"pbdoc( 通用状态码描述 )pbdoc")
  .def(
    "__repr__", [](const protocol::srv::MotionSequenceShow::Response & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::srv::MotionSequenceShow::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - motion_id = %d"
          "\n│  - result = %s"
          "\n│  - code = %d"
          "\n│  - describe: = %s"
          "\n└───────────────────────────────────────────────────---",
          _res.motion_id, std::string(_res.result ? "True" : "False").c_str(),
          _res.code, _res.describe.c_str()));
    })
  ;
}

void DefineActNavigationResult(py::object m)
{
  py::class_<protocol::action::Navigation::Result>(
    m, "ActNavigation_Result",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "result", &protocol::action::Navigation::Result::result,
    R"pbdoc( 结果 )pbdoc")
  .def(
    "__repr__", [](const protocol::action::Navigation::Result & _res) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: protocol::action::Navigation::Result"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - result = %d"
          "\n└───────────────────────────────────────────────────---",
          _res.result));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
