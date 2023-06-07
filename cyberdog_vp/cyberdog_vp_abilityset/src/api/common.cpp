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
#include <vector>
#include <memory>
#include <map>

#include "vpapy.hpp"
#include "cyberdog_vp_abilityset/common.hpp"

PYBIND11_MAKE_OPAQUE(
  std::vector<cyberdog_visual_programming_abilityset::MsgMotionSequenceGait,
  std::allocator<cyberdog_visual_programming_abilityset::MsgMotionSequenceGait>>)
PYBIND11_MAKE_OPAQUE(
  std::vector<cyberdog_visual_programming_abilityset::MsgMotionSequencePace,
  std::allocator<cyberdog_visual_programming_abilityset::MsgMotionSequencePace>>)

PYBIND11_MAKE_OPAQUE(std::vector<cyberdog_visual_programming_abilityset::MsgPersonnel>)

PYBIND11_MAKE_OPAQUE(std::vector<cyberdog_visual_programming_abilityset::MsgFaceRes>)
PYBIND11_MAKE_OPAQUE(std::map<std::string, cyberdog_visual_programming_abilityset::MsgFaceRes>)

PYBIND11_MAKE_OPAQUE(std::vector<cyberdog_visual_programming_abilityset::MsgPreset>)
PYBIND11_MAKE_OPAQUE(std::map<std::string, cyberdog_visual_programming_abilityset::MsgPreset>)

PYBIND11_MAKE_OPAQUE(std::vector<cyberdog_visual_programming_abilityset::MsgTrainingWords>)
PYBIND11_MAKE_OPAQUE(
  std::map<std::string,
  cyberdog_visual_programming_abilityset::MsgTrainingWords>)
PYBIND11_MAKE_OPAQUE(
  std::vector<cyberdog_visual_programming_abilityset::DialogueResponse,
  std::allocator<cyberdog_visual_programming_abilityset::DialogueResponse>>)

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineCommonType(py::object m)
{
  DefineBuiltinInterfaces(m);
  DefineStdMsgs(m);
  DefineSensorMsgs(m);
  DefineGeometryMsgs(m);
  DefineNavMsgs(m);
  DefineProtocol(m);

  DefineState(m);

  DefineAudioPlaySeviceResponse(m);
  DefineAudioGetVolumeSeviceResponse(m);
  DefineAudioSetVolumeSeviceResponse(m);
  DefineDialogueResponse(m);
  DefineMsgDialogueResponseList(m);
  DefineAudioGetUserDialogueResponse(m);
  DefineLedConstraint(m);
  DefineLedSeviceResponse(m);
  DefineMotionResultServiceResponse(m);
  DefineMotionSequenceServiceResponse(m);
  DefineMotionServoCmdResponse(m);
  DefineMotionParams(m);
  DefineMotionId(m);

  DefineMsgMotionSequenceGaitList(m);
  DefineMsgMotionSequencePaceList(m);

  DefineMotionSequence(m);
  DefineObstacleMeta(m);
  DefineTofObstacle(m);
  DefineTofPayload(m);

  DefineMsgPersonnel(m);
  DefineMsgPersonnelList(m);
  DefineSrvPersonnelResponse(m);

  DefineFaceSeviceResponse(m);
  DefineFaceRecognizedSeviceResponse(m);
  DefineVoiceprintRecognizedResponse(m);

  DefineGestureData(m);
  DefineGestureType(m);
  DefineGestureRecognizedSeviceResponse(m);
  DefineGestureRecognizedMessageResponse(m);

  DefineSkeletonType(m);
  DefineSkeletonRecognizedSeviceResponse(m);
  DefineSkeletonRecognizedMessageResponse(m);

  DefineTrainingWordsRecognizedSeviceResponse(m);
  DefineTrainingWordsRecognizedMessageResponse(m);

  DefineMapPresetSeviceResponse(m);
  DefineNavigationActionResponse(m);
}

void DefineState(py::object m)
{
  py::enum_<VPA::StateCode>(m, "StateCode")
  .value("invalid", VPA::StateCode::invalid, R"pbdoc( 无效 )pbdoc")
  .value("success", VPA::StateCode::success, R"pbdoc( 成功 )pbdoc")
  .value("fail", VPA::StateCode::fail, R"pbdoc( 失败 )pbdoc")
  .value("no_data_update", VPA::StateCode::no_data_update, R"pbdoc( 无数据更新 )pbdoc")
  .value(
    "command_waiting_execute", VPA::StateCode::command_waiting_execute,
    R"pbdoc( 待执行时发生错误 )pbdoc")
  .value(
    "service_client_interrupted", VPA::StateCode::service_client_interrupted,
    R"pbdoc( 客户端在请求服务出现时被打断 )pbdoc")
  .value(
    "service_appear_timeout", VPA::StateCode::service_appear_timeout,
    R"pbdoc( 等待服务出现/启动超时 )pbdoc")
  .value(
    "service_request_interrupted", VPA::StateCode::service_request_interrupted,
    R"pbdoc( 请求服务中断 )pbdoc")
  .value(
    "service_request_timeout", VPA::StateCode::service_request_timeout,
    R"pbdoc( 请求服务超时/延迟 )pbdoc")
  .value(
    "spin_future_interrupted", VPA::StateCode::spin_future_interrupted,
    R"pbdoc( 请求服务中断 )pbdoc")
  .value("spin_future_timeout", VPA::StateCode::spin_future_timeout, R"pbdoc( 请求服务超时/延迟 )pbdoc")
  ;

  py::class_<VPA::State>(m, "State", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("code", &VPA::State::code, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("describe", &VPA::State::describe, R"pbdoc( 描述 )pbdoc")
  .def(
    "__repr__", [](const VPA::State & _state) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: State"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - code = %d"
          "\n│  - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _state.code,
          _state.describe.c_str()));
    })
  ;
}

void DefineAudioPlaySeviceResponse(py::object m)
{
  py::class_<VPA::AudioPlaySeviceResponse>(m, "AudioPlaySeviceResponse", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::AudioPlaySeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::AudioPlaySeviceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::AudioPlaySeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: AudioPlaySeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - status = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.status));
    })
  ;
}

void DefineAudioGetVolumeSeviceResponse(py::object m)
{
  py::class_<VPA::AudioGetVolumeSeviceResponse>(
    m, "AudioGetVolumeSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::AudioGetVolumeSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::AudioGetVolumeSeviceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::AudioGetVolumeSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: AudioGetVolumeSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - volume = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.volume));
    })
  ;
}

void DefineAudioSetVolumeSeviceResponse(py::object m)
{
  py::class_<VPA::AudioSetVolumeSeviceResponse>(
    m, "AudioSetVolumeSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::AudioSetVolumeSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::AudioSetVolumeSeviceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::AudioSetVolumeSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: AudioSetVolumeSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - success = %s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          std::string(_ret.response.success ? "True" : "False").c_str()));
    })
  ;
}

void DefineDialogueResponse(py::object m)
{
  py::class_<VPA::DialogueResponse>(
    m, "DialogueResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("time_ns", &VPA::DialogueResponse::time_ns, R"pbdoc( 时间 )pbdoc")
  .def_readwrite("data", &VPA::DialogueResponse::data, R"pbdoc( 数据 )pbdoc")
  .def(
    "__repr__", [](const VPA::DialogueResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: DialogueResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - time_ns = %ld"
          "\n│  - data = '%s'"
          "\n└────────────────────────────────────────---",
          _ret.time_ns,
          _ret.data.c_str()));
    })
  ;
}

void DefineMsgDialogueResponseList(py::object m)
{
  using MsgDialogueResponseList = std::vector<VPA::DialogueResponse>;
  py::bind_vector<MsgDialogueResponseList>(m, "MsgDialogueResponseList")
  .def(py::init<>())
  .def("empty", &MsgDialogueResponseList::empty)
  .def("size", &MsgDialogueResponseList::size)
  .def("max_size", &MsgDialogueResponseList::max_size)
  .def("capacity", &MsgDialogueResponseList::capacity)
  .def(
    "at",
    (VPA::DialogueResponse & (MsgDialogueResponseList::*) (const size_t)) &
    MsgDialogueResponseList::at)
  .def(
    "front",
    (VPA::DialogueResponse & (MsgDialogueResponseList::*) ()) & MsgDialogueResponseList::front)
  .def(
    "back",
    (VPA::DialogueResponse & (MsgDialogueResponseList::*) ()) & MsgDialogueResponseList::back)
  .def(
    "__len__", [](const MsgDialogueResponseList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgDialogueResponseList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgDialogueResponseList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgDialogueResponseList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgDialogueResponseVector(_data, "│  ").c_str()));
    })
  ;
}

void DefineAudioGetUserDialogueResponse(py::object m)
{
  py::class_<VPA::AudioGetUserDialogueResponse>(
    m, "AudioGetUserDialogueResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::AudioGetUserDialogueResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::AudioGetUserDialogueResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::AudioGetUserDialogueResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: AudioGetUserDialogueResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "%s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          VPA::msgDialogueResponseVector(_ret.response, "│    ").c_str()));
    })
  ;
}

void DefineLedConstraint(py::object m)
{
  py::enum_<VPA::LedConstraint>(m, "LedConstraint")
  .value("target_head", VPA::LedConstraint::target_head, R"pbdoc( 头灯 )pbdoc")
  .value("target_tail", VPA::LedConstraint::target_tail, R"pbdoc( 尾灯 )pbdoc")
  .value("target_mini", VPA::LedConstraint::target_mini, R"pbdoc( 眼灯 )pbdoc")

  .value("effect_line_on", VPA::LedConstraint::effect_line_on, R"pbdoc( [灯带]常亮 )pbdoc")
  .value("effect_line_blink", VPA::LedConstraint::effect_line_blink, R"pbdoc( [灯带]闪烁 )pbdoc")
  .value(
    "effect_line_blink_fast", VPA::LedConstraint::effect_line_blink_fast,
    R"pbdoc( [灯带]快速闪烁 )pbdoc")
  .value("effect_line_breath", VPA::LedConstraint::effect_line_breath, R"pbdoc( [灯带]呼吸 )pbdoc")
  .value(
    "effect_line_breath_fast", VPA::LedConstraint::effect_line_breath_fast,
    R"pbdoc( [灯带]快速呼吸 )pbdoc")
  .value(
    "effect_line_one_by_one", VPA::LedConstraint::effect_line_one_by_one,
    R"pbdoc( [灯带]逐个点亮 )pbdoc")
  .value(
    "effect_line_one_by_one_fast", VPA::LedConstraint::effect_line_one_by_one_fast,
    R"pbdoc( [灯带]快速逐个点亮 )pbdoc")
  .value(
    "effect_line_back_and_forth", VPA::LedConstraint::effect_line_back_and_forth,
    R"pbdoc( [灯带]往返逐个点亮 )pbdoc")
  .value(
    "effect_line_trailing_race", VPA::LedConstraint::effect_line_trailing_race,
    R"pbdoc( [灯带]拖尾流跑马 )pbdoc")

  .value(
    "system_effect_line_off", VPA::LedConstraint::system_effect_line_off,
    R"pbdoc( [灯带]常灭 )pbdoc")

  .value(
    "system_effect_line_red_on", VPA::LedConstraint::system_effect_line_red_on,
    R"pbdoc( [灯带]红灯常亮 )pbdoc")
  .value(
    "system_effect_line_red_blink", VPA::LedConstraint::system_effect_line_red_blink,
    R"pbdoc( [灯带]红灯闪烁 )pbdoc")
  .value(
    "system_effect_line_red_blink_fast", VPA::LedConstraint::system_effect_line_red_blink_fast,
    R"pbdoc( [灯带]红灯快速闪烁 )pbdoc")
  .value(
    "system_effect_line_red_breath", VPA::LedConstraint::system_effect_line_red_breath,
    R"pbdoc( [灯带]红灯呼吸 )pbdoc")
  .value(
    "system_effect_line_red_breath_fast", VPA::LedConstraint::system_effect_line_red_breath_fast,
    R"pbdoc( [灯带]红灯快速呼吸 )pbdoc")
  .value(
    "system_effect_line_red_one_by_one", VPA::LedConstraint::system_effect_line_red_one_by_one,
    R"pbdoc( [灯带]红灯逐个点亮 )pbdoc")
  .value(
    "system_effect_line_red_one_by_one_fast",
    VPA::LedConstraint::system_effect_line_red_one_by_one_fast,
    R"pbdoc( [灯带]红灯快速逐个点亮 )pbdoc")

  .value(
    "system_effect_line_blue_on", VPA::LedConstraint::system_effect_line_blue_on,
    R"pbdoc( [灯带]红灯常亮 )pbdoc")
  .value(
    "system_effect_line_blue_blink", VPA::LedConstraint::system_effect_line_blue_blink,
    R"pbdoc( [灯带]红灯闪烁 )pbdoc")
  .value(
    "system_effect_line_blue_blink_fast", VPA::LedConstraint::system_effect_line_blue_blink_fast,
    R"pbdoc( [灯带]红灯快速闪烁 )pbdoc")
  .value(
    "system_effect_line_blue_breath", VPA::LedConstraint::system_effect_line_blue_breath,
    R"pbdoc( [灯带]红灯呼吸 )pbdoc")
  .value(
    "system_effect_line_blue_breath_fast", VPA::LedConstraint::system_effect_line_blue_breath_fast,
    R"pbdoc( [灯带]红灯快速呼吸 )pbdoc")
  .value(
    "system_effect_line_blue_one_by_one", VPA::LedConstraint::system_effect_line_blue_one_by_one,
    R"pbdoc( [灯带]红灯逐个点亮 )pbdoc")
  .value(
    "system_effect_line_blue_one_by_one_fast",
    VPA::LedConstraint::system_effect_line_blue_one_by_one_fast, R"pbdoc( [灯带]红灯快速逐个点亮 )pbdoc")

  .value(
    "system_effect_line_yellow_on", VPA::LedConstraint::system_effect_line_yellow_on,
    R"pbdoc( [灯带]红灯常亮 )pbdoc")
  .value(
    "system_effect_line_yellow_blink", VPA::LedConstraint::system_effect_line_yellow_blink,
    R"pbdoc( [灯带]红灯闪烁 )pbdoc")
  .value(
    "system_effect_line_yellow_blink_fast",
    VPA::LedConstraint::system_effect_line_yellow_blink_fast,
    R"pbdoc( [灯带]红灯快速闪烁 )pbdoc")
  .value(
    "system_effect_line_yellow_breath", VPA::LedConstraint::system_effect_line_yellow_breath,
    R"pbdoc( [灯带]红灯呼吸 )pbdoc")
  .value(
    "system_effect_line_yellow_breath_fast",
    VPA::LedConstraint::system_effect_line_yellow_breath_fast,
    R"pbdoc( [灯带]红灯快速呼吸 )pbdoc")
  .value(
    "system_effect_line_yellow_one_by_one",
    VPA::LedConstraint::system_effect_line_yellow_one_by_one,
    R"pbdoc( [灯带]红灯逐个点亮 )pbdoc")
  .value(
    "system_effect_line_yellow_one_by_one_fast",
    VPA::LedConstraint::system_effect_line_yellow_one_by_one_fast, R"pbdoc( [灯带]红灯快速逐个点亮 )pbdoc")

  .value(
    "effect_mini_circular_breath",
    VPA::LedConstraint::effect_mini_circular_breath, R"pbdoc( [眼灯]圆形缩放 )pbdoc")
  .value(
    "effect_mini_circular_ring",
    VPA::LedConstraint::effect_mini_circular_ring, R"pbdoc( [眼灯]画圆环 )pbdoc")

  .value(
    "system_effect_mini_off",
    VPA::LedConstraint::system_effect_mini_off, R"pbdoc( [眼灯]常灭 )pbdoc")

  .value(
    "system_effect_mini_rectangle_color",
    VPA::LedConstraint::system_effect_mini_rectangle_color, R"pbdoc( [眼灯]方块变色 )pbdoc")
  .value(
    "system_effect_mini_centre_color",
    VPA::LedConstraint::system_effect_mini_centre_color, R"pbdoc( [眼灯]中间彩带 )pbdoc")
  .value(
    "system_effect_mini_three_circular",
    VPA::LedConstraint::system_effect_mini_three_circular, R"pbdoc( [眼灯]三圆呼吸 )pbdoc")
  .value(
    "system_effect_mini_one_by_one",
    VPA::LedConstraint::system_effect_mini_one_by_one, R"pbdoc( [眼灯]彩带逐个点亮 )pbdoc")
  ;
}

void DefineLedSeviceResponse(py::object m)
{
  py::class_<VPA::LedSeviceResponse>(m, "LedSeviceResponse", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::LedSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::LedSeviceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::LedSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: LedSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - code = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.code));
    })
  ;
}

void DefineMotionResultServiceResponse(py::object m)
{
  py::class_<VPA::MotionResultServiceResponse>(m, "MotionResultServiceResponse", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::MotionResultServiceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::MotionResultServiceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::MotionResultServiceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: MotionResultServiceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - motion_id: = %d"
          "\n│    - result = %s"
          "\n│    - code = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.motion_id,
          std::string(_ret.response.result ? "True" : "False").c_str(),
          _ret.response.code));
    })
  ;
}

void DefineMotionSequenceServiceResponse(py::object m)
{
  py::class_<VPA::MotionSequenceServiceResponse>(
    m, "MotionSequenceServiceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::MotionSequenceServiceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::MotionSequenceServiceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::MotionSequenceServiceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: MotionSequenceServiceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - motion_id = %d"
          "\n│    - result = %s"
          "\n│    - code = %d"
          "\n│    - describe: = %s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.motion_id,
          std::string(_ret.response.result ? "True" : "False").c_str(),
          _ret.response.code,
          _ret.response.describe.c_str()));
    })
  ;
}

void DefineMotionServoCmdResponse(py::object m)
{
  py::class_<VPA::MotionServoCmdResponse>(m, "MotionServoCmdResponse", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::MotionServoCmdResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::MotionServoCmdResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::MotionServoCmdResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: MotionServoCmdResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - motion_id: = %d"
          "\n│    - result = %s"
          "\n│    - code = %d"
          "\n│    - cmd_id = %d"
          "\n│    - order_process_bar = %d"
          "\n│    - status = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.motion_id,
          std::string(_ret.response.result ? "True" : "False").c_str(),
          _ret.response.code,
          _ret.response.cmd_id,
          _ret.response.order_process_bar,
          _ret.response.status));
    })
  ;
}

void DefineMotionParams(py::object m)
{
  py::class_<VPA::DefaultAndMaximum>(m, "DefaultAndMaximum", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "minimum_value", &VPA::DefaultAndMaximum::minimum_value,
    R"pbdoc( 最小值 )pbdoc")
  .def_readwrite(
    "default_value", &VPA::DefaultAndMaximum::default_value,
    R"pbdoc( 默认值 )pbdoc")
  .def_readwrite(
    "maximum_value", &VPA::DefaultAndMaximum::maximum_value,
    R"pbdoc( 最大值 )pbdoc")
  .def_readwrite(
    "unit", &VPA::DefaultAndMaximum::unit,
    R"pbdoc( 单位 )pbdoc")
  .def(
    "__repr__", [](const VPA::DefaultAndMaximum & _def) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: DefaultAndMaximum"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - minimum_value = %lf"
          "\n│  - default_value = %lf"
          "\n│  - maximum_value = %lf"
          "\n│  - unit: = '%s'"
          "\n└────────────────────────────────────────---",
          _def.minimum_value,
          _def.default_value,
          _def.maximum_value,
          _def.unit.c_str()));
    })
  ;

  py::class_<VPA::MotionParams>(m, "MotionParams", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "front_leg_lift", &VPA::MotionParams::front_leg_lift,
    R"pbdoc( 前腿抬腿高度 )pbdoc")
  .def_readwrite(
    "back_leg_lift", &VPA::MotionParams::back_leg_lift,
    R"pbdoc( 后腿抬腿高度 )pbdoc")
  .def_readwrite(
    "centroid_x", &VPA::MotionParams::centroid_x,
    R"pbdoc( 质心X轴约束 )pbdoc")
  .def_readwrite(
    "centroid_y", &VPA::MotionParams::centroid_y,
    R"pbdoc( 质心Y轴约束 )pbdoc")
  .def_readwrite(
    "centroid_z", &VPA::MotionParams::centroid_z,
    R"pbdoc( 质心Z轴约束 )pbdoc")
  .def_readwrite(
    "fulcrum_x", &VPA::MotionParams::fulcrum_x,
    R"pbdoc( 支点X轴约束 )pbdoc")
  .def_readwrite(
    "fulcrum_y", &VPA::MotionParams::fulcrum_y,
    R"pbdoc( 支点Y轴约束 )pbdoc")
  .def_readwrite(
    "fulcrum_z", &VPA::MotionParams::fulcrum_z,
    R"pbdoc( 支点Z轴约束 )pbdoc")
  .def_readwrite(
    "roll", &VPA::MotionParams::roll,
    R"pbdoc( 机身翻滚 )pbdoc")
  .def_readwrite(
    "pitch", &VPA::MotionParams::pitch,
    R"pbdoc( 机身俯仰 )pbdoc")
  .def_readwrite(
    "yaw", &VPA::MotionParams::yaw,
    R"pbdoc( 机身偏航 )pbdoc")
  .def_readwrite(
    "x_velocity", &VPA::MotionParams::x_velocity,
    R"pbdoc( x速度 )pbdoc")
  .def_readwrite(
    "y_velocity", &VPA::MotionParams::y_velocity,
    R"pbdoc( y速度 )pbdoc")
  .def_readwrite(
    "z_velocity", &VPA::MotionParams::z_velocity,
    R"pbdoc( z角速度 )pbdoc")
  .def_readwrite(
    "distance", &VPA::MotionParams::distance,
    R"pbdoc( 期望距离 )pbdoc")
  .def_readwrite(
    "duration", &VPA::MotionParams::duration,
    R"pbdoc( 期望时间 )pbdoc")
  .def_readwrite(
    "delta", &VPA::MotionParams::delta,
    R"pbdoc( 变化量 )pbdoc")
  .def(
    "__repr__", [](const VPA::MotionParams & _par) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: MotionParams"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - front_leg_lift:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - back_leg_lift:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - centroid_x:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - centroid_y:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - centroid_z:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - fulcrum_x:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - fulcrum_y:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - fulcrum_z:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - roll:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - pitch:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - yaw:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - x_velocity:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - y_velocity:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - z_velocity:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - distance:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - duration:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n│  - delta:"
          "\n│    - minimum_value = %lf"
          "\n│    - default_value = %lf"
          "\n│    - maximum_value = %lf"
          "\n│    - unit: = '%s'"
          "\n└────────────────────────────────────────---",
          _par.front_leg_lift.minimum_value,
          _par.front_leg_lift.default_value,
          _par.front_leg_lift.maximum_value,
          _par.front_leg_lift.unit.c_str(),
          _par.back_leg_lift.minimum_value,
          _par.back_leg_lift.default_value,
          _par.back_leg_lift.maximum_value,
          _par.back_leg_lift.unit.c_str(),
          _par.centroid_x.minimum_value,
          _par.centroid_x.default_value,
          _par.centroid_x.maximum_value,
          _par.centroid_x.unit.c_str(),
          _par.centroid_y.minimum_value,
          _par.centroid_y.default_value,
          _par.centroid_y.maximum_value,
          _par.centroid_y.unit.c_str(),
          _par.centroid_z.minimum_value,
          _par.centroid_z.default_value,
          _par.centroid_z.maximum_value,
          _par.centroid_z.unit.c_str(),
          _par.fulcrum_x.minimum_value,
          _par.fulcrum_x.default_value,
          _par.fulcrum_x.maximum_value,
          _par.fulcrum_x.unit.c_str(),
          _par.fulcrum_y.minimum_value,
          _par.fulcrum_y.default_value,
          _par.fulcrum_y.maximum_value,
          _par.fulcrum_y.unit.c_str(),
          _par.fulcrum_z.minimum_value,
          _par.fulcrum_z.default_value,
          _par.fulcrum_z.maximum_value,
          _par.fulcrum_z.unit.c_str(),
          _par.roll.minimum_value,
          _par.roll.default_value,
          _par.roll.maximum_value,
          _par.roll.unit.c_str(),
          _par.pitch.minimum_value,
          _par.pitch.default_value,
          _par.pitch.maximum_value,
          _par.pitch.unit.c_str(),
          _par.yaw.minimum_value,
          _par.yaw.default_value,
          _par.yaw.maximum_value,
          _par.yaw.unit.c_str(),
          _par.x_velocity.minimum_value,
          _par.x_velocity.default_value,
          _par.x_velocity.maximum_value,
          _par.x_velocity.unit.c_str(),
          _par.y_velocity.minimum_value,
          _par.y_velocity.default_value,
          _par.y_velocity.maximum_value,
          _par.y_velocity.unit.c_str(),
          _par.z_velocity.minimum_value,
          _par.z_velocity.default_value,
          _par.z_velocity.maximum_value,
          _par.z_velocity.unit.c_str(),
          _par.distance.minimum_value,
          _par.distance.default_value,
          _par.distance.maximum_value,
          _par.distance.unit.c_str(),
          _par.duration.minimum_value,
          _par.duration.default_value,
          _par.duration.maximum_value,
          _par.duration.unit.c_str(),
          _par.delta.minimum_value,
          _par.delta.default_value,
          _par.delta.maximum_value,
          _par.delta.unit.c_str()
      ));
    })
  ;
}

void DefineMotionId(py::object m)
{
  py::enum_<VPA::MotionId>(m, "MotionId")
  .value(
    "emergency_stop",
    VPA::MotionId::emergency_stop,
    R"pbdoc( 急停 )pbdoc")
  .value(
    "get_down",
    VPA::MotionId::get_down,
    R"pbdoc( 趴下 )pbdoc")
  .value(
    "resume_standing",
    VPA::MotionId::resume_standing,
    R"pbdoc( 恢复站立 )pbdoc")
  .value(
    "servo_standing",
    VPA::MotionId::servo_standing,
    R"pbdoc( 伺服站立 )pbdoc")
  .value(
    "back_flip",
    VPA::MotionId::back_flip,
    R"pbdoc( 后空翻 )pbdoc")
  .value(
    "front_flip",
    VPA::MotionId::front_flip,
    R"pbdoc( 前空翻 )pbdoc")
  .value(
    "bow",
    VPA::MotionId::bow,
    R"pbdoc( 作揖 )pbdoc")
  .value(
    "roll_left",
    VPA::MotionId::roll_left,
    R"pbdoc( 向左侧躺后恢复 )pbdoc")
  .value(
    "walk_the_dog",
    VPA::MotionId::walk_the_dog,
    R"pbdoc( 遛狗 )pbdoc")
  .value(
    "jump_stair",
    VPA::MotionId::jump_stair,
    R"pbdoc( 跳上台阶 )pbdoc")
  .value(
    "right_somersault",
    VPA::MotionId::right_somersault,
    R"pbdoc( 右侧空翻 )pbdoc")
  .value(
    "left_somersault",
    VPA::MotionId::left_somersault,
    R"pbdoc( 左侧空翻 )pbdoc")
  .value(
    "run_and_jump_front_flip",
    VPA::MotionId::run_and_jump_front_flip,
    R"pbdoc( 跑跳前空翻 )pbdoc")
  .value(
    "jump3d_left90deg",
    VPA::MotionId::jump3d_left90deg,
    R"pbdoc( 3D跳:左转90度 )pbdoc")
  .value(
    "jump3d_right90deg",
    VPA::MotionId::jump3d_right90deg,
    R"pbdoc( 3D跳:右转90度 )pbdoc")
  .value(
    "jump3d_forward60cm",
    VPA::MotionId::jump3d_forward60cm,
    R"pbdoc( 3D跳:前跳60cm )pbdoc")
  .value(
    "jump3d_forward30cm",
    VPA::MotionId::jump3d_forward30cm,
    R"pbdoc( 3D跳:前跳30cm )pbdoc")
  .value(
    "jump3d_left20cm",
    VPA::MotionId::jump3d_left20cm,
    R"pbdoc( 3D跳:左跳20cm )pbdoc")
  .value(
    "jump3d_right20cm",
    VPA::MotionId::jump3d_right20cm,
    R"pbdoc( 3D跳:右跳20cm )pbdoc")
  .value(
    "jump3d_up30cm",
    VPA::MotionId::jump3d_up30cm,
    R"pbdoc( 3D跳:向上30cm )pbdoc")
  .value(
    "jump3d_down_stair",
    VPA::MotionId::jump3d_down_stair,
    R"pbdoc( 3D跳:跳下台阶 )pbdoc")
  .value(
    "roll_right",
    VPA::MotionId::roll_right,
    R"pbdoc( 向右侧躺后恢复 )pbdoc")
  .value(
    "dance_collection",
    VPA::MotionId::dance_collection,
    R"pbdoc( 舞蹈集合 )pbdoc")
  .value(
    "hold_left_hand",
    VPA::MotionId::hold_left_hand,
    R"pbdoc( 握左手 )pbdoc")
  .value(
    "hold_right_hand",
    VPA::MotionId::hold_right_hand,
    R"pbdoc( 握右手 )pbdoc")
  .value(
    "sit_down",
    VPA::MotionId::sit_down,
    R"pbdoc( 坐下 )pbdoc")
  .value(
    "butt_circle",
    VPA::MotionId::butt_circle,
    R"pbdoc( 屁股画圆 )pbdoc")
  .value(
    "head_circle",
    VPA::MotionId::head_circle,
    R"pbdoc( 头画圆 )pbdoc")
  .value(
    "stretch_the_body",
    VPA::MotionId::stretch_the_body,
    R"pbdoc( 伸展身体 )pbdoc")
  .value(
    "shake_ass_left",
    VPA::MotionId::shake_ass_left,
    R"pbdoc( 向左摇晃屁股 )pbdoc")
  .value(
    "shake_ass_right",
    VPA::MotionId::shake_ass_right,
    R"pbdoc( 向右摇晃屁股 )pbdoc")
  .value(
    "shake_ass_from_side_to_side",
    VPA::MotionId::shake_ass_from_side_to_side,
    R"pbdoc( 左右摇晃屁股 )pbdoc")
  .value(
    "ballet",
    VPA::MotionId::ballet,
    R"pbdoc( 芭蕾舞 )pbdoc")
  .value(
    "space_walk",
    VPA::MotionId::space_walk,
    R"pbdoc( 太空步 )pbdoc")
  .value(
    "front_leg_jumping",
    VPA::MotionId::front_leg_jumping,
    R"pbdoc( 前腿开合跳 )pbdoc")
  .value(
    "hind_leg_jumping",
    VPA::MotionId::hind_leg_jumping,
    R"pbdoc( 后腿开合跳 )pbdoc")
  .value(
    "lift_the_left_leg_and_nod",
    VPA::MotionId::lift_the_left_leg_and_nod,
    R"pbdoc( 左腿抬起并点头 )pbdoc")
  .value(
    "lift_the_right_leg_and_nod",
    VPA::MotionId::lift_the_right_leg_and_nod,
    R"pbdoc( 右腿抬起并点头 )pbdoc")
  .value(
    "left_front_right_back_legs_apart",
    VPA::MotionId::left_front_right_back_legs_apart,
    R"pbdoc( 左前右后岔开腿 )pbdoc")
  .value(
    "right_front_left_back_legs_apart",
    VPA::MotionId::right_front_left_back_legs_apart,
    R"pbdoc( 右前左后岔开腿 )pbdoc")
  .value(
    "walk_nodding",
    VPA::MotionId::walk_nodding,
    R"pbdoc( 走路点头 )pbdoc")
  .value(
    "walking_with_divergence_and_adduction_alternately",
    VPA::MotionId::walking_with_divergence_and_adduction_alternately,
    R"pbdoc( 岔开内收交替走路 )pbdoc")
  .value(
    "nodding_in_place",
    VPA::MotionId::nodding_in_place,
    R"pbdoc( 原地踏步点头 )pbdoc")
  .value(
    "front_legs_jump_back_and_forth",
    VPA::MotionId::front_legs_jump_back_and_forth,
    R"pbdoc( 前腿前后跳 )pbdoc")
  .value(
    "hind_legs_jump_back_and_forth",
    VPA::MotionId::hind_legs_jump_back_and_forth,
    R"pbdoc( 后腿前后跳 )pbdoc")
  .value(
    "alternately_front_leg_lift",
    VPA::MotionId::alternately_front_leg_lift,
    R"pbdoc( 前腿交替抬起 )pbdoc")
  .value(
    "alternately_hind_leg_lift",
    VPA::MotionId::alternately_hind_leg_lift,
    R"pbdoc( 后腿交替抬起 )pbdoc")
  .value(
    "jump_collection",
    VPA::MotionId::jump_collection,
    R"pbdoc( 跳跃合集 )pbdoc")
  .value(
    "stretching_left_and_right",
    VPA::MotionId::stretching_left_and_right,
    R"pbdoc( 左右伸腿踏步 )pbdoc")
  .value(
    "jump_forward_and_backward",
    VPA::MotionId::jump_forward_and_backward,
    R"pbdoc( 前后摆腿跳跃 )pbdoc")
  .value(
    "step_left_and_right",
    VPA::MotionId::step_left_and_right,
    R"pbdoc( 左右摆腿踏步 )pbdoc")
  .value(
    "right_leg_back_and_forth_stepping",
    VPA::MotionId::right_leg_back_and_forth_stepping,
    R"pbdoc( 右腿前后踏步 )pbdoc")
  .value(
    "left_leg_back_and_forth_stepping",
    VPA::MotionId::left_leg_back_and_forth_stepping,
    R"pbdoc( 左腿前后踏步 )pbdoc")
  .value(
    "squat_down_on_all_fours",
    VPA::MotionId::squat_down_on_all_fours,
    R"pbdoc( 四足蹲起 )pbdoc")
  .value(
    "push_ups",
    VPA::MotionId::push_ups,
    R"pbdoc( 俯卧撑 )pbdoc")
  .value(
    "bow_to_each_other",
    VPA::MotionId::bow_to_each_other,
    R"pbdoc( 作揖比心 )pbdoc")
  .value(
    "absolute_force_control_attitude",
    VPA::MotionId::absolute_force_control_attitude,
    R"pbdoc( 绝对力控姿态（绝对姿态） )pbdoc")
  .value(
    "relatively_force_control_attitude",
    VPA::MotionId::relatively_force_control_attitude,
    R"pbdoc( 相对力控姿态 )pbdoc")
  .value(
    "absolute_position_control_attitude",
    VPA::MotionId::absolute_position_control_attitude,
    R"pbdoc( 绝对位控姿态（过渡站立） )pbdoc")
  .value(
    "relatively_position_control_attitude",
    VPA::MotionId::relatively_position_control_attitude,
    R"pbdoc( 相对位控姿态 )pbdoc")
  .value(
    "jump_back_and_forth",
    VPA::MotionId::jump_back_and_forth,
    R"pbdoc( 前后跳 )pbdoc")
  .value(
    "small_jump_walking",
    VPA::MotionId::small_jump_walking,
    R"pbdoc( 小跳行走 )pbdoc")
  .value(
    "trot_walking",
    VPA::MotionId::trot_walking,
    R"pbdoc( 小跑行走 )pbdoc")
  .value(
    "automatic_frequency_conversion_walking",
    VPA::MotionId::automatic_frequency_conversion_walking,
    R"pbdoc( 自动变频行走 )pbdoc")
  .value(
    "run_fast_walking",
    VPA::MotionId::run_fast_walking,
    R"pbdoc( 快跑行走 )pbdoc")
  .value(
    "sequence_custom",
    VPA::MotionId::sequence_custom,
    R"pbdoc( 序列动作 )pbdoc")
  .value(
    "illegal_motion_id",
    VPA::MotionId::illegal_motion_id,
    R"pbdoc( 非法运动ID )pbdoc")
  ;
}

void DefineMsgMotionSequenceGaitList(py::object m)
{
  using MsgMotionSequenceGaitList = std::vector<VPA::MsgMotionSequenceGait,
      std::allocator<VPA::MsgMotionSequenceGait>>;
  py::class_<MsgMotionSequenceGaitList>(m, "MsgMotionSequenceGaitList")
  .def(py::init<>())
  .def("empty", &MsgMotionSequenceGaitList::empty)
  .def("size", &MsgMotionSequenceGaitList::size)
  .def("max_size", &MsgMotionSequenceGaitList::max_size)
  .def("capacity", &MsgMotionSequenceGaitList::capacity)
  .def("clear", &MsgMotionSequenceGaitList::clear)
  .def(
    "push_back", (void (MsgMotionSequenceGaitList::*)(
      const VPA::MsgMotionSequenceGait &)) & MsgMotionSequenceGaitList::push_back)
  .def("pop_back", &MsgMotionSequenceGaitList::pop_back)
  .def(
    "at",
    (VPA::MsgMotionSequenceGait & (MsgMotionSequenceGaitList::*) (const size_t)) &
    MsgMotionSequenceGaitList::at)
  .def(
    "front",
    (VPA::MsgMotionSequenceGait & (MsgMotionSequenceGaitList::*) ()) &
    MsgMotionSequenceGaitList::front)
  .def(
    "back",
    (VPA::MsgMotionSequenceGait & (MsgMotionSequenceGaitList::*) ()) &
    MsgMotionSequenceGaitList::back)
  .def(
    "__len__", [](const MsgMotionSequenceGaitList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgMotionSequenceGaitList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgMotionSequenceGaitList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgMotionSequenceGaitList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::sequenceGaitVector(_data, "│  ").c_str()));
    })
  ;
}

void DefineMsgMotionSequencePaceList(py::object m)
{
  using MsgMotionSequencePaceList = std::vector<VPA::MsgMotionSequencePace,
      std::allocator<VPA::MsgMotionSequencePace>>;
  py::class_<MsgMotionSequencePaceList>(m, "MsgMotionSequencePaceList")
  .def(py::init<>())
  .def("empty", &MsgMotionSequencePaceList::empty)
  .def("size", &MsgMotionSequencePaceList::size)
  .def("max_size", &MsgMotionSequencePaceList::max_size)
  .def("capacity", &MsgMotionSequencePaceList::capacity)
  .def("clear", &MsgMotionSequencePaceList::clear)
  .def(
    "push_back", (void (MsgMotionSequencePaceList::*)(
      const VPA::MsgMotionSequencePace &)) & MsgMotionSequencePaceList::push_back)
  .def("pop_back", &MsgMotionSequencePaceList::pop_back)
  .def(
    "at",
    (VPA::MsgMotionSequencePace & (MsgMotionSequencePaceList::*) (const size_t)) &
    MsgMotionSequencePaceList::at)
  .def(
    "front",
    (VPA::MsgMotionSequencePace & (MsgMotionSequencePaceList::*) ()) &
    MsgMotionSequencePaceList::front)
  .def(
    "back",
    (VPA::MsgMotionSequencePace & (MsgMotionSequencePaceList::*) ()) &
    MsgMotionSequencePaceList::back)
  .def(
    "__len__", [](const MsgMotionSequencePaceList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgMotionSequencePaceList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgMotionSequencePaceList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgMotionSequencePaceList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::sequencePaceVector(_data, "│  ").c_str()));
    })
  ;
}

void DefineMotionSequence(py::object m)
{
  py::class_<VPA::MotionSequence>(m, "MotionSequence", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("name", &VPA::MotionSequence::name, R"pbdoc( 名称 )pbdoc")
  .def_readwrite("describe", &VPA::MotionSequence::describe, R"pbdoc( 描述 )pbdoc")
  .def_readwrite("gait_list", &VPA::MotionSequence::gait_list, R"pbdoc( 步态列表 )pbdoc")
  .def_readwrite("pace_list", &VPA::MotionSequence::pace_list, R"pbdoc( 步伐列表 )pbdoc")
  .def(
    "__repr__", [](const VPA::MotionSequence & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MotionSequence"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - name: = %s"
          "\n│  - describe: = %s"
          "\n│  - gait_list:"
          "%s"
          "\n│  - pace_list:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          _data.name.c_str(),
          _data.describe.c_str(),
          VPA::sequenceGaitVector(_data.gait_list, "│    ").c_str(),
          VPA::sequencePaceVector(_data.pace_list, "│    ").c_str()));
    })
  ;
}

void DefineObstacleMeta(py::object m)
{
  py::class_<VPA::ObstacleMeta>(m, "ObstacleMeta", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("detected", &VPA::ObstacleMeta::detected, R"pbdoc( 检测到 )pbdoc")
  .def(
    "__repr__", [](const VPA::ObstacleMeta & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: ObstacleMeta"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - detected = %s"
          "\n└───────────────────────────────────────────────────---",
          _data.detected ? "True" : "False"));
    })
  ;
}

void DefineTofObstacle(py::object m)
{
  py::class_<VPA::TofObstacle>(m, "TofObstacle", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("head_left", &VPA::TofObstacle::head_left, R"pbdoc( 头部左侧 )pbdoc")
  .def_readwrite("head_right", &VPA::TofObstacle::head_right, R"pbdoc( 头部右侧 )pbdoc")
  .def_readwrite("rear_left", &VPA::TofObstacle::rear_left, R"pbdoc( 尾部左侧 )pbdoc")
  .def_readwrite("rear_right", &VPA::TofObstacle::rear_right, R"pbdoc( 尾部右侧 )pbdoc")
  .def(
    "__repr__", [](const VPA::TofObstacle & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: TofObstacle"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - head_left:"
          "\n│    - detected = %s"
          "\n│  - head_right:"
          "\n│    - detected = %s"
          "\n│  - rear_left:"
          "\n│    - detected = %s"
          "\n│  - rear_right:"
          "\n│    - detected = %s"
          "\n└───────────────────────────────────────────────────---",
          _data.head_left.detected ? "True" : "False",
          _data.head_right.detected ? "True" : "False",
          _data.rear_left.detected ? "True" : "False",
          _data.rear_right.detected ? "True" : "False"));
    })
  ;
}

void DefineTofPayload(py::object m)
{
  py::class_<VPA::TofPayload>(m, "TofPayload", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("head", &VPA::TofPayload::head, R"pbdoc( 头部 )pbdoc")
  .def_readwrite("rear", &VPA::TofPayload::rear, R"pbdoc( 尾部 )pbdoc")
  .def(
    "__repr__", [](const VPA::TofPayload & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: TofPayload"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - head:"
          "\n│    - left_head:"
          "\n│      - header:"
          "\n│        - stamp:"
          "\n│          - sec = %d"
          "\n│          - nanosec = %d"
          "\n│        - frame_id = '%s'"
          "\n│      - data_available = %s"
          "\n│      - tof_position = %d"
          "\n│      - data = %ld"
          "\n│    - right_head:"
          "\n│      - header:"
          "\n│        - stamp:"
          "\n│          - sec = %d"
          "\n│          - nanosec = %d"
          "\n│        - frame_id = '%s'"
          "\n│      - data_available = %s"
          "\n│      - tof_position = %d"
          "\n│      - data = %ld"
          "\n│  - rear:"
          "\n│    - left_rear:"
          "\n│      - header:"
          "\n│        - stamp:"
          "\n│          - sec = %d"
          "\n│          - nanosec = %d"
          "\n│        - frame_id = '%s'"
          "\n│      - data_available = %s"
          "\n│      - tof_position = %d"
          "\n│      - data = %ld"
          "\n│    - right_rear:"
          "\n│      - header:"
          "\n│        - stamp:"
          "\n│          - sec = %d"
          "\n│          - nanosec = %d"
          "\n│        - frame_id = '%s'"
          "\n│      - data_available = %s"
          "\n│      - tof_position = %d"
          "\n│      - data = %ld"
          "\n└───────────────────────────────────────────────────---",
          _data.head.left_head.header.stamp.sec,
          _data.head.left_head.header.stamp.nanosec,
          _data.head.left_head.header.frame_id.c_str(),
          _data.head.left_head.data_available ? "True" : "False",
          _data.head.left_head.tof_position,
          _data.head.left_head.data.size(),
          _data.head.right_head.header.stamp.sec,
          _data.head.right_head.header.stamp.nanosec,
          _data.head.right_head.header.frame_id.c_str(),
          _data.head.right_head.data_available ? "True" : "False",
          _data.head.right_head.tof_position,
          _data.head.right_head.data.size(),
          _data.rear.left_rear.header.stamp.sec,
          _data.rear.left_rear.header.stamp.nanosec,
          _data.rear.left_rear.header.frame_id.c_str(),
          _data.rear.left_rear.data_available ? "True" : "False",
          _data.rear.left_rear.tof_position,
          _data.rear.left_rear.data.size(),
          _data.rear.right_rear.header.stamp.sec,
          _data.rear.right_rear.header.stamp.nanosec,
          _data.rear.right_rear.header.frame_id.c_str(),
          _data.rear.right_rear.data_available ? "True" : "False",
          _data.rear.right_rear.tof_position,
          _data.rear.right_rear.data.size()));
    })
  ;
}

void DefineMsgPersonnel(py::object m)
{
  py::class_<VPA::MsgPersonnel>(m, "MsgPersonnel", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("id", &VPA::MsgPersonnel::id, R"pbdoc( 人员id )pbdoc")
  .def_readwrite("username", &VPA::MsgPersonnel::username, R"pbdoc( 人员昵称 )pbdoc")
  .def_readwrite("voicestatus", &VPA::MsgPersonnel::voicestatus, R"pbdoc( 声纹录入状态 )pbdoc")
  .def_readwrite("facestatus", &VPA::MsgPersonnel::facestatus, R"pbdoc( 人脸录入状态 )pbdoc")

  .def(
    "__repr__", [](const VPA::MsgPersonnel & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgPersonnel"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - id: = %d"
          "\n│  - username: = %s"
          "\n│  - voicestatus: = %d"
          "\n│  - facestatus: = %d"
          "\n└───────────────────────────────────────────────────---",
          _data.id,
          _data.username.c_str(),
          _data.voicestatus,
          _data.facestatus));
    })
  ;
}

void DefineMsgPersonnelList(py::object m)
{
  using MsgPersonnelList = std::vector<VPA::MsgPersonnel>;
  py::bind_vector<MsgPersonnelList>(m, "MsgPersonnelVector")
  .def(py::init<>())
  .def("empty", &MsgPersonnelList::empty)
  .def("size", &MsgPersonnelList::size)
  .def("max_size", &MsgPersonnelList::max_size)
  .def("capacity", &MsgPersonnelList::capacity)
  .def("at", (VPA::MsgPersonnel & (MsgPersonnelList::*) (const size_t)) & MsgPersonnelList::at)
  .def("front", (VPA::MsgPersonnel & (MsgPersonnelList::*) ()) & MsgPersonnelList::front)
  .def("back", (VPA::MsgPersonnel & (MsgPersonnelList::*) ()) & MsgPersonnelList::back)
  .def(
    "__len__", [](const MsgPersonnelList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgPersonnelList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgPersonnelList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgPersonnelList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgPersonnelVector(_data, "  ").c_str()));
    })
  ;
}

void DefineSrvPersonnelResponse(py::object m)
{
  py::class_<VPA::SrvPersonnel::Response>(m, "SrvPersonnelResponse", py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("code", &VPA::SrvPersonnel::Response::code, R"pbdoc( 错误码 )pbdoc")
  .def_readwrite("result", &VPA::SrvPersonnel::Response::result, R"pbdoc( 结果 )pbdoc")
  .def(
    "__repr__", [](const VPA::SrvPersonnel::Response & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: SrvPersonnel::Response"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - code: = %d"
          "\n│  - result:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          _data.code,
          VPA::msgPersonnelVector(_data.result, "│  ").c_str()));
    })
  ;
}

void DefineFaceSeviceResponse(py::object m)
{
  py::class_<VPA::FaceSeviceResponse>(
    m, "FaceSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::FaceSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite(
    "response", &VPA::FaceSeviceResponse::response,
    R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::FaceSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: FaceSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - result: = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.result));
    })
  ;
}

void DefineFaceRecognizedSeviceResponse(py::object m)
{
  using MsgFaceResList = std::vector<VPA::MsgFaceRes>;
  py::bind_vector<MsgFaceResList>(m, "MsgFaceResList")
  .def(py::init<>())
  .def("empty", &MsgFaceResList::empty)
  .def("size", &MsgFaceResList::size)
  .def("max_size", &MsgFaceResList::max_size)
  .def("capacity", &MsgFaceResList::capacity)
  .def("at", (VPA::MsgFaceRes & (MsgFaceResList::*) (const size_t)) & MsgFaceResList::at)
  .def("front", (VPA::MsgFaceRes & (MsgFaceResList::*) ()) & MsgFaceResList::front)
  .def("back", (VPA::MsgFaceRes & (MsgFaceResList::*) ()) & MsgFaceResList::back)
  .def(
    "__len__", [](const MsgFaceResList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgFaceResList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgFaceResList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgFaceResList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgFaceResVector(_data, "│  ").c_str()));
    })
  ;

  using MsgFaceResDictionary = std::map<std::string, VPA::MsgFaceRes>;
  py::bind_map<MsgFaceResDictionary>(m, "MsgFaceResDictionary")
  .def(py::init<>())
  .def("empty", &MsgFaceResDictionary::empty)
  .def("size", &MsgFaceResDictionary::size)
  .def("max_size", &MsgFaceResDictionary::max_size)
  .def(
    "has_key", [](const MsgFaceResDictionary & v, const std::string & k) {
      return static_cast<bool>(!v.empty() && (v.find(k) != v.end()));
    })
  .def(
    "__len__", [](const MsgFaceResDictionary & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgFaceResDictionary & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgFaceResDictionary & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgFaceResDictionary"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgFaceResMap(_data, "│  ").c_str()));
    })
  ;

  py::class_<VPA::FaceRecognizedSeviceResponse>(
    m, "FaceRecognizedSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::FaceRecognizedSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("list", &VPA::FaceRecognizedSeviceResponse::list, R"pbdoc( 列表 )pbdoc")
  .def_readwrite("dictionary", &VPA::FaceRecognizedSeviceResponse::dictionary, R"pbdoc( 字典 )pbdoc")
  .def(
    "__repr__", [](const VPA::FaceRecognizedSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: FaceRecognizedSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - list:"
          "%s"
          "\n│  - dictionary:"
          "%s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          VPA::msgFaceResVector(_ret.list, "│    ").c_str(),
          VPA::msgFaceResMap(_ret.dictionary, "│    ").c_str()));
    })
  ;
}

void DefineTrainingWordsRecognizedSeviceResponse(py::object m)
{
  using MsgTrainingWordsList = std::vector<VPA::MsgTrainingWords>;
  py::bind_vector<MsgTrainingWordsList>(m, "MsgTrainingWordsList")
  .def(py::init<>())
  .def("empty", &MsgTrainingWordsList::empty)
  .def("size", &MsgTrainingWordsList::size)
  .def("max_size", &MsgTrainingWordsList::max_size)
  .def("capacity", &MsgTrainingWordsList::capacity)
  .def(
    "at",
    (VPA::MsgTrainingWords & (MsgTrainingWordsList::*) (const size_t)) &
    MsgTrainingWordsList::at)
  .def(
    "front",
    (VPA::MsgTrainingWords & (MsgTrainingWordsList::*) ()) & MsgTrainingWordsList::front)
  .def("back", (VPA::MsgTrainingWords & (MsgTrainingWordsList::*) ()) & MsgTrainingWordsList::back)
  .def(
    "__len__", [](const MsgTrainingWordsList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgTrainingWordsList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgTrainingWordsList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgTrainingWordsList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgTrainingWordsVector(_data, "│  ").c_str()));
    })
  ;

  using MsgTrainingWordsDictionary = std::map<std::string, VPA::MsgTrainingWords>;
  py::bind_map<MsgTrainingWordsDictionary>(m, "MsgTrainingWordsDictionary")
  .def(py::init<>())
  .def("empty", &MsgTrainingWordsDictionary::empty)
  .def("size", &MsgTrainingWordsDictionary::size)
  .def("max_size", &MsgTrainingWordsDictionary::max_size)
  .def(
    "has_key", [](const MsgTrainingWordsDictionary & v, const std::string & k) {
      return static_cast<bool>(!v.empty() && (v.find(k) != v.end()));
    })
  .def(
    "__len__", [](const MsgTrainingWordsDictionary & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgTrainingWordsDictionary & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgTrainingWordsDictionary & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgTrainingWordsDictionary"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgTrainingWordsMap(_data, "│  ").c_str()));
    })
  ;

  py::class_<VPA::TrainingWordsRecognizedSeviceResponse>(
    m, "TrainingWordsRecognizedSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::TrainingWordsRecognizedSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite(
    "response", &VPA::TrainingWordsRecognizedSeviceResponse::response,
    R"pbdoc( 反馈 )pbdoc")
  .def_readwrite(
    "dictionary", &VPA::TrainingWordsRecognizedSeviceResponse::dictionary,
    R"pbdoc( 字典 )pbdoc")
  .def(
    "__repr__", [](const VPA::TrainingWordsRecognizedSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: TrainingWordsRecognizedSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - training_set:"
          "%s"
          "\n│  - dictionary:"
          "%s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          VPA::msgTrainingWordsVector(_ret.response.training_set, "│      ").c_str(),
          VPA::msgTrainingWordsMap(_ret.dictionary, "│    ").c_str()));
    })
  ;
}

void DefineTrainingWordsRecognizedMessageResponse(py::object m)
{
  py::class_<VPA::TrainingWordsRecognizedMessageResponse>(
    m, "TrainingWordsRecognizedMessageResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::TrainingWordsRecognizedMessageResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite(
    "response", &VPA::TrainingWordsRecognizedMessageResponse::response,
    R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::TrainingWordsRecognizedMessageResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: TrainingWordsRecognizedMessageResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - trigger: = %s"
          "\n│    - type: = %s"
          "\n│    - value: = %s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.trigger.c_str(),
          _ret.response.type.c_str(),
          _ret.response.value.c_str()));
    })
  ;
}

void DefineVoiceprintRecognizedResponse(py::object m)
{
  py::class_<VPA::VoiceprintRecognizedResponse>(
    m, "VoiceprintRecognizedResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::VoiceprintRecognizedResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("list", &VPA::VoiceprintRecognizedResponse::list, R"pbdoc( 识别人员列表 )pbdoc")
  .def_readwrite("data", &VPA::VoiceprintRecognizedResponse::data, R"pbdoc( 识别状态数据 )pbdoc")
  .def(
    "__repr__", [](const VPA::VoiceprintRecognizedResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: VoiceprintRecognizedResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - list: = %s"
          "\n│  - data: = %s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          VPA::stringVector(_ret.list).c_str(),
          std::string(_ret.data ? "True" : "False").c_str()));
    })
  ;
}

void DefineGestureData(py::object m)
{
  py::class_<VPA::GestureData>(
    m, "GestureData",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite(
    "pulling_hand_or_two_fingers_in", &VPA::GestureData::pulling_hand_or_two_fingers_in,
    R"pbdoc( 手掌拉近 )pbdoc")
  .def_readwrite(
    "pushing_hand_or_two_fingers_away",
    &VPA::GestureData::pushing_hand_or_two_fingers_away, R"pbdoc( 手掌推开 )pbdoc")
  .def_readwrite(
    "sliding_hand_or_two_fingers_up", &VPA::GestureData::sliding_hand_or_two_fingers_up,
    R"pbdoc( 手向上抬 )pbdoc")
  .def_readwrite(
    "sliding_hand_or_two_fingers_down",
    &VPA::GestureData::sliding_hand_or_two_fingers_down, R"pbdoc( 手向下压 )pbdoc")
  .def_readwrite(
    "sliding_hand_or_two_fingers_left",
    &VPA::GestureData::sliding_hand_or_two_fingers_left, R"pbdoc( 手向左推 )pbdoc")
  .def_readwrite(
    "sliding_hand_or_two_fingers_right",
    &VPA::GestureData::sliding_hand_or_two_fingers_right, R"pbdoc( 手向右推 )pbdoc")
  .def_readwrite("stop_sign", &VPA::GestureData::stop_sign, R"pbdoc( 停止手势 )pbdoc")
  .def_readwrite("thumb_down", &VPA::GestureData::thumb_down, R"pbdoc( 大拇指朝下 )pbdoc")
  .def_readwrite("thumb_up", &VPA::GestureData::thumb_up, R"pbdoc( 大拇指朝上 )pbdoc")
  .def_readwrite(
    "zooming_in_with_hand_or_two_fingers",
    &VPA::GestureData::zooming_in_with_hand_or_two_fingers, R"pbdoc( 张开手掌或手指 )pbdoc")
  .def_readwrite(
    "zooming_out_with_hand_or_two_fingers",
    &VPA::GestureData::zooming_out_with_hand_or_two_fingers, R"pbdoc( 闭合手掌或手指 )pbdoc")
  .def(
    "__repr__", [](const VPA::GestureData & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: GestureData"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - pulling_hand_or_two_fingers_in = %s"
          "\n│  - pushing_hand_or_two_fingers_away = %s"
          "\n│  - sliding_hand_or_two_fingers_up = %s"
          "\n│  - sliding_hand_or_two_fingers_down = %s"
          "\n│  - sliding_hand_or_two_fingers_left = %s"
          "\n│  - sliding_hand_or_two_fingers_right = %s"
          "\n│  - stop_sign = %s"
          "\n│  - thumb_down = %s"
          "\n│  - thumb_up = %s"
          "\n│  - zooming_in_with_hand_or_two_fingers = %s"
          "\n│  - zooming_out_with_hand_or_two_fingers = %s"
          "\n└────────────────────────────────────────---",
          std::string(_ret.pulling_hand_or_two_fingers_in ? "True" : "False").c_str(),
          std::string(_ret.pushing_hand_or_two_fingers_away ? "True" : "False").c_str(),
          std::string(_ret.sliding_hand_or_two_fingers_up ? "True" : "False").c_str(),
          std::string(_ret.sliding_hand_or_two_fingers_down ? "True" : "False").c_str(),
          std::string(_ret.sliding_hand_or_two_fingers_left ? "True" : "False").c_str(),
          std::string(_ret.sliding_hand_or_two_fingers_right ? "True" : "False").c_str(),
          std::string(_ret.stop_sign ? "True" : "False").c_str(),
          std::string(_ret.thumb_down ? "True" : "False").c_str(),
          std::string(_ret.thumb_up ? "True" : "False").c_str(),
          std::string(_ret.zooming_in_with_hand_or_two_fingers ? "True" : "False").c_str(),
          std::string(_ret.zooming_out_with_hand_or_two_fingers ? "True" : "False").c_str()));
    })
  ;
}

void DefineGestureType(py::object m)
{
  py::enum_<VPA::GestureType>(m, "GestureType")
  .value(
    "no_gesture",
    VPA::GestureType::no_gesture,
    R"pbdoc( 无手势 )pbdoc")
  .value(
    "pulling_hand_or_two_fingers_in",
    VPA::GestureType::pulling_hand_or_two_fingers_in,
    R"pbdoc( 手掌拉近 )pbdoc")
  .value(
    "pushing_hand_or_two_fingers_away",
    VPA::GestureType::pushing_hand_or_two_fingers_away,
    R"pbdoc( 手掌推开 )pbdoc")
  .value(
    "sliding_hand_or_two_fingers_up",
    VPA::GestureType::sliding_hand_or_two_fingers_up,
    R"pbdoc( 手向上抬 )pbdoc")
  .value(
    "sliding_hand_or_two_fingers_down",
    VPA::GestureType::sliding_hand_or_two_fingers_down,
    R"pbdoc( 手向下压 )pbdoc")
  .value(
    "sliding_hand_or_two_fingers_left",
    VPA::GestureType::sliding_hand_or_two_fingers_left,
    R"pbdoc( 手向左推 )pbdoc")
  .value(
    "sliding_hand_or_two_fingers_right",
    VPA::GestureType::sliding_hand_or_two_fingers_right,
    R"pbdoc( 手向右推 )pbdoc")
  .value(
    "stop_sign",
    VPA::GestureType::stop_sign,
    R"pbdoc( 停止手势 )pbdoc")
  .value(
    "thumb_down",
    VPA::GestureType::thumb_down,
    R"pbdoc( 大拇指朝下 )pbdoc")
  .value(
    "thumb_up",
    VPA::GestureType::thumb_up,
    R"pbdoc( 大拇指朝上 )pbdoc")
  .value(
    "zooming_in_with_hand_or_two_fingers",
    VPA::GestureType::zooming_in_with_hand_or_two_fingers,
    R"pbdoc( 张开手掌或手指 )pbdoc")
  .value(
    "zooming_out_with_hand_or_two_fingers",
    VPA::GestureType::zooming_out_with_hand_or_two_fingers,
    R"pbdoc( 闭合手掌或手指 )pbdoc")
  .value(
    "id_upper_limit",
    VPA::GestureType::id_upper_limit,
    R"pbdoc( 手势id上限 )pbdoc")
  ;
}

void DefineGestureRecognizedSeviceResponse(py::object m)
{
  py::class_<VPA::GestureRecognizedSeviceResponse>(
    m, "GestureRecognizedSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::GestureRecognizedSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::GestureRecognizedSeviceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::GestureRecognizedSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: GestureRecognizedSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - code = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.code));
    })
  ;
}

void DefineGestureRecognizedMessageResponse(py::object m)
{
  py::class_<VPA::GestureRecognizedMessageResponse>(
    m, "GestureRecognizedMessageResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::GestureRecognizedMessageResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("data", &VPA::GestureRecognizedMessageResponse::data, R"pbdoc( 识别状态数据 )pbdoc")
  .def(
    "__repr__", [](const VPA::GestureRecognizedMessageResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: GestureRecognizedMessageResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - data:"
          "\n│    - pulling_hand_or_two_fingers_in = %s"
          "\n│    - pushing_hand_or_two_fingers_away = %s"
          "\n│    - sliding_hand_or_two_fingers_up = %s"
          "\n│    - sliding_hand_or_two_fingers_down = %s"
          "\n│    - sliding_hand_or_two_fingers_left = %s"
          "\n│    - sliding_hand_or_two_fingers_right = %s"
          "\n│    - stop_sign = %s"
          "\n│    - thumb_down = %s"
          "\n│    - thumb_up = %s"
          "\n│    - zooming_in_with_hand_or_two_fingers = %s"
          "\n│    - zooming_out_with_hand_or_two_fingers = %s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          std::string(_ret.data.pulling_hand_or_two_fingers_in ? "True" : "False").c_str(),
          std::string(_ret.data.pushing_hand_or_two_fingers_away ? "True" : "False").c_str(),
          std::string(_ret.data.sliding_hand_or_two_fingers_up ? "True" : "False").c_str(),
          std::string(_ret.data.sliding_hand_or_two_fingers_down ? "True" : "False").c_str(),
          std::string(_ret.data.sliding_hand_or_two_fingers_left ? "True" : "False").c_str(),
          std::string(_ret.data.sliding_hand_or_two_fingers_right ? "True" : "False").c_str(),
          std::string(_ret.data.stop_sign ? "True" : "False").c_str(),
          std::string(_ret.data.thumb_down ? "True" : "False").c_str(),
          std::string(_ret.data.thumb_up ? "True" : "False").c_str(),
          std::string(_ret.data.zooming_in_with_hand_or_two_fingers ? "True" : "False").c_str(),
          std::string(_ret.data.zooming_out_with_hand_or_two_fingers ? "True" : "False").c_str()));
    })
  ;
}

void DefineSkeletonType(py::object m)
{
  py::enum_<VPA::SkeletonType>(m, "SkeletonType")
  .value("squat", VPA::SkeletonType::SPORT_SQUAT, R"pbdoc( 深蹲 )pbdoc")
  .value("highknees", VPA::SkeletonType::SPORT_HIGHKNEES, R"pbdoc( 高抬腿 )pbdoc")
  .value("situp", VPA::SkeletonType::SPORT_SITUP, R"pbdoc( 仰卧起坐 )pbdoc")
  .value("pressup", VPA::SkeletonType::SPORT_PRESSUP, R"pbdoc( 俯卧撑 )pbdoc")
  .value("plank", VPA::SkeletonType::SPORT_PLANK, R"pbdoc( 平板支撑 )pbdoc")
  .value("jumpjack", VPA::SkeletonType::SPORT_JUMPJACK, R"pbdoc( 开合跳 )pbdoc")
  ;
}

void DefineSkeletonRecognizedSeviceResponse(py::object m)
{
  py::class_<VPA::SkeletonRecognizedSeviceResponse>(
    m, "SkeletonRecognizedSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::SkeletonRecognizedSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::SkeletonRecognizedSeviceResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::SkeletonRecognizedSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: SkeletonRecognizedSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - result = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.result));
    })
  ;
}

void DefineSkeletonRecognizedMessageResponse(py::object m)
{
  py::class_<VPA::SkeletonRecognizedMessageResponse>(
    m, "SkeletonRecognizedMessageResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::SkeletonRecognizedMessageResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::SkeletonRecognizedMessageResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::SkeletonRecognizedMessageResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: SkeletonRecognizedMessageResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - algo_switch = %d"
          "\n│    - sport_type = %d"
          "\n│    - counts = %d"
          "\n│    - duration = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.algo_switch,
          _ret.response.sport_type,
          _ret.response.counts,
          _ret.response.duration));
    })
  ;
}

void DefineMapPresetSeviceResponse(py::object m)
{
  using MsgPresetList = std::vector<VPA::MsgPreset>;
  py::bind_vector<MsgPresetList>(m, "MsgPresetList")
  .def(py::init<>())
  .def("empty", &MsgPresetList::empty)
  .def("size", &MsgPresetList::size)
  .def("max_size", &MsgPresetList::max_size)
  .def("capacity", &MsgPresetList::capacity)
  .def("at", (VPA::MsgPreset & (MsgPresetList::*) (const size_t)) & MsgPresetList::at)
  .def("front", (VPA::MsgPreset & (MsgPresetList::*) ()) & MsgPresetList::front)
  .def("back", (VPA::MsgPreset & (MsgPresetList::*) ()) & MsgPresetList::back)
  .def(
    "__len__", [](const MsgPresetList & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgPresetList & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgPresetList & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgPresetList"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgPresetVector(_data, "│  ").c_str()));
    })
  ;

  using MsgPresetDictionary = std::map<std::string, VPA::MsgPreset>;
  py::bind_map<MsgPresetDictionary>(m, "MsgPresetDictionary")
  .def(py::init<>())
  .def("empty", &MsgPresetDictionary::empty)
  .def("size", &MsgPresetDictionary::size)
  .def("max_size", &MsgPresetDictionary::max_size)
  .def(
    "has_key", [](const MsgPresetDictionary & v, const std::string & k) {
      return static_cast<bool>(!v.empty() && (v.find(k) != v.end()));
    })
  .def(
    "__len__", [](const MsgPresetDictionary & v) {
      return v.size();
    })
  .def(
    "__iter__", [](MsgPresetDictionary & v) {
      return py::make_iterator(v.begin(), v.end());
    }, py::keep_alive<0, 1>())
  .def(
    "__repr__", [](const MsgPresetDictionary & _data) {
      return std::string(
        FORMAT(
          "┌───────────────────────────────────────────────────---"
          "\n│- type: MsgPresetDictionary"
          "\n├───────────────────────────────────────────────────---"
          "\n│- data:"
          "%s"
          "\n└───────────────────────────────────────────────────---",
          VPA::msgPresetMap(_data, "│  ").c_str()));
    })
  ;

  py::class_<VPA::MapPresetSeviceResponse>(
    m, "MapPresetSeviceResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::MapPresetSeviceResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("map_name", &VPA::MapPresetSeviceResponse::map_name, R"pbdoc( 地图名称 )pbdoc")
  .def_readwrite("is_outdoor", &VPA::MapPresetSeviceResponse::is_outdoor, R"pbdoc( 是否为户外地图 )pbdoc")
  .def_readwrite("list", &VPA::MapPresetSeviceResponse::list, R"pbdoc( 列表 )pbdoc")
  .def_readwrite("dictionary", &VPA::MapPresetSeviceResponse::dictionary, R"pbdoc( 字典 )pbdoc")
  .def(
    "__repr__", [](const VPA::MapPresetSeviceResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: MapPresetSeviceResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - map_name = %s"
          "\n│  - is_outdoor = %s"
          "\n│  - list:"
          "%s"
          "\n│  - dictionary:"
          "%s"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.map_name.c_str(),
          std::string(_ret.is_outdoor ? "True" : "False").c_str(),
          VPA::msgPresetVector(_ret.list, "│    ").c_str(),
          VPA::msgPresetMap(_ret.dictionary, "│    ").c_str()));
    })
  ;
}

void DefineNavigationActionResponse(py::object m)
{
  py::class_<VPA::NavigationActionResponse>(
    m, "NavigationActionResponse",
    py::dynamic_attr())
  .def(py::init<>())
  .def_readwrite("state", &VPA::NavigationActionResponse::state, R"pbdoc( 状态 )pbdoc")
  .def_readwrite("response", &VPA::NavigationActionResponse::response, R"pbdoc( 反馈 )pbdoc")
  .def(
    "__repr__", [](const VPA::NavigationActionResponse & _ret) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: NavigationActionResponse"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n│  - response:"
          "\n│    - result = %d"
          "\n└────────────────────────────────────────---",
          _ret.state.code,
          _ret.state.describe.c_str(),
          _ret.response.result));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
