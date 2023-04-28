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

#include "vptpy.hpp"
#include "cyberdog_vp_terminal/interface.hpp"

namespace cyberdog_visual_programming_terminal_py
{
namespace VPT = cyberdog_visual_programming_terminal;

void DefineInterfaceNetwork(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Network>(m, "InterfaceCyberdogNetwork", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogNetwork object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Network::Network_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help"
  )
  .def(
    "list", &VPT::Interface::Cyberdog::Network::Network_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List"
  )
  .def(
    "tree", &VPT::Interface::Cyberdog::Network::Network_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree"
  )
  .def(
    "state", &VPT::Interface::Cyberdog::Network::Network_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State"
  )
  .def(
    "data", &VPT::Interface::Cyberdog::Network::Network_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data"
  )

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Network &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Network"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceFollow(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Follow>(m, "InterfaceCyberdogFollow", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogFollow object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help"
  )
  .def(
    "list", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "add_personnel", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 添加人员 )pbdoc",
    py::arg("fun") = "AddPersonnel")
  .def(
    "delete_personnel", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 删除人员 )pbdoc",
    py::arg("fun") = "DeletePersonnel")
  .def(
    "cancel_follow", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 取消跟随 )pbdoc",
    py::arg("fun") = "CancelFollow")
  .def(
    "follow_personnel", &VPT::Interface::Cyberdog::Follow::Follow_, R"pbdoc( 跟踪人员 )pbdoc",
    py::arg("fun") = "FollowPersonnel")
  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Follow &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Follow"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceMotion(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Motion>(m, "InterfaceCyberdogMotion", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogMotion object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "request", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 请求 )pbdoc",
    py::arg("fun") = "Request")

  .def(
    "emergency_stop", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 急停 )pbdoc",
    py::arg("fun") = "EmergencyStop")
  .def(
    "get_down", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 趴下 )pbdoc",
    py::arg("fun") = "GetDown")
  .def(
    "resume_standing", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 恢复站立 )pbdoc",
    py::arg("fun") = "ResumeStanding")
  .def(
    "back_flip", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 后空翻 )pbdoc",
    py::arg("fun") = "BackFlip")
  .def(
    "front_flip", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 前空翻 )pbdoc",
    py::arg("fun") = "FrontFlip")
  .def(
    "bow", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 作揖 )pbdoc",
    py::arg("fun") = "Bow")
  .def(
    "roll_left", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 向左侧躺后恢复 )pbdoc",
    py::arg("fun") = "RollLeft")
  .def(
    "walk_the_dog", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 遛狗 )pbdoc",
    py::arg("fun") = "WalkTheDog")
  .def(
    "jump_stair", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 跳上台阶 )pbdoc",
    py::arg("fun") = "JumpStair")
  .def(
    "right_somersault", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 右侧空翻 )pbdoc",
    py::arg("fun") = "RightSomersault")
  .def(
    "left_somersault", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 左侧空翻 )pbdoc",
    py::arg("fun") = "LeftSomersault")
  .def(
    "run_and_jump_front_flip", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 跑跳前空翻 )pbdoc",
    py::arg("fun") = "RunAndJumpFrontFlip")
  .def(
    "jump3d_left90deg", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:左转90度 )pbdoc",
    py::arg("fun") = "Jump3dLeft90deg")
  .def(
    "jump3d_right90deg", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:右转90度 )pbdoc",
    py::arg("fun") = "Jump3dRight90deg")
  .def(
    "jump3d_forward60cm", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:前跳60cm )pbdoc",
    py::arg("fun") = "Jump3dForward60cm")
  .def(
    "jump3d_forward30cm", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:前跳30cm )pbdoc",
    py::arg("fun") = "Jump3dForward30cm")
  .def(
    "jump3d_left20cm", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:左跳20cm )pbdoc",
    py::arg("fun") = "Jump3dLeft20cm")
  .def(
    "jump3d_right20cm", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:右跳20cm )pbdoc",
    py::arg("fun") = "Jump3dRight20cm")
  .def(
    "jump3d_up30cm", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:向上30cm )pbdoc",
    py::arg("fun") = "Jump3dUp30cm")
  .def(
    "jump3d_down_stair", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 3D跳:跳下台阶 )pbdoc",
    py::arg("fun") = "Jump3dDownStair")
  .def(
    "roll_right", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 向右侧躺后恢复 )pbdoc",
    py::arg("fun") = "RollRight")
  .def(
    "dance_collection", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 舞蹈集合 )pbdoc",
    py::arg("fun") = "DanceCollection")
  .def(
    "hold_left_hand", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 握左手 )pbdoc",
    py::arg("fun") = "HoldLeftHand")
  .def(
    "hold_right_hand", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 握右手 )pbdoc",
    py::arg("fun") = "HoldRightHand")
  .def(
    "sit_down", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 坐下 )pbdoc",
    py::arg("fun") = "SitDown")
  .def(
    "butt_circle", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 屁股画圆 )pbdoc",
    py::arg("fun") = "ButtCircle")
  .def(
    "head_circle", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 头画圆 )pbdoc",
    py::arg("fun") = "HeadCircle")
  .def(
    "stretch_the_body", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 伸展身体 )pbdoc",
    py::arg("fun") = "StretchTheBody")
  .def(
    "shake_ass_left", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 向左摇晃屁股 )pbdoc",
    py::arg("fun") = "ShakeAssLeft")
  .def(
    "shake_ass_right", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 向右摇晃屁股 )pbdoc",
    py::arg("fun") = "ShakeAssRight")
  .def(
    "shake_ass_from_side_to_side", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 左右摇晃屁股 )pbdoc",
    py::arg("fun") = "ShakeAssFromSideToSide")
  .def(
    "ballet", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 芭蕾舞 )pbdoc",
    py::arg("fun") = "Ballet")
  .def(
    "space_walk", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 太空步 )pbdoc",
    py::arg("fun") = "SpaceWalk")
  .def(
    "front_leg_jumping", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 前腿开合跳 )pbdoc",
    py::arg("fun") = "FrontLegJumping")
  .def(
    "hind_leg_jumping", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 后腿开合跳 )pbdoc",
    py::arg("fun") = "HindLegJumping")
  .def(
    "lift_the_left_leg_and_nod", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 左腿抬起并点头 )pbdoc",
    py::arg("fun") = "LiftTheLeftLegAndNod")
  .def(
    "lift_the_right_leg_and_nod", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 右腿抬起并点头 )pbdoc",
    py::arg("fun") = "LiftTheRightLegAndNod")
  .def(
    "left_front_right_back_legs_apart", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 左前右后岔开腿 )pbdoc",
    py::arg("fun") = "LeftFrontRightBackLegsApart")
  .def(
    "right_front_left_back_legs_apart", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 右前左后岔开腿 )pbdoc",
    py::arg("fun") = "RightFrontLeftBackLegsApart")
  .def(
    "walk_nodding", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 走路点头 )pbdoc",
    py::arg("fun") = "WalkNodding")
  .def(
    "walking_with_divergence_and_adduction_alternately", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 岔开内收交替走路 )pbdoc",
    py::arg("fun") = "WalkingWithDivergenceAndAdductionAlternately")
  .def(
    "nodding_in_place", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 原地踏步点头 )pbdoc",
    py::arg("fun") = "NoddingInPlace")
  .def(
    "front_legs_jump_back_and_forth", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 前腿前后跳 )pbdoc",
    py::arg("fun") = "FrontLegsJumpBackAndForth")
  .def(
    "hind_legs_jump_back_and_forth", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 后腿前后跳 )pbdoc",
    py::arg("fun") = "HindLegsJumpBackAndForth")
  .def(
    "alternately_front_leg_lift", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 前腿交替抬起 )pbdoc",
    py::arg("fun") = "AlternatelyFrontLegLift")
  .def(
    "alternately_hind_leg_lift", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 后腿交替抬起 )pbdoc",
    py::arg("fun") = "AlternatelyHindLegLift")
  .def(
    "jump_collection", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 跳跃合集 )pbdoc",
    py::arg("fun") = "JumpCollection")
  .def(
    "stretching_left_and_right", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 左右伸腿踏步 )pbdoc",
    py::arg("fun") = "StretchingLeftAndRight")
  .def(
    "jump_forward_and_backward", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 前后摆腿跳跃 )pbdoc",
    py::arg("fun") = "JumpForwardAndBackward")
  .def(
    "step_left_and_right", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 左右摆腿踏步 )pbdoc",
    py::arg("fun") = "StepLeftAndRight")
  .def(
    "right_leg_back_and_forth_stepping", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 右腿前后踏步 )pbdoc",
    py::arg("fun") = "RightLegBackAndForthStepping")
  .def(
    "left_leg_back_and_forth_stepping", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 左腿前后踏步 )pbdoc",
    py::arg("fun") = "LeftLegBackAndForthStepping")
  .def(
    "squat_down_on_all_fours", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 四足蹲起 )pbdoc",
    py::arg("fun") = "SquatDownOnAllFours")
  .def(
    "push_ups", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 俯卧撑 )pbdoc",
    py::arg("fun") = "PushUps")
  .def(
    "bow_to_each_other", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 作揖比心 )pbdoc",
    py::arg("fun") = "BowToEachOther")
  .def(
    "absolute_attitude", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 绝对力控姿态 )pbdoc",
    py::arg("fun") = "AbsoluteAttitude")
  .def(
    "relatively_force_control_attitude", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 相对力控姿态 )pbdoc",
    py::arg("fun") = "RelativelyForceControlAttitude")
  .def(
    "transition_standing", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 绝对位控姿态（过渡站立） )pbdoc",
    py::arg("fun") = "TransitionStanding")
  .def(
    "relatively_position_control_attitude", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 相对位控姿态 )pbdoc",
    py::arg("fun") = "RelativelyPositionControlAttitude")
  .def(
    "jump_back_and_forth", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 前后跳 )pbdoc",
    py::arg("fun") = "JumpBackAndForth")
  .def(
    "trot_walking", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 小跳行走 )pbdoc",
    py::arg("fun") = "TrotWalking")
  .def(
    "automatic_frequency_conversion_walking", &VPT::Interface::Cyberdog::Motion::Motion_,
    R"pbdoc( 自动变频行走 )pbdoc",
    py::arg("fun") = "AutomaticFrequencyConversionWalking")
  .def(
    "run_fast_walking", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 快跑行走 )pbdoc",
    py::arg("fun") = "RunFastWalking")
  .def(
    "turn", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 转向 )pbdoc",
    py::arg("fun") = "Turn")
  .def(
    "go_straight", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 直行 )pbdoc",
    py::arg("fun") = "GoStraight")
  .def(
    "lateral_movement", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 横移 )pbdoc",
    py::arg("fun") = "LateralMovement")
  .def(
    "run_sequence", &VPT::Interface::Cyberdog::Motion::Motion_, R"pbdoc( 运行序列 )pbdoc",
    py::arg("fun") = "RunSequence")
  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Motion &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Motion"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceNavigation(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Navigation>(
    m, "InterfaceCyberdogNavigation",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogNavigation object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "get_preset", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 获取预置点 )pbdoc",
    py::arg("fun") = "GetPreset"
  )
  .def(
    "turn_on_navigation", &VPT::Interface::Cyberdog::Navigation::Navigation_,
    R"pbdoc( 开启（进入）导航模式 )pbdoc",
    py::arg("fun") = "TurnOnNavigation"
  )
  .def(
    "turn_off_navigation", &VPT::Interface::Cyberdog::Navigation::Navigation_,
    R"pbdoc( 关闭（退出）导航模式 )pbdoc",
    py::arg("fun") = "TurnOffNavigation"
  )
  .def(
    "cancel_navigation", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 取消导航 )pbdoc",
    py::arg("fun") = "CancelNavigation"
  )
  .def(
    "navigation_to_preset", &VPT::Interface::Cyberdog::Navigation::Navigation_,
    R"pbdoc( 导航到预置点 )pbdoc",
    py::arg("fun") = "NavigationToPreset"
  )
  .def(
    "navigation_to_coordinates", &VPT::Interface::Cyberdog::Navigation::Navigation_,
    R"pbdoc( 导航到坐标点 )pbdoc",
    py::arg("fun") = "NavigationToCoordinates"
  )
  .def(
    "navigation_to_pose", &VPT::Interface::Cyberdog::Navigation::Navigation_,
    R"pbdoc( 导航到目标位姿 )pbdoc",
    py::arg("fun") = "NavigationToPose"
  )
  .def(
    "to_preset", &VPT::Interface::Cyberdog::Navigation::Navigation_, R"pbdoc( 去预置点 )pbdoc",
    py::arg("fun") = "ToPreset"
  )

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Navigation &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Navigation"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceTask(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Task>(m, "InterfaceCyberdogTask", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogTask object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "start", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 开始任务 )pbdoc",
    py::arg("fun") = "Start")
  .def(
    "stop", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 结束任务 )pbdoc",
    py::arg("fun") = "Stop")
  .def(
    "block", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 设置块 )pbdoc",
    py::arg("fun") = "Block")
  .def(
    "breakpoint_block", &VPT::Interface::Cyberdog::Task::Task_, R"pbdoc( 设置断点块 )pbdoc",
    py::arg("fun") = "BreakpointBlock")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Task &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Task"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceTrain(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Train>(m, "InterfaceCyberdogTrain", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogTrain object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Train::Train_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Train::Train_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Train::Train_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Train::Train_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "get_training_words_set", &VPT::Interface::Cyberdog::Train::Train_, R"pbdoc( 获取训练词集合 )pbdoc",
    py::arg("fun") = "GetTrainingWordsSet")
  .def(
    "training_words_recognized", &VPT::Interface::Cyberdog::Train::Train_, R"pbdoc( 识别到训练词 )pbdoc",
    py::arg("fun") = "TrainingWordsRecognized")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Train &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Train"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfacePersonnel(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Personnel>(
    m, "InterfaceCyberdogPersonnel",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogPersonnel object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Personnel::Personnel_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Personnel::Personnel_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Personnel::Personnel_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Personnel::Personnel_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "get_data", &VPT::Interface::Cyberdog::Personnel::Personnel_, R"pbdoc( 获取人员数据 )pbdoc",
    py::arg("fun") = "GetData")
  .def(
    "face_recognized", &VPT::Interface::Cyberdog::Personnel::Personnel_, R"pbdoc( 识别到目标人脸 )pbdoc",
    py::arg("fun") = "FaceRecognized")
  .def(
    "voiceprint_recognized", &VPT::Interface::Cyberdog::Personnel::Personnel_,
    R"pbdoc( 识别到目标人员声纹 )pbdoc",
    py::arg("fun") = "VoiceprintRecognized")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Personnel &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Personnel"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceGesture(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Gesture>(
    m, "InterfaceCyberdogGesture",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogGesture object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "turn_on_recognition", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 打开识别手势功能 )pbdoc",
    py::arg("fun") = "TurnOnRecognition")
  .def(
    "turn_off_recognition", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 关闭识别手势功能 )pbdoc",
    py::arg("fun") = "TurnOffRecognition")
  .def(
    "recognized_designated_gesture", &VPT::Interface::Cyberdog::Gesture::Gesture_,
    R"pbdoc( 识别到指定手势 )pbdoc",
    py::arg("fun") = "RecognizedDesignatedGesture")
  .def(
    "recognized_any_gesture", &VPT::Interface::Cyberdog::Gesture::Gesture_,
    R"pbdoc( 识别到任意手势 )pbdoc",
    py::arg("fun") = "RecognizedAnyGesture")
  .def(
    "recognized", &VPT::Interface::Cyberdog::Gesture::Gesture_, R"pbdoc( 开始识别并识别到任意手势 )pbdoc",
    py::arg("fun") = "Recognized")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Gesture &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Gesture"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceSkeleton(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Skeleton>(
    m, "InterfaceCyberdogSkeleton",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogSkeleton object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "turn_on_recognition", &VPT::Interface::Cyberdog::Skeleton::Skeleton_,
    R"pbdoc( 打开识别骨骼（点）功能 )pbdoc",
    py::arg("fun") = "TurnOnRecognition")
  .def(
    "turn_off_recognition", &VPT::Interface::Cyberdog::Skeleton::Skeleton_,
    R"pbdoc( 关闭识别骨骼（点）功能 )pbdoc",
    py::arg("fun") = "TurnOffRecognition")
  .def(
    "blocking_recognized", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 阻塞式识别到 )pbdoc",
    py::arg("fun") = "BlockingRecognized")
  .def(
    "instant_recognized", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 瞬时式识别到 )pbdoc",
    py::arg("fun") = "InstantRecognized")
  .def(
    "sports_recognition", &VPT::Interface::Cyberdog::Skeleton::Skeleton_, R"pbdoc( 运动识别 )pbdoc",
    py::arg("fun") = "SportsRecognition")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Skeleton &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Skeleton"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceBms(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Bms>(m, "InterfaceCyberdogBms", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogBms object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Bms::Bms_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Bms::Bms_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Bms::Bms_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Bms::Bms_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Bms::Bms_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Bms &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Bms"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceLed(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Led>(m, "InterfaceCyberdogLed", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogLed object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "play", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 设置led系统灯效 )pbdoc",
    py::arg("fun") = "Play")
  .def(
    "play_rgb", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 设置led用户定义RGB灯效 )pbdoc",
    py::arg("fun") = "PlayRgb")
  .def(
    "freed", &VPT::Interface::Cyberdog::Led::Led_, R"pbdoc( 释放led设备 )pbdoc",
    py::arg("fun") = "Freed")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Led &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Led"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceAudio(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Audio>(m, "InterfaceCyberdogAudio", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogAudio object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "play", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 阻塞式播放语音 )pbdoc",
    py::arg("fun") = "Play")
  .def(
    "instantly_play", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 立即播放语音 )pbdoc",
    py::arg("fun") = "InstantlyPlay")
  .def(
    "get_volume", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 获取音量 )pbdoc",
    py::arg("fun") = "GetVolume")
  .def(
    "set_volume", &VPT::Interface::Cyberdog::Audio::Audio_, R"pbdoc( 设置音量 )pbdoc",
    py::arg("fun") = "SetVolume")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Audio &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Audio"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceTouch(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Touch>(m, "InterfaceCyberdogTouch", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogTouch object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Touch::Touch_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Touch::Touch_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Touch::Touch_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Touch::Touch_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Touch::Touch_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Touch &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Touch"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceGps(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Gps>(m, "InterfaceCyberdogGps", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogGps object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Gps::Gps_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Gps::Gps_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Gps::Gps_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Gps::Gps_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Gps::Gps_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Gps &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Gps"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceTof(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Tof>(m, "InterfaceCyberdogTof", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogTof object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Tof::Tof_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Tof::Tof_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Tof::Tof_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Tof::Tof_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Tof::Tof_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "obstacle", &VPT::Interface::Cyberdog::Tof::Tof_, R"pbdoc( 障碍物 )pbdoc",
    py::arg("fun") = "Obstacle")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Tof &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Tof"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceLidar(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Lidar>(m, "InterfaceCyberdogLidar", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogLidar object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Lidar::Lidar_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Lidar::Lidar_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Lidar::Lidar_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Lidar::Lidar_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Lidar::Lidar_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Lidar &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Lidar"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceUltrasonic(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Ultrasonic>(
    m, "InterfaceCyberdogUltrasonic",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogUltrasonic object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Ultrasonic::Ultrasonic_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Ultrasonic::Ultrasonic_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Ultrasonic::Ultrasonic_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Ultrasonic::Ultrasonic_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Ultrasonic::Ultrasonic_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Ultrasonic &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Ultrasonic"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceOdometer(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Odometer>(m, "InterfaceCyberdogOdometer", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogOdometer object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Odometer::Odometer_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Odometer::Odometer_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Odometer::Odometer_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Odometer::Odometer_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Odometer::Odometer_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Odometer &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Odometer"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceImu(py::object m)
{
  py::class_<VPT::Interface::Cyberdog::Imu>(m, "InterfaceCyberdogImu", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdogImu object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Imu::Imu_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Imu::Imu_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Imu::Imu_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Imu::Imu_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "data", &VPT::Interface::Cyberdog::Imu::Imu_, R"pbdoc( 数据 )pbdoc",
    py::arg("fun") = "Data")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog::Imu &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog::Imu"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceCyberdog(py::object m)
{
  py::class_<VPT::Interface::Cyberdog>(m, "InterfaceCyberdog", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceCyberdog object )pbdoc")
  .def(
    "help", &VPT::Interface::Cyberdog::Cyberdog_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Cyberdog::Cyberdog_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Cyberdog::Cyberdog_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Cyberdog::Cyberdog_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")
  .def(
    "set_log", &VPT::Interface::Cyberdog::Cyberdog_, R"pbdoc( 设置日志 )pbdoc",
    py::arg("fun") = "SetLog")
  .def(
    "shutdown", &VPT::Interface::Cyberdog::Cyberdog_, R"pbdoc( 终止 )pbdoc",
    py::arg("fun") = "Shutdown")

  .def_readwrite("network", &VPT::Interface::Cyberdog::network_, R"pbdoc( 网络 )pbdoc")
  .def_readwrite("follow", &VPT::Interface::Cyberdog::follow_, R"pbdoc( 跟随 )pbdoc")
  .def_readwrite("motion", &VPT::Interface::Cyberdog::motion_, R"pbdoc( 运动 )pbdoc")
  .def_readwrite("navigation", &VPT::Interface::Cyberdog::navigation_, R"pbdoc( 导航 )pbdoc")
  .def_readwrite("task", &VPT::Interface::Cyberdog::task_, R"pbdoc( 任务 )pbdoc")
  .def_readwrite("train", &VPT::Interface::Cyberdog::train_, R"pbdoc( 训练 )pbdoc")
  .def_readwrite("personnel", &VPT::Interface::Cyberdog::personnel_, R"pbdoc( 人员 )pbdoc")
  .def_readwrite("gesture", &VPT::Interface::Cyberdog::gesture_, R"pbdoc( 手势识别 )pbdoc")
  .def_readwrite("skeleton", &VPT::Interface::Cyberdog::skeleton_, R"pbdoc( 运动识别（基于骨骼点） )pbdoc")

  .def_readwrite("bms", &VPT::Interface::Cyberdog::bms_, R"pbdoc( 电池管理系统 )pbdoc")
  .def_readwrite("led", &VPT::Interface::Cyberdog::led_, R"pbdoc( LED灯 )pbdoc")
  .def_readwrite("audio", &VPT::Interface::Cyberdog::audio_, R"pbdoc( 语音模块 )pbdoc")
  .def_readwrite("touch", &VPT::Interface::Cyberdog::touch_, R"pbdoc( 触摸板 )pbdoc")
  .def_readwrite("gps", &VPT::Interface::Cyberdog::gps_, R"pbdoc( 全球定位系统 )pbdoc")
  .def_readwrite("tof", &VPT::Interface::Cyberdog::tof_, R"pbdoc( 激光测距 )pbdoc")
  .def_readwrite("lidar", &VPT::Interface::Cyberdog::lidar_, R"pbdoc( 雷达 )pbdoc")
  .def_readwrite("ultrasonic", &VPT::Interface::Cyberdog::ultrasonic_, R"pbdoc( 超声波 )pbdoc")
  .def_readwrite("odometer", &VPT::Interface::Cyberdog::odometer_, R"pbdoc( 里程计 )pbdoc")
  .def_readwrite("imu", &VPT::Interface::Cyberdog::imu_, R"pbdoc( 惯导 )pbdoc")

  .def(
    "__repr__", [](const VPT::Interface::Cyberdog &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Cyberdog"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceVisualInterface(py::object m)
{
  py::class_<VPT::Interface::Visual::Interface>(m, "InterfaceVisualInterface", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceVisualInterface object )pbdoc")
  .def(
    "help", &VPT::Interface::Visual::Interface::Interface_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Visual::Interface::Interface_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Visual::Interface::Interface_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Visual::Interface::Interface_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "__repr__", [](const VPT::Interface::Visual::Interface &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Visual::Interface"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceVisualDebugAbilityset(py::object m)
{
  py::class_<VPT::Interface::Visual::Debugger::Abilityset>(
    m, "InterfaceVisualDebugAbilityset",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceVisualDebugAbilityset object )pbdoc")
  .def(
    "help", &VPT::Interface::Visual::Debugger::Abilityset::Abilityset_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Visual::Debugger::Abilityset::Abilityset_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Visual::Debugger::Abilityset::Abilityset_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Visual::Debugger::Abilityset::Abilityset_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "launch", &VPT::Interface::Visual::Debugger::Abilityset::Abilityset_, R"pbdoc( 启动 )pbdoc",
    py::arg("fun") = "Launch")

  .def(
    "__repr__", [](const VPT::Interface::Visual::Debugger::Abilityset &) {
      return std::string(
        ONE_FORMAT(
          "┌──────────────────────────────────────────────---"
          "\n│- type: Interface::Visual::Debugger::Abilityset"
          "\n└──────────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceVisualDebuggerEngine(py::object m)
{
  py::class_<VPT::Interface::Visual::Debugger::Engine>(
    m, "InterfaceVisualDebuggerEngine",
    py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceVisualDebuggerEngine object )pbdoc")
  .def(
    "help", &VPT::Interface::Visual::Debugger::Engine::Engine_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Visual::Debugger::Engine::Engine_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Visual::Debugger::Engine::Engine_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Visual::Debugger::Engine::Engine_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def(
    "mock_app_request", &VPT::Interface::Visual::Debugger::Engine::Engine_,
    R"pbdoc( 模拟APP请求 )pbdoc",
    py::arg("fun") = "MockAppRequest")

  .def(
    "__repr__", [](const VPT::Interface::Visual::Debugger::Engine &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Visual::Debugger::Engine"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceVisualDebugger(py::object m)
{
  py::class_<VPT::Interface::Visual::Debugger>(m, "InterfaceVisualDebugger", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceVisualDebugger object )pbdoc")
  .def(
    "help", &VPT::Interface::Visual::Debugger::Debugger_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Visual::Debugger::Debugger_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Visual::Debugger::Debugger_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "state", &VPT::Interface::Visual::Debugger::Debugger_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")

  .def_readwrite("abilityset", &VPT::Interface::Visual::Debugger::abilityset_, R"pbdoc( 能力集 )pbdoc")
  .def_readwrite("engine", &VPT::Interface::Visual::Debugger::engine_, R"pbdoc( 引擎 )pbdoc")

  .def(
    "__repr__", [](const VPT::Interface::Visual::Debugger &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Visual::Debugger"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceVisual(py::object m)
{
  py::class_<VPT::Interface::Visual>(m, "InterfaceVisual", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceVisual object )pbdoc")
  .def(
    "help", &VPT::Interface::Visual::Visual_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Visual::Visual_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Visual::Visual_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")

  .def_readwrite("interface", &VPT::Interface::Visual::interface_, R"pbdoc( 接口 )pbdoc")
  .def_readwrite("debugger", &VPT::Interface::Visual::debugger_, R"pbdoc( 调试 )pbdoc")

  .def(
    "__repr__", [](const VPT::Interface::Visual &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Visual"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceTypeEnum(py::object m)
{
  py::class_<VPT::Interface::Type::Enum>(m, "InterfaceTypeEnum", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceTypeEnum object )pbdoc")
  .def(
    "help", &VPT::Interface::Type::Enum::Enum_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Type::Enum::Enum_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Type::Enum::Enum_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "StateCode", &VPT::Interface::Type::Enum::Enum_, R"pbdoc( 状态码 )pbdoc",
    py::arg("fun") = "StateCode")
  .def(
    "LedConstraint", &VPT::Interface::Type::Enum::Enum_, R"pbdoc( LED约束 )pbdoc",
    py::arg("fun") = "LedConstraint")

  .def(
    "__repr__", [](const VPT::Interface::Type::Enum &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Type::Enum"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceTypeClass(py::object m)
{
  py::class_<VPT::Interface::Type::Class>(m, "InterfaceTypeClass", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceTypeClass object )pbdoc")
  .def(
    "help", &VPT::Interface::Type::Class::Class_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Type::Class::Class_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Type::Class::Class_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")
  .def(
    "Time", &VPT::Interface::Type::Class::Class_, R"pbdoc( 时间戳 )pbdoc",
    py::arg("fun") = "Time")
  .def(
    "Header", &VPT::Interface::Type::Class::Class_, R"pbdoc( 消息头 )pbdoc",
    py::arg("fun") = "Header")
  .def(
    "LaserScan", &VPT::Interface::Type::Class::Class_, R"pbdoc( 激光扫描数据 )pbdoc",
    py::arg("fun") = "LaserScan")
  .def(
    "Range", &VPT::Interface::Type::Class::Class_, R"pbdoc( 范围测量 )pbdoc",
    py::arg("fun") = "Range")
  .def(
    "Odometry", &VPT::Interface::Type::Class::Class_, R"pbdoc( 里程计 )pbdoc",
    py::arg("fun") = "Odometry")
  .def(
    "Imu", &VPT::Interface::Type::Class::Class_, R"pbdoc( 惯性导航 )pbdoc",
    py::arg("fun") = "Imu")
  .def(
    "Point", &VPT::Interface::Type::Class::Class_, R"pbdoc( 点 )pbdoc",
    py::arg("fun") = "Point")
  .def(
    "Quaternion", &VPT::Interface::Type::Class::Class_, R"pbdoc( 四元数 )pbdoc",
    py::arg("fun") = "Quaternion")
  .def(
    "Pose", &VPT::Interface::Type::Class::Class_, R"pbdoc( 位姿 )pbdoc",
    py::arg("fun") = "Pose")
  .def(
    "Vector3", &VPT::Interface::Type::Class::Class_, R"pbdoc( 三元向量 )pbdoc",
    py::arg("fun") = "Vector3")
  .def(
    "Twist", &VPT::Interface::Type::Class::Class_, R"pbdoc( 6轴速度 )pbdoc",
    py::arg("fun") = "Twist")
  .def(
    "PoseWithCovariance", &VPT::Interface::Type::Class::Class_, R"pbdoc( 具有协方差的位姿 )pbdoc",
    py::arg("fun") = "PoseWithCovariance")
  .def(
    "TwistWithCovariance", &VPT::Interface::Type::Class::Class_, R"pbdoc( 具有协方差的6轴速度 )pbdoc",
    py::arg("fun") = "TwistWithCovariance")
  .def(
    "BmsStatus", &VPT::Interface::Type::Class::Class_, R"pbdoc( 电池消息 )pbdoc",
    py::arg("fun") = "BmsStatus")
  .def(
    "TouchStatus", &VPT::Interface::Type::Class::Class_, R"pbdoc( 具有协方差的6轴速度 )pbdoc",
    py::arg("fun") = "TouchStatus")
  .def(
    "GpsPayload", &VPT::Interface::Type::Class::Class_, R"pbdoc( 全球定位系统消息 )pbdoc",
    py::arg("fun") = "GpsPayload")
  .def(
    "SingleTofPayload", &VPT::Interface::Type::Class::Class_, R"pbdoc( 单个TOF消息 )pbdoc",
    py::arg("fun") = "SingleTofPayload")
  .def(
    "HeadTofPayload", &VPT::Interface::Type::Class::Class_, R"pbdoc( 头部TOF消息 )pbdoc",
    py::arg("fun") = "HeadTofPayload")
  .def(
    "RearTofPayload", &VPT::Interface::Type::Class::Class_, R"pbdoc( 尾部TOF消息 )pbdoc",
    py::arg("fun") = "RearTofPayload")
  .def(
    "TofPayload", &VPT::Interface::Type::Class::Class_, R"pbdoc( TOF消息 )pbdoc",
    py::arg("fun") = "TofPayload")
  .def(
    "State", &VPT::Interface::Type::Class::Class_, R"pbdoc( 状态 )pbdoc",
    py::arg("fun") = "State")
  .def(
    "DefaultAndMaximum", &VPT::Interface::Type::Class::Class_, R"pbdoc( 参数默认类型 )pbdoc",
    py::arg("fun") = "DefaultAndMaximum")
  .def(
    "MotionParams", &VPT::Interface::Type::Class::Class_, R"pbdoc( 运动参数消息 )pbdoc",
    py::arg("fun") = "MotionParams")
  .def(
    "MotionResultServiceResponse", &VPT::Interface::Type::Class::Class_, R"pbdoc( 运动服务反馈类型 )pbdoc",
    py::arg("fun") = "MotionResultServiceResponse")
  .def(
    "MotionServoCmdResponse", &VPT::Interface::Type::Class::Class_, R"pbdoc( 运动伺服指令反馈类型 )pbdoc",
    py::arg("fun") = "MotionServoCmdResponse")
  .def(
    "MotionSequenceServiceResponse", &VPT::Interface::Type::Class::Class_,
    R"pbdoc( 序列运动服务反馈类型 )pbdoc",
    py::arg("fun") = "MotionSequenceServiceResponse")
  .def(
    "LedSeviceResponse", &VPT::Interface::Type::Class::Class_, R"pbdoc( LED服务反馈类型 )pbdoc",
    py::arg("fun") = "LedSeviceResponse")
  .def(
    "ConnectorStatus", &VPT::Interface::Type::Class::Class_, R"pbdoc( 连接状态类型 )pbdoc",
    py::arg("fun") = "ConnectorStatus")
  .def(
    "MotionSequenceGait", &VPT::Interface::Type::Class::Class_, R"pbdoc( 序列步态消息类型 )pbdoc",
    py::arg("fun") = "MotionSequenceGait")
  .def(
    "MotionSequenceParam", &VPT::Interface::Type::Class::Class_, R"pbdoc( 序列步伐（参数）消息类型 )pbdoc",
    py::arg("fun") = "MotionSequenceParam")
  .def(
    "SequenceMeta", &VPT::Interface::Type::Class::Class_, R"pbdoc( 序列元消息类型 )pbdoc",
    py::arg("fun") = "SequenceMeta")
  .def(
    "MotionSequence", &VPT::Interface::Type::Class::Class_, R"pbdoc( 运动序列消息类型 )pbdoc",
    py::arg("fun") = "MotionSequence")
  .def(
    "AudioPlaySeviceResponse", &VPT::Interface::Type::Class::Class_, R"pbdoc( 语音播放服务反馈消息类型 )pbdoc",
    py::arg("fun") = "AudioPlaySeviceResponse")
  .def(
    "AudioGetVolumeSeviceResponse", &VPT::Interface::Type::Class::Class_,
    R"pbdoc( 语音获取音量服务反馈消息类型 )pbdoc",
    py::arg("fun") = "AudioGetVolumeSeviceResponse")
  .def(
    "AudioSetVolumeSeviceResponse", &VPT::Interface::Type::Class::Class_,
    R"pbdoc( 语音获取音量服务反馈消息类型 )pbdoc",
    py::arg("fun") = "AudioSetVolumeSeviceResponse")
  .def(
    "FaceRecognitionResult", &VPT::Interface::Type::Class::Class_, R"pbdoc( 人脸识别结果信息类型 )pbdoc",
    py::arg("fun") = "FaceRecognitionResult")
  .def(
    "FaceRecognizedSeviceResponse", &VPT::Interface::Type::Class::Class_,
    R"pbdoc( 人脸识别反馈消息类型 )pbdoc",
    py::arg("fun") = "FaceRecognizedSeviceResponse")
  .def(
    "GestureType", &VPT::Interface::Type::Class::Class_, R"pbdoc( 手势类型 )pbdoc",
    py::arg("fun") = "GestureType")
  .def(
    "GestureData", &VPT::Interface::Type::Class::Class_, R"pbdoc( 手势数据 )pbdoc",
    py::arg("fun") = "GestureData")
  .def(
    "GestureRecognized", &VPT::Interface::Type::Class::Class_, R"pbdoc( 手势识别消息类型 )pbdoc",
    py::arg("fun") = "GestureRecognized")

  .def(
    "__repr__", [](const VPT::Interface::Type::Class &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Type::Class"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterfaceType(py::object m)
{
  py::class_<VPT::Interface::Type>(m, "InterfaceType", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create InterfaceType object )pbdoc")
  .def(
    "help", &VPT::Interface::Type::Type_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Type::Type_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Type::Type_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")

  .def_readwrite("enum", &VPT::Interface::Type::enum_, R"pbdoc( 枚举 )pbdoc")
  .def_readwrite("clas", &VPT::Interface::Type::class_, R"pbdoc( 结构 )pbdoc")

  .def(
    "__repr__", [](const VPT::Interface::Type &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface::Type"
          "\n└────────────────────────────────────────---"));
    })
  ;
}

void DefineInterface(py::object m)
{
  py::class_<VPT::Interface>(m, "Interface", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Interface object )pbdoc")
  .def(
    "help", &VPT::Interface::Interface_, R"pbdoc( 帮助文档 )pbdoc",
    py::arg("fun") = "Help")
  .def(
    "list", &VPT::Interface::Interface_, R"pbdoc( 接口列表 )pbdoc",
    py::arg("fun") = "List")
  .def(
    "tree", &VPT::Interface::Interface_, R"pbdoc( 接口树 )pbdoc",
    py::arg("fun") = "Tree")

  .def_readwrite("visual", &VPT::Interface::visual_, R"pbdoc( 可视化 )pbdoc")
  .def_readwrite("type", &VPT::Interface::type_, R"pbdoc( 类型 )pbdoc")
  .def_readwrite("cyberdog", &VPT::Interface::cyberdog_, R"pbdoc( 铁蛋 )pbdoc")

  .def(
    "__repr__", [](const VPT::Interface &) {
      return std::string(
        ONE_FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Interface"
          "\n└────────────────────────────────────────---"));
    })
  ;
}
}   // namespace cyberdog_visual_programming_terminal_py
