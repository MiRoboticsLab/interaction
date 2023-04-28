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
#include "cyberdog_vp_abilityset/motion.hpp"

namespace cyberdog_visual_programming_abilityset_py
{
namespace VPA = cyberdog_visual_programming_abilityset;

void DefineMotion(py::object m)
{
  py::class_<VPA::Motion, VPA::Base>(m, "Motion", py::dynamic_attr())
  .def(py::init<>(), R"pbdoc( create Motion object )pbdoc")

  .def_readonly(
    "params", &VPA::Motion::params_,
    R"pbdoc( 运动参数 )pbdoc")
  .def_readonly(
    "motion_id", &VPA::Motion::motion_id_,
    R"pbdoc( 运动ID )pbdoc")

  .def(
    "request",
    &VPA::Motion::Request,
    R"pbdoc( 请求 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::emergency_stop,
    py::arg("duration") = 0)

  .def(
    "emergency_stop",
    &VPA::Motion::Request,
    R"pbdoc( 急停 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::emergency_stop,
    py::arg("duration") = 0)

  .def(
    "get_down",
    &VPA::Motion::Request,
    R"pbdoc( 趴下 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::get_down,
    py::arg("duration") = 0)

  .def(
    "resume_standing",
    &VPA::Motion::Request,
    R"pbdoc( 恢复站立 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::resume_standing,
    py::arg("duration") = 0)

  .def(
    "back_flip",
    &VPA::Motion::Request,
    R"pbdoc( 后空翻 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::back_flip,
    py::arg("duration") = 0)

  .def(
    "front_flip",
    &VPA::Motion::Request,
    R"pbdoc( 前空翻 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::front_flip,
    py::arg("duration") = 0)

  .def(
    "bow",
    &VPA::Motion::Request,
    R"pbdoc( 作揖 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::bow,
    py::arg("duration") = 0)

  .def(
    "roll_left",
    &VPA::Motion::Request,
    R"pbdoc( 向左侧躺后恢复 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::roll_left,
    py::arg("duration") = 0)

  .def(
    "walk_the_dog",
    &VPA::Motion::WalkTheDog,
    R"pbdoc( 遛狗 )pbdoc",
    py::arg("front_leg_lift") = VPA::motion_params_.front_leg_lift.default_value,
    py::arg("back_leg_lift") = VPA::motion_params_.back_leg_lift.default_value
  )

  .def(
    "jump_stair",
    &VPA::Motion::Request,
    R"pbdoc( 跳上台阶 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump_stair,
    py::arg("duration") = 0)

  .def(
    "right_somersault",
    &VPA::Motion::Request,
    R"pbdoc( 右侧空翻 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::right_somersault,
    py::arg("duration") = 0)

  .def(
    "left_somersault",
    &VPA::Motion::Request,
    R"pbdoc( 左侧空翻 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::left_somersault,
    py::arg("duration") = 0)

  .def(
    "run_and_jump_front_flip",
    &VPA::Motion::Request,
    R"pbdoc( 跑跳前空翻 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::run_and_jump_front_flip,
    py::arg("duration") = 0)

  .def(
    "jump3d_left90deg",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:左转90度 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_left90deg,
    py::arg("duration") = 0)

  .def(
    "jump3d_right90deg",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:右转90度 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_right90deg,
    py::arg("duration") = 0)

  .def(
    "jump3d_forward60cm",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:前跳60cm )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_forward60cm,
    py::arg("duration") = 0)

  .def(
    "jump3d_forward30cm",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:前跳30cm )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_forward30cm,
    py::arg("duration") = 0)

  .def(
    "jump3d_left20cm",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:左跳20cm )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_left20cm,
    py::arg("duration") = 0)

  .def(
    "jump3d_right20cm",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:右跳20cm )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_right20cm,
    py::arg("duration") = 0)

  .def(
    "jump3d_up30cm",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:向上30cm )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_up30cm,
    py::arg("duration") = 0)

  .def(
    "jump3d_down_stair",
    &VPA::Motion::Request,
    R"pbdoc( 3D跳:跳下台阶 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump3d_down_stair,
    py::arg("duration") = 0)

  .def(
    "roll_right",
    &VPA::Motion::Request,
    R"pbdoc( 向右侧躺后恢复 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::roll_right,
    py::arg("duration") = 0)

  .def(
    "dance_collection",
    &VPA::Motion::Request,
    R"pbdoc( 舞蹈集合 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::dance_collection,
    py::arg("duration") = 0)

  .def(
    "hold_left_hand",
    &VPA::Motion::Request,
    R"pbdoc( 握左手 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::hold_left_hand,
    py::arg("duration") = 0)

  .def(
    "hold_right_hand",
    &VPA::Motion::Request,
    R"pbdoc( 握右手 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::hold_right_hand,
    py::arg("duration") = 0)

  .def(
    "sit_down",
    &VPA::Motion::Request,
    R"pbdoc( 坐下 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::sit_down,
    py::arg("duration") = 0)

  .def(
    "butt_circle",
    &VPA::Motion::Request,
    R"pbdoc( 屁股画圆 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::butt_circle,
    py::arg("duration") = 0)

  .def(
    "head_circle",
    &VPA::Motion::Request,
    R"pbdoc( 头画圆 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::head_circle,
    py::arg("duration") = 0)

  .def(
    "stretch_the_body",
    &VPA::Motion::Request,
    R"pbdoc( 伸展身体 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::stretch_the_body,
    py::arg("duration") = 0)

  .def(
    "shake_ass_left",
    &VPA::Motion::Request,
    R"pbdoc( 向左摇晃屁股 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::shake_ass_left,
    py::arg("duration") = 0)

  .def(
    "shake_ass_right",
    &VPA::Motion::Request,
    R"pbdoc( 向右摇晃屁股 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::shake_ass_right,
    py::arg("duration") = 0)

  .def(
    "shake_ass_from_side_to_side",
    &VPA::Motion::Request,
    R"pbdoc( 左右摇晃屁股 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::shake_ass_from_side_to_side,
    py::arg("duration") = 0)

  .def(
    "ballet",
    &VPA::Motion::Request,
    R"pbdoc( 芭蕾舞 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::ballet,
    py::arg("duration") = 0)

  .def(
    "space_walk",
    &VPA::Motion::Request,
    R"pbdoc( 太空步 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::space_walk,
    py::arg("duration") = 0)

  .def(
    "front_leg_jumping",
    &VPA::Motion::Request,
    R"pbdoc( 前腿开合跳 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::front_leg_jumping,
    py::arg("duration") = 0)

  .def(
    "hind_leg_jumping",
    &VPA::Motion::Request,
    R"pbdoc( 后腿开合跳 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::hind_leg_jumping,
    py::arg("duration") = 0)

  .def(
    "lift_the_left_leg_and_nod",
    &VPA::Motion::Request,
    R"pbdoc( 左腿抬起并点头 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::lift_the_left_leg_and_nod,
    py::arg("duration") = 0)

  .def(
    "lift_the_right_leg_and_nod",
    &VPA::Motion::Request,
    R"pbdoc( 右腿抬起并点头 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::lift_the_right_leg_and_nod,
    py::arg("duration") = 0)

  .def(
    "left_front_right_back_legs_apart",
    &VPA::Motion::Request,
    R"pbdoc( 左前右后岔开腿 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::left_front_right_back_legs_apart,
    py::arg("duration") = 0)

  .def(
    "right_front_left_back_legs_apart",
    &VPA::Motion::Request,
    R"pbdoc( 右前左后岔开腿 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::right_front_left_back_legs_apart,
    py::arg("duration") = 0)

  .def(
    "walk_nodding",
    &VPA::Motion::Request,
    R"pbdoc( 走路点头 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::walk_nodding,
    py::arg("duration") = 0)

  .def(
    "walking_with_divergence_and_adduction_alternately",
    &VPA::Motion::Request,
    R"pbdoc( 岔开内收交替走路 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::walking_with_divergence_and_adduction_alternately,
    py::arg("duration") = 0)

  .def(
    "nodding_in_place",
    &VPA::Motion::Request,
    R"pbdoc( 原地踏步点头 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::nodding_in_place,
    py::arg("duration") = 0)

  .def(
    "front_legs_jump_back_and_forth",
    &VPA::Motion::Request,
    R"pbdoc( 前腿前后跳 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::front_legs_jump_back_and_forth,
    py::arg("duration") = 0)

  .def(
    "hind_legs_jump_back_and_forth",
    &VPA::Motion::Request,
    R"pbdoc( 后腿前后跳 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::hind_legs_jump_back_and_forth,
    py::arg("duration") = 0)

  .def(
    "alternately_front_leg_lift",
    &VPA::Motion::Request,
    R"pbdoc( 前腿交替抬起 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::alternately_front_leg_lift,
    py::arg("duration") = 0)

  .def(
    "alternately_hind_leg_lift",
    &VPA::Motion::Request,
    R"pbdoc( 后腿交替抬起 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::alternately_hind_leg_lift,
    py::arg("duration") = 0)

  .def(
    "jump_collection",
    &VPA::Motion::Request,
    R"pbdoc( 跳跃合集 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump_collection,
    py::arg("duration") = 0)

  .def(
    "stretching_left_and_right",
    &VPA::Motion::Request,
    R"pbdoc( 左右伸腿踏步 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::stretching_left_and_right,
    py::arg("duration") = 0)

  .def(
    "jump_forward_and_backward",
    &VPA::Motion::Request,
    R"pbdoc( 前后摆腿跳跃 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::jump_forward_and_backward,
    py::arg("duration") = 0)

  .def(
    "step_left_and_right",
    &VPA::Motion::Request,
    R"pbdoc( 左右摆腿踏步 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::step_left_and_right,
    py::arg("duration") = 0)

  .def(
    "right_leg_back_and_forth_stepping",
    &VPA::Motion::Request,
    R"pbdoc( 右腿前后踏步 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::right_leg_back_and_forth_stepping,
    py::arg("duration") = 0)

  .def(
    "left_leg_back_and_forth_stepping",
    &VPA::Motion::Request,
    R"pbdoc( 左腿前后踏步 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::left_leg_back_and_forth_stepping,
    py::arg("duration") = 0)

  .def(
    "squat_down_on_all_fours",
    &VPA::Motion::Request,
    R"pbdoc( 四足蹲起 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::squat_down_on_all_fours,
    py::arg("duration") = 0)

  .def(
    "push_ups",
    &VPA::Motion::Request,
    R"pbdoc( 俯卧撑 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::push_ups,
    py::arg("duration") = 0)

  .def(
    "bow_to_each_other",
    &VPA::Motion::Request,
    R"pbdoc( 作揖比心 )pbdoc",
    py::arg("motion_id") = VPA::MotionId::bow_to_each_other,
    py::arg("duration") = 0)

  .def(
    "absolute_attitude",
    &VPA::Motion::AbsoluteForceControlAttitude,
    R"pbdoc( 绝对力控姿态（绝对姿态） )pbdoc",
    py::arg("centroid_z") = VPA::motion_params_.centroid_z.default_value,
    py::arg("roll") = VPA::motion_params_.roll.default_value,
    py::arg("pitch") = VPA::motion_params_.pitch.default_value,
    py::arg("yaw") = VPA::motion_params_.yaw.default_value,
    py::arg("duration") = 1
  )

  .def(
    "relatively_force_control_attitude",
    &VPA::Motion::RelativelyForceControlAttitude,
    R"pbdoc( 相对力控姿态 )pbdoc",
    py::arg("centroid_x") = VPA::motion_params_.centroid_x.default_value,
    py::arg("centroid_y") = VPA::motion_params_.centroid_y.default_value,
    py::arg("centroid_z") = VPA::motion_params_.centroid_z.default_value,
    py::arg("roll") = VPA::motion_params_.roll.default_value,
    py::arg("pitch") = VPA::motion_params_.pitch.default_value,
    py::arg("yaw") = VPA::motion_params_.yaw.default_value,
    py::arg("duration") = 1
  )

  .def(
    "transition_standing",
    &VPA::Motion::AbsolutePositionControlAttitude,
    R"pbdoc( 绝对位控姿态（过渡站立） )pbdoc",
    py::arg("centroid_z") = VPA::motion_params_.centroid_z.default_value,
    py::arg("duration") = 1
  )

  .def(
    "relatively_position_control_attitude",
    &VPA::Motion::RelativelyPositionControlAttitude,
    R"pbdoc( 相对位控姿态 )pbdoc",
    py::arg("centroid_x") = VPA::motion_params_.centroid_x.default_value,
    py::arg("centroid_y") = VPA::motion_params_.centroid_y.default_value,
    py::arg("centroid_z") = VPA::motion_params_.centroid_z.default_value,
    py::arg("roll") = VPA::motion_params_.roll.default_value,
    py::arg("pitch") = VPA::motion_params_.pitch.default_value,
    py::arg("yaw") = VPA::motion_params_.yaw.default_value,
    py::arg("fulcrum_x") = VPA::motion_params_.fulcrum_x.default_value,
    py::arg("fulcrum_y") = VPA::motion_params_.fulcrum_y.default_value,
    py::arg("fulcrum_z") = VPA::motion_params_.fulcrum_z.default_value,
    py::arg("duration") = 1
  )

  .def(
    "jump_back_and_forth",
    &VPA::Motion::JumpBackAndForth,
    R"pbdoc( 前后跳 )pbdoc",
    py::arg("x_velocity") = VPA::motion_params_.x_velocity.default_value,
    py::arg("y_velocity") = VPA::motion_params_.y_velocity.default_value,
    py::arg("z_velocity") = VPA::motion_params_.z_velocity.default_value,
    py::arg("front_leg_lift") = VPA::motion_params_.front_leg_lift.default_value,
    py::arg("back_leg_lift") = VPA::motion_params_.back_leg_lift.default_value,
    py::arg("distance") = VPA::motion_params_.distance.default_value,
    py::arg("duration") = VPA::motion_params_.duration.default_value,
    py::arg("compensation_frame_size") = 0
  )

  .def(
    "small_jump_walking",
    &VPA::Motion::SmallJumpWalking,
    R"pbdoc( 小跳行走 )pbdoc",
    py::arg("x_velocity") = VPA::motion_params_.x_velocity.default_value,
    py::arg("y_velocity") = VPA::motion_params_.y_velocity.default_value,
    py::arg("z_velocity") = VPA::motion_params_.z_velocity.default_value,
    py::arg("front_leg_lift") = VPA::motion_params_.front_leg_lift.default_value,
    py::arg("back_leg_lift") = VPA::motion_params_.back_leg_lift.default_value,
    py::arg("distance") = VPA::motion_params_.distance.default_value,
    py::arg("duration") = VPA::motion_params_.duration.default_value,
    py::arg("compensation_frame_size") = 0
  )

  .def(
    "trot_walking",
    &VPA::Motion::TrotWalking,
    R"pbdoc( 小跑行走 )pbdoc",
    py::arg("x_velocity") = VPA::motion_params_.x_velocity.default_value,
    py::arg("y_velocity") = VPA::motion_params_.y_velocity.default_value,
    py::arg("z_velocity") = VPA::motion_params_.z_velocity.default_value,
    py::arg("front_leg_lift") = VPA::motion_params_.front_leg_lift.default_value,
    py::arg("back_leg_lift") = VPA::motion_params_.back_leg_lift.default_value,
    py::arg("distance") = VPA::motion_params_.distance.default_value,
    py::arg("duration") = VPA::motion_params_.duration.default_value,
    py::arg("compensation_frame_size") = 0
  )

  .def(
    "automatic_frequency_conversion_walking",
    &VPA::Motion::AutomaticFrequencyConversionWalking,
    R"pbdoc( 自动变频行走 )pbdoc",
    py::arg("x_velocity") = VPA::motion_params_.x_velocity.default_value,
    py::arg("y_velocity") = VPA::motion_params_.y_velocity.default_value,
    py::arg("z_velocity") = VPA::motion_params_.z_velocity.default_value,
    py::arg("front_leg_lift") = VPA::motion_params_.front_leg_lift.default_value,
    py::arg("back_leg_lift") = VPA::motion_params_.back_leg_lift.default_value,
    py::arg("distance") = VPA::motion_params_.distance.default_value,
    py::arg("duration") = VPA::motion_params_.duration.default_value,
    py::arg("compensation_frame_size") = 0
  )

  .def(
    "run_fast_walking",
    &VPA::Motion::RunFastWalking,
    R"pbdoc( 快跑行走 )pbdoc",
    py::arg("x_velocity") = VPA::motion_params_.x_velocity.default_value,
    py::arg("y_velocity") = VPA::motion_params_.y_velocity.default_value,
    py::arg("z_velocity") = VPA::motion_params_.z_velocity.default_value,
    py::arg("front_leg_lift") = VPA::motion_params_.front_leg_lift.default_value,
    py::arg("back_leg_lift") = VPA::motion_params_.back_leg_lift.default_value,
    py::arg("distance") = VPA::motion_params_.distance.default_value,
    py::arg("duration") = VPA::motion_params_.duration.default_value,
    py::arg("compensation_frame_size") = 0
  )

  .def(
    "turn",
    &VPA::Motion::Turn,
    R"pbdoc( 转向 )pbdoc",
    py::arg("angle") = 1,
    py::arg("duration") = 1
  )

  .def(
    "go_straight",
    &VPA::Motion::GoStraight,
    R"pbdoc( 直行 )pbdoc",
    py::arg("velocity") = VPA::motion_params_.x_velocity.default_value,
    py::arg("journey") = 0,
    py::arg("time") = 1
  )

  .def(
    "lateral_movement",
    &VPA::Motion::LateralMovement,
    R"pbdoc( 横移 )pbdoc",
    py::arg("velocity") = VPA::motion_params_.y_velocity.default_value,
    py::arg("journey") = 0,
    py::arg("time") = 1
  )

  .def(
    "run_sequence",
    &VPA::Motion::RunSequence,
    R"pbdoc( 运行序列 )pbdoc",
    py::arg("sequence")
  )

  .def(
    "__repr__", [](const VPA::Motion & _motion) {
      return std::string(
        FORMAT(
          "┌────────────────────────────────────────---"
          "\n│- type: Motion"
          "\n├────────────────────────────────────────---"
          "\n│- data:"
          "\n│  - state:"
          "\n│    - code = %d"
          "\n│    - describe = '%s'"
          "\n└────────────────────────────────────────---",
          _motion.state_.code, _motion.state_.describe.c_str()));
    })
  ;
}
}  // namespace cyberdog_visual_programming_abilityset_py
