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

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "cyberdog_vp_terminal/debug_abilityset.hpp"

namespace cyberdog_visual_programming_terminal
{
std::mutex cursor_mutex_;     /**< 光标互斥锁 */
DebugAbilityset::DebugAbilityset()
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  DEBUG("%s Creating object(node)", this->logger_.c_str());
}

DebugAbilityset::~DebugAbilityset()
{
  DEBUG("%s Destroy object(node)", this->logger_.c_str());
}

bool DebugAbilityset::Init(const std::shared_ptr<VPA::Cyberdog> & _cyberdog_ptr)
{
  try {
    this->cyberdog_ptr_ = _cyberdog_ptr;
    this->walking_type = FUNCTION::automatic_frequency_conversion_walking;
    this->namespace_ = "namespace";
  } catch (const std::exception & e) {
    ERROR("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void DebugAbilityset::Stop()
{
  try {
    if (this->cyberdog_ptr_ != nullptr) {
      this->cyberdog_ptr_->task_.Stop();
    }
  } catch (const std::exception & e) {
    ERROR("%s Stop cyberdog failed: %s", this->logger_.c_str(), e.what());
  }
}

bool DebugAbilityset::SetCyberdog(bool _log)
{
  try {
    if (this->cyberdog_ptr_ == nullptr) {
      return false;
    }
    this->cyberdog_ptr_->SetLog(_log);
  } catch (const std::exception & e) {
    ERROR("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

std::string DebugAbilityset::KeyboardTeleop(const int _key)
{
  double _x = this->params_window.body.at(PARAMETERS::velocity).second.at(PARAMETERS::x).value;
  double _y = this->params_window.body.at(PARAMETERS::velocity).second.at(PARAMETERS::y).value;
  double _z = this->params_window.body.at(PARAMETERS::velocity).second.at(PARAMETERS::z).value;
  double x = 0;
  double y = 0;
  double z = 0;
  switch (_key) {
    case 'w':
    case 'W':
      x = _x;
      break;
    case 'x':
    case 'X':
      x = -_x;
      break;
    case 'a':
      z = _z;
      break;
    case 'A':
      y = _y;
      break;
    case 'd':
      z = -_z;
      break;
    case 'D':
      y = -_y;
      break;
    case 'q':
      x = _x;
      z = _z;
      break;
    case 'Q':
      x = _x;
      y = _y;
      break;
    case 'e':
      x = _x;
      z = -_z;
      break;
    case 'E':
      x = _x;
      y = -_y;
      break;
    case 'z':
      x = -_x;
      z = -_z;
      break;
    case 'Z':
      x = -_x;
      y = _y;
      break;
    case 'c':
      x = -_x;
      z = _z;
      break;
    case 'C':
      x = -_x;
      y = -_y;
      break;
    case 's':
    case 'S':
      // 停止
      break;
  }
  VPA::MotionServoCmdResponse ret;
  switch (this->walking_type) {
    case FUNCTION::jump_back_and_forth:
      x = y = z = 0.0;
      ret = this->cyberdog_ptr_->motion_.JumpBackAndForth(
        x, y, z,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value,
        0.0, 0.0,
        static_cast<uint>(
          this->params_window.body.at(PARAMETERS::compensation).second.at(PARAMETERS::motion_frame).
          value));
      break;
    case FUNCTION::small_jump_walking:
      ret = this->cyberdog_ptr_->motion_.SmallJumpWalking(
        x, y, z,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value,
        0.0, 0.0,
        static_cast<uint>(
          this->params_window.body.at(PARAMETERS::compensation).second.at(PARAMETERS::motion_frame).
          value));
      break;
    case FUNCTION::trot_walking:
      ret = this->cyberdog_ptr_->motion_.TrotWalking(
        x, y, z,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value,
        0.0, 0.0,
        static_cast<uint>(
          this->params_window.body.at(PARAMETERS::compensation).second.at(PARAMETERS::motion_frame).
          value));
      break;
    case FUNCTION::automatic_frequency_conversion_walking:
      ret = this->cyberdog_ptr_->motion_.AutomaticFrequencyConversionWalking(
        x, y, z,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value,
        0.0, 0.0,
        static_cast<uint>(
          this->params_window.body.at(PARAMETERS::compensation).second.at(PARAMETERS::motion_frame).
          value));
      break;
    case FUNCTION::run_fast_walking:
      ret = this->cyberdog_ptr_->motion_.RunFastWalking(
        x, y, z,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value,
        0.0, 0.0,
        static_cast<uint>(
          this->params_window.body.at(PARAMETERS::compensation).second.at(PARAMETERS::motion_frame).
          value));
      break;
    default:
      return "";
  }
  return std::string(
    FORMAT(
      " - motion mode: %s (%s)"
      "\n - operate : key = '%c'"
      "\n - request :"
      "\n   - vel_des = [%lf, %lf, %lf]"
      "\n   - step_height = [%lf, %lf, 0.0]"
      "\n - response:"
      "\n   - state:"
      "\n     - code = %d"
      "\n     - describe = '%s'"
      "\n   - response:"
      "\n     - motion_id: = %d"
      "\n     - result = %s"
      "\n     - code = %d"
      "\n     - cmd_id = %d"
      "\n     - order_process_bar = %d"
      "\n     - status = %d",
      function_window.body.at(static_cast<uint>(this->walking_type)).second.first.c_str(),
      function_window.body.at(static_cast<uint>(this->walking_type)).second.second.c_str(),
      _key, x, y, z,
      this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
      this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value,
      ret.state.code, ret.state.describe.c_str(),
      ret.response.motion_id, std::string(ret.response.result ? "True" : "False").c_str(),
      ret.response.code, ret.response.cmd_id,
      ret.response.order_process_bar, ret.response.status));
}

std::string DebugAbilityset::TriggerFunction(const std::string _function)
{
  auto get_function_id = [&]() -> uint {
      uint id = 0;
      for (auto meta : function_window.body) {
        if (meta.second.first == _function) {
          break;
        }
        ++id;
      }
      return (id < function_window.body.size()) ? id : 0;
    };
  uint function_id = get_function_id();
  VPA::MotionResultServiceResponse motion_ret;
  VPA::State state_ret;
  VPA::AudioPlaySeviceResponse audio_ret;
  VPA::LedSeviceResponse led_ret;
  switch (function_id) {
    case FUNCTION::emergency_stop:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::emergency_stop);
      break;
    case FUNCTION::get_down:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::get_down);
      break;
    case FUNCTION::resume_standing:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::resume_standing);
      break;
    case FUNCTION::back_flip:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::back_flip);
      break;
    case FUNCTION::front_flip:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::front_flip);
      break;
    case FUNCTION::bow:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::bow);
      break;
    case FUNCTION::roll_left:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::roll_left);
      break;
    case FUNCTION::walk_the_dog:
      motion_ret = this->cyberdog_ptr_->motion_.WalkTheDog(
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::front_foot).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::rears_foot).value);
      break;
    case FUNCTION::jump_stair:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump_stair);
      break;
    case FUNCTION::right_somersault:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::right_somersault);
      break;
    case FUNCTION::left_somersault:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::left_somersault);
      break;
    case FUNCTION::run_and_jump_front_flip:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::run_and_jump_front_flip);
      break;
    case FUNCTION::jump3d_left90deg:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_left90deg);
      break;
    case FUNCTION::jump3d_right90deg:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_right90deg);
      break;
    case FUNCTION::jump3d_forward60cm:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_forward60cm);
      break;
    case FUNCTION::jump3d_forward30cm:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_forward30cm);
      break;
    case FUNCTION::jump3d_left20cm:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_left20cm);
      break;
    case FUNCTION::jump3d_right20cm:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_right20cm);
      break;
    case FUNCTION::jump3d_up30cm:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_up30cm);
      break;
    case FUNCTION::jump3d_down_stair:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump3d_down_stair);
      break;
    case FUNCTION::roll_right:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::roll_right);
      break;
    case FUNCTION::dance_collection:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::dance_collection);
      break;
    case FUNCTION::hold_left_hand:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::hold_left_hand);
      break;
    case FUNCTION::hold_right_hand:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::hold_right_hand);
      break;
    case FUNCTION::sit_down:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::sit_down);
      break;
    case FUNCTION::butt_circle:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::butt_circle);
      break;
    case FUNCTION::head_circle:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::head_circle);
      break;
    case FUNCTION::stretch_the_body:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::stretch_the_body);
      break;
    case FUNCTION::shake_ass_left:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::shake_ass_left);
      break;
    case FUNCTION::shake_ass_right:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::shake_ass_right);
      break;
    case FUNCTION::shake_ass_from_side_to_side:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::shake_ass_from_side_to_side);
      break;
    case FUNCTION::ballet:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::ballet);
      break;
    case FUNCTION::space_walk:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::space_walk);
      break;
    case FUNCTION::front_leg_jumping:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::front_leg_jumping);
      break;
    case FUNCTION::hind_leg_jumping:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::hind_leg_jumping);
      break;
    case FUNCTION::lift_the_left_leg_and_nod:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::lift_the_left_leg_and_nod);
      break;
    case FUNCTION::lift_the_right_leg_and_nod:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::lift_the_right_leg_and_nod);
      break;
    case FUNCTION::left_front_right_back_legs_apart:
      motion_ret = this->cyberdog_ptr_->motion_.Request(
        VPA::MotionId::left_front_right_back_legs_apart);
      break;
    case FUNCTION::right_front_left_back_legs_apart:
      motion_ret = this->cyberdog_ptr_->motion_.Request(
        VPA::MotionId::right_front_left_back_legs_apart);
      break;
    case FUNCTION::walk_nodding:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::walk_nodding);
      break;
    case FUNCTION::walking_with_divergence_and_adduction_alternately:
      motion_ret = this->cyberdog_ptr_->motion_.Request(
        VPA::MotionId::walking_with_divergence_and_adduction_alternately);
      break;
    case FUNCTION::nodding_in_place:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::nodding_in_place);
      break;
    case FUNCTION::front_legs_jump_back_and_forth:
      motion_ret = this->cyberdog_ptr_->motion_.Request(
        VPA::MotionId::front_legs_jump_back_and_forth);
      break;
    case FUNCTION::hind_legs_jump_back_and_forth:
      motion_ret =
        this->cyberdog_ptr_->motion_.Request(VPA::MotionId::hind_legs_jump_back_and_forth);
      break;
    case FUNCTION::alternately_front_leg_lift:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::alternately_front_leg_lift);
      break;
    case FUNCTION::alternately_hind_leg_lift:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::alternately_hind_leg_lift);
      break;
    case FUNCTION::jump_collection:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump_collection);
      break;
    case FUNCTION::stretching_left_and_right:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::stretching_left_and_right);
      break;
    case FUNCTION::jump_forward_and_backward:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::jump_forward_and_backward);
      break;
    case FUNCTION::step_left_and_right:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::step_left_and_right);
      break;
    case FUNCTION::right_leg_back_and_forth_stepping:
      motion_ret = this->cyberdog_ptr_->motion_.Request(
        VPA::MotionId::right_leg_back_and_forth_stepping);
      break;
    case FUNCTION::left_leg_back_and_forth_stepping:
      motion_ret = this->cyberdog_ptr_->motion_.Request(
        VPA::MotionId::left_leg_back_and_forth_stepping);
      break;
    case FUNCTION::bow_to_each_other:
      motion_ret = this->cyberdog_ptr_->motion_.Request(VPA::MotionId::bow_to_each_other);
      break;
    case FUNCTION::absolute_force_control_attitude:
      motion_ret = this->cyberdog_ptr_->motion_.AbsoluteForceControlAttitude(
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::x).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::y).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::duration).value);
      break;
    case FUNCTION::relatively_force_control_attitude:
      motion_ret = this->cyberdog_ptr_->motion_.RelativelyForceControlAttitude(
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::x).value,
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::y).value,
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::x).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::y).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::duration).value);
      break;
    case FUNCTION::absolute_position_control_attitude:
      motion_ret = this->cyberdog_ptr_->motion_.AbsolutePositionControlAttitude(
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::duration).value);
      break;
    case FUNCTION::relatively_position_control_attitude:
      motion_ret = this->cyberdog_ptr_->motion_.RelativelyPositionControlAttitude(
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::x).value,
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::y).value,
        this->params_window.body.at(PARAMETERS::centroid).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::x).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::y).value,
        this->params_window.body.at(PARAMETERS::angle).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::fulcrum).second.at(PARAMETERS::x).value,
        this->params_window.body.at(PARAMETERS::fulcrum).second.at(PARAMETERS::y).value,
        this->params_window.body.at(PARAMETERS::fulcrum).second.at(PARAMETERS::z).value,
        this->params_window.body.at(PARAMETERS::scale).second.at(PARAMETERS::duration).value);
      break;
    case FUNCTION::jump_back_and_forth:
      this->walking_type = FUNCTION::jump_back_and_forth;
      break;
    case FUNCTION::small_jump_walking:
      this->walking_type = FUNCTION::small_jump_walking;
      break;
    case FUNCTION::trot_walking:
      this->walking_type = FUNCTION::trot_walking;
      break;
    case FUNCTION::automatic_frequency_conversion_walking:
      this->walking_type = FUNCTION::automatic_frequency_conversion_walking;
      break;
    case FUNCTION::run_fast_walking:
      this->walking_type = FUNCTION::run_fast_walking;
      break;
    case FUNCTION::Audio_set_play:
      audio_ret = this->cyberdog_ptr_->audio_.OnlinePlay();
      break;
    case FUNCTION::Led_set_play:
      led_ret = this->cyberdog_ptr_->led_.Play();
      break;
    default:
      break;
  }
  std::string ret_message = "";
  std::ostringstream ret_str;
  ret_str << "motion mode : " << _function <<
    " (" << function_window.body.at(static_cast<uint>(function_id)).second.second << ")";
  switch (function_id) {
    case FUNCTION::emergency_stop:
    case FUNCTION::get_down:
    case FUNCTION::resume_standing:
    case FUNCTION::back_flip:
    case FUNCTION::front_flip:
    case FUNCTION::bow:
    case FUNCTION::roll_left:
    case FUNCTION::jump_stair:
    case FUNCTION::right_somersault:
    case FUNCTION::left_somersault:
    case FUNCTION::run_and_jump_front_flip:
    case FUNCTION::jump3d_left90deg:
    case FUNCTION::jump3d_right90deg:
    case FUNCTION::jump3d_forward60cm:
    case FUNCTION::jump3d_forward30cm:
    case FUNCTION::jump3d_left20cm:
    case FUNCTION::jump3d_right20cm:
    case FUNCTION::jump3d_up30cm:
    case FUNCTION::jump3d_down_stair:
    case FUNCTION::roll_right:
    case FUNCTION::dance_collection:
    case FUNCTION::hold_left_hand:
    case FUNCTION::hold_right_hand:
    case FUNCTION::sit_down:
    case FUNCTION::butt_circle:
    case FUNCTION::head_circle:
    case FUNCTION::stretch_the_body:
    case FUNCTION::shake_ass_left:
    case FUNCTION::shake_ass_right:
    case FUNCTION::shake_ass_from_side_to_side:
    case FUNCTION::ballet:
    case FUNCTION::space_walk:
    case FUNCTION::front_leg_jumping:
    case FUNCTION::hind_leg_jumping:
    case FUNCTION::lift_the_left_leg_and_nod:
    case FUNCTION::lift_the_right_leg_and_nod:
    case FUNCTION::left_front_right_back_legs_apart:
    case FUNCTION::right_front_left_back_legs_apart:
    case FUNCTION::walk_nodding:
    case FUNCTION::walking_with_divergence_and_adduction_alternately:
    case FUNCTION::nodding_in_place:
    case FUNCTION::front_legs_jump_back_and_forth:
    case FUNCTION::hind_legs_jump_back_and_forth:
    case FUNCTION::alternately_front_leg_lift:
    case FUNCTION::alternately_hind_leg_lift:
    case FUNCTION::jump_collection:
    case FUNCTION::stretching_left_and_right:
    case FUNCTION::jump_forward_and_backward:
    case FUNCTION::step_left_and_right:
    case FUNCTION::right_leg_back_and_forth_stepping:
    case FUNCTION::left_leg_back_and_forth_stepping:
    case FUNCTION::bow_to_each_other:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request : (default)"
          "\n - response:"
          "\n   - state:"
          "\n     - code = %d"
          "\n     - describe = '%s'"
          "\n   - response:"
          "\n     - motion_id: = %d"
          "\n     - result = %s"
          "\n     - code = %d",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str(),
          motion_ret.state.code, motion_ret.state.describe.c_str(),
          motion_ret.response.motion_id,
          std::string(motion_ret.response.result ? "True" : "False").c_str(),
          motion_ret.response.code));
      break;
    case FUNCTION::absolute_force_control_attitude:
    case FUNCTION::relatively_force_control_attitude:
    case FUNCTION::absolute_position_control_attitude:
    case FUNCTION::relatively_position_control_attitude:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request :"
          "\n   - rpy_des = [%lf, %lf, %lf]"
          "\n   - pos_des = [0.0, 0.0, %lf]"
          "\n - response:"
          "\n   - state:"
          "\n     - code = %d"
          "\n     - describe = '%s'"
          "\n   - response:"
          "\n     - motion_id: = %d"
          "\n     - result = %s"
          "\n     - code = %d",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str(),
          this->params_window.body.at(1).second.at(0).value,
          this->params_window.body.at(1).second.at(1).value,
          this->params_window.body.at(1).second.at(2).value,
          this->params_window.body.at(2).second.at(2).value,
          motion_ret.state.code, motion_ret.state.describe.c_str(),
          motion_ret.response.motion_id,
          std::string(motion_ret.response.result ? "True" : "False").c_str(),
          motion_ret.response.code));
      break;
    case FUNCTION::walk_the_dog:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request :"
          "\n   - step_height = [%lf, %lf, 0.0]"
          "\n - response:"
          "\n   - state:"
          "\n     - code = %d"
          "\n     - describe = '%s'"
          "\n   - response:"
          "\n     - motion_id: = %d"
          "\n     - result = %s"
          "\n     - code = %d",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str(),
          this->params_window.body.at(2).second.at(0).value,
          this->params_window.body.at(2).second.at(1).value,
          motion_ret.state.code, motion_ret.state.describe.c_str(),
          motion_ret.response.motion_id,
          std::string(motion_ret.response.result ? "True" : "False").c_str(),
          motion_ret.response.code));
      break;
    case FUNCTION::jump_back_and_forth:
    case FUNCTION::small_jump_walking:
    case FUNCTION::trot_walking:
    case FUNCTION::automatic_frequency_conversion_walking:
    case FUNCTION::run_fast_walking:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request : change <walking_type>"
          "\n - response: (null)",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str()));
      break;
    case FUNCTION::Led_set_play:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request : (default)"
          "\n - response:"
          "\n   - state:"
          "\n     - code = %d"
          "\n     - describe = '%s'"
          "\n   - response:"
          "\n     - code = %d",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str(),
          led_ret.state.code, led_ret.state.describe.c_str(),
          led_ret.response.code));
      break;
    case FUNCTION::Audio_set_play:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request : (default)"
          "\n - response:"
          "\n   - state:"
          "\n     - code = %d"
          "\n     - describe = '%s'"
          "\n   - response:"
          "\n     - status = %d",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str(),
          audio_ret.state.code, audio_ret.state.describe.c_str(),
          audio_ret.response.status));
      break;
    default:
      ret_message = std::string(
        FORMAT(
          " - motion mode: %s (%s)"
          "\n - operate : key = 'enter'"
          "\n - request : (null)"
          "\n - response: (null)",
          _function.c_str(),
          function_window.body.at(static_cast<uint>(function_id)).second.second.c_str()));
      break;
  }
  return ret_message;
}

bool DebugAbilityset::UpdateStatus()
{
  bool need_update = false;
  uint count = this->state_window.body.size();
  std::string data;
  for (uint i = 0; i < count; i++) {
    data = "";
    switch (i) {
      case static_cast<uint>(STATES::version):
        data = "v-2.0";
        break;
      case static_cast<uint>(STATES::wifi_ssid):
        data = this->cyberdog_ptr_->network_.data_.ssid;
        break;
      case static_cast<uint>(STATES::wifi_signal):
        data = std::to_string(this->cyberdog_ptr_->network_.data_.strength);
        break;
      case static_cast<uint>(STATES::robot_ip):
        data = this->cyberdog_ptr_->network_.data_.robot_ip;
        break;
      case static_cast<uint>(STATES::provider_ip):
        data = this->cyberdog_ptr_->network_.data_.provider_ip;
        break;
      case static_cast<uint>(STATES::electricity):
        data = std::to_string(this->cyberdog_ptr_->bms_.data_.batt_soc);
        break;
      case static_cast<uint>(STATES::temperature):
        data = std::to_string(this->cyberdog_ptr_->bms_.data_.batt_temp);
        break;
      case static_cast<uint>(STATES::voltage):
        data = std::to_string(this->cyberdog_ptr_->bms_.data_.batt_volt);
        break;
      case static_cast<uint>(STATES::current):
        data = std::to_string(this->cyberdog_ptr_->bms_.data_.batt_curr);
        break;
      case static_cast<uint>(STATES::audio):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->audio_.state_.code];
        break;
      case static_cast<uint>(STATES::led):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->led_.state_.code];
        break;
      case static_cast<uint>(STATES::touch):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->touch_.state_.code];
        break;
      case static_cast<uint>(STATES::gps):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->gps_.state_.code];
        break;
      case static_cast<uint>(STATES::lidar):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->lidar_.state_.code];
        break;
      case static_cast<uint>(STATES::tof):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->tof_.state_.code];
        break;
      case static_cast<uint>(STATES::ultrasonic):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->ultrasonic_.state_.code];
        break;
      case static_cast<uint>(STATES::imu):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->imu_.state_.code];
        break;
      case static_cast<uint>(STATES::odometer):
        data = VPA::StateDescribe_[this->cyberdog_ptr_->odometer_.state_.code];
        break;
      default:
        data = "";
        break;
    }
    if ((!data.empty()) &&
      (this->state_window.body.at(i).value != data))
    {
      this->state_window.body.at(i).value = data;
      this->state_window.body.at(i).update = true;
      this->state_window.body.at(i).index = 0;
      need_update = true;
    }
  }
  return need_update;
}

std::vector<std::string> DebugAbilityset::GetVector(const std::string & _message, char _delim)
{
  std::vector<std::string> _vector;
  std::stringstream message_str;
  message_str.str(_message);
  std::string elems;
  while (std::getline(message_str, elems, _delim)) {
    _vector.push_back(elems);
  }
  return _vector;
}

void DebugAbilityset::InitStateWins(WINDOW ** wins)
{
  auto get_height = [&]() -> uint {
      return static_cast<uint>(3 + state_window.body.size() * 2);
    };
  this->state_win_width_ = 3 + state_window.length.front() + state_window.length.back();
  wins[0] = newwin(get_height(), this->state_win_width_, 0, 0);
  box(wins[0], 0, 0);
  uint y_ = 0;
  auto add_str = [&](uint _y, uint _x, uint _width, std::string _str) {
      if (_str.length() < _width) {
        _x += ((_width - _str.length()) / 2);
      }
      mvwhline(wins[0], y_, state_window.length.front() + 2, ' ', state_window.length.back());
      mvwprintw(wins[0], _y, _x, "%s", _str.c_str());
    };
  auto add_line = [&](uint _y, const chtype front, const chtype middle, const chtype back) {
      mvwaddch(wins[0], _y, 0, front);
      mvwhline(wins[0], _y, 1, ACS_HLINE, this->state_win_width_ - 1);
      mvwaddch(wins[0], _y, this->state_win_width_ - 1, back);
      mvwaddch(wins[0], _y, state_window.length.front() + 1, middle);
    };
  add_str(++y_, 0, this->state_win_width_, state_window.head);
  add_line(++y_, ACS_LTEE, ACS_TTEE, ACS_RTEE);
  for (auto meta : state_window.body) {
    add_str(++y_, 1, state_window.length.front(), meta.key);
    mvwaddch(wins[0], y_, state_window.length.front() + 1, ACS_VLINE);
    add_str(y_, state_window.length.front() + 2, state_window.length.back(), meta.value);
    add_line(++y_, ACS_LTEE, ACS_PLUS, ACS_RTEE);
  }
  add_line(y_, ACS_LLCORNER, ACS_BTEE, ACS_LRCORNER);
}

bool DebugAbilityset::UpdataStateWins(WINDOW ** wins)
{
  uint width_ = state_window.length.back();
  int y_ = 1, index_ = -1;
  bool ret = false;
  auto updata_state_meta = [&](uint _y, uint _x) {
      this->state_window.body.at(index_).update = false;
      mvwhline(wins[0], _y, _x, ' ', width_);
      std::string msg = this->state_window.body.at(index_).value;
      if (msg.length() < width_) {
        _x += ((width_ - msg.length()) / 2);
      } else {
        if ((msg.length() - this->state_window.body.at(index_).index) < width_) {
          this->state_window.body.at(index_).index = 0;
        }
        msg = msg.substr(this->state_window.body.at(index_).index++, width_);
      }
      mvwprintw(wins[0], _y, _x, "%s", msg.c_str());
    };
  for (auto meta : state_window.body) {
    y_ += 2;
    index_++;
    if (!meta.update && !(meta.value.length() > state_window.length.back())) {
      continue;
    }
    ret = true;
    wmove(wins[0], y_, state_window.length.front() + 1);
    if ((meta.value == VPA::StateDescribe_[VPA::StateCode::success]) ||
      (meta.key == "version") ||                                                // 版本信息
      ((meta.key == "wifi-ssid") && !meta.value.empty()) ||                     // wifi 名称
      ((meta.key == "wifi-signal") && (std::stoi(meta.value) > 50)) ||          // wifi 信号强度
      ((meta.key == "robot-ip") && !meta.value.empty()) ||                      // robot ip
      ((meta.key == "provider-ip") && !meta.value.empty()) ||                   // provider ip
      ((meta.key == "electricity(%)") && (std::stoi(meta.value) > 50)) ||       // 电池电量
      ((meta.key == "temperature(*C)") && (std::stoi(meta.value) > 5)) ||       // 电池温度
      ((meta.key == "voltage(mV)") && (std::stoi(meta.value) > 20 * 1000)) ||   // 电池电压
      ((meta.key == "current(mA)") && (std::abs(std::stoi(meta.value)) < 2 * 1000))   // 电池电流
    )
    {   // 合法范围
      updata_state_meta(y_, state_window.length.front() + 2);
    } else {
      int message_format = (     // 非法范围 : 异常范围
        ((meta.key == "wifi-ssid") && meta.value.empty()) ||
        ((meta.key == "wifi-signal") && (std::stoi(meta.value) < 10)) ||
        ((meta.key == "robot-ip") && meta.value.empty()) ||
        ((meta.key == "provider-ip") && meta.value.empty()) ||
        ((meta.key == "electricity(%)") && (std::stoi(meta.value) < 10)) ||
        ((meta.key == "temperature(*C)") && (std::stoi(meta.value) < 5)) ||
        ((meta.key == "voltage(mV)") && (std::stoi(meta.value) < 10 * 1000)) ||
        ((meta.key == "current(mA)") && (std::abs(std::stoi(meta.value)) > 4 * 1000))
        ) ?
        (COLOR_PAIR(this->error_message_) | A_BOLD | A_UNDERLINE | A_BLINK) :
        (COLOR_PAIR(this->warning_message_) | A_BOLD);
      //  A_NORMAL//原始;A_BLINK/闪烁;A_UNDERLINE/下划线;A_REVERSE/反显;A_BOLD/加粗
      wattron(wins[0], message_format);
      mvwhline(wins[0], y_, state_window.length.front() + 2, ' ', width_);
      updata_state_meta(y_, state_window.length.front() + 2);
      wattroff(wins[0], message_format);
    }
  }
  return ret;
}

uint DebugAbilityset::GetX(uint begin, uint end)
{
  uint ret = 0;
  for (size_t i = begin; i < end; i++) {
    ret += params_window.length.at(i) + 1;
  }
  return ret;
}

void DebugAbilityset::DrawParamsWin(WINDOW * win, int n)
{
  int width = 1 + this->GetX(0, 5);
  auto cursor_reset = [&](int _y = -1, int _x = -1) {
      int y_ = (_y == -1) ? this->begin_lines_ : _y;
      int x_ = (_x == -1) ? this->begin_cols_ : _x;
      wmove(win, y_, x_);
    };
  auto add_str = [&](uint _width, std::string str, int _x = 0) {
      uint y_, x_;
      getyx(win, y_, x_);
      x_ += ((_width - str.length()) / 2) + _x;
      mvwprintw(win, y_, x_, "%s", str.c_str());
    };
  auto add_ch_absolute = [&](const chtype ch, int _x = 0) {
      uint y_, x_;
      getyx(win, y_, x_);
      x_ = _x;
      mvwaddch(win, y_, x_, ch);
    };
  auto add_line = [&](const chtype front, const chtype middle, const chtype back) {
      // uint length_index = 0;
      // for (auto meta_length : params_window.length) {
      //     if (length_index++ == 0)
      //         waddch(win, front);
      //     else
      //         waddch(win, middle);
      //     for (size_t i = 0; i < meta_length; i++)
      //         waddch(win, ACS_HLINE);
      // }
      // waddch(win, back);
      uint y_, x_;
      getyx(win, y_, x_);
      mvwaddch(win, y_, 0, front);
      mvwhline(win, y_, 1, ACS_HLINE, width - 2);
      mvwaddch(win, y_, width - 1, back);
      cursor_reset(y_);
      for (size_t i = 1; i < params_window.length.size(); i++) {
        add_ch_absolute(middle, this->GetX(0, i));
      }
    };
  auto add_divider = [&]() {
      add_line(ACS_LTEE, ACS_PLUS, ACS_RTEE);
    };
  auto add_tail = [&]() {
      add_line(ACS_LLCORNER, ACS_BTEE, ACS_LRCORNER);
    };
  auto add_title = [&](int _y = 1) {
      cursor_reset(_y);
      add_str(width, params_window.body.at(n).first.first);
      mvwaddch(win, _y + 1, 0, ACS_LTEE);
      mvwhline(win, _y + 1, 1, ACS_HLINE, width - 2);
      mvwaddch(win, _y + 1, width - 1, ACS_RTEE);
      cursor_reset(_y + 1);
      add_ch_absolute(ACS_TTEE, this->GetX(0, 1));
      add_ch_absolute(ACS_TTEE, this->GetX(0, 4));
    };
  auto add_head = [&](int _y = 3) {
      int head_index = 0;
      for (size_t i = 0; i < 4; i++) {
        cursor_reset(_y + i, 0);
        switch (i) {
          case 0:
            add_ch_absolute(ACS_VLINE, this->GetX(0, 1));
            add_str(this->GetX(1, 4), params_window.head.at(head_index++));
            add_ch_absolute(ACS_VLINE, this->GetX(0, 4));
            break;
          case 1:
            add_str(this->GetX(0, 1), params_window.head.at(head_index++));
            for (size_t j = 1; j < 4; j++) {
              if (j == 1) {
                add_ch_absolute(ACS_LTEE, this->GetX(0, 1));
              } else {
                waddch(win, ACS_TTEE);
              }

              for (size_t k = 0; k < params_window.length.at(j); k++) {
                waddch(win, ACS_HLINE);
              }
            }
            add_ch_absolute(ACS_RTEE, this->GetX(0, 4));
            add_str(params_window.length.back(), params_window.head.at(head_index++));
            break;
          case 2:
            for (size_t j = 1; j < 4; j++) {
              add_ch_absolute(ACS_VLINE, this->GetX(0, j));
              add_str(params_window.length.at(j), params_window.head.at(head_index++));
            }
            add_ch_absolute(ACS_VLINE, this->GetX(0, 4));
            break;
          case 3:
            add_divider();
            break;
          default:
            break;
        }
      }
    };
  auto add_meta = [&](const TunableParameters meta) {
      uint length_index = 0;
      for (auto meta_length : params_window.length) {
        add_ch_absolute(ACS_VLINE, this->GetX(0, length_index));
        std::ostringstream meta_str;
        switch (length_index++) {
          case 0:
            meta_str << meta.key;
            break;
          case 1:
            meta_str << meta.params.minimum_value;
            break;
          case 2:
            meta_str << meta.value;
            break;
          case 3:
            meta_str << meta.params.maximum_value;
            break;
          case 4:
            meta_str << meta.params.unit;
            break;
          default:
            break;
        }
        add_str(meta_length, meta_str.str().c_str());
      }
    };
  auto add_body = [&](int _y = 7) {
      uint index = 0;
      for (auto meta : params_window.body.at(n).second) {
        cursor_reset(_y + index++, 0);
        if (index == params_window.body.at(n).second.size()) {
          add_divider();
          cursor_reset(_y + index++, 0);
        }
        add_meta(meta);
      }
      cursor_reset(_y + index++, 0);
      add_tail();
    };

  box(win, 0, 0);
  cursor_reset();
  add_title();
  add_head();
  add_body();
  refresh();
}

void DebugAbilityset::WinUpdateFocus(
  WINDOW ** wins, int size, WINDOW * win, const int key,
  bool able)
{
  int index = 0;
  auto win_focus_enable = [&](const int color, const chtype place) {
      int y_ = params_window.body.at(index).first.second +
        ((params_window.body.at(index).first.second ==
        static_cast<int>(params_window.body.at(index).second.size() - 1)) ? 8 : 7);
      int x_ = this->GetX(0, 2) + 1;
      wattron(win, COLOR_PAIR(color));
      mvwhline(win, y_, x_, place, params_window.length.at(2));
      wmove(win, y_, x_);
      std::string value =
        std::to_string(
        params_window.body.at(index).second.at(
          params_window.body.at(index).first.
          second).value);
      int vx_ = x_ + ((params_window.length.at(2) - value.length()) / 2);
      mvwprintw(win, y_, vx_, "%s", value.c_str());
      wmove(win, y_, x_);
      wattroff(win, COLOR_PAIR(color));
    };
  do {
    if (win == wins[index]) {
      switch (key) {
        case 9:
          if (able) {
            win_focus_enable(this->focus_column_, ACS_CKBOARD);
          } else {
            win_focus_enable(this->default_column_, ' ');
          }
          break;
        case KEY_DOWN:
          win_focus_enable(this->focus_window_, ' ');
          if (++params_window.body.at(index).first.second ==
            static_cast<int>(params_window.body.at(index).second.size()))
          {
            params_window.body.at(index).first.second = 0;
          }
          win_focus_enable(this->focus_column_, ACS_CKBOARD);
          break;
        case KEY_UP:
          win_focus_enable(this->focus_window_, ' ');
          if (params_window.body.at(index).first.second-- == 0) {
            params_window.body.at(index).first.second = params_window.body.at(index).second.size() -
              1;
          }
          win_focus_enable(this->focus_column_, ACS_CKBOARD);
          break;
        case KEY_LEFT:
          if (params_window.body.at(index).first.second ==
            static_cast<int>(params_window.body.at(index).second.size() - 1))
          {
            params_window.body.at(index).second.at(params_window.body.at(index).first.second).value
              -=
              0.1;
          } else {
            params_window.body.at(index).second.at(params_window.body.at(index).first.second).value
              -=
              params_window.body.at(index).second.at(params_window.body.at(index).first.second).
              params.maximum_value *
              params_window.body.at(index).second.back().value / 100;
          }
          if (params_window.body.at(index).second.at(params_window.body.at(index).first.second).
            value < params_window.body.at(index).second.at(
              params_window.body.at(
                index).first.second).params.minimum_value)
          {
            params_window.body.at(index).second.at(params_window.body.at(index).first.second).value
              =
              params_window.body.at(index).second.at(params_window.body.at(index).first.second).
              params.minimum_value;
          }
          win_focus_enable(this->focus_column_, ACS_CKBOARD);
          break;
        case KEY_RIGHT:
          if (params_window.body.at(index).first.second ==
            static_cast<int>(params_window.body.at(index).second.size() - 1))
          {
            params_window.body.at(index).second.at(params_window.body.at(index).first.second).value
              +=
              0.1;
          } else {
            params_window.body.at(index).second.at(params_window.body.at(index).first.second).value
              +=
              params_window.body.at(index).second.at(params_window.body.at(index).first.second).
              params.maximum_value *
              params_window.body.at(index).second.back().value / 100;
          }
          if (params_window.body.at(index).second.at(params_window.body.at(index).first.second).
            value > params_window.body.at(index).second.at(
              params_window.body.at(
                index).first.second).params.maximum_value)
          {
            params_window.body.at(index).second.at(params_window.body.at(index).first.second).value
              =
              params_window.body.at(index).second.at(params_window.body.at(index).first.second).
              params.maximum_value;
          }
          win_focus_enable(this->focus_column_, ACS_CKBOARD);
          break;
        default:
          break;
      }
    }
  } while (++index < size);
}

void DebugAbilityset::InitParamsWins(WINDOW ** wins, int size)
{
  int index = 0, origin_x = this->state_win_width_, origin_y = 0;
  auto get_height = [&]() -> uint {
      return static_cast<uint>(
        this->params_win_head_height_ + params_window.body.at(index).second.size());
    };
  auto get_width = [&]() -> uint {
      uint ret = 1;
      for (auto meta : params_window.length) {
        ret += meta + 1;
      }
      return ret;
    };
  for (index = 0; index < size; ++index) {
    wins[index] = newwin(get_height(), get_width(), origin_y, origin_x);
    this->DrawParamsWin(wins[index], index);
    origin_y += this->incremental_y_;
    origin_x += this->incremental_x_;
  }
}

void DebugAbilityset::Launch()
{
  INFO("DebugAbilityset launch...");
  auto get_forbidden_zone_row = [&]() -> uint {     // 获取禁止区行数
      uint letf_row = 3 + this->state_window.body.size() * 2;
      uint right_row = this->params_win_head_height_ + params_window.body.back().second.size() +
        (params_window.body.size() - 1) * this->incremental_y_ + this->function_win_height_;
      return (letf_row > right_row) ? letf_row : right_row;
    };
  auto get_forbidden_zone_col = [&]() -> uint {     // 获取禁止区列数
      uint letf_col = 3 + state_window.length.front() + state_window.length.back();
      uint right_col = 1 +
        this->GetX(0, 5) + (params_window.body.size() - 1) * this->incremental_x_;
      return static_cast<uint>(letf_col + right_col);
    };
  uint min_row = 1 + get_forbidden_zone_row();      // this->base_window.help_info.size()
  uint min_col = get_forbidden_zone_col();
  winsize ws{};
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
  if ((ws.ws_col >= min_col) && (ws.ws_row >= min_row)) {
    // setlocale(LC_ALL, "en_US.utf8");
    // setlocale(LC_ALL, "en_GB.utf8");
    // setlocale(LC_ALL, "zh_CN.UTF-8");
    setlocale(LC_ALL, "");
    INFO("当前语言环境：%s\n", setlocale(LC_ALL, NULL));
    int params_wins_size = 6;
    WINDOW * state_wins[1];
    WINDOW * params_wins[6];
    WINDOW * function_win = nullptr;
    ITEM ** function_items;
    MENU * function_menu;
    PANEL * my_panels[8];
    PANEL * top;
    int items_choices = function_window.body.size();

    auto init_color = [&]() {                   // 初始化所有颜色
        init_pair(this->default_window_, COLOR_WHITE, COLOR_BLACK);
        init_pair(this->focus_window_, COLOR_WHITE, COLOR_MAGENTA);
        init_pair(this->default_column_, COLOR_WHITE, COLOR_BLACK);
        init_pair(this->focus_column_, COLOR_BLACK, COLOR_CYAN);
        init_pair(this->warning_message_, COLOR_BLACK, COLOR_YELLOW);
        init_pair(this->error_message_, COLOR_BLACK, COLOR_RED);
      };
    auto add_function_win_line = [&](uint y_) {                 // 为功能窗口添加线
        mvwaddch(function_win, y_, 0, ACS_LTEE);
        mvwhline(function_win, y_, 1, ACS_HLINE, this->function_win_width_ - 1);
        mvwaddch(function_win, y_, this->function_win_width_ - 1, ACS_RTEE);
      };
    auto update_function_win_cmd = [&](
      std::string msg, bool is_focus = true) {  // 更新功能窗口指令
        uint _y = this->function_win_height_ - 2, _x = 1;
        mvwhline(function_win, _y, _x, ' ', this->function_win_width_ - 2);
        mvwprintw(function_win, _y, _x, "%s", msg.c_str());
        if (is_focus) {
          wbkgd(panel_window(top), COLOR_PAIR(this->focus_window_));
        }
        refresh();
        update_panels();
        doupdate();
      };
    auto clear_free_zone = [&]() {              // 清空自由区
        // int y, x;
        // getyx(stdscr, y, x);
        uint free_zone_row_beg = get_forbidden_zone_row();
        winsize now_ws{};
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &now_ws);
        uint _y = free_zone_row_beg;
        for (; _y < now_ws.ws_row; _y++) {
          move(_y, 0);
          clrtoeol();
          mvhline(_y, 0, ' ', now_ws.ws_col);
        }
        move(free_zone_row_beg, 0);
        refresh();
        // move(y, x);
      };
    auto update_free_zone = [&](
      const std::vector<std::string> info, uint style = 0) {       // 更新自由区
        uint free_zone_row_beg = get_forbidden_zone_row();
        winsize now_ws{};
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &now_ws);
        uint _y = free_zone_row_beg;
        for (; _y < now_ws.ws_row; _y++) {
          move(_y, 0);
          clrtoeol();
          mvhline(_y, 0, ' ', now_ws.ws_col);
        }
        refresh();
        move(free_zone_row_beg, 0);
        switch (style) {
          case 1: attron(COLOR_PAIR(this->default_window_) | A_BOLD | A_BLINK);break;
          case 2: attron(COLOR_PAIR(this->default_window_) | A_BOLD | A_UNDERLINE | A_BLINK);break;
          default: attron(COLOR_PAIR(this->default_window_));break;
        }
        _y = free_zone_row_beg;
        for (auto meta : info) {
          if (_y < now_ws.ws_row) {
            mvprintw(_y, 0, "%s", meta.c_str());
            _y += meta.length() / now_ws.ws_col +
              static_cast<uint>((meta.length() % now_ws.ws_col) ? 1 : 0);
          } else {
            attron(COLOR_PAIR(this->warning_message_) | A_BOLD | A_UNDERLINE | A_BLINK);
            mvprintw(
              --_y, 0,
              "The current window space is insufficient.");
            attroff(COLOR_PAIR(this->warning_message_) | A_BOLD | A_UNDERLINE | A_BLINK);
            break;
          }
        }
        switch (style) {
          case 1: attroff(COLOR_PAIR(this->default_window_) | A_BOLD | A_BLINK);break;
          case 2: attroff(COLOR_PAIR(this->default_window_) | A_BOLD | A_UNDERLINE | A_BLINK);break;
          default: attroff(COLOR_PAIR(this->default_window_));break;
        }
        refresh();
        update_panels();
        doupdate();
      };
    auto init_function_win = [&]() {            // 初始化功能窗口
        this->function_win_width_ = 1 +
          this->GetX(0, 5) + (params_window.body.size() - 1) * this->incremental_x_;
        int origin_x = this->state_win_width_,
          origin_y = this->params_win_head_height_ + params_window.body.back().second.size() +
          (params_window.body.size() - 1) * this->incremental_y_;
        function_win = newwin(
          this->function_win_height_, this->function_win_width_, origin_y,
          origin_x);
        box(function_win, 0, 0);
        uint y_ = 1, x_ = ((this->function_win_width_ - function_window.head.length()) / 2);
        mvwprintw(function_win, y_, x_, "%s", function_window.head.c_str());
        add_function_win_line(2);
        add_function_win_line(this->function_win_height_ - 3);
        update_function_win_cmd(">> ", false);
        function_items = reinterpret_cast<ITEM **>(calloc(items_choices, sizeof(ITEM *)));
        for (int i = 0; i < items_choices; i++) {
          function_items[i] = new_item(
            function_window.body.at(
              i).second.first.c_str(), function_window.body.at(i).second.second.c_str());
        }
        function_menu = new_menu(reinterpret_cast<ITEM **>(function_items));
        // menu_opts_on();  // 默认全部打开
        // O_ONEVALUE      // 此菜单只能选择一项: 单选菜单
        // O_SHOWDESC      // 菜单时显示项目描述
        // O_ROWMAJOR      // 以行优先顺序显示菜单
        // O_IGNORECASE    // 模式匹配时忽略大小写
        // O_SHOWMATCH     // 时将光标移动到项目名称内
        // O_NONCYCLIC     // 不要环绕下一个项目和上一个项目，请求菜单的另一端
        menu_opts_off(function_menu, O_ONEVALUE);
        menu_opts_off(function_menu, O_SHOWMATCH);
        menu_opts_off(function_menu, O_NONCYCLIC);
        set_menu_win(function_menu, function_win);
        set_menu_sub(
          function_menu,
          derwin(
            function_win, this->function_win_height_ - 3, this->function_win_width_ - 2, 3,
            1));
        set_menu_format(function_menu, this->function_menu_height_, 1);
        set_menu_mark(function_menu, "$ ");
        post_menu(function_menu);
        // set_menu_fore(function_menu, COLOR_PAIR(1) | A_REVERSE);
        // set_menu_back(function_menu, COLOR_PAIR(2));
        // set_menu_grey(function_menu, COLOR_PAIR(3));
      };
    auto free_mutually_exclusive_items = [&]() {                // 释放互斥选项
        for (int i = 0; i < items_choices; i++) {
          item_opts_on(function_items[i], O_SELECTABLE);
        }
      };
    auto free_mutually_already_selected_items = [&]() {         // 释放已选选项
        ITEM ** items;
        items = menu_items(function_menu);
        for (int i = 0; i < item_count(function_menu); ++i) {
          if (item_value(items[i]) == TRUE) {
            set_item_value(items[i], FALSE);
          }
        }
        wbkgd(panel_window(top), COLOR_PAIR(this->focus_window_));
      };
    auto update_mutually_exclusive_items = [&]() {              // 更新互斥选项
        free_mutually_exclusive_items();
        ITEM ** items;
        items = menu_items(function_menu);
        for (int j = 0; j < item_count(function_menu); j++) {
          if (item_value(items[j]) == TRUE) {
            for (int i = 0; i < items_choices; i++) {
              if ((i != j) &&
                (function_window.body.at(i).first ==
                function_window.body.at(j).first))
              {
                item_opts_off(function_items[i], O_SELECTABLE);
              }
            }
          }
        }
      };
    auto init_win = [&]() {                     // 初始化窗口
        this->InitStateWins(state_wins);
        this->InitParamsWins(params_wins, 6);
        init_function_win();
      };
    auto free_function_win = [&]() {            // 释放功能窗口
        unpost_menu(function_menu);
        free_menu(function_menu);
        for (int i = 0; i < items_choices; ++i) {
          free_item(function_items[i]);
        }
      };
    auto init_panels = [&]() {                  // 初始化面板
        my_panels[0] = new_panel(params_wins[0]);
        my_panels[1] = new_panel(params_wins[1]);
        my_panels[2] = new_panel(params_wins[2]);
        my_panels[3] = new_panel(params_wins[3]);
        my_panels[4] = new_panel(params_wins[4]);
        my_panels[5] = new_panel(params_wins[5]);
        my_panels[6] = new_panel(function_win);
        my_panels[7] = new_panel(state_wins[0]);
      };
    auto init_panels_order = [&]() {            // 初始化面板顺序：设置指向下一个面板的用户指针
        set_panel_userptr(my_panels[0], my_panels[1]);
        set_panel_userptr(my_panels[1], my_panels[2]);
        set_panel_userptr(my_panels[2], my_panels[3]);
        set_panel_userptr(my_panels[3], my_panels[4]);
        set_panel_userptr(my_panels[4], my_panels[5]);
        set_panel_userptr(my_panels[5], my_panels[0]);
        set_panel_userptr(my_panels[6], my_panels[0]);
      };
    auto update_win_focus = [&](int color, bool able) {         // 更新窗口焦点
        this->WinUpdateFocus(params_wins, params_wins_size, panel_window(top), 9, able);
        wbkgd(panel_window(top), COLOR_PAIR(color));
      };
    auto set_top_win = [&]() {                  // 设置顶部窗口
        top = my_panels[5];
        update_win_focus(this->focus_window_, true);
        top_panel(top);
        update_panels();
        doupdate();
      };
    auto is_function_win = [&]() -> bool {      // 是功能窗口
        return static_cast<bool>(panel_window(top) == function_win);
      };
    auto trigger_function = [&]() {             // 执行功能
        std::vector<std::string> function_vector;
        std::stringstream message_str;
        message_str.str(function_window.request);
        std::string elems;
        while (std::getline(message_str, elems, ' ')) {
          if (elems == "help") {
            update_free_zone(this->base_window.help_info);
          } else {
            function_vector.push_back(elems);
          }
        }
        for (auto meta : function_vector) {
          update_free_zone(this->GetVector(this->TriggerFunction(meta)));
        }
      };
    auto start_function = [&]() {               // 启动功能
        char temp[200];
        ITEM ** items;
        items = menu_items(function_menu);
        temp[0] = '\0';
        for (int i = 0; i < item_count(function_menu); ++i) {
          if (item_value(items[i]) == TRUE) {
            snprintf(temp, sizeof(temp), "%s ", item_name(items[i]));
            // strcat(temp, item_name(items[i]));
            // strcat(temp, " ");
            set_item_value(items[i], FALSE);
          }
        }
        function_window.request = temp;
        update_function_win_cmd(function_window.flag + function_window.request);
        clear_free_zone();
        trigger_function();
      };
    auto refresh_window = [&]() {               // 刷新窗口
        init_win();
        init_panels();
        init_panels_order();
        set_top_win();
      };
    initscr();
    if ((has_colors() == TRUE) && this->SetCyberdog(false)) {
      start_color();
      // raw();
      cbreak();
      noecho();
      // nodelay(stdscr, TRUE);   // 超级耗CPU
      keypad(stdscr, TRUE);
      init_color();
      refresh_window();
      bool is_return = false;
      std::thread updata_thread([&] {
          while (!is_return) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::lock_guard<std::mutex> guard(cursor_mutex_);
            this->UpdateStatus();
            if (this->UpdataStateWins(state_wins)) {
              update_panels();
              doupdate();
            }
          }
        });
      updata_thread.detach();
      int key;
      while ((key = getch()) != 27) {
        std::lock_guard<std::mutex> guard(cursor_mutex_);
        switch (key) {
          case 9:        // tab
            update_win_focus(this->default_window_, false);
            top = (PANEL *)panel_userptr(top);  // NOLINT
            // reinterpret_cast<PANEL *>(panel_userptr(top));
            top_panel(top);
            update_win_focus(this->focus_window_, true);
            break;
          case KEY_DOWN:
            if (is_function_win()) {
              // menu_driver(function_menu, REQ_SCR_DLINE);
              menu_driver(function_menu, REQ_DOWN_ITEM);
              wbkgd(panel_window(top), COLOR_PAIR(this->focus_window_));
            } else {
              this->WinUpdateFocus(params_wins, params_wins_size, panel_window(top), key);
            }
            break;
          case KEY_UP:
            if (is_function_win()) {
              // menu_driver(function_menu, REQ_SCR_ULINE);
              menu_driver(function_menu, REQ_UP_ITEM);
              wbkgd(panel_window(top), COLOR_PAIR(this->focus_window_));
            } else {
              this->WinUpdateFocus(params_wins, params_wins_size, panel_window(top), key);
            }
            break;
          case KEY_LEFT:
            if (is_function_win()) {
              free_mutually_exclusive_items();
              free_mutually_already_selected_items();
            } else {
              this->WinUpdateFocus(params_wins, params_wins_size, panel_window(top), key);
            }
            break;
          case KEY_RIGHT:
            if (is_function_win()) {
              menu_driver(function_menu, REQ_TOGGLE_ITEM);
              update_mutually_exclusive_items();
              update_win_focus(this->focus_window_, true);
            } else {
              this->WinUpdateFocus(params_wins, params_wins_size, panel_window(top), key);
            }
            break;
          case KEY_NPAGE:
            if (is_function_win()) {
              menu_driver(function_menu, REQ_SCR_DPAGE);
              wbkgd(panel_window(top), COLOR_PAIR(this->focus_window_));
            }
            break;
          case KEY_PPAGE:
            if (is_function_win()) {
              menu_driver(function_menu, REQ_SCR_UPAGE);
              wbkgd(panel_window(top), COLOR_PAIR(this->focus_window_));
            }
            break;
          case ' ':
            if (!is_function_win()) {
              update_win_focus(this->default_window_, false);
              top = my_panels[6];
              top_panel(top);
              update_win_focus(this->focus_window_, true);
            }
            break;
          case 10:          // Enter
            if (is_function_win()) {
              free_mutually_exclusive_items();
              start_function();
            }
            break;
          case 'w':
          case 'x':
          case 'a':
          case 'd':
          case 'q':
          case 'e':
          case 'z':
          case 'c':
          case 's':
          case 'W':
          case 'X':
          case 'A':
          case 'D':
          case 'Q':
          case 'E':
          case 'Z':
          case 'C':
          case 'S':
            update_free_zone(this->GetVector(this->KeyboardTeleop(key)));
            break;
        }
        update_panels();
        doupdate();
      }
      is_return = true;
      update_free_zone(this->base_window.cyberdog_info, 1);
      sleep(3);
      clear_free_zone();
      free_function_win();
      this->SetCyberdog(true);
    } else {
      WARN("Your terminal does not support color\n");
    }
    endwin();
  } else {
    WARN(
      "The minimum terminal window size is {row * col} = {%d * %d}, "
      "but the current window size is {row * col} = {%d * %d}, "
      "please expand the terminal window.", min_row, min_col, ws.ws_row,
      ws.ws_col);
  }
  INFO("DebugAbilityset shutdown.");
}

}   // namespace cyberdog_visual_programming_terminal

// waddch(win, ACS_ULCORNER);      // ┌
// waddch(win, ACS_LLCORNER);      // └
// waddch(win, ACS_URCORNER);      // ┐
// waddch(win, ACS_LRCORNER);      // ┘
// waddch(win, ACS_LTEE);      // ├
// waddch(win, ACS_RTEE);      // ┤
// waddch(win, ACS_BTEE);      // ┴
// waddch(win, ACS_TTEE);      // ┬
// waddch(win, ACS_HLINE);      // ─
// waddch(win, ACS_VLINE);      // │
// waddch(win, ACS_PLUS);      // ┼
// waddch(win, ACS_S1);      // ┼
// waddch(win, ACS_S9);      // ┼
// waddch(win, ACS_DIAMOND);      // ┼
// waddch(win, ACS_CKBOARD);      // ┼
// waddch(win, ACS_DEGREE);      // ┼
// waddch(win, ACS_PLMINUS);      // ┼
// waddch(win, ACS_BULLET);      // ┼
// waddch(win, ACS_LARROW);      // ┼
// waddch(win, ACS_RARROW);      // ┼
// waddch(win, ACS_DARROW);      // ┼
// waddch(win, ACS_UARROW);      // ┼
// waddch(win, ACS_BOARD);      // ┼
// waddch(win, ACS_LANTERN);      // ┼
// waddch(win, ACS_S3);      // ┼
// waddch(win, ACS_S7);      // ┼
// waddch(win, ACS_LEQUAL);      // ┼
// waddch(win, ACS_GEQUAL);      // ┼
// waddch(win, ACS_PI);      // ┼
// waddch(win, ACS_NEQUAL);      // ┼
// waddch(win, ACS_STERLING);      // ┼
// waddch(win, ACS_BSSB);      // ┼
// waddch(win, ACS_SSBB);      // ┼
// waddch(win, ACS_BBSS);      // ┼
// waddch(win, ACS_SBBS);      // ┼
// waddch(win, ACS_SBSS);      // ┼
// waddch(win, ACS_SSSB);      // ┼
// waddch(win, ACS_SSBS);      // ┼
// waddch(win, ACS_BSSS);      // ┼
// waddch(win, ACS_BSBS);      // ┼
// waddch(win, ACS_SBSB);      // ┼
// waddch(win, ACS_SSSS);      // ┼
