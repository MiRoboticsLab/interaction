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
#include <memory>

#include "cyberdog_vp_abilityset/motion.hpp"

namespace cyberdog_visual_programming_abilityset
{
std::mutex compensation_frame_mtx_;               /*!< 补偿帧互斥量 */

bool Motion::SetData(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->servo_cmd_deviation_coefficient_ = toml::find<double>(
      _params_toml, "vp", "init", "params", "servo_cmd_deviation_coefficient");
    this->odom_deviation_coefficient_ = toml::find<double>(
      _params_toml, "vp", "init", "params", "odom_deviation_coefficient");
    this->servo_cmd_end_frame_time_consuming_ = toml::find<uint>(
      _params_toml, "vp", "init", "params", "servo_cmd_end_frame_time_consuming");
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return python_.Init();
}

bool Motion::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    uint compensation_frame_hz = toml::find_or(
      _params_toml, "vp", "init", "params", "compensation_frame_hz", 20);
    if (compensation_frame_hz > 0) {
      this->timer_cb_group_ =
        this->node_immortal_ptr_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      this->compensation_frame_timer_ = this->node_immortal_ptr_->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / compensation_frame_hz)),
        std::bind(&Motion::CompensationFrame, this),
        this->timer_cb_group_);
    }

    this->result_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->sequence_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->status_sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->pub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->result_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvMotionResultCmd>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "motion", "motion_result_cmd"),
      ClientQos.get_rmw_qos_profile(),
      this->result_cli_cb_group_);
    this->sequence_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvMotionSequenceShow>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "sequence", "motion_sequence"),
      ClientQos.get_rmw_qos_profile(),
      this->sequence_cli_cb_group_);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->sub_cb_group_;
    this->sub_ptr_ = this->node_mortal_ptr_->create_subscription<MsgMotionServoResponse>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "motion_servo_res"),
      1,
      std::bind(&Motion::ServoResponse, this, std::placeholders::_1),
      sub_option);

    sub_option.callback_group = this->status_sub_cb_group_;
    this->status_sub_ptr_ = this->node_mortal_ptr_->create_subscription<MsgMotionStatus>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "motion_status"),
      1,
      std::bind(&Motion::MotionStatusResponse, this, std::placeholders::_1),
      sub_option);

    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->pub_cb_group_;
    this->pub_ptr_ = this->node_mortal_ptr_->create_publisher<MsgMotionServoCmd>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "motion_servo_cmd"),
      PublisherQos,
      pub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Motion::InitDependent(
  const std::function<AudioPlaySeviceResponse(const std::string, const int8_t)> & _FOnlinePlay,
  const std::function<AudioPlaySeviceResponse(const uint16_t, const int8_t)> & _FOfflinePlay,
  const std::function<bool(
    MsgOdometry &,
    double &,
    const double)> & _OdomIfCumulativeDistance,
  const std::function<bool(Processor)> & _FGetProcessor)
{
  this->FOnlinePlay = _FOnlinePlay;
  this->FOfflinePlay = _FOfflinePlay;
  this->FIfCumulativeDistance = _OdomIfCumulativeDistance;
  this->FGetProcessor = _FGetProcessor;
}

void Motion::MotionStatusResponse(const MsgMotionStatus::SharedPtr msg_ptr)
{
  this->old_motion_id_ = this->motion_id_;
  this->motion_id_ = msg_ptr->motion_id;
  Debug(
    "[MotionStatusResponse] The motion status:"
    "\n\tmotion_id = %d"
    "\n\tcontact = %d"
    "\n\tswitch_status = %d"
    "\n\torder_process_bar = %d"
    "\n\tori_error = %d"
    "\n\tfootpos_error = %d"
    "\n\tmotor_error = %s",
    static_cast<int>(msg_ptr->motion_id),
    static_cast<int>(msg_ptr->contact),
    static_cast<int>(msg_ptr->switch_status),
    static_cast<int>(msg_ptr->order_process_bar),
    static_cast<int>(msg_ptr->ori_error),
    static_cast<int>(msg_ptr->footpos_error),
    intVectorToString(msg_ptr->motor_error).c_str());
}

void Motion::ServoResponse(const MsgMotionServoResponse::SharedPtr msg_ptr)
{
  this->servo_response_msg_ = *msg_ptr;
  WarnIf(
    !msg_ptr->result,
    "[ServoResponse] The motion control result command returns abnormally, please check."
    "\n\tmotion_id = %d\n\tresult = False\n\tcode = %d\n\torder_process_bar = %d\n\tstatus = %d",
    static_cast<int>(msg_ptr->result),
    static_cast<int>(msg_ptr->code),
    static_cast<int>(msg_ptr->order_process_bar),
    static_cast<int>(msg_ptr->status));
}

void Motion::CompensationFrame()
{
  if (this->compensation_frame_size_ > 0) {
    std::lock_guard<std::mutex> guard(compensation_frame_mtx_);
    this->compensation_frame_size_--;
    this->pub_ptr_->publish(this->compensation_frame_);
  }
}

std::shared_ptr<SrvMotionResultCmd::Request> Motion::GetResultRequest()
{
  std::shared_ptr<SrvMotionResultCmd::Request> request_ptr =
    std::make_shared<SrvMotionResultCmd::Request>();
  request_ptr->motion_id = 0;             // int32       # 机器人运控姿态
  request_ptr->cmd_source = SrvMotionResultCmd::Request::VIS;
  request_ptr->vel_des.resize(3, 0);      // float32[3]  # x y(最大值1.5） yaw（最大值2.0）m/s
  request_ptr->rpy_des.resize(3, 0);      // float32[3]  # roll pitch yaw（最大值0.4）  rad
  request_ptr->pos_des.resize(3, 0);      // float32[3]  # x y（最大值0.2）z（最大值0.3)  m/s
  request_ptr->acc_des.resize(6, 0);      // float32[3]  # acc for jump m^2/s
  request_ptr->ctrl_point.resize(3, 0);   // float32[3]  # 姿势 ctrl 点 m
  request_ptr->foot_pose.resize(6, 0);    // float32[3]  # 前/后脚姿势 x,y,z m
  request_ptr->step_height.resize(2, 0);  // float32[2]  # 小跑时的步高 m
  request_ptr->duration = 0;              // int32       # 命令执行时间毫秒
  request_ptr->value = 0;                 // int32
  request_ptr->toml_data = "";            // string
  return request_ptr;
}

std::shared_ptr<SrvMotionSequenceShow::Request> Motion::GetSequenceRequest(
  const MotionSequence & _sequence)
{
  std::shared_ptr<SrvMotionSequenceShow::Request> request_ptr =
    std::make_shared<SrvMotionSequenceShow::Request>();
  request_ptr->motion_id = MotionId::sequence_custom;
  request_ptr->duration = 0;
  request_ptr->gait_list = _sequence.gait_list;
  request_ptr->pace_list = _sequence.pace_list;
  auto set_gait_toml = [&]() {
      size_t list_size = _sequence.gait_list.size();
      std::ostringstream data_str;
      data_str << std::string(
        FORMAT(
          "# Gait Def (Gait toml)."
          "\n# This is the toml format constraint information for the sequence action gait."
          "\n# + request:"
          "\n#   - task: %s"
          "\n#   - time: %s"
          "\n# + sequence:"
          "\n#   - name: %s"
          "\n#   - describe: %s"
          "\n#   - size: %ld"
          "\n# + section:"
          "\n#   - contact: [右前, 左前, 右后, 左后](是否着地)"
          "\n#   - duration: n*30(ms)\n\n",
          this->task_id_.c_str(),
          GetTime(static_cast<int>(TimeMode::_Y_M_D_H_M_S)).c_str(),
          _sequence.name.c_str(), _sequence.describe.c_str(), list_size
      ));
      size_t list_inde = 0;
      for (auto & meta : _sequence.gait_list) {
        ++list_inde;
        std::string toml_meta = std::string(
          FORMAT(
            "# %ld/%ld"
            "\n[[section]]"
            "\ncontact  = [%d, %d, %d, %d]"
            "\nduration = %ld\n\n",
            list_inde, list_size,
            meta.right_forefoot,
            meta.left_forefoot,
            meta.right_hindfoot,
            meta.left_hindfoot,
            meta.duration
        ));
        data_str << toml_meta;
        request_ptr->gait_toml_list.push_back(
          std::string(FORMAT("# Gait toml meta.\n\n%s", toml_meta.c_str())));
      }
      request_ptr->gait_toml = data_str.str();
    };
  auto set_pace_toml = [&]() {
      size_t list_size = _sequence.pace_list.size();
      std::ostringstream data_str;
      data_str << std::string(
        FORMAT(
          "# Gait Params (Pace toml)."
          "\n# This is the toml format constraint information for the sequence action pace."
          "\n# + request:"
          "\n#   - task: %s"
          "\n#   - time: %s"
          "\n# + sequence:"
          "\n#   - name: %s"
          "\n#   - describe: %s"
          "\n#   - size: %ld"
          "\n# + step:"
          "\n#   - mode: 11(固定)"
          "\n#   - gait_id: 110(固定)"
          "\n#   - contact: 落地系数 * 1e1"
          "\n#   - life_count: 0(固定)"
          "\n#   - vel_des: [X轴线速度, Y轴线速度, Z轴线速度]"
          "\n#   - pos_des: [X轴质心位置偏移量(m ), Y轴质心位置偏移量(m ), Z轴质心位置偏移量(m )]"
          "\n#   - rpy_des: [X轴质心姿态偏移量(R°), Y轴质心姿态偏移量(P°), Z轴质心姿态偏移量(Y°)]"
          "\n#   - acc_des: [X轴位置权重, Y轴位置权重, Z轴位置权重, X-R轴姿态权重, Y-P轴姿态权重, Z-Y轴姿态权重]"  // NOLINT
          "\n#   - foot_pose: [右前足落地点X轴偏移量(m), 右前足落地点Y轴偏移量(m), 左前足落地点X轴偏移量(m), 左前足落地点Y轴偏移量(m), 右后足落地点X轴偏移量(m), 右后足落地点Y轴偏移量(m)]"  // NOLINT
          "\n#   - ctrl_point: [左后足落地点X轴偏移量(m), 左后足落地点Y轴偏移量(m), 摩擦系数]"
          "\n#   - step_height: [前方左右足抬腿高度捏合值(m), 后方左右足抬腿高度捏合值(m)]"
          "\n#   - value: 是否使用 MPC 轨迹"
          "\n#   - duration: 期望耗时\n\n",
          this->task_id_.c_str(),
          GetTime(static_cast<int>(TimeMode::_Y_M_D_H_M_S)).c_str(),
          _sequence.name.c_str(), _sequence.describe.c_str(), list_size
      ));
      size_t list_inde = 0;
      for (auto & meta : _sequence.pace_list) {
        ++list_inde;
        std::string toml_meta = std::string(
          FORMAT(
            "# %ld/%ld"
            "\n[[step]]"
            "\nmode = 11"
            "\ngait_id = 110"
            "\ncontact = %d"
            "\nlife_count = 0"
            "\nvel_des = [%lf, %lf, %lf]"
            "\nrpy_des = [%lf, %lf, %lf]"
            "\npos_des = [%lf, %lf, %lf]"
            "\nacc_des = [%lf, %lf, %lf, %lf, %lf, %lf]"
            "\nctrl_point = [%lf, %lf, %lf]"
            "\nfoot_pose = [%lf, %lf, %lf, %lf, %lf, %lf]"
            "\nstep_height = [%lf, %lf]"
            "\nvalue = %d"
            "\nduration = %ld\n\n",
            list_inde,
            list_size,
            /* contact */
            static_cast<int>(meta.landing_gain * 1e1),
            /* vel_des */
            meta.twist.linear.x,
            meta.twist.linear.y,
            meta.twist.linear.z,
            /* rpy_des */
            meta.centroid.orientation.x,
            meta.centroid.orientation.y,
            meta.centroid.orientation.z,
            /* pos_des */
            meta.centroid.position.x,
            meta.centroid.position.y,
            meta.centroid.position.z,
            /* acc_des */
            meta.weight.linear.x,
            meta.weight.linear.y,
            meta.weight.linear.z,
            meta.weight.angular.x,
            meta.weight.angular.y,
            meta.weight.angular.z,
            /* ctrl_point */
            meta.left_hindfoot.x,
            meta.left_hindfoot.y,
            meta.friction_coefficient,
            /* foot_pose */
            meta.right_forefoot.x,
            meta.right_forefoot.y,
            meta.left_forefoot.x,
            meta.left_forefoot.y,
            meta.right_hindfoot.x,
            meta.right_hindfoot.y,
            /* step_height */
            static_cast<double>(
              int32_t(meta.right_forefoot.w * 1e3) +
              int32_t(meta.left_forefoot.w * 1e3) *
              1e3),
            static_cast<double>(
              int32_t(meta.right_hindfoot.w * 1e3) +
              int32_t(meta.left_hindfoot.w * 1e3) *
              1e3),
            /* value */
            meta.use_mpc_track,
            /* duration */
            meta.duration
        ));
        request_ptr->duration += meta.duration;
        data_str << toml_meta;
        request_ptr->pace_toml_list.push_back(
          std::string(FORMAT("# Pace toml meta.\n\n%s", toml_meta.c_str())));
      }
      request_ptr->pace_toml = data_str.str();
    };
  set_gait_toml();
  set_pace_toml();
  Debug(
    "Now Request toml data is:"
    "\n+ motion_id = %d"
    "\n+ duration = %ld"
    "\n+ gait_toml:"
    "\n%s"
    "\n+ pace_toml:"
    "\n%s",
    request_ptr->motion_id,
    request_ptr->duration,
    request_ptr->gait_toml.c_str(),
    request_ptr->pace_toml.c_str());
  return request_ptr;
}

std::shared_ptr<MsgMotionServoCmd> Motion::GetServoCmd(
  const int32_t _motion_id,
  const int32_t _cmd_type)
{
  std::shared_ptr<MsgMotionServoCmd> servo_cmd_ptr = std::make_shared<MsgMotionServoCmd>();
  servo_cmd_ptr->motion_id = _motion_id;    // int32  # 机器人运控姿态
  servo_cmd_ptr->cmd_type = _cmd_type;      // int32  # 指令类型约束,
  //  1: Data, 2: End；（之前的Start帧可以省略）Data帧根据控制需求写入以下参数，End帧无需填写参数
  servo_cmd_ptr->cmd_source = MsgMotionServoCmd::VIS;  // int32  # 指令来源
  servo_cmd_ptr->vel_des.resize(3, 0);      // float32[3]  # x y(最大值1.5） yaw（最大值2.0）m/s
  servo_cmd_ptr->rpy_des.resize(3, 0);      // float32[3]  # roll pitch yaw（最大值0.4）rad
  servo_cmd_ptr->pos_des.resize(3, 0);      // float32[3]  # x y（最大值0.2）z（最大值0.3)m/s
  servo_cmd_ptr->acc_des.resize(3, 0);      // float32[3]  # acc for jump m^2/s
  servo_cmd_ptr->ctrl_point.resize(3, 0);   // float32[3]  # 姿势 ctrl 点 m
  servo_cmd_ptr->foot_pose.resize(3, 0);    // float32[3]  # 前/后脚姿势 x,y,z m
  servo_cmd_ptr->step_height.resize(2, 0);  // float32[2]  # 小跑时的步高 m
  // servo_cmd_ptr->duration = 0;           // int32       # 命令执行时间毫秒,当前暂不开放
  return servo_cmd_ptr;
}

bool Motion::RequestResultSrv(
  MotionResultServiceResponse & _response,
  std::string _interface_name,
  std::shared_ptr<SrvMotionResultCmd::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));  // 保险起见等待伺服结束
    if (!rclcpp::ok()) {
      Warn(
        "[%s] Client interrupted while requesting for motion result service to appear.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_request_interrupted;
      return false;
    }
    if (!this->result_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout))) {
      Warn(
        "[%s] Waiting for motion service to appear(start) timeout.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_appear_timeout;
      this->FOnlinePlay(
        FORMAT(
          "期待运动服务就绪，等待%d秒已超时，运动失败",
          _service_start_timeout), -1);
      return false;
    }
    Debug(
      "The %s interface is requesting motion result service.",
      _interface_name.c_str());
    auto result = this->result_cli_ptr_->async_send_request(_request_ptr);
    result.wait();
    auto result_ptr = result.get();
    _response.state.code = StateCode::success;
    _response.response = *result_ptr;
    WarnIf(
      !result_ptr->result,
      "[%s] The motion control result command returns abnormally, please check."
      "\n\tmotion_id = %d\n\tresult = %d\n\tcode = %d",
      _interface_name.c_str(),
      static_cast<int>(result_ptr->motion_id),
      static_cast<int>(result_ptr->result),
      static_cast<int>(result_ptr->code));
    if (!result_ptr->result) {
      this->FOfflinePlay(31053, -1);
      this->FOnlinePlay(FORMAT("运动服务反馈错误码为%d", result_ptr->code), -1);
    }
    return true;
  } catch (...) {
    Warn("[%s] RequestResultSrv() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

bool Motion::RequestSequenceSrv(
  MotionSequenceServiceResponse & _response,
  std::string _interface_name,
  std::shared_ptr<SrvMotionSequenceShow::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));  // 保险起见等待伺服结束
    if (!rclcpp::ok()) {
      Warn(
        "[%s] Client interrupted while requesting for motion sequence service to appear.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_request_interrupted;
      return false;
    }
    if (!this->sequence_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout))) {
      Warn(
        "[%s] Waiting for sequence service to appear(start) timeout.",
        _interface_name.c_str());
      _response.state.code = StateCode::service_appear_timeout;
      return false;
    }
    Debug(
      "The %s interface is requesting motion sequence service.",
      _interface_name.c_str());
    auto result = this->sequence_cli_ptr_->async_send_request(_request_ptr);
    result.wait();
    auto result_ptr = result.get();
    _response.state.code = StateCode::success;
    _response.response = *result_ptr;
    WarnIf(
      !result_ptr->result,
      "[%s] The motion control result command returns abnormally, please check."
      "\n\tmotion_id = %d\n\tresult = %d\n\tcode = %d\n\tdescribe = %s",
      _interface_name.c_str(),
      static_cast<int>(result_ptr->motion_id),
      static_cast<int>(result_ptr->result),
      static_cast<int>(result_ptr->code),
      result_ptr->describe.c_str());
    return true;
  } catch (...) {
    Warn("[%s] RequestResultSrv() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

bool Motion::PublisherServoCmd(
  MotionServoCmdResponse & _response,
  std::string _interface_name,
  std::shared_ptr<MsgMotionServoCmd> _servo_cmd_ptr,
  const uint & _compensation_frame_size)
{
  try {
    this->pub_ptr_->publish(*_servo_cmd_ptr);
    if (_compensation_frame_size > 0) {
      std::lock_guard<std::mutex> guard(compensation_frame_mtx_);
      this->compensation_frame_size_ = _compensation_frame_size;
      this->compensation_frame_ = *_servo_cmd_ptr;
    }
    _response.response = this->servo_response_msg_;
    return true;
  } catch (...) {
    Warn("[%s] PublisherServoCmd() is failed.", _interface_name.c_str());
    _response.state.code = StateCode::fail;
  }
  return false;
}

bool Motion::PublisherServoEnd(
  MotionServoCmdResponse & _response,
  const int32_t _motion_id,
  const std::string _interface_name)
{
  std::shared_ptr<MsgMotionServoCmd> servo_cmd_ptr = this->GetServoCmd(_motion_id, 2);
  if (!this->PublisherServoCmd(_response, _interface_name, servo_cmd_ptr)) {
    Warn("[PublisherServoEnd] is false.");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(this->servo_cmd_end_frame_time_consuming_));
  return true;
}

MotionResultServiceResponse Motion::Request(
  const int32_t _motion_id,
  const int32_t _duration)
{
  MotionResultServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(motion_id = %d,"
    " duration = %d)",
    static_cast<int>(_motion_id),
    static_cast<int>(_duration));
  Info("%s ...", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  auto request = [&](const int32_t __motion_id, const int32_t __duration = 0) {
      auto request_ptr = this->GetResultRequest();
      request_ptr->motion_id = __motion_id;
      request_ptr->duration = __duration;
      if (!this->RequestResultSrv(ret, funs, request_ptr, 3)) {
        Warn(
          "[%s] Request result service is error.",
          funs.c_str());
      }
      return static_cast<int>(ret.state.code == StateCode::success);
    };
  auto sit_down_twist_ass_compensation = [&]() -> bool {
      bool is_sit_down = static_cast<bool>(
        ((this->old_motion_id_ == static_cast<int>(MotionId::dance_collection)) ||
        (this->old_motion_id_ == static_cast<int>(MotionId::sit_down)) ||
        (this->old_motion_id_ == static_cast<int>(MotionId::shake_ass_left)) ||
        (this->old_motion_id_ == static_cast<int>(MotionId::shake_ass_right)) ||
        (this->old_motion_id_ == static_cast<int>(MotionId::shake_ass_from_side_to_side))) &&
        (this->motion_id_ ==
        static_cast<int>(MotionId::relatively_position_control_attitude_insert_frame_2)));
      if ((_motion_id == MotionId::shake_ass_left) ||
        (_motion_id == MotionId::shake_ass_right) ||
        (_motion_id == MotionId::shake_ass_from_side_to_side))
      { // 正向补偿：使狗坐下
        return is_sit_down ? true : request(MotionId::sit_down);
      }
      // 逆向补偿：使狗站立
      return (is_sit_down && (_motion_id != MotionId::resume_standing)) ? request(
        MotionId::resume_standing) : true;
    };
  if (sit_down_twist_ass_compensation()) {
    request(_motion_id, _duration);
  }
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  Info("%s is ok, %s.", funs.c_str(), ret.state.describe.c_str());
  return ret;
}

void Motion::ServiceBaseMotionAttitude(
  MotionResultServiceResponse & _response,
  const std::string _interface_name,
  const int32_t _motion_id,
  const MsgPose & _centroid,
  const MsgPoint & _fulcrum)
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = _motion_id;
  request_ptr->pos_des[0] = _centroid.position.x;
  request_ptr->pos_des[1] = _centroid.position.y;
  request_ptr->pos_des[2] = _centroid.position.z;
  request_ptr->rpy_des[0] = Angle2Radian(_centroid.orientation.x);
  request_ptr->rpy_des[1] = Angle2Radian(_centroid.orientation.y);
  request_ptr->rpy_des[2] = Angle2Radian(_centroid.orientation.z);
  request_ptr->ctrl_point[0] = _fulcrum.x;
  request_ptr->ctrl_point[1] = _fulcrum.y;
  request_ptr->ctrl_point[2] = _fulcrum.z;
  request_ptr->duration = static_cast<int32_t>(_centroid.orientation.w * 1000);   // 秒转毫秒
  if (!this->RequestResultSrv(_response, _interface_name, request_ptr, 3)) {
    Warn(
      "[%s] Request result service is error.",
      _interface_name.c_str());
  }
}

MotionResultServiceResponse Motion::AbsoluteForceControlAttitude(
  const double _centroid_z,
  const double _roll,
  const double _pitch,
  const double _yaw,
  const double _duration)
{
  MotionResultServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(centroid_z = %lf,"
    " roll = %lf, pitch = %lf, yaw = %lf,"
    " duration = %d)",
    _centroid_z,
    _roll,
    _pitch,
    _yaw,
    static_cast<int>(_duration));
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  MsgPose centroid;
  centroid.position.x = 0.0;
  centroid.position.y = 0.0;
  centroid.position.z = _centroid_z;
  centroid.orientation.x = _roll;
  centroid.orientation.y = _pitch;
  centroid.orientation.z = _yaw;
  centroid.orientation.w = _duration;
  MsgPoint fulcrum;
  fulcrum.x = 0.0;
  fulcrum.y = 0.0;
  fulcrum.z = 0.0;
  this->ServiceBaseMotionAttitude(
    ret,
    std::string(__FUNCTION__),
    MsgMotionID::FORCECONTROL_DEFINITIVELY,
    centroid,
    fulcrum);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionResultServiceResponse Motion::RelativelyForceControlAttitude(
  const double _centroid_x,
  const double _centroid_y,
  const double _centroid_z,
  const double _roll,
  const double _pitch,
  const double _yaw,
  const double _duration)
{
  MotionResultServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(centroid_x = %lf, centroid_y = %lf, centroid_z = %lf,"
    " roll = %lf, pitch = %lf, yaw = %lf,"
    " duration = %d)",
    _centroid_x,
    _centroid_y,
    _centroid_z,
    _roll,
    _pitch,
    _yaw,
    static_cast<int>(_duration));
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  MsgPose centroid;
  centroid.position.x = _centroid_x;
  centroid.position.y = _centroid_y;
  centroid.position.z = _centroid_z;
  centroid.orientation.x = _roll;
  centroid.orientation.y = _pitch;
  centroid.orientation.z = _yaw;
  centroid.orientation.w = _duration;
  MsgPoint fulcrum;
  fulcrum.x = 0.0;
  fulcrum.y = 0.0;
  fulcrum.z = 0.0;
  this->ServiceBaseMotionAttitude(
    ret,
    std::string(__FUNCTION__),
    MsgMotionID::FORCECONTROL_RELATIVEYLY,
    centroid,
    fulcrum);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionResultServiceResponse Motion::AbsolutePositionControlAttitude(
  const double _centroid_z,
  const double _duration)
{
  MotionResultServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(centroid_z = %lf,"
    " duration = %d)",
    _centroid_z,
    static_cast<int>(_duration));
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  MsgPose centroid;
  centroid.position.x = 0.0;
  centroid.position.y = 0.0;
  centroid.position.z = _centroid_z;
  centroid.orientation.x = 0.0;
  centroid.orientation.y = 0.0;
  centroid.orientation.z = 0.0;
  centroid.orientation.w = _duration;
  MsgPoint fulcrum;
  fulcrum.x = 0.0;
  fulcrum.y = 0.0;
  fulcrum.z = 0.0;
  this->ServiceBaseMotionAttitude(
    ret,
    std::string(__FUNCTION__),
    MsgMotionID::POSECONTROL_DEFINITIVELY,
    centroid,
    fulcrum);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionResultServiceResponse Motion::RelativelyPositionControlAttitude(
  const double _centroid_x,
  const double _centroid_y,
  const double _centroid_z,
  const double _roll,
  const double _pitch,
  const double _yaw,
  const double _fulcrum_x,
  const double _fulcrum_y,
  const double _fulcrum_z,
  const double _duration)
{
  MotionResultServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(centroid_x = %lf, centroid_y = %lf, centroid_z = %lf,"
    " _roll = %lf, _pitch = %lf, _yaw = %lf,"
    " fulcrum_x = %lf, fulcrum_y = %lf, fulcrum_z = %lf,"
    " duration = %d)",
    _centroid_x,
    _centroid_y,
    _centroid_z,
    _roll,
    _pitch,
    _yaw,
    _fulcrum_x,
    _fulcrum_y,
    _fulcrum_z,
    static_cast<int>(_duration));
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  MsgPose centroid;
  centroid.position.x = _centroid_x;
  centroid.position.y = _centroid_y;
  centroid.position.z = _centroid_z;
  centroid.orientation.x = _roll;
  centroid.orientation.y = _pitch;
  centroid.orientation.z = _yaw;
  centroid.orientation.w = _duration;
  MsgPoint fulcrum;
  fulcrum.x = _fulcrum_x;
  fulcrum.y = _fulcrum_y;
  fulcrum.z = _fulcrum_z;
  this->ServiceBaseMotionAttitude(
    ret,
    std::string(__FUNCTION__),
    MsgMotionID::POSECONTROL_RELATIVEYLY,
    centroid,
    fulcrum);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

void Motion::ServiceBaseMotionLh(
  MotionResultServiceResponse & _response,
  const std::string _interface_name,
  const int32_t _motion_id,
  const double _front_leg_lift,
  const double _back_leg_lift)
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = _motion_id;
  request_ptr->step_height[0] = _front_leg_lift;
  request_ptr->step_height[1] = _back_leg_lift;
  if (!this->RequestResultSrv(_response, _interface_name, request_ptr)) {
    Warn(
      "[%s] Request result service is error.",
      _interface_name.c_str());
  }
}

MotionResultServiceResponse Motion::WalkTheDog(
  const double _front_leg_lift,
  const double _back_leg_lift)
{
  MotionResultServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf)", _front_leg_lift,
    _back_leg_lift);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->ServiceBaseMotionLh(
    ret,
    std::string(__FUNCTION__), MsgMotionID::PASSIVE_TROT,
    _front_leg_lift, _back_leg_lift);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

void Motion::ServoBaseMotionLh(
  MotionServoCmdResponse & _response,
  const std::string _interface_name,
  const int32_t _motion_id,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const uint & _compensation_frame_size)
{
  auto servo_cmd_ptr = this->GetServoCmd();
  servo_cmd_ptr->motion_id = _motion_id;
  servo_cmd_ptr->step_height[0] = _front_leg_lift;
  servo_cmd_ptr->step_height[1] = _back_leg_lift;
  if (!this->PublisherServoCmd(
      _response, _interface_name, servo_cmd_ptr,
      _compensation_frame_size))
  {
    Warn(
      "[%s] Publisher servo cmd is error.",
      _interface_name.c_str());
  }
}

void Motion::ServoBaseMotionVxyzLh(
  MotionServoCmdResponse & _response,
  const std::string _interface_name,
  const int32_t _motion_id,
  const double _x_velocity,
  const double _y_velocity,
  const double _z_velocity,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const double _distance,
  const double _duration,
  const uint & _compensation_frame_size)
{
  auto servo_cmd_ptr = this->GetServoCmd();
  servo_cmd_ptr->motion_id = _motion_id;
  servo_cmd_ptr->vel_des[0] = _x_velocity;
  servo_cmd_ptr->vel_des[1] = _y_velocity;
  servo_cmd_ptr->vel_des[2] = Angle2Radian(_z_velocity);
  servo_cmd_ptr->step_height[0] = _front_leg_lift;
  servo_cmd_ptr->step_height[1] = _back_leg_lift;
  uint32_t sleep_ms = 20;                 // 请求运动指令:50hz(要求不小于20hz)
  uint32_t max_processor_ms = 1000 / 2;   // 判断暂停请求:2hz
  uint32_t processor_ms = 0;              // 当前已经停止的时间（用于判断是否有暂停任务请求）
  auto publisher_servo_cmd = [&]() -> uint {
      uint ret = 0;   // 0: 失败且没必要继续; 1: 成功且继续; 2: 失败且有必要继续。
      if (this->PublisherServoCmd(
          _response, _interface_name, servo_cmd_ptr,
          _compensation_frame_size))
      {
        ret = (this->servo_response_msg_.motion_id == _motion_id) ? 1 : 2;
        processor_ms += sleep_ms;
        if (processor_ms > max_processor_ms) {
          processor_ms = 0;
          // 应对长时间运行时的暂停请求
          auto begin = std::chrono::steady_clock::now();
          this->FGetProcessor(Processor::task);
          uint32_t time_consuming_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin).count();
          if (sleep_ms > time_consuming_ms) {
            std::this_thread::sleep_for(
              std::chrono::milliseconds(
                uint32_t(sleep_ms - time_consuming_ms)));
          }
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
      } else {
        Warn(
          "[%s] Publisher servo cmd is error.",
          _interface_name.c_str());
      }
      return ret;
    };
  if ((this->motion_id_ != _motion_id) &&
    (this->motion_id_ != MotionId::resume_standing) &&
    (this->motion_id_ != MotionId::servo_standing))
  {  // 执行伺服指令前先确保机器人处于站立状态(恢复站立、伺服站立)
    if (!this->Request(MotionId::resume_standing).response.result) {
      Warn(
        "The current motion id=%d, "
        "the recovery to stand (id=%d) failed, "
        "and the servo command (id=%d) request failed.",
        this->motion_id_,
        static_cast<int>(MotionId::resume_standing),
        _motion_id);
      return;
    }
  }
  if (std::abs(_distance) > 0.001) {
    double cumulative = -1;                 // 累计距离(负数标识初始状态，用于初始化起点里程计)
    double distance = std::abs(_distance) / this->odom_deviation_coefficient_;  // 目标距离
    MsgOdometry now_odom;
    while (rclcpp::ok() && !this->FIfCumulativeDistance(now_odom, cumulative, distance)) {
      if (!publisher_servo_cmd()) {
        break;
      }
    }
    this->PublisherServoEnd(_response, _motion_id, _interface_name);
  } else if (std::abs(_duration) > 0.0009) {
    // 策略:不暂停时绝对时间，暂停时更新绝对时间
    std::chrono::time_point<std::chrono::system_clock> deadline =
      std::chrono::system_clock::now() +
      std::chrono::milliseconds(int64_t(std::abs(_duration) * 1000));
    uint pub_ser_cmd = 0;
    while (rclcpp::ok() && std::chrono::system_clock::now() < deadline) {
      std::chrono::milliseconds margin =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::system_clock::now());
      auto begin = std::chrono::steady_clock::now();
      pub_ser_cmd = publisher_servo_cmd();
      if (pub_ser_cmd == 0) {
        break;
      }
      if ((pub_ser_cmd == 2) ||
        (std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - begin).count() > 1))
      {  // 50hz请求时，“运控未就绪”或“任务被暂停”（执行过程耗时超过1秒，则视为执行中被暂停了）
         // 则视为本次伺服无效，重新计时
        deadline = std::chrono::system_clock::now() + margin;
      }
    }
    this->PublisherServoEnd(_response, _motion_id, _interface_name);
  } else {
    if (!this->PublisherServoCmd(
        _response, _interface_name, servo_cmd_ptr,
        _compensation_frame_size))
    {
      Warn(
        "[%s] Publisher servo cmd is error.",
        _interface_name.c_str());
    }
  }
}

MotionServoCmdResponse Motion::JumpBackAndForth(
  const double _x_velocity,
  const double _y_velocity,
  const double _z_velocity,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const double _distance,
  const double _duration,
  const uint _compensation_frame_size)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf, %lf, %lf, %lf, %lf)",
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift, _distance,
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->ServoBaseMotionVxyzLh(
    ret,
    std::string(__FUNCTION__), MsgMotionID::JUMP_BOUND,
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift,
    _distance, _duration, _compensation_frame_size);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionServoCmdResponse Motion::SmallJumpWalking(
  const double _x_velocity,
  const double _y_velocity,
  const double _z_velocity,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const double _distance,
  const double _duration,
  const uint _compensation_frame_size)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf, %lf, %lf, %lf, %lf)",
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift, _distance,
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->ServoBaseMotionVxyzLh(
    ret,
    std::string(__FUNCTION__), MsgMotionID::JUMP_PRONK,
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift,
    _distance, _duration, _compensation_frame_size);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionServoCmdResponse Motion::AutomaticFrequencyConversionWalking(
  const double _x_velocity,
  const double _y_velocity,
  const double _z_velocity,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const double _distance,
  const double _duration,
  const uint _compensation_frame_size)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf, %lf, %lf, %lf, %lf)",
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift,
    _distance, _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->ServoBaseMotionVxyzLh(
    ret,
    std::string(__FUNCTION__), MsgMotionID::WALK_ADAPTIVELY,
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift,
    _distance, _duration, _compensation_frame_size);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionServoCmdResponse Motion::TrotWalking(
  const double _x_velocity,
  const double _y_velocity,
  const double _z_velocity,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const double _distance,
  const double _duration,
  const uint _compensation_frame_size)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf, %lf, %lf, %lf, %lf)",
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift, _distance,
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->ServoBaseMotionVxyzLh(
    ret,
    std::string(__FUNCTION__), MsgMotionID::WALK_USERTROT,
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift,
    _distance, _duration, _compensation_frame_size);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionServoCmdResponse Motion::RunFastWalking(
  const double _x_velocity,
  const double _y_velocity,
  const double _z_velocity,
  const double _front_leg_lift,
  const double _back_leg_lift,
  const double _distance,
  const double _duration,
  const uint _compensation_frame_size)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf, %lf, %lf, %lf, %lf)",
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift, _distance,
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  this->ServoBaseMotionVxyzLh(
    ret,
    std::string(__FUNCTION__), MsgMotionID::WALK_FLYTROT,
    _x_velocity, _y_velocity, _z_velocity,
    _front_leg_lift, _back_leg_lift,
    _distance, _duration, _compensation_frame_size);
  ret.state.describe = this->GetDescribe(funs, ret.state.code);
  return ret;
}

MotionServoCmdResponse Motion::Turn(const double _angle, double _duration)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT("(%lf, %lf)", _angle, _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  auto sign = [](const double data) -> int {
      return static_cast<int>((data > 0) ? 1 : -1);
    };
  double z_velocity = _angle / _duration;
  if (fabs(z_velocity) > motion_params_.z_velocity.maximum_value) {
    z_velocity = sign(z_velocity) * motion_params_.z_velocity.maximum_value;
    _duration = fabs(_angle / motion_params_.z_velocity.maximum_value);
  }
  return this->AutomaticFrequencyConversionWalking(
    0.0, 0.0, z_velocity,
    motion_params_.front_leg_lift.default_value, motion_params_.back_leg_lift.default_value,
    0.0, _duration);
}

MotionServoCmdResponse Motion::GoStraight(
  const double x_velocity,
  const double _distance,
  const double _duration)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf)", x_velocity, _distance,
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  return this->AutomaticFrequencyConversionWalking(
    x_velocity, 0.0, 0.0,
    motion_params_.front_leg_lift.default_value, motion_params_.back_leg_lift.default_value,
    _distance, _duration);
}

MotionServoCmdResponse Motion::LateralMovement(
  const double y_velocity,
  const double _distance,
  const double _duration)
{
  MotionServoCmdResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf)", y_velocity, _distance,
    _duration);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  return this->AutomaticFrequencyConversionWalking(
    0.0, y_velocity, 0.0,
    motion_params_.front_leg_lift.default_value, motion_params_.back_leg_lift.default_value,
    _distance, _duration);
}

MotionSequenceServiceResponse Motion::RunSequence(const MotionSequence & _sequence)
{
  MotionSequenceServiceResponse ret;
  std::string funs = std::string(__FUNCTION__) +
    std::string(
    FORMAT(
      "(const MotionSequence & _sequence)"
      "\n + sequence:"
      "\n   - name: %s"
      "\n   - describe: %s"
      "\n   - gait_list: %ld"
      "\n   - pace_list: %ld",
      _sequence.name.c_str(),
      _sequence.describe.c_str(),
      _sequence.gait_list.size(),
      _sequence.pace_list.size()
    ));

  std::string sequence_str = std::string(
    FORMAT(
      "\n┌───────────────────────────────────────────────────---"
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
      _sequence.name.c_str(),
      _sequence.describe.c_str(),
      sequenceGaitVector(_sequence.gait_list, "    ").c_str(),
      sequencePaceVector(_sequence.pace_list, "    ").c_str()));
  Info("%s", funs.c_str());
  Debug("sequence msg is:\n%s", sequence_str.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    return ret;
  }
  if (this->RequestSequenceSrv(
      ret, std::string(__FUNCTION__),
      this->GetSequenceRequest(_sequence)))
  {
    Warn(
      "[%s] Request sequence Service is error.",
      funs.c_str());
  }
  return ret;
}

bool Motion::Choreographer(
  const std::string _type,
  const py::args _args)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %s) ...",
    _type.c_str(),
    pyArgsToString(_args).c_str());
  Info("%s", funs.c_str());
  try {
    // py::object self = py::get_object_handle(this);  // 新版本 pybind11 接口
    py::handle self = py::cast(this);
    uint64_t pythonId = PyLong_AsLong(PyLong_FromVoidPtr(self.ptr()));
    return python_.Choreographer(uint64_t(pythonId), _type, _args);
  } catch (const std::exception & e) {
    Error("%s error:%s", funs.c_str(), e.what());
  }
  return false;
}

bool Motion::Choreographer(const py::kwargs _kwargs)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s) ...",
    pyKwargsToString(_kwargs).c_str());
  Info("%s", funs.c_str());
  try {
    // py::object self = py::get_object_handle(this);  // 新版本 pybind11 接口
    py::handle self = py::cast(this);
    uint64_t pythonId = PyLong_AsLong(PyLong_FromVoidPtr(self.ptr()));
    return python_.Choreographer(uint64_t(pythonId), _kwargs);
  } catch (const std::exception & e) {
    Error("%s error:%s", funs.c_str(), e.what());
  }
  return false;
}
}   // namespace cyberdog_visual_programming_abilityset
