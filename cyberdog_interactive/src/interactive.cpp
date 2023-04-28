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
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_interactive/interactive.hpp"

namespace cyberdog
{
namespace interaction
{
Interactive::Interactive(const std::string & name)
{
  INFO("Creating [Interactive] object(node)");
  this->node_ptr_ = rclcpp::Node::make_shared(name);
}

Interactive::~Interactive()
{
  INFO("Destroy [Interactive] object(node)");
}

bool Interactive::Init()
{
  INFO("Initializing ...");
  try {
    std::string node_config_dir =
      ament_index_cpp::get_package_share_directory("cyberdog_interactive") +
      "/config/interactive.toml";
    toml::value params_toml;

    INFO("Params config file dir:<%s>", node_config_dir.c_str());

    if (access(node_config_dir.c_str(), F_OK)) {
      ERROR("Params config file does not exist");
      return false;
    }

    if (access(node_config_dir.c_str(), R_OK)) {
      ERROR("Params config file does not have read permissions");
      return false;
    }

    if (access(node_config_dir.c_str(), W_OK)) {
      ERROR("Params config file does not have write permissions");
      return false;
    }

    if (!cyberdog::common::CyberdogToml::ParseFile(
        node_config_dir.c_str(), params_toml))
    {
      ERROR("Params config file is not in toml format");
      return false;
    }

    this->obstacle_distance_ = toml::find_or(
      params_toml, "interactive", "initialization", "detection", "tof", "obstacle_distance",
      0.000001);

    this->obstacle_percentage_ = toml::find_or(
      params_toml, "interactive", "initialization", "detection", "tof", "obstacle_percentage",
      80.0);
    if (this->obstacle_percentage_ > 100) {
      this->obstacle_percentage_ = 100;
    } else if (this->obstacle_percentage_ < 0) {
      this->obstacle_percentage_ = 1;
    }
    this->obstacle_percentage_ /= 100;

    this->sensitivity_s_ = toml::find_or(
      params_toml, "interactive", "initialization", "detection", "tof", "sensitivity_s", 1.0);
    if (this->sensitivity_s_ < 0) {
      this->sensitivity_s_ = 0.1;  // 10 Hz : 100 毫秒
    }

    this->BTOTCWS_cb_group_ = this->node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    float hz = toml::find<float>(
      params_toml, "interactive", "initialization", "hz", "BTOTCWS");
    if (!(hz > 0)) {
      hz = 1;
    }
    this->BTOTCWS_timer_ = this->node_ptr_->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / hz)),
      std::bind(&Interactive::BeingTouchedOnTheChinWhileSitting, this),
      this->BTOTCWS_cb_group_);

    rclcpp::SubscriptionOptions sub_option;
    this->tof_sub_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_option.callback_group = this->tof_sub_cb_group_;
    this->tof_sub_head_ptr_ = this->node_ptr_->create_subscription<MsgHeadTofPayload>(
      "head_tof_payload",
      rclcpp::SensorDataQoS(),
      std::bind(&Interactive::SubHeadCB, this, std::placeholders::_1),
      sub_option);

    this->status_sub_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_option.callback_group = this->status_sub_cb_group_;
    this->status_sub_ptr_ = this->node_ptr_->create_subscription<MsgMotionStatus>(
      "motion_status",
      rclcpp::ParametersQoS(),
      std::bind(&Interactive::MotionStatusResponse, this, std::placeholders::_1),
      sub_option);

    this->result_cli_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->result_cli_ptr_ = this->node_ptr_->create_client<SrvMotionResultCmd>(
      "motion_result_cmd",
      rclcpp::SystemDefaultsQoS().get_rmw_qos_profile(),
      this->result_cli_cb_group_);

    this->play_pub_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->play_pub_cb_group_;
    this->topic_pub_ = this->node_ptr_->create_publisher<MsgAudioPlayExtend>(
      "speech_play_extend",
      rclcpp::ParametersQoS(),
      pub_option);

    INFO(
      "obstacle_distance %f, obstacle_percentage %f, sensitivity_s %f",
      this->obstacle_distance_,
      this->obstacle_percentage_,
      this->sensitivity_s_);
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

void Interactive::SubHeadCB(const MsgHeadTofPayload::SharedPtr msg_ptr)
{
  auto detection_obstacle = [&](MsgSingleTofPayload & payload, ObstacleMeta & obstacle) {
      if (!payload.data_available) {
        obstacle.detected = false;
        obstacle.validity_period_time = std::chrono::system_clock::now();
        return;
      }
      if ((obstacle.detected) &&
        (std::chrono::system_clock::now() < obstacle.validity_period_time))
      {
        return;
      }
      size_t obstacle_point = 0;
      for (auto ranging : payload.data) {
        if (ranging < this->obstacle_distance_) {
          ++obstacle_point;
        }
      }
      float obstacle_percentage = static_cast<float>(
        static_cast<float>(obstacle_point) /
        static_cast<float>(payload.data.size()));
      if (obstacle_percentage < this->obstacle_percentage_) {
        obstacle.detected = false;
        obstacle.validity_period_time = std::chrono::system_clock::now();
      } else {
        obstacle.detected = true;
        obstacle.validity_period_time = std::chrono::system_clock::now() +
          std::chrono::milliseconds(static_cast<int64_t>(1000 * this->sensitivity_s_));
      }
    };
  detection_obstacle(msg_ptr->left_head, this->head_left);
  detection_obstacle(msg_ptr->right_head, this->head_right);
  // INFO("head_left %d, head_right %d", this->head_left.detected, this->head_right.detected);
}

void Interactive::MotionStatusResponse(const MsgMotionStatus::SharedPtr msg_ptr)
{
  if (this->motion_id != msg_ptr->motion_id) {
    this->old_motion_id = this->motion_id;
  }
  this->motion_id = msg_ptr->motion_id;
}

bool Interactive::RequestResultSrv(
  std::shared_ptr<SrvMotionResultCmd::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    if (!rclcpp::ok()) {
      return false;
    }
    if (!this->result_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout))) {
      return false;
    }
    auto result = this->result_cli_ptr_->async_send_request(_request_ptr);
    // result.wait();
    std::future_status status =
      result.wait_for(std::chrono::seconds(10));
    if (status != std::future_status::ready) {
      WARN("Request motion timedout (10s) or deferred.");
      return false;
    }
    auto result_ptr = result.get();
    INFO(
      "The motion control result command returns:"
      "\n\tmotion_id = %d"
      "\n\tresult = %d"
      "\n\tcode = %d",
      static_cast<int>(result_ptr->motion_id),
      static_cast<int>(result_ptr->result),
      static_cast<int>(result_ptr->code));
    return true;
  } catch (const std::exception & e) {
    ERROR("RequestResultSrv failed: <%s>", e.what());
  }
  return false;
}

std::shared_ptr<SrvMotionResultCmd::Request> Interactive::GetResultRequest()
{
  std::shared_ptr<SrvMotionResultCmd::Request> request_ptr =
    std::make_shared<SrvMotionResultCmd::Request>();
  request_ptr->motion_id = 0;             // int32       # 机器人运控姿态
  request_ptr->vel_des.resize(3, 0);      // float32[3]  # x y(最大值1.5） yaw（最大值2.0）m/s
  request_ptr->rpy_des.resize(3, 0);      // float32[3]  # roll pitch yaw（最大值0.4）  rad
  request_ptr->pos_des.resize(3, 0);      // float32[3]  # x y（最大值0.2）z（最大值0.3)  m/s
  request_ptr->acc_des.resize(6, 0);      // float32[3]  # acc for jump m^2/s
  request_ptr->ctrl_point.resize(3, 0);   // float32[3]  # 姿势 ctrl 点 m
  request_ptr->foot_pose.resize(6, 0);    // float32[3]  # 前/后脚姿势 x,y,z m
  request_ptr->step_height.resize(2, 0);  // float32[2]  # 小跑时的步高 m
  request_ptr->duration = 0;              // int32       # 命令执行时间毫秒
  return request_ptr;
}

void Interactive::RelativePosture()
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = 212;
  request_ptr->duration = 100;
  if (!this->RequestResultSrv(request_ptr, 3)) {
    ERROR("RelativePosture failed.");
  }
}

void Interactive::SitDown()
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = 143;
  if (!this->RequestResultSrv(request_ptr, 3)) {
    ERROR("SitDown failed.");
  }
}

void Interactive::ShakeAssLeft()
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = 148;
  if (!this->RequestResultSrv(request_ptr, 3)) {
    ERROR("ShakeAssLeft failed.");
  }
}

void Interactive::ShakeAssRight()
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = 149;
  if (!this->RequestResultSrv(request_ptr, 3)) {
    ERROR("ShakeAssRight failed.");
  }
}

void Interactive::ShakeAssFromSideToSide()
{
  auto request_ptr = this->GetResultRequest();
  request_ptr->motion_id = 150;
  if (!this->RequestResultSrv(request_ptr, 3)) {
    ERROR("ShakeAssFromSideToSide failed.");
  }
}

void Interactive::WoofWoof(uint16_t play_id)
{
  MsgAudioPlayExtend audio_msg;
  audio_msg.module_name = "cyberdog_vp";
  audio_msg.is_online = false;
  // audio_msg.text = "汪汪";
  audio_msg.speech.module_name = "cyberdog_vp";
  audio_msg.speech.play_id = play_id;
  this->topic_pub_->publish(audio_msg);
}

void Interactive::BeingTouchedOnTheChinWhileSitting()
{
  try {
    if (((this->old_motion_id == 143) ||
      (this->old_motion_id == 148) ||
      (this->old_motion_id == 149) ||
      (this->old_motion_id == 150)) &&
      (this->motion_id == 214))
    {
      INFO("当前机器人已坐下");
      if (this->head_left.detected && this->head_right.detected) {
        INFO("检测到来回触摸下巴");
        this->WoofWoof(4001);
        this->RelativePosture();
        this->ShakeAssFromSideToSide();
        // this->SitDown();
      } else if (this->head_left.detected) {
        INFO("检测到触摸下巴左侧");
        this->WoofWoof(4000);
        this->RelativePosture();
        this->ShakeAssRight();
        // this->SitDown();
      } else if (this->head_right.detected) {
        INFO("检测到触摸下巴右侧");
        this->WoofWoof(4000);
        this->RelativePosture();
        this->ShakeAssLeft();
        // this->SitDown();
      } else {
        INFO("如果想和我玩，就摸摸我的下巴");
      }
      sleep(1);
    } else {
      // INFO("如果想和我玩，就让我坐下吧");
    }
  } catch (const std::exception & e) {
    ERROR("BeingTouchedOnTheChinWhileSitting failed: <%s>", e.what());
  }
}
}   // namespace interaction
}   // namespace cyberdog
