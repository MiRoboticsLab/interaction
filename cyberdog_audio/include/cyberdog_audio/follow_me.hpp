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
#ifndef CYBERDOG_AUDIO__FOLLOW_ME_HPP_
#define CYBERDOG_AUDIO__FOLLOW_ME_HPP_

#include <memory>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/action/navigation.hpp"
#include "protocol/srv/stop_algo_task.hpp"
#include "protocol/srv/ble_scan.hpp"
#include "protocol/msg/algo_task_status.hpp"
namespace cyberdog
{
namespace interaction
{
class Uwb_Tracking_Client : public rclcpp::Node
{
public:
  using Navigation = protocol::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;

  explicit Uwb_Tracking_Client(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_tracking_action_client", options), is_running(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<Navigation>(
      this,
      "start_algo_task");
    stop_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    stop_algo_task_client_ =
      this->create_client<protocol::srv::StopAlgoTask>(
      "stop_algo_task",
      rmw_qos_profile_services_default, stop_callback_group_);
    current_connected_bluetooth_client_ =
      this->create_client<protocol::srv::BLEScan>(
      "get_connected_bluetooth_info",
      rmw_qos_profile_services_default, stop_callback_group_);
    tracking_status_sub_ = this->create_subscription<protocol::msg::AlgoTaskStatus>(
      "algo_task_status", 10,
      std::bind(&Uwb_Tracking_Client::get_tracing_status, this, std::placeholders::_1));
  }
  void get_tracing_status(const protocol::msg::AlgoTaskStatus::SharedPtr msg)
  {
    tracking_status = msg->task_status;
  }
  bool is_uwb_tracking()
  {
    INFO("algor status,task_sub_status:%d", tracking_status);
    return (tracking_status == 11) ? true : false;
  }
  int get_uwb_device()
  {
    int number = 0;
    if (!current_connected_bluetooth_client_->wait_for_service(std::chrono::seconds(3))) {
      ERROR("current_connected_bluetooth_devices server not avaiable");
      return number;
    }
    auto req = std::make_shared<protocol::srv::BLEScan::Request>();
    req->scan_seconds = 0;
    auto future_result = current_connected_bluetooth_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
      auto devices = future_result.get()->device_info_list;
      number = devices.size();
    } else {
      INFO("request get_connected_bluetooth_info service failed!");
    }
    return number;
  }
  void send_goal()
  {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "Uwb tracking action server not available after waiting");
      return;
    }
    auto goal_msg = Navigation::Goal();
    goal_msg.nav_type = Navigation::Goal::NAVIGATION_TYPE_START_UWB_TRACKING;
    goal_msg.relative_pos = Navigation::Goal::TRACING_AUTO;
    RCLCPP_INFO(this->get_logger(), "Uwb Tracking Sending goal");
    auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&Uwb_Tracking_Client::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(
      &Uwb_Tracking_Client::feedback_callback, this, std::placeholders::_1,
      std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&Uwb_Tracking_Client::result_callback, this, std::placeholders::_1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    is_running.store(true);
  }

  void cancel_goal()
  {
    if (is_uwb_tracking() || is_running.load()) {
      INFO("Uwb tracking cancel goal");
      // this->client_ptr_->async_cancel_all_goals();
      if (!stop_algo_task_client_->wait_for_service(std::chrono::seconds(2))) {
        INFO("Uwb tracking service server not avalible");
      } else {
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::StopAlgoTask::Request>();
        req->task_id = protocol::srv::StopAlgoTask::Request::ALGO_TASK_UWB_TRACKING;
        auto future_result = stop_algo_task_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO("Uwb tracking service success.");
        } else {
          INFO("Uwb tracking service Failed to call stop services.");
        }
      }
    }
  }

private:
  void goal_response_callback(std::shared_future<GoalHandleNavigation::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Uwb Tracking Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Uwb Tracking Goal accepted by server, waiting for result");
    }
  }
  void feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const Navigation::Feedback> feedback)
  {
    INFO_MILLSECONDS(5000, "Uwb Tracking feedback:%d", feedback->feedback_code);
  }
  void result_callback(const GoalHandleNavigation::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Uwb tracking Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Uwb tracking Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Uwb tracking Unknown result code");
        return;
    }
    INFO("Uwb tracking Result received:%d", result.result->result);
    is_running.store(false);
  }

private:
  rclcpp_action::Client<Navigation>::SharedPtr client_ptr_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::Client<protocol::srv::StopAlgoTask>::SharedPtr stop_algo_task_client_;
  rclcpp::Client<protocol::srv::BLEScan>::SharedPtr current_connected_bluetooth_client_;
  rclcpp::Subscription<protocol::msg::AlgoTaskStatus>::SharedPtr tracking_status_sub_{nullptr};
  std::atomic_bool is_running;
  int tracking_status = 0;
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__FOLLOW_ME_HPP_
