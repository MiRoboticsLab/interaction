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
#include <map>

#include "cyberdog_vp_abilityset/navigation.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Navigation::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Navigation::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->algo_status_sub_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assisted_relocation_timer_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->get_preset_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->cancel_navigation_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->navigation_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->algo_status_sub_cb_group_;
    this->algo_status_sub_ptr_ = this->node_mortal_ptr_->create_subscription<MsgAlgoStatus>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "slam_navigation", "algo_task_status"),
      SubscriptionQos,
      std::bind(&Navigation::SubAlgoStatusCB, this, std::placeholders::_1),
      sub_option);

    double assisted_relocation_hz = 0.3;
    this->assisted_relocation_timer_ = this->node_mortal_ptr_->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / assisted_relocation_hz)),
      std::bind(&Navigation::AssistedRelocation, this),
      this->assisted_relocation_timer_cb_group_);
    this->assisted_relocation_timer_->cancel();

    this->get_preset_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvGetPreset>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "slam_preset", "map_labels"),
      ClientQos.get_rmw_qos_profile(),
      this->get_preset_cli_cb_group_);
    this->cancel_navigation_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvCancelNavigation>(
      toml::find_or(
        _params_toml, "vp", "init", "service", "slam_navigation", "stop_algo_task"),
      ClientQos.get_rmw_qos_profile(),
      this->cancel_navigation_cli_cb_group_);
    this->navigation_cli_ptr_ = rclcpp_action::create_client<ActNavigation>(
      this->node_mortal_ptr_,
      toml::find_or(
        _params_toml, "vp", "init", "action", "slam_navigation", "start_algo_task"),
      this->navigation_cli_cb_group_);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Navigation::InitDependent(
  const std::function<MotionServoCmdResponse(const double, double)> & _Turn,
  const std::function<AudioPlaySeviceResponse(const std::string, const int8_t)> & _Play,
  const std::function<State(const std::string, const int8_t)> & _InstantlyPlay)
{
  this->FTurn = _Turn;
  this->FPlay = _Play;
  this->FInstantlyPlay = _InstantlyPlay;
}

void Navigation::SubAlgoStatusCB(const MsgAlgoStatus::SharedPtr _msg_ptr)
{
  this->algo_status_ = *_msg_ptr;
}

void Navigation::AssistedRelocation()
{
  if (this->assisted_relocation_interact_) {
    this->FInstantlyPlay("请注意避让，让我看看周围环境", this->assisted_relocation_volume_);
  }
  this->FTurn(60, 2);
}

SrvGetPreset::Response Navigation::RequestPresetSrv(
  int _service_start_timeout)
{
  SrvGetPreset::Response ret;
  try {
    if (rclcpp::ok()) {
      if (this->get_preset_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout)))
      {
        Debug(
          "[%s] Requesting preset service.",
          this->logger_.c_str());
        std::shared_ptr<SrvGetPreset::Request> request_ptr =
          std::make_shared<SrvGetPreset::Request>();
        request_ptr->map_name = "";
        request_ptr->map_id = 0;
        auto result = this->get_preset_cli_ptr_->async_send_request(request_ptr);
        // result.wait();
        std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
        if (status == std::future_status::ready) {
          auto result_ptr = result.get();
          ret = *result_ptr;
        } else {
          this->state_.code = StateCode::service_request_timeout;
          Warn(
            "[%s] Waiting for preset service to response timeout.",
            this->logger_.c_str());
        }
      } else {
        this->state_.code = StateCode::service_appear_timeout;
        Warn(
          "[%s] Waiting for preset service to appear(start) timeout.",
          this->logger_.c_str());
      }
    } else {
      this->state_.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for preset service to appear.",
        this->logger_.c_str());
    }
  } catch (...) {
    this->state_.code = StateCode::fail;
    Error("[%s] RequestPresetSrv() is failed.", this->logger_.c_str());
  }
  return ret;
}

SrvCancelNavigation::Response Navigation::RequestCancelSrv(
  const std::shared_ptr<SrvCancelNavigation::Request> _request_ptr,
  int _service_start_timeout)
{
  SrvCancelNavigation::Response ret;
  try {
    if (rclcpp::ok()) {
      if (this->cancel_navigation_cli_ptr_->wait_for_service(
          std::chrono::seconds(
            _service_start_timeout)))
      {
        Debug(
          "[%s] Requesting cancel navigation service.",
          this->logger_.c_str());
        auto result = this->cancel_navigation_cli_ptr_->async_send_request(_request_ptr);
        // result.wait();
        std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
        if (status == std::future_status::ready) {
          auto result_ptr = result.get();
          ret = *result_ptr;
        } else {
          this->state_.code = StateCode::service_request_timeout;
          Warn(
            "[%s] Waiting for cancel navigation service to response timeout.",
            this->logger_.c_str());
        }
      } else {
        this->state_.code = StateCode::service_appear_timeout;
        Warn(
          "[%s] Waiting for cancel navigation service to appear(start) timeout.",
          this->logger_.c_str());
      }
    } else {
      this->state_.code = StateCode::service_request_interrupted;
      Warn(
        "[%s] Client interrupted while requesting for cancel navigation service to appear.",
        this->logger_.c_str());
    }
  } catch (...) {
    this->state_.code = StateCode::fail;
    Error("[%s] RequestCancelSrv() is failed.", this->logger_.c_str());
  }
  return ret;
}

void Navigation::NavigationFeedbackCB(
  rclcpp_action::Client<ActNavigation>::GoalHandle::SharedPtr goal_handel_ptr,
  const std::shared_ptr<const ActNavigation::Feedback> feedback_ptr)
{
  try {
    std::hash<rclcpp_action::GoalUUID> goal_id_hash_fun;
    size_t goal_hash = goal_id_hash_fun(goal_handel_ptr->get_goal_id());
    Info(
      "Navigation:\n\tgoal_id_hash:%ld\n\tfeedback_code:%d\n\tfeedback_msg:%s",
      goal_hash,
      feedback_ptr->feedback_code,
      feedback_ptr->feedback_msg.c_str());
    if (feedback_ptr->feedback_code == 3107) {
      if (this->assisted_relocation_) {
        if (this->assisted_relocation_interact_) {
          this->FPlay("尝试定位失败，即将开始辅助定位", this->assisted_relocation_volume_);
        }
        this->assisted_relocation_timer_->reset();
      }
    } else if (!this->assisted_relocation_timer_->is_canceled()) {
      this->assisted_relocation_timer_->cancel();
    }
    if (this->assisted_relocation_interact_) {
      switch (feedback_ptr->feedback_code) {
        case 3106:
          this->FPlay("定位超时", this->assisted_relocation_volume_);
          break;
        case 3108:
          this->FPlay("定位成功", this->assisted_relocation_volume_);
          break;
        case 3109:
          this->FPlay("定位失败", this->assisted_relocation_volume_);
          break;
        default:
          break;
      }
    }
  } catch (const std::exception & e) {
    Error(
      "[%s] NavigationFeedbackCB() is failed: %s.",
      this->logger_.c_str(),
      e.what());
  }
}

ActNavigation::Result Navigation::RequestNavigationAct(
  const ActNavigation::Goal & _goal,
  int _timeout)
{
  ActNavigation::Result ret;
  ret.result = ActNavigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  try {
    if (rclcpp::ok()) {
      Debug(
        "[%s] Requesting navigation action.",
        this->logger_.c_str());
      rclcpp_action::Client<ActNavigation>::SendGoalOptions goal_options;
      goal_options.feedback_callback =
        std::bind(
        &Navigation::NavigationFeedbackCB, this, std::placeholders::_1,
        std::placeholders::_2);
      auto goal_handle = this->navigation_cli_ptr_->async_send_goal(_goal, goal_options);
      // result.wait();
      if (goal_handle.wait_for(std::chrono::seconds(_timeout)) == std::future_status::ready) {
        if (goal_handle.get() != nullptr) {
          auto result =
            this->navigation_cli_ptr_->async_get_result(goal_handle.get());
          if (result.wait_for(std::chrono::seconds(_timeout)) == std::future_status::ready) {
            auto result_ptr = result.get().result;
            ret = *result_ptr;
            switch (result_ptr->result) {
              case ActNavigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS:
                Info("Navigation to is success.");
                break;
              case ActNavigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT:
                Warn("Navigation to is accept.");
                break;
              case ActNavigation::Result::NAVIGATION_RESULT_TYPE_UNAVALIBLE:
                Warn("Navigation to is unsvslible.");
                break;
              case ActNavigation::Result::NAVIGATION_RESULT_TYPE_FAILED:
                Warn("Navigation to is failed.");
                break;
              case ActNavigation::Result::NAVIGATION_RESULT_TYPE_REJECT:
                Warn("Navigation to is perject.");
                break;
              case ActNavigation::Result::NAVIGATION_RESULT_TYPE_CANCEL:
                Warn("Navigation to is cancel.");
                break;
              default:
                Warn("Navigation result is undefault.");
                break;
            }
          } else {
            this->state_.code = StateCode::action_result_timeout;
            Warn(
              "[%s] Waiting for navigation action to result timeout.",
              this->logger_.c_str());
          }
        } else {
          this->state_.code = StateCode::action_request_rejected;
          Warn(
            "[%s] Waiting for navigation action to request rejected.",
            this->logger_.c_str());
        }
      } else {
        this->state_.code = StateCode::action_request_timeout;
        Warn(
          "[%s] Waiting for navigation action to request timeout.",
          this->logger_.c_str());
      }
    }
  } catch (...) {
    this->state_.code = StateCode::fail;
    Error("[%s] RequestNavigationAct() is failed.", this->logger_.c_str());
  }
  return ret;
}

MapPresetSeviceResponse Navigation::GetPreset(const int _timeout)
{
  transient_state_.code = StateCode::success;
  MapPresetSeviceResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d)",
    _timeout);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  SrvGetPreset::Response response =
    this->RequestPresetSrv(static_cast<int>((_timeout > 0) ? _timeout : 5));
  ret.state = this->GetState(
    funs,
    (response.success == SrvGetPreset::Response::RESULT_SUCCESS) ?
    StateCode::success : StateCode::fail);
  ret.map_name = response.label.map_name;
  ret.is_outdoor = response.label.is_outdoor;
  ret.list = response.label.labels;
  for (const MsgPreset & meta : ret.list) {
    ret.dictionary.insert(
      std::map<std::string, MsgPreset>::value_type(
        meta.label_name, meta));
  }
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::TurnOnNavigation(
  const bool _outdoor,
  const bool _assisted_relocation,
  const bool _interact,
  const int _volume)
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%d, %d, %d, %d)",
    _outdoor, _assisted_relocation, _interact, _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  auto target_algorithm_ready = [&]() -> int {  // 0:空闲-自启 1:已启-就绪 2:占用冲突
      Info("Algo status is %d.", this->algo_status_.task_status);
      if (this->algo_status_.task_status == 101) {
        return 0;
      }
      if ((_outdoor && (this->algo_status_.task_status == 18)) ||   // 视觉定位成功
        (!_outdoor && (this->algo_status_.task_status == 8)))       // 激光定位成功
      {
        return 1;
      }
      return 2;
    };
  int algorithm_ready = target_algorithm_ready();
  if (algorithm_ready == 0) {
    this->assisted_relocation_ = _assisted_relocation;
    this->assisted_relocation_interact_ = _interact;
    this->assisted_relocation_volume_ = _volume;
    ActNavigation::Goal goal;
    goal.nav_type = ActNavigation::Goal::NAVIGATION_TYPE_START_LOCALIZATION;
    goal.outdoor = _outdoor;
    ret.response = this->RequestNavigationAct(goal);
    ret.state = this->GetState(
      funs, (ret.response.result ==
      ActNavigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS) ?
      StateCode::success : StateCode::fail);
  } else if (algorithm_ready == 1) {
    Info("%s The current navigation algorithm is ready.", funs.c_str());
    ret.state = this->GetState(funs, StateCode::success);
    ret.response.result = ActNavigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
  } else {
    Warn(
      "[%s] The current navigation algorithm status is {task_status=%d; task_sub_status=%d}, "
      "and there is an occupation conflict with the current request(outdoor=%d).",
      funs.c_str(),
      this->algo_status_.task_status,
      this->algo_status_.task_sub_status,
      _outdoor);
    ret.state = this->GetState(funs, StateCode::fail);
    ret.response.result = ActNavigation::Result::NAVIGATION_RESULT_TYPE_UNAVALIBLE;
  }
  this->assisted_relocation_ = false;
  this->assisted_relocation_interact_ = false;
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::TurnOffNavigation()
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  // ActNavigation::Goal goal;
  // goal.nav_type = ActNavigation::Goal::NAVIGATION_TYPE_STOP_LOCALIZATION;
  // goal.outdoor = false;
  // ret.response = this->RequestNavigationAct(goal);
  // ret.state = this->GetState(
  //   funs, (ret.response.result ==
  //   ActNavigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS) ?
  //   StateCode::success : StateCode::fail);
  std::shared_ptr<SrvCancelNavigation::Request> request_ptr =
    std::make_shared<SrvCancelNavigation::Request>();
  request_ptr->task_id = SrvCancelNavigation::Request::ALGO_TASK_ALL;
  request_ptr->map_name = "";
  SrvCancelNavigation::Response ret_response = this->RequestCancelSrv(request_ptr, 30);
  ret.response.result = ret_response.result;
  ret.state = this->GetState(
    funs, (ret.response.result ==
    SrvCancelNavigation::Response::SUCCESS) ? StateCode::success : StateCode::fail);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::NavigationToPreset(
  const std::string _preset_name)
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string preset_name = '%s')",
    _preset_name.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  MapPresetSeviceResponse preset = this->GetPreset();
  if (preset.dictionary.count(_preset_name)) {
    ret = this->NavigationToCoordinates(
      preset.is_outdoor,
      static_cast<double>(preset.dictionary[_preset_name].physic_x),
      static_cast<double>(preset.dictionary[_preset_name].physic_y));
  } else {
    Warn("%s is failed, target(%s) lable is not find.", funs.c_str(), _preset_name.c_str());
    ret.state = this->GetState(funs, StateCode::fail);
    ret.response.result = 3;
  }
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::NavigationToCoordinates(
  const bool is_outdoor,
  const double _x,
  const double _y,
  const double _z,
  const double _roll,
  const double _pitch,
  const double _yaw)
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%lf, %lf, %lf, %lf, %lf, %lf)", _x, _y,
    _z, _roll, _pitch, _yaw);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  MsgPoseStamped target;
  target.header.stamp = this->node_mortal_ptr_->now();
  target.header.frame_id = this->logger_;
  target.pose.position.x = _x;
  target.pose.position.y = _y;
  target.pose.position.z = _z;
  target.pose.orientation = RPY2Qrientation(_roll, _pitch, _yaw);
  ActNavigation::Goal goal;
  goal.nav_type = ActNavigation::Goal::NAVIGATION_TYPE_START_AB;
  goal.outdoor = is_outdoor;
  goal.poses.push_back(target);
  ret.response = this->RequestNavigationAct(goal);
  ret.state = this->GetState(
    funs, (ret.response.result ==
    ActNavigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS) ? StateCode::success : StateCode::fail);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::NavigationToPose(const MsgPose & _pose)
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "({(x=%lf, y=%lf, z=%lf), (x=%lf, y=%lf, z=%lf, w=%lf)})",
    _pose.position.x, _pose.position.y, _pose.position.z,
    _pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  MsgPoseStamped target;
  target.header.stamp = this->node_mortal_ptr_->now();
  target.header.frame_id = this->logger_;
  target.pose = _pose;
  ActNavigation::Goal goal;
  goal.nav_type = ActNavigation::Goal::NAVIGATION_TYPE_START_AB;
  goal.poses.push_back(target);
  ret.response = this->RequestNavigationAct(goal);
  ret.state = this->GetState(
    funs, (ret.response.result ==
    ActNavigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS) ? StateCode::success : StateCode::fail);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::CancelNavigation(const int _timeout)
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  std::shared_ptr<SrvCancelNavigation::Request> request_ptr =
    std::make_shared<SrvCancelNavigation::Request>();
  request_ptr->task_id = SrvCancelNavigation::Request::ALGO_TASK_AB;
  request_ptr->map_name = "";
  SrvCancelNavigation::Response ret_response =
    this->RequestCancelSrv(request_ptr, static_cast<int>((_timeout > 0) ? _timeout : 5));
  ret.response.result = ret_response.result;
  ret.state = this->GetState(
    funs, (ret.response.result ==
    SrvCancelNavigation::Response::SUCCESS) ? StateCode::success : StateCode::fail);
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

NavigationActionResponse Navigation::ToPreset(
  const std::string _preset_name,
  const bool _assisted_relocation,
  const bool _interact,
  const int _volume)
{
  transient_state_.code = StateCode::success;
  NavigationActionResponse ret;
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %d, %d, %d)",
    _preset_name.c_str(), _assisted_relocation, _interact, _volume);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    ret.state = this->GetState(funs, this->state_.code);
    transient_state_ = ret.state;
    return ret;
  }
  MapPresetSeviceResponse preset = this->GetPreset();
  if (preset.dictionary.count(_preset_name)) {
    ret = this->TurnOnNavigation(
      preset.is_outdoor,
      _assisted_relocation, _interact,
      _volume);
    if (ret.state.code == StateCode::success) {
      ret = this->NavigationToCoordinates(
        preset.is_outdoor,
        static_cast<double>(preset.dictionary[_preset_name].physic_x),
        static_cast<double>(preset.dictionary[_preset_name].physic_y));
    }
  } else {
    Warn("%s is failed, target(%s) lable is not find.", funs.c_str(), _preset_name.c_str());
    ret.state = this->GetState(funs, StateCode::fail);
    ret.response.result = 3;
  }
  if (ret.state.code != StateCode::success) {
    transient_state_ = ret.state;
  }
  return ret;
}

State Navigation::AddPreset(const std::string _preset_name)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string preset_name = '%s')",
    _preset_name.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  // code ***
  return this->GetState(funs, StateCode::success);
}

State Navigation::DeletePreset(const std::string _preset_name)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string preset_name = '%s')",
    _preset_name.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  // code ***
  return this->GetState(funs, StateCode::success);
}
}   // namespace cyberdog_visual_programming_abilityset
