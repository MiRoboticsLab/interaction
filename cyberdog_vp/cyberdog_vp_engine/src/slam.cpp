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

#include "cyberdog_vp_engine/slam.hpp"

namespace cyberdog_visual_programming_engine
{
Slam::Slam()
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

Slam::~Slam()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool Slam::Init(
  const rclcpp::Node::SharedPtr & _node_ptr,
  const toml::value & _params_toml)
{
  try {
    INFO("%s Initializing ...", this->logger_.c_str());
    this->node_ptr_ = _node_ptr;
    this->params_toml_ = _params_toml;
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return this->InitData();
}

bool Slam::InitData()
{
  try {
    INFO("%s Initializing data ...", this->logger_.c_str());

    this->preset_cli_cb_group_ = this->node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    this->preset_cli_ptr_ = this->node_ptr_->create_client<PresetSrv>(
      toml::find_or(
        this->params_toml_, "vp", "init", "service", "slam_preset", "map_labels"),
      ClientQos.get_rmw_qos_profile(),
      this->preset_cli_cb_group_);
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Slam::RequestPresetSrv(
  PresetSrv::Response & _response,
  std::shared_ptr<PresetSrv::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    if (rclcpp::ok()) {
      if (this->preset_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout))) {
        INFO(
          "[%s] Requesting preset service.",
          this->logger_.c_str());
        auto result = this->preset_cli_ptr_->async_send_request(_request_ptr);
        // result.wait();
        std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
        if (status == std::future_status::ready) {
          auto result_ptr = result.get();
          _response = *result_ptr;
          return true;
        } else {
          this->state_ = StateEnum::service_request_timeout;
          this->describe_ = "Waiting for preset service to response timeout.";
        }
      } else {
        this->state_ = StateEnum::service_appear_timeout;
        this->describe_ = "Waiting for preset service to appear(start) timeout.";
      }
    } else {
      this->state_ = StateEnum::service_request_interrupted;
      this->describe_ = "Client interrupted while requesting for preset service to appear.";
    }
  } catch (...) {
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "RequestPresetSrv() is failed.";
  }
  WARN(
    "[%s] %s",
    this->logger_.c_str(),
    this->describe_.c_str());
  return false;
}

bool Slam::PresetsToJson(
  const OperateMsg & _msg,
  const PresetSrv::Response & _presets,
  GRPCMsg & _frontend_msg)
{
  try {
    auto get_preset_msg = [&](const PresetMsg & _preset) -> std::string {
        return std::string(
          "In the current preset database, the " +
          _preset.label_name + "(" +
          std::to_string(_preset.physic_x) + ", " +
          std::to_string(_preset.physic_y) + ", 0.0) preset is in " +
          std::string(_presets.label.is_outdoor ? "outdoor" : "indoor") + " map " +
          _presets.label.map_name + ".");
      };
    rapidjson::Document robotend_json;
    robotend_json.SetObject();
    rapidjson::Document::AllocatorType & allocator = robotend_json.GetAllocator();
    rapidjson::Value response_json(rapidjson::kObjectType);
    response_json.SetObject();
    response_json.AddMember("type", rapidjson::StringRef(_msg.type.c_str()), allocator);
    response_json.AddMember("id", rapidjson::StringRef(_msg.id.c_str()), allocator);
    auto get_value = [&](const std::string & _value) -> rapidjson::Value {
        rapidjson::Value ret;
        ret.SetString(_value.c_str(), _value.length(), allocator);
        return ret;
      };
    rapidjson::Value list_json(rapidjson::kArrayType);
    if (_presets.success == PresetSrv::Response::RESULT_SUCCESS) {
      if (_msg.mode == OperateMsg::MODE_PRESET) {
        for (const PresetMsg & now_meta : _presets.label.labels) {
          std::string preset_style = "[" +
            std::to_string(now_meta.physic_x) + ", " +
            std::to_string(now_meta.physic_y) + ", 0.0]";

          rapidjson::Value now_meta_json(rapidjson::kObjectType);
          now_meta_json.AddMember(
            "id",
            get_value(now_meta.label_name), allocator);
          now_meta_json.AddMember(
            "mode",
            get_value(_msg.mode), allocator);
          now_meta_json.AddMember(
            "style",
            get_value(preset_style), allocator);
          now_meta_json.AddMember(
            "condition",
            get_value(_presets.label.map_name), allocator);
          now_meta_json.AddMember(
            "describe",
            get_value(get_preset_msg(now_meta)), allocator);
          list_json.PushBack(now_meta_json, allocator);
        }
      }
    }
    response_json.AddMember("list", list_json, allocator);
    robotend_json.AddMember("response", response_json, allocator);
    CyberdogJson::Document2String(robotend_json, _frontend_msg.data);
  } catch (const std::exception & e) {
    WARN(
      "Error presets to json message:\nerror:%s",
      e.what());
    return false;
  }
  return true;
}

bool Slam::RespondToRequests(const OperateMsg & _msg, GRPCMsg & msg_)
{
  this->state_ = StateEnum::normally;
  this->describe_ = "normally";
  try {
    if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
      std::shared_ptr<PresetSrv::Request> request_ptr =
        std::make_shared<PresetSrv::Request>();
      request_ptr->map_name = "";
      PresetSrv::Response presets;
      this->RequestPresetSrv(presets, request_ptr, 5);
      this->PresetsToJson(_msg, presets, msg_);
    }
    if (msg_.data.empty()) {
      this->getRobotendMsg(_msg, msg_);
    }
  } catch (const std::exception & e) {
    WARN(
      "Error respond to requests to frontend message:\nerror:%s",
      e.what());
    return false;
  }
  return true;
}

void Slam::getRobotendMsg(const OperateMsg & _msg, GRPCMsg & msg_)
{
  // {
  //     "feedback": {
  //       "type": "type",
  //       "id": "id",
  //       "target_id": "target_id",
  //       "operate": "operate",
  //       "state": 0,
  //       "describe": "describe"
  //     }
  // }
  std::string target_id = (_msg.target_id.empty()) ? "" : _msg.target_id.front();
  rapidjson::Document task_json(rapidjson::kObjectType);
  CyberdogJson::Add(task_json, "type", _msg.type);
  CyberdogJson::Add(task_json, "id", _msg.id);
  CyberdogJson::Add(task_json, "target_id", target_id);
  CyberdogJson::Add(task_json, "operate", _msg.operate);
  CyberdogJson::Add(task_json, "state", static_cast<int>(this->state_));
  CyberdogJson::Add(task_json, "describe", this->describe_);
  rapidjson::Document robotend_json(rapidjson::kObjectType);
  CyberdogJson::Add(robotend_json, "feedback", task_json);
  CyberdogJson::Document2String(robotend_json, msg_.data);
}
}   // namespace cyberdog_visual_programming_engine
