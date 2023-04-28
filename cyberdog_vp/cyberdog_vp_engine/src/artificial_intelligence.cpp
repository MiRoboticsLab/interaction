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

#include "cyberdog_vp_engine/artificial_intelligence.hpp"

namespace cyberdog_visual_programming_engine
{
ArtificialIntelligence::ArtificialIntelligence()
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

ArtificialIntelligence::~ArtificialIntelligence()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool ArtificialIntelligence::Init(
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

bool ArtificialIntelligence::InitData()
{
  try {
    INFO("%s Initializing data ...", this->logger_.c_str());

    this->personnel_cli_cb_group_ = this->node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->training_words_cli_cb_group_ = this->node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    this->personnel_cli_ptr_ = this->node_ptr_->create_client<PersonnelSrv>(
      toml::find_or(
        this->params_toml_, "vp", "init", "service", "personnel", "all_user_search"),
      ClientQos.get_rmw_qos_profile(),
      this->personnel_cli_cb_group_);
    this->training_words_cli_ptr_ = this->node_ptr_->create_client<TrainingWordsSrv>(
      toml::find_or(
        this->params_toml_, "vp", "init", "service", "training_words", "query_all_train_plan"),
      ClientQos.get_rmw_qos_profile(),
      this->personnel_cli_cb_group_);
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool ArtificialIntelligence::RequestPersonnelSrv(
  PersonnelSrv::Response & _response,
  std::shared_ptr<PersonnelSrv::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    if (rclcpp::ok()) {
      if (this->personnel_cli_ptr_->wait_for_service(std::chrono::seconds(_service_start_timeout)))
      {
        INFO(
          "[%s] Requesting personnel service.",
          this->logger_.c_str());
        auto result = this->personnel_cli_ptr_->async_send_request(_request_ptr);
        // result.wait();
        std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
        if (status == std::future_status::ready) {
          auto result_ptr = result.get();
          _response = *result_ptr;
          INFO(
            "[%s] Requesting personnel service response code is %d.",
            this->logger_.c_str(), result_ptr->code);
          return true;
        } else {
          this->state_ = StateEnum::service_request_timeout;
          this->describe_ = "Waiting for personnel service to response timeout.";
        }
      } else {
        this->state_ = StateEnum::service_appear_timeout;
        this->describe_ = "Waiting for personnel service to appear(start) timeout.";
      }
    } else {
      this->state_ = StateEnum::service_request_interrupted;
      this->describe_ = "Client interrupted while requesting for personnel service to appear.";
    }
  } catch (...) {
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "RequestPersonnelSrv() is failed.";
  }
  WARN(
    "[%s] %s",
    this->logger_.c_str(),
    this->describe_.c_str());
  return false;
}

bool ArtificialIntelligence::PersonnelsToJson(
  const OperateMsg & _msg,
  const PersonnelSrv::Response & _personnels,
  GRPCMsg & _frontend_msg)
{
  try {
    auto get_meta_msg = [&](
      const std::string & _msg_head,
      const uint _state) -> std::string {   // 请求
        std::string now_msg = _msg_head;
        switch (_state) {
          case 0:   // 未录入
            now_msg += "has not been recorded";
            break;
          case 1:   // 已录入
            now_msg += "has been recorded";
            break;
          case 2:   // 正在录入
            now_msg += "is being recorded";
            break;
          default:  // 位置状态，不可用
            now_msg += "unknown status, unavailable";
            break;
        }
        return now_msg;
      };
    auto get_personnel_msg = [&](const PersonnelMsg & _personnel) -> std::string {
        return std::string(
          "In the current personnel database,  " +
          _personnel.username + " " +
          get_meta_msg("voiceprint features ", _personnel.voicestatus) + ", " +
          get_meta_msg("facial features ", _personnel.facestatus) + ".");
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
    if (!_personnels.code) {
      for (const PersonnelMsg & now_meta : _personnels.result) {
        if (((_msg.mode == OperateMsg::MODE_FACE) &&
          (now_meta.facestatus == 1)) ||
          ((_msg.mode == OperateMsg::MODE_VOICEPRINT) &&
          (now_meta.voicestatus == 1)) ||
          (_msg.mode == OperateMsg::MODE_PERSONNEL))
        {
          rapidjson::Value now_meta_json(rapidjson::kObjectType);
          now_meta_json.AddMember(
            "id",
            get_value(std::to_string(now_meta.id)), allocator);
          now_meta_json.AddMember(
            "mode",
            get_value(_msg.mode), allocator);
          now_meta_json.AddMember(
            "style",
            get_value(std::to_string(now_meta.facestatus)), allocator);
          now_meta_json.AddMember(
            "condition",
            get_value(std::to_string(now_meta.voicestatus)), allocator);
          now_meta_json.AddMember(
            "describe",
            get_value(now_meta.username), allocator);
          list_json.PushBack(now_meta_json, allocator);
          INFO(
            "%s %s",
            this->logger_.c_str(),
            get_personnel_msg(now_meta).c_str());
        }
      }
    }
    response_json.AddMember("list", list_json, allocator);
    robotend_json.AddMember("response", response_json, allocator);
    CyberdogJson::Document2String(robotend_json, _frontend_msg.data);
  } catch (const std::exception & e) {
    WARN(
      "%s Error personnels to json message:\nerror:%s",
      this->logger_.c_str(),
      e.what());
    return false;
  }
  return true;
}

bool ArtificialIntelligence::RequestTrainingWordsSrv(
  TrainingWordsSrv::Response & _response,
  std::shared_ptr<TrainingWordsSrv::Request> _request_ptr,
  int _service_start_timeout)
{
  try {
    if (rclcpp::ok()) {
      if (this->training_words_cli_ptr_->wait_for_service(
          std::chrono::seconds(
            _service_start_timeout)))
      {
        INFO(
          "[%s] Requesting personnel service.",
          this->logger_.c_str());
        auto result = this->training_words_cli_ptr_->async_send_request(_request_ptr);
        // result.wait();
        std::future_status status = result.wait_for(std::chrono::seconds(_service_start_timeout));
        if (status == std::future_status::ready) {
          auto result_ptr = result.get();
          _response = *result_ptr;
          return true;
        } else {
          this->state_ = StateEnum::service_request_timeout;
          this->describe_ = "Waiting for personnel service to response timeout.";
        }
      } else {
        this->state_ = StateEnum::service_appear_timeout;
        this->describe_ = "Waiting for personnel service to appear(start) timeout.";
      }
    } else {
      this->state_ = StateEnum::service_request_interrupted;
      this->describe_ = "Client interrupted while requesting for personnel service to appear.";
    }
  } catch (...) {
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "RequestPersonnelSrv() is failed.";
  }
  WARN(
    "[%s] %s",
    this->logger_.c_str(),
    this->describe_.c_str());
  return false;
}

bool ArtificialIntelligence::TrainingWordsToJson(
  const OperateMsg & _msg,
  const TrainingWordsSrv::Response & _training_words,
  GRPCMsg & _frontend_msg)
{
  try {
    auto get_training_words_msg = [&](const TrainingWordsMsg & _training_words) -> std::string {
        return std::string(
          "The current training word is '" +
          _training_words.trigger + "', which is of type '" +
          _training_words.type + "' and bound to the action '" +
          _training_words.value + "'.");
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
    if (!_training_words.training_set.empty()) {
      for (const TrainingWordsMsg & now_meta : _training_words.training_set) {
        rapidjson::Value now_meta_json(rapidjson::kObjectType);
        now_meta_json.AddMember(
          "id",
          get_value(now_meta.trigger), allocator);
        now_meta_json.AddMember(
          "mode",
          get_value(_msg.mode), allocator);
        now_meta_json.AddMember(
          "style",
          get_value(now_meta.type), allocator);
        now_meta_json.AddMember(
          "condition",
          get_value(now_meta.value), allocator);
        now_meta_json.AddMember(
          "describe",
          get_value(get_training_words_msg(now_meta)), allocator);
        list_json.PushBack(now_meta_json, allocator);
      }
    }
    response_json.AddMember("list", list_json, allocator);
    robotend_json.AddMember("response", response_json, allocator);
    CyberdogJson::Document2String(robotend_json, _frontend_msg.data);
  } catch (const std::exception & e) {
    WARN(
      "Error personnels to json message:\nerror:%s",
      e.what());
    return false;
  }
  return true;
}

bool ArtificialIntelligence::RespondToRequests(const OperateMsg & _msg, GRPCMsg & msg_)
{
  this->state_ = StateEnum::normally;
  this->describe_ = "normally";
  try {
    if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
      if ((_msg.mode == OperateMsg::MODE_PERSONNEL) ||
        (_msg.mode == OperateMsg::MODE_FACE) ||
        (_msg.mode == OperateMsg::MODE_VOICEPRINT))
      {
        PersonnelSrv::Response personnels;
        std::shared_ptr<PersonnelSrv::Request> request_ptr =
          std::make_shared<PersonnelSrv::Request>();
        request_ptr->command = "";
        this->RequestPersonnelSrv(personnels, request_ptr, 5);
        this->PersonnelsToJson(_msg, personnels, msg_);
      } else if (_msg.mode == OperateMsg::MODE_TAAINING_WORDS) {
        TrainingWordsSrv::Response training_words;
        this->RequestTrainingWordsSrv(
          training_words,
          std::make_shared<TrainingWordsSrv::Request>(), 5);
        this->TrainingWordsToJson(_msg, training_words, msg_);
      } else if (_msg.mode == OperateMsg::MODE_ALL) {
        PersonnelSrv::Response personnels;
        std::shared_ptr<PersonnelSrv::Request> request_ptr =
          std::make_shared<PersonnelSrv::Request>();
        request_ptr->command = "";
        this->RequestPersonnelSrv(personnels, request_ptr, 5);

        TrainingWordsSrv::Response training_words;
        this->RequestTrainingWordsSrv(
          training_words,
          std::make_shared<TrainingWordsSrv::Request>(), 5);

        this->AIToJson(_msg, personnels, training_words, msg_);
      }
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

bool ArtificialIntelligence::AIToJson(
  const OperateMsg & _msg,
  const PersonnelSrv::Response & _personnels,
  const TrainingWordsSrv::Response & _training_words,
  GRPCMsg & _frontend_msg)
{
  try {
    auto get_meta_msg = [&](
      const std::string & _msg_head,
      const uint _state) -> std::string {   // 请求
        std::string now_msg = _msg_head;
        switch (_state) {
          case 0:   // 未录入
            now_msg += "has not been recorded";
            break;
          case 1:   // 已录入
            now_msg += "has been recorded";
            break;
          case 2:   // 正在录入
            now_msg += "is being recorded";
            break;
          default:  // 位置状态，不可用
            now_msg += "unknown status, unavailable";
            break;
        }
        return now_msg;
      };
    auto get_personnel_msg = [&](const PersonnelMsg & _personnel) -> std::string {
        return std::string(
          "In the current personnel database,  " +
          _personnel.username + " " +
          get_meta_msg("voiceprint features ", _personnel.voicestatus) + ", " +
          get_meta_msg("facial features ", _personnel.facestatus) + ".");
      };
    auto get_training_words_msg = [&](const TrainingWordsMsg & _training_words) -> std::string {
        return std::string(
          "The current training word is '" +
          _training_words.trigger + "', which is of type '" +
          _training_words.type + "' and bound to the action '" +
          _training_words.value + "'.");
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
    if (!_personnels.code) {
      for (const PersonnelMsg & now_meta : _personnels.result) {
        rapidjson::Value now_meta_json(rapidjson::kObjectType);
        now_meta_json.AddMember(
          "id",
          get_value(std::to_string(now_meta.id)), allocator);
        now_meta_json.AddMember(
          "mode",
          get_value(OperateMsg::MODE_PERSONNEL), allocator);
        now_meta_json.AddMember(
          "style",
          get_value(std::to_string(now_meta.facestatus)), allocator);
        now_meta_json.AddMember(
          "condition",
          get_value(std::to_string(now_meta.voicestatus)), allocator);
        now_meta_json.AddMember(
          "describe",
          get_value(now_meta.username), allocator);
        list_json.PushBack(now_meta_json, allocator);
        INFO(
          "%s %s",
          this->logger_.c_str(),
          get_personnel_msg(now_meta).c_str());
      }
    }
    if (!_training_words.training_set.empty()) {
      for (const TrainingWordsMsg & now_meta : _training_words.training_set) {
        rapidjson::Value now_meta_json(rapidjson::kObjectType);
        now_meta_json.AddMember(
          "id",
          get_value(now_meta.trigger), allocator);
        now_meta_json.AddMember(
          "mode",
          get_value(OperateMsg::MODE_TAAINING_WORDS), allocator);
        now_meta_json.AddMember(
          "style",
          get_value(now_meta.type), allocator);
        now_meta_json.AddMember(
          "condition",
          get_value(now_meta.value), allocator);
        now_meta_json.AddMember(
          "describe",
          get_value(get_training_words_msg(now_meta)), allocator);
        list_json.PushBack(now_meta_json, allocator);
      }
    }
    response_json.AddMember("list", list_json, allocator);
    robotend_json.AddMember("response", response_json, allocator);
    CyberdogJson::Document2String(robotend_json, _frontend_msg.data);
  } catch (const std::exception & e) {
    WARN(
      "Error personnels to json message:\nerror:%s",
      e.what());
    return false;
  }
  return true;
}

void ArtificialIntelligence::getRobotendMsg(const OperateMsg & _msg, GRPCMsg & msg_)
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
