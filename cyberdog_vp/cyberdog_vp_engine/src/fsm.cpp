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

#include "cyberdog_vp_engine/fsm.hpp"

namespace cyberdog_visual_programming_engine
{
Fsm::Fsm()
: cyberdog::machine::MachineActuator("vp_engine")
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

Fsm::~Fsm()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool Fsm::Init(
  const rclcpp::Node::SharedPtr & _node_ptr,
  const toml::value & _params_toml)
{
  try {
    INFO("%s Initializing ...", this->logger_.c_str());
    this->node_ptr_ = _node_ptr;
    this->params_toml_ = _params_toml;
    if (!this->MachineActuatorInit(
        std::string(
          ament_index_cpp::get_package_share_directory("params") +
          std::string("/toml_config/manager/state_machine_config.toml")),
        this->node_ptr_))
    {
      ERROR("Init failed, actuator init error.");
      return false;
    }
    this->RegisterStateCallback(OperateMsg::FSM_SET_UP, std::bind(&Fsm::OnSetUp, this));
    this->RegisterStateCallback(OperateMsg::FSM_TEAR_DOWN, std::bind(&Fsm::ONTearDown, this));
    this->RegisterStateCallback(OperateMsg::FSM_SELF_CHECK, std::bind(&Fsm::OnSelfCheck, this));
    this->RegisterStateCallback(OperateMsg::FSM_ACTIVE, std::bind(&Fsm::OnActive, this));
    this->RegisterStateCallback(OperateMsg::FSM_DE_ACTIVE, std::bind(&Fsm::OnDeActive, this));
    this->RegisterStateCallback(OperateMsg::FSM_PROTECTED, std::bind(&Fsm::OnProtected, this));
    this->RegisterStateCallback(OperateMsg::FSM_LOW_POWER, std::bind(&Fsm::OnLowPower, this));
    this->RegisterStateCallback(OperateMsg::FSM_OTA, std::bind(&Fsm::OnOTA, this));
    this->RegisterStateCallback(OperateMsg::FSM_ERROR, std::bind(&Fsm::OnError, this));
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return this->InitData() && this->ActuatorStart();
}

bool Fsm::InitData()
{
  try {
    INFO("%s Initializing data ...", this->logger_.c_str());

    this->timer_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->pub_info_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->pub_audio_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->update_info_timer_ = this->node_ptr_->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / 5)),
      std::bind(&Fsm::UpdateInfo, this),
      this->timer_cb_group_);

    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->pub_info_cb_group_;
    this->info_pub_ = this->node_ptr_->create_publisher<OperateMsg>(
      toml::find_or(
        this->params_toml_, "vp", "init", "topic", "fsm", "cyberdog_vp_engine_fsm"),
      PublisherQos,
      pub_option);

    pub_option.callback_group = this->pub_audio_cb_group_;
    this->audio_pub_ = this->node_ptr_->create_publisher<AudioPlayExtendMsg>(
      toml::find_or(
        this->params_toml_, "vp", "init", "topic", "audio", "speech_play_extend"),
      PublisherQos,
      pub_option);

    this->info_.type = OperateMsg::TYPE_TASK;
    this->info_.id = "cyberdog_visual_programming_engine_fsm";
    this->info_.fsm = OperateMsg::FSM_UNINITIALIZED;
    this->info_.describe = "未初始化";
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

int32_t Fsm::OnSetUp()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_SET_UP;
  this->info_.describe = "资源加载";   //  调试阶段设为：OperateMsg::FSM_ACTIVE;
  return 0;
}

int32_t Fsm::ONTearDown()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_TEAR_DOWN;
  this->info_.describe = "资源释放";
  return 0;
}

int32_t Fsm::OnSelfCheck()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_SELF_CHECK;
  this->info_.describe = "自检";
  return 0;
}

int32_t Fsm::OnActive()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_ACTIVE;
  this->info_.describe = "活跃";
  return 0;
}

int32_t Fsm::OnDeActive()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_DE_ACTIVE;
  this->info_.describe = "静默";
  return 0;
}

int32_t Fsm::OnProtected()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_PROTECTED;
  this->info_.describe = "低电量";
  return 0;
}

int32_t Fsm::OnLowPower()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_LOW_POWER;
  this->info_.describe = "低功耗";
  return 0;
}

int32_t Fsm::OnOTA()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_OTA;
  this->info_.describe = "远程升级";
  return 0;
}

int32_t Fsm::OnError()
{
  INFO("%s Trigger once %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  this->info_.fsm = OperateMsg::FSM_ERROR;
  this->info_.describe = "错误";
  return 0;
}

void Fsm::UpdateInfo()
{
  try {
    this->info_pub_->publish(this->info_);
  } catch (const std::exception & e) {
    ERROR("%s Update info failed: %s.", this->logger_.c_str(), e.what());
  }
}

void Fsm::AudioPlay(const std::string & _message)
{
  AudioPlayExtendMsg audio_msg;
  audio_msg.module_name = "cyberdog_vp";
  audio_msg.is_online = true;
  audio_msg.text = _message;
  // audio_msg.speech.xxx
  this->audio_pub_->publish(audio_msg);
}

bool Fsm::JudgeOperateLegality(const OperateMsg & _msg)
{
  try {
    std::string fsm_state = this->info_.fsm;
    std::string fsm_describe_zh = this->info_.describe;
    if (LEGAL_OPERATE_CONDITION.count(_msg.type) == 1) {
      if (LEGAL_OPERATE_CONDITION.at(_msg.type).count(fsm_state) == 1) {
        auto & legal_operates = LEGAL_OPERATE_CONDITION.at(_msg.type).at(fsm_state);
        if ((!legal_operates.empty()) &&
          (std::find(
            legal_operates.begin(), legal_operates.end(),
            _msg.operate) != legal_operates.end()))
        {
          return true;
        }
      }
    }
    this->state_ = StateEnum::abnormally_operate;
    this->describe_ = "The current robot is in [" + fsm_state +
      "] mode and does not support <" + _msg.operate + "> operation.";
    WARN("%s %s", this->logger_.c_str(), this->describe_.c_str());
    std::string operate_describe_zh = "";
    if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
      operate_describe_zh = "查询";
    } else if (_msg.operate == OperateMsg::OPERATE_SAVE) {
      operate_describe_zh = "保存";
    } else if (_msg.operate == OperateMsg::OPERATE_DELETE) {
      operate_describe_zh = "删除";
    } else if (_msg.operate == OperateMsg::OPERATE_DEBUG) {
      operate_describe_zh = "调试";
    } else if (_msg.operate == OperateMsg::OPERATE_RUN) {
      operate_describe_zh = "运行";
    } else if (_msg.operate == OperateMsg::OPERATE_SUSPEND) {
      operate_describe_zh = "暂停";
    } else if (_msg.operate == OperateMsg::OPERATE_RECOVER) {
      operate_describe_zh = "继续";
    } else if (_msg.operate == OperateMsg::OPERATE_SHUTDOWN) {
      operate_describe_zh = "终止";
    } else if (_msg.operate == OperateMsg::OPERATE_START) {
      operate_describe_zh = "开始";
    } else if (_msg.operate == OperateMsg::OPERATE_STOP) {
      operate_describe_zh = "停止";
    } else {
      operate_describe_zh = "非法";
    }
    this->AudioPlay(
      std::string(
        "当前机器人处于" + fsm_describe_zh + "模式, 不支持 " +
        _msg.type + " 模块的" + operate_describe_zh + "操作。"));
  } catch (const std::exception & e) {
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "Judge operate legality failed.";
    ERROR("%s %s: %s.", this->logger_.c_str(), this->describe_.c_str(), e.what());
  }
  return false;
}

void Fsm::getRobotendMsg(const OperateMsg & _msg, GRPCMsg & msg_)
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

bool Fsm::RespondToRequests(const OperateMsg & _msg, GRPCMsg & msg_)
{
  this->state_ = StateEnum::normally;
  this->describe_ = "normally";
  try {
    if (this->JudgeOperateLegality(_msg)) {
      return true;
    }
  } catch (const std::exception & e) {
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "Respond to requests failed.";
    ERROR("%s %s: %s.", this->logger_.c_str(), this->describe_.c_str(), e.what());
  }
  this->getRobotendMsg(_msg, msg_);
  return false;
}
}   // namespace cyberdog_visual_programming_engine
