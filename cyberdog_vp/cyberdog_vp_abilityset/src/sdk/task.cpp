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

#include "cyberdog_vp_abilityset/task.hpp"
namespace cyberdog_visual_programming_abilityset
{
std::mutex task_state_cvm_;                       /*!< 任务状态cvm */
std::condition_variable task_state_cv_;           /*!< 任务状态cv */

bool Task::SetData(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    this->processor_srv_ = std::string(
      toml::find<std::string>(
        _params_toml, "vp", "init", "service", "task_processor") + "_" + this->task_id_);
    this->fsm_.type = MsgVisualProgrammingOperate::TYPE_TASK;
    this->fsm_.id = "cyberdog_visual_programming_engine_fsm";
    this->fsm_.describe = "活跃";
    this->fsm_.fsm = MsgVisualProgrammingOperate::FSM_ACTIVE;
    this->task_state_ = MsgVisualProgrammingOperate::STATE_NULL;
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Task::SetMechanism(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());

    this->operate_cli_cb_group_ =
      this->node_mortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->frontend_option_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->robotend_grpc_pub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->fsm_sub_cb_group_ =
      this->node_immortal_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->operate_cli_ptr_ = this->node_mortal_ptr_->create_client<SrvVisualProgrammingOperate>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "service", "engine"),
      ClientQos.get_rmw_qos_profile(),
      this->operate_cli_cb_group_);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->frontend_option_sub_cb_group_;
    this->frontend_option_sub_ptr_ =
      this->node_immortal_ptr_->create_subscription<MsgVisualProgrammingOperate>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "task_option_request"),
      SubscriptionQos,
      std::bind(&Task::FrontendOperateCB, this, std::placeholders::_1),
      sub_option);

    sub_option.callback_group = this->fsm_sub_cb_group_;
    this->fsm_sub_ = this->node_immortal_ptr_->create_subscription<MsgVisualProgrammingOperate>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "fsm"),
      SubscriptionQos,
      std::bind(&Task::FsmCB, this, std::placeholders::_1),
      sub_option);

    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->robotend_grpc_pub_cb_group_;
    this->robotend_grpc_pub_ptr_ = this->node_immortal_ptr_->create_publisher<MsgString>(
      toml::find<std::string>(
        _params_toml, "vp", "init", "topic", "robotend"),
      PublisherQos,
      pub_option);
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Task::InitDependent(
  const std::function<void(bool)> & _FShutdown,
  const std::function<AudioPlaySeviceResponse(
    const std::string, const int8_t)> & _FAudioPlay)
{
  this->FShutdown = _FShutdown;
  this->FAudioPlay = _FAudioPlay;
}

bool Task::PubRobotendGrpc(rapidjson::Document & _json_msg)
{
  try {
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> pretty_writer(buffer);
    pretty_writer.SetMaxDecimalPlaces(4);
    _json_msg.Accept(pretty_writer);
    Debug(
      "Task %s publish robotend message to grpc :\n%s",
      this->task_id_.c_str(),
      buffer.GetString());

    MsgString msg;
    if (!cyberdog::common::CyberdogJson::Document2String(_json_msg, msg.data)) {
      Warn("%s PublisherGrpc failed", this->logger_.c_str());
      return false;
    }
    this->robotend_grpc_pub_ptr_->publish(msg);
  } catch (const std::exception & e) {
    Error("%s PublisherGrpc failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Task::RequestEngine(const std::string & _operate)
{
  try {
    if (!rclcpp::ok()) {
      Warn("Client interrupted while requesting for service to appear.");
      return false;
    }
    if (!this->operate_cli_ptr_->wait_for_service(
        std::chrono::seconds(this->timeout_wait_for_service_)))
    {
      Warn("Waiting for backend vp engine bridge service to appear(start) timeout.");
      return false;
    }
    std::shared_ptr<SrvVisualProgrammingOperate::Request> request_ptr =
      std::make_shared<SrvVisualProgrammingOperate::Request>();
    request_ptr->type = SrvVisualProgrammingOperate::Request::TYPE_FROM;
    request_ptr->form.type = MsgVisualProgrammingOperate::TYPE_TASK;
    request_ptr->form.id = "task_" + GetTime();
    request_ptr->form.target_id.push_back(this->task_id_);
    request_ptr->form.operate = _operate;
    // request_ptr->form.describe = "";
    // request_ptr->form.style = "";
    request_ptr->form.mode = MsgVisualProgrammingOperate::TYPE_TASK;
    // request_ptr->form.condition = "";
    // request_ptr->form.body = "";
    // request_ptr->form.condition = "";
    auto result = this->operate_cli_ptr_->async_send_request(request_ptr);
    std::future_status status =
      result.wait_for(std::chrono::seconds(this->timeout_wait_for_));
    if (status != std::future_status::ready) {
      Warn("Request vp engine module timedout or deferred.");
      return false;
    }
    auto response_ptr = result.get();
    if (response_ptr->code != SrvVisualProgrammingOperate::Response::CODE_SUCCESS) {
      Warn(
        "Request(%s) vp engine service module failed, code is %d.",
        _operate.c_str(), response_ptr->code);
      return false;
    }
    Info(
      "Request(%s) vp engine service module success.", _operate.c_str());
  } catch (const std::exception & e) {
    Warn(
      "Requests(%s) vp engine service failed : %s.",
      _operate.c_str(), e.what());
    return false;
  }
  return true;
}

void Task::SetTaskState(const std::string _state)
{
  {
    // std::lock_guard<std::mutex> lk(task_state_cvm_);
    std::scoped_lock lk(task_state_cvm_);
    this->task_state_ = _state;
  }
  task_state_cv_.notify_all();
}

void Task::FrontendOperateCB(const MsgVisualProgrammingOperate::SharedPtr _msg_ptr)
{
  try {
    if (_msg_ptr->target_id.empty()) {
      return;
    }
    if (_msg_ptr->target_id.front() == this->task_id_) {
      // 终止：手动结束
      if (_msg_ptr->operate == MsgVisualProgrammingOperate::OPERATE_SHUTDOWN) {
        this->SetTaskState(MsgVisualProgrammingOperate::STATE_SHUTDOWN);
        Info(
          "[%s] The current task is about to be terminated and "
          "the cyberdog capability set is about to be shutdown.",
          _msg_ptr->id.c_str());
        this->FShutdown(true);
      } else if (_msg_ptr->operate == MsgVisualProgrammingOperate::OPERATE_SUSPEND) {   // 暂停
        Info(
          "[%s] The current task is about to be suspend.",
          _msg_ptr->id.c_str());
        this->SetTaskState(MsgVisualProgrammingOperate::STATE_SUSPEND);
      } else if (_msg_ptr->operate == MsgVisualProgrammingOperate::OPERATE_RECOVER) {   // 继续
        Info(
          "[%s] The current task is about to be run.",
          _msg_ptr->id.c_str());
        this->SetTaskState(MsgVisualProgrammingOperate::STATE_RUN);
      } else {
        Warn(
          "[%s] Invalid task operation command received, \n\ttask_id : %s\n\toperate : %s",
          _msg_ptr->id.c_str(),
          _msg_ptr->target_id.front().c_str(),
          _msg_ptr->operate.c_str());
      }
    }
  } catch (const std::exception & e) {
    Warn(
      "[%s] Error responding to task operate, \n\ttask_id : %s\n\toperate : %s\nerror : %s",
      _msg_ptr->id.c_str(),
      _msg_ptr->target_id.front().c_str(),
      _msg_ptr->operate.c_str(), e.what());
  }
}

void Task::FsmCB(const MsgVisualProgrammingOperate::SharedPtr msg)
{
  this->fsm_ = *msg;
}

bool Task::JudgeFSM()
{
  try {
    if ((this->fsm_.fsm == MsgVisualProgrammingOperate::FSM_ACTIVE) ||
      (this->fsm_.fsm == MsgVisualProgrammingOperate::FSM_PROTECTED))
    {
      return true;
    }
  } catch (const std::exception & e) {
    Error("%s Judge FSM failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

void Task::FSMReciprocalCompensation(StateCode & fsm_state_co)
{
  try {
    std::string fsm_state = this->fsm_.fsm;
    std::string fsm_describe_zh = "";
    fsm_state_co = StateCode::fsm_does_not_allow;
    rapidjson::Document robotend_json;

    auto update_fsm_describe = [&]() {
        std::string fsm_state_zh = "";
        std::string fsm_hint_zh = "";
        std::string hint_end = "主人再下发任务，我就乖乖执行。";
        if (fsm_state == MsgVisualProgrammingOperate::FSM_UNINITIALIZED) {
          fsm_state_zh = "未初始化";
          fsm_hint_zh = "等我初始化完成后，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_SET_UP) {
          fsm_state_zh = "资源加载";
          fsm_hint_zh = "等我资源加载完成后，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_TEAR_DOWN) {
          fsm_state_zh = "资源释放";
          fsm_hint_zh = "好累呀，我要休息了，等我睡醒后，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_SELF_CHECK) {
          fsm_state_zh = "自检";
          fsm_hint_zh = "等我自检完成后，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_ACTIVE) {
          fsm_state_zh = "活跃";
          fsm_hint_zh = "执行任务喽。";
          fsm_state_co = StateCode::success;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_DE_ACTIVE) {
          fsm_state_zh = "静默";
          fsm_hint_zh = "主人如果想要我执行任务，请先呼唤我的名字唤醒我，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_PROTECTED) {
          fsm_state_zh = "低电量";
          fsm_hint_zh = "主人我又饿又累，能不能给我充点电，等我有电了，" + hint_end;
          fsm_state_co = StateCode::success;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_LOW_POWER) {
          fsm_state_zh = "低功耗";
          fsm_hint_zh = "主人如果想要我执行任务，请先呼唤我的名字唤醒我，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_OTA) {
          fsm_state_zh = "远程升级";
          fsm_hint_zh = "主人我正在学习新的技能，等我学会了，" + hint_end;
        } else if (fsm_state == MsgVisualProgrammingOperate::FSM_ERROR) {
          fsm_state_zh = "错误";
          fsm_hint_zh = "主人我好像遇到了困难，需要主人帮我重启一下，" + hint_end;
        } else {
          fsm_state_zh = "非法";
          fsm_hint_zh = "主人我好像遇到了说不清道不明的事情，需要主人帮我重启一下，" +
            hint_end;
        }
        fsm_describe_zh = "抱歉主人，我当前处于" + fsm_state_zh +
          "模式，安全起见该模式下我不能执行任务，望见谅呀。" + fsm_hint_zh;
      };
    update_fsm_describe();
    Warn(
      "%s Task %s : %s",
      this->logger_.c_str(),
      this->task_id_.c_str(),
      fsm_describe_zh.c_str());
    this->FAudioPlay(fsm_describe_zh, 66);
  } catch (const std::exception & e) {
    Error(
      "%s Task %s Shutdown failed: %s.",
      this->logger_.c_str(),
      this->task_id_.c_str(),
      e.what());
  }
}

void Task::GetBlockJson(
  rapidjson::Document & robotend_json, std::string _type,
  std::string _describe, int _state)
{
  // {
  //   "feedback": {
  //       "type": "MsgVisualProgrammingOperate::TYPE_TASK",
  //       "id": "id",
  //       "target_id": "target_id",
  //       "operate": "operate",
  //       "state": 0,
  //       "describe": "describe"
  //   },
  //   "block": {
  //       "type": "type",
  //       "id": "id"
  //   },
  // }
  Debug(
    "%s %s block of %s task.",
    _type.c_str(),
    this->block_id_.c_str(),
    this->task_id_.c_str());
  int state = _state;
  std::string describe = _describe;
  if (state == 0) {
    state = transient_state_.code;
    describe = transient_state_.describe;
  }
  rapidjson::Document feedback_json(rapidjson::kObjectType);
  CyberdogJson::Add(feedback_json, "type", MsgVisualProgrammingOperate::TYPE_TASK);
  CyberdogJson::Add(feedback_json, "id", std::string("task_" + GetTime()));
  CyberdogJson::Add(feedback_json, "target_id", this->task_id_);
  CyberdogJson::Add(feedback_json, "operate", MsgVisualProgrammingOperate::OPERATE_RUN);
  CyberdogJson::Add(feedback_json, "state", static_cast<int>(state));
  CyberdogJson::Add(feedback_json, "describe", this->task_state_ + " task. " + describe);
  rapidjson::Document block_json(rapidjson::kObjectType);
  CyberdogJson::Add(block_json, "type", _type);
  CyberdogJson::Add(block_json, "id", this->block_id_);
  CyberdogJson::Add(robotend_json, "feedback", feedback_json);
  CyberdogJson::Add(robotend_json, "block", block_json);
}

bool Task::GetProcessor(Processor _processor)
{
  try {
    if (!this->is_activation_) {
      return true;
    }
    if (rclcpp::ok()) {
      if (!this->JudgeFSM()) {
        std::string error_describe = "The current robot is in [" + this->fsm_.fsm +
          "] mode and does not support <run> operation.";
        StateCode error_state;
        this->FSMReciprocalCompensation(error_state);
        rapidjson::Document robotend_json(rapidjson::kObjectType);
        this->GetBlockJson(
          robotend_json, MsgVisualProgrammingOperate::STEP_END, error_describe,
          static_cast<int>(error_state));
        this->PubRobotendGrpc(robotend_json);
        this->Stop();
        this->FShutdown(true);
      } else {
        std::unique_lock<std::mutex> lk(task_state_cvm_);
        switch (_processor) {
          case Processor::task:
            task_state_cv_.wait(
              lk, [&] {
                return this->task_state_ == MsgVisualProgrammingOperate::STATE_RUN;
              });
            return true;
            break;
          default:
            break;
        }
      }
    }
  } catch (const std::exception & e) {
    Warn("%s Processor service failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

State Task::Start()
{
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  if (!this->RequestEngine(MsgVisualProgrammingOperate::OPERATE_START)) {
    this->Stop();
    return this->GetState(funs, StateCode::fail);
  }
  this->SetTaskState(MsgVisualProgrammingOperate::STATE_RUN);
  this->is_activation_ = true;
  return this->GetState(funs, StateCode::success);
}

State Task::Stop()
{
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  if (!this->block_id_.empty()) {
    rapidjson::Document robotend_json(rapidjson::kObjectType);
    this->GetBlockJson(robotend_json, MsgVisualProgrammingOperate::STEP_END);
    this->PubRobotendGrpc(robotend_json);
  }
  if (!this->RequestEngine(MsgVisualProgrammingOperate::OPERATE_STOP)) {
    this->FShutdown(true);
    return this->GetState(funs, StateCode::fail);
  }
  this->SetTaskState(MsgVisualProgrammingOperate::STATE_SHUTDOWN);
  this->is_activation_ = false;
  return this->GetState(funs, StateCode::success);
}

State Task::Recover()
{
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  if (!this->RequestEngine(MsgVisualProgrammingOperate::OPERATE_RECOVER)) {
    return this->GetState(funs, StateCode::fail);
  }
  this->SetTaskState(MsgVisualProgrammingOperate::STATE_RUN);
  return this->GetState(funs, StateCode::success);
}

State Task::Block(const std::string _block_id)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s)", _block_id.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  this->GetProcessor(Processor::task);
  if (!this->block_id_.empty()) {
    rapidjson::Document robotend_json(rapidjson::kObjectType);
    this->GetBlockJson(robotend_json, MsgVisualProgrammingOperate::STEP_END);
    this->PubRobotendGrpc(robotend_json);
  }
  this->block_id_ = _block_id;
  rapidjson::Document robotend_json(rapidjson::kObjectType);
  this->GetBlockJson(robotend_json, MsgVisualProgrammingOperate::STEP_BEGIN);
  return this->PubRobotendGrpc(robotend_json) ? this->GetState(
    funs,
    StateCode::success) : this->GetState(funs, StateCode::fail);
}

State Task::BreakpointBlock(const std::string _block_id)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s)", _block_id.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  if (!this->RequestEngine(MsgVisualProgrammingOperate::OPERATE_SUSPEND)) {
    this->Stop();
    return this->GetState(funs, StateCode::fail);
  }
  this->SetTaskState(MsgVisualProgrammingOperate::STATE_SUSPEND);
  return this->Block(_block_id);
}
}   // namespace cyberdog_visual_programming_abilityset
