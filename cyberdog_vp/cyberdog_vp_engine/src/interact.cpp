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

#include "cyberdog_vp_engine/interact.hpp"

namespace cyberdog_visual_programming_engine
{
Interact::Interact()
: node_config_dir_("/config/abilityset.toml")
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
  py::scoped_interpreter guard{};
  this->frontend_node_ptr_ = rclcpp::Node::make_shared("cyberdog_vp_engine_frontend");
  this->backend_node_ptr_ = rclcpp::Node::make_shared("cyberdog_vp_engine_backend");
  this->robotend_node_ptr_ = rclcpp::Node::make_shared("cyberdog_vp_engine_robotend");
  this->fsm_node_ptr_ = rclcpp::Node::make_shared("cyberdog_vp_engine_fsm");

  if (this->Init()) {
    INFO("Visual programming engine node is running ...");
    rclcpp::executors::MultiThreadedExecutor cyberdog_vpe_exec;
    cyberdog_vpe_exec.add_node(this->frontend_node_ptr_);
    cyberdog_vpe_exec.add_node(this->backend_node_ptr_);
    cyberdog_vpe_exec.add_node(this->robotend_node_ptr_);
    cyberdog_vpe_exec.add_node(this->fsm_node_ptr_);
    cyberdog_vpe_exec.spin();
    rclcpp::shutdown();
  } else {
    rclcpp::shutdown();
    exit(-1);
  }
}

Interact::~Interact()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool Interact::Init()
{
  try {
    INFO("%s Initializing ...", this->logger_.c_str());
    auto judge_thirdparty_tools = [&](const std::string _cmd) -> bool {
        int shell_code;
        std::string shell_message;
        if (Shell(_cmd, shell_code, shell_message)) {
          INFO(
            "%s %s\ncode: %d\nmessage:%s", this->logger_.c_str(),
            _cmd.c_str(), shell_code, shell_message.c_str());
        } else {
          ERROR(
            "%s %s\ncode: %d\nmessage:%s", this->logger_.c_str(),
            _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    if (!judge_thirdparty_tools(this->judge_at_) ||
      !judge_thirdparty_tools(this->judge_cron_) ||
      !judge_thirdparty_tools(this->judge_pyflakes_))
    {
      ERROR(
        "%s The running environment is abnormal (missing dependent third-party tools)",
        this->logger_.c_str());
      return false;
    }
    this->params_pkg_dir_ = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    this->node_config_dir_ = this->params_pkg_dir_ + this->node_config_dir_;
    if (!JudgeConfileFile(this->node_config_dir_)) {
      return false;
    }
    if (!cyberdog::common::CyberdogToml::ParseFile(
        this->node_config_dir_.c_str(), this->params_toml_))
    {
      ERROR("%s Params config file is not in toml format.", this->logger_.c_str());
      return false;
    }
    this->use_backend_ = toml::find<bool>(
      this->params_toml_, "vp", "init", "environment", "backend");
    this->use_voice_ctrl_ = toml::find<bool>(
      this->params_toml_, "vp", "init", "environment", "voice_operation");

    this->frontend_msg_cb_group_ = this->frontend_node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->audio_msg_cb_group_ = this->robotend_node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->grpc_pub_cb_group_ = this->robotend_node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->http_cli_cb_group_ = this->backend_node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->operate_srv_cb_group_ = this->frontend_node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->frontend_msg_cb_group_;
    this->grpc_sub_ = this->frontend_node_ptr_->create_subscription<GRPCMsg>(
      toml::find<std::string>(
        this->params_toml_, "vp", "init", "topic", "frontend"),
      SubscriptionQos,
      std::bind(&Interact::FrontendMsgCallback, this, std::placeholders::_1),
      sub_option);

    if (this->use_voice_ctrl_) {
      INFO("%s Turn on voice control.", this->logger_.c_str());
      sub_option.callback_group = this->audio_msg_cb_group_;
      this->asr_sub_ = this->robotend_node_ptr_->create_subscription<ASRMsg>(
        toml::find<std::string>(
          this->params_toml_, "vp", "init", "topic", "training_words"),
        SubscriptionQos,
        std::bind(&Interact::ASRMsgCallback, this, std::placeholders::_1),
        sub_option);
    }

    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->grpc_pub_cb_group_;
    this->grpc_pub_ = this->robotend_node_ptr_->create_publisher<GRPCMsg>(
      toml::find<std::string>(
        this->params_toml_, "vp", "init", "topic", "robotend"),
      PublisherQos,
      pub_option);

    this->http_cli_ = this->backend_node_ptr_->create_client<HTPSrv>(
      toml::find<std::string>(
        this->params_toml_, "vp", "init", "service", "backend"),
      ClientQos.get_rmw_qos_profile(),
      this->http_cli_cb_group_);

    this->operate_srv_ = this->frontend_node_ptr_->create_service<OperateSrv>(
      toml::find<std::string>(
        this->params_toml_, "vp", "init", "service", "engine"),
      std::bind(&Interact::OperateResponse, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->operate_srv_cb_group_);
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return this->InitWorkspace() && this->InitData();
}

bool Interact::InitWorkspace()
{
  try {
    INFO("%s Init Workspace...", this->logger_.c_str());
    std::string workspace_name = "";
    if (!GetWorkspace(workspace_name)) {
      ERROR(
        "%s Init get workspace failed.",
        this->logger_.c_str());
      return false;
    }

    auto shell = [&](const std::string & _cmd) -> bool {
        int shell_code;
        std::string shell_message;
        INFO(
          "%s cmd<%s>",
          this->logger_.c_str(),
          _cmd.c_str());
        if (Shell(_cmd, shell_code, shell_message)) {
          INFO(
            "%s %s\ncode: %d\nmessage:%s", this->logger_.c_str(),
            _cmd.c_str(), shell_code, shell_message.c_str());
        } else {
          ERROR(
            "%s %s\ncode: %d\nmessage:%s", this->logger_.c_str(),
            _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    auto difference_create_path = [&](std::string _path) {
        if (access(_path.c_str(), F_OK) != 0) {
          if (!Mkdir(_path)) {
            WARN(
              "%s Mkdir path<%s> failed.",
              this->logger_.c_str(),
              _path.c_str());
            if (!shell(std::string("mkdir -p " + _path))) {
              ERROR(
                "%s <mkdir -p %s> failed.",
                this->logger_.c_str(),
                _path.c_str());
              return false;
            }
          }
        }
        return true;
      };
    auto difference_create_workspace = [&]() {
        if (access(workspace_name.c_str(), F_OK) != 0) {
          if (!difference_create_path(workspace_name)) {
            ERROR(
              "%s create path %s is failed.",
              this->logger_.c_str(),
              workspace_name.c_str());
            return false;
          }
          std::string source_workspace =
            ament_index_cpp::get_package_share_directory("cyberdog_vp") +
            "/workspace/*";
          std::string cmd = "cp -r " + source_workspace + " " + workspace_name + "/";
          if (!shell(cmd)) {
            ERROR(
              "%s Cp workspace(%s --> %s) failed.",
              this->logger_.c_str(),
              source_workspace.c_str(), workspace_name.c_str());
            return false;
          }
        }
        return true;
      };
    auto difference_create_toml_file = [&](std::string _type) {
        std::string target_file = workspace_name + "/" + _type + "/" + _type + ".toml";
        if (!JudgeConfileFile(target_file)) {
          WARN(
            "%s Judge confile file <%s> failed.",
            this->logger_.c_str(),
            target_file.c_str());
          std::string source_file = ament_index_cpp::get_package_share_directory("cyberdog_vp") +
            "/workspace/" + _type + "/" + _type + ".toml";
          if (!shell(std::string("cp " + source_file + " " + target_file))) {
            ERROR(
              "%s <cp %s %s> failed.",
              this->logger_.c_str(),
              source_file.c_str(),
              target_file.c_str());
            return false;
          }
        }
        return true;
      };

    if (!difference_create_workspace() ||
      !difference_create_path(std::string(workspace_name + "/log")) ||
      !difference_create_path(std::string(workspace_name + "/" + OperateMsg::TYPE_MODULE)) ||
      !difference_create_path(
        std::string(
          workspace_name + "/" + OperateMsg::TYPE_MODULE + "/" +
          PYTHON_SRC)) ||
      !difference_create_path(std::string(workspace_name + "/" + OperateMsg::TYPE_TASK)) ||
      !difference_create_path(
        std::string(
          workspace_name + "/" + OperateMsg::TYPE_TASK + "/" +
          PYTHON_SRC)) ||
      !difference_create_toml_file(OperateMsg::TYPE_MODULE) ||
      !difference_create_toml_file(OperateMsg::TYPE_TASK))
    {
      ERROR(
        "%s Init workspace failed.",
        this->logger_.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    ERROR(
      "%s Init workspace failed.%s",
      this->logger_.c_str(),
      e.what());
    return false;
  }
  return true;
}

bool Interact::InitData()
{
  INFO("%s Init data...", this->logger_.c_str());
  this->py_interpreter_ptr_ = std::make_shared<PythonInterpreter>();
  this->py_interpreter_ptr_->Init();
  if (!this->module_.Init(
      this->frontend_node_ptr_,
      this->py_interpreter_ptr_,
      this->params_toml_))
  {
    ERROR("%s Init module failed.", this->logger_.c_str());
    return false;
  }
  if (!this->task_.Init(
      this->frontend_node_ptr_,
      this->py_interpreter_ptr_,
      this->params_toml_))
  {
    ERROR("%s Init task failed.", this->logger_.c_str());
    return false;
  }
  if (!this->AI_.Init(
      this->robotend_node_ptr_,
      this->params_toml_))
  {
    ERROR("%s Init AI failed.", this->logger_.c_str());
    return false;
  }
  if (!this->SLAM_.Init(
      this->robotend_node_ptr_,
      this->params_toml_))
  {
    ERROR("%s Init SLAM failed.", this->logger_.c_str());
    return false;
  }
  if (!this->fsm_.Init(
      this->fsm_node_ptr_,
      this->params_toml_))
  {
    ERROR("%s Init fsm failed.", this->logger_.c_str());
    return false;
  }

  return true;
}

std::shared_ptr<HTPSrv::Request> Interact::GetRequest(const std::string & _msg)
{
  std::shared_ptr<HTPSrv::Request> request_ptr = std::make_shared<HTPSrv::Request>();
  request_ptr->method = HTPSrv::Request::HTTP_METHOD_POST;  // 0:get,1:post
  request_ptr->url = "programs";                            // 请求地址
  request_ptr->params = _msg;                               // 请求参数
  request_ptr->milsecs = 3000;                              // 超时设置，单位毫秒，默认3秒
  return request_ptr;
}

bool Interact::RequestsBackend(
  const std::shared_ptr<HTPSrv::Request> & _request_ptr,
  BackendMsg & _response)
{
  try {
    this->CoutJson("The requests http service json data is", _request_ptr->params);
    if (!rclcpp::ok()) {
      WARN("Client interrupted while requesting for service to appear.");
      return false;
    }
    if (!this->http_cli_->wait_for_service(std::chrono::seconds(3))) {
      WARN("Waiting for backend http bridge service to appear(start) timeout.");
      return false;
    }
    auto result = this->http_cli_->async_send_request(_request_ptr);
    std::future_status status =
      result.wait_for(std::chrono::seconds(5));
    if (status != std::future_status::ready) {
      WARN("Request backend module timedout or deferred.");
      return false;
    }
    auto response_ptr = result.get();
    std::string response_data = response_ptr->data;
    if (response_data.empty()) {
      WARN(
        "Request http service module failed, data is empty");
      return false;
    }
    this->CoutJson("The http service response json data is", response_data);
    BackendMessage response_json(response_data);
    _response = response_json.backend_;
    if ((response_json.state_ == BackendMessage::CommonEnum::efficient) &&
      (std::stoi(response_json.backend_.code) !=
      static_cast<int>(BackendMessage::HttpCode::Success)))
    {
      WARN(
        "Request http service module failed, %s.",
        response_json.errors.at(std::stoi(response_json.backend_.code)).c_str());
      return false;
    }
  } catch (const std::exception & e) {
    WARN(
      "Requests http service failed:\nmessage:%s\nerror:%s",
      _request_ptr->params.c_str(), e.what());
    return false;
  }
  return true;
}

void Interact::CoutJson(const std::string & _msg_head, const std::string & _msg)
{
  try {
    rapidjson::Document doc;
    if (cyberdog::common::CyberdogJson::String2Document(_msg, doc)) {
      rapidjson::StringBuffer buffer;
      rapidjson::PrettyWriter<rapidjson::StringBuffer> pretty_writer(buffer);
      pretty_writer.SetMaxDecimalPlaces(4);
      doc.Accept(pretty_writer);
      INFO("%s %s :\n%s", this->logger_.c_str(), _msg_head.c_str(), buffer.GetString());
    }
  } catch (const std::exception & e) {
    WARN("Error responding to message:\nmessage:%s\nerror:%s", _msg.c_str(), e.what());
  }
}

void Interact::FrontendMsgCallback(const GRPCMsg::SharedPtr _frontend_msg_ptr)
{
  try {
    OperateMsg frontend;
    if (this->RespondToFrontAndBackEndRequests(
        CommonEnum::frontend, _frontend_msg_ptr->data,
        frontend))
    {
      INFO("Respond To frontend Requests is success.");
    } else {
      WARN("Respond To frontend Requests is error.");
    }
  } catch (const std::exception & e) {
    WARN(
      "Error responding to frontend message:\nmessage:%s\nerror:%s",
      _frontend_msg_ptr->data.c_str(), e.what());
  }
}

void Interact::ASRMsgCallback(const ASRMsg::SharedPtr _msg_ptr)
{
  try {
    INFO(
      "Automatic speech recognition:\n\ttrigger=%s\n\ttype=%s\n\tvalue=%s",
      _msg_ptr->trigger.c_str(),
      _msg_ptr->type.c_str(),
      _msg_ptr->value.c_str());
    if (_msg_ptr->type != "vp_task") {
      WARN(
        "Automatic speech recognition is illegal type:\n\ttrigger=%s\n\ttype=%s\n\tvalue=%s",
        _msg_ptr->trigger.c_str(),
        _msg_ptr->type.c_str(),
        _msg_ptr->value.c_str());
      return;
    }
    std::string frontend_requests = "";
    if (!this->task_.BuildFrontendOperate(_msg_ptr->value, frontend_requests)) {
      WARN("Build frontend operate is fail.");
      return;
    }
    OperateMsg frontend;
    if (this->RespondToFrontAndBackEndRequests(
        CommonEnum::frontend, frontend_requests,
        frontend))
    {
      INFO("Automatic speech recognition request bot is success.");
    } else {
      WARN("Automatic speech recognition request bot is error.");
    }
  } catch (const std::exception & e) {
    WARN(
      "Error responding to frontend message:\nmessage:%s\nerror:%s",
      _msg_ptr->trigger.c_str(), e.what());
  }
}

void Interact::OperateResponse(
  const std::shared_ptr<OperateSrv::Request> request_ptr,
  std::shared_ptr<OperateSrv::Response> response_ptr)
{
  OperateMsg now_operate;
  GRPCMsg respond_msg;
  auto responds = [&]() -> bool {   // 响应
      response_ptr->json = respond_msg.data;
      this->CoutJson("The respond json data is", response_ptr->json);
      this->grpc_pub_->publish(respond_msg);
      return true;
    };
  try {
    if (request_ptr->type == OperateSrv::Request::TYPE_JSON) {
      FrontendMessage json_msg(request_ptr->json);
      if (json_msg.state_ == FrontendMessage::CommonEnum::efficient) {
        now_operate = json_msg.frontend_;
      } else {
        json_msg.getRobotendMsg(respond_msg);
        response_ptr->code = json_msg.state_;
        responds();
        return;
      }
    } else {
      now_operate = request_ptr->form;
    }

    if (now_operate.target_id.empty()) {
      WARN("Respond operate task message is error(target_id is empty).");
      return;
    }
    INFO(
      "Respond operate task message: {"
      "type : %s, "
      "id : %s, "
      "target_id : %s, "
      "describe : %s, "
      "style : %s, "
      "operate : %s, "
      "mode : %s, "
      "condition : %s, "
      "body : %s, "
      "fsm : %s}",
      now_operate.type.c_str(),
      now_operate.id.c_str(),
      now_operate.target_id.front().c_str(),
      now_operate.describe.c_str(),
      now_operate.style.c_str(),
      now_operate.operate.c_str(),
      now_operate.mode.c_str(),
      now_operate.condition.c_str(),
      now_operate.body.c_str(),
      now_operate.fsm.c_str());
    if (this->fsm_.RespondToRequests(now_operate, respond_msg)) {
      if (this->task_.RespondToRequests(now_operate, respond_msg)) {
        INFO("Responding to task operate(%s) message ok,", now_operate.id.c_str());
        response_ptr->code = OperateSrv::Response::CODE_SUCCESS;
      } else {
        WARN(
          "Responding to task operate message error,"
          "\n\tid : %s\n\terror : Task status transfer failed", now_operate.id.c_str());
        response_ptr->code = OperateSrv::Response::CODE_TASK;
      }
    } else {
      WARN(
        "Responding to task operate message error,"
        "\n\tid : %s\n\terror : The state machine does not allow.", now_operate.id.c_str());
      response_ptr->code = OperateSrv::Response::CODE_FSM;
    }
  } catch (const std::exception & e) {
    WARN("Operate service response is failed: <%s>", e.what());
    response_ptr->code = OperateSrv::Response::CODE_CATHC;
  }
  responds();
}

bool Interact::RespondToFrontAndBackEndRequests(
  const CommonEnum & _type, const std::string & _requests,
  OperateMsg & _frontend)
{
  std::string head = (_type == CommonEnum::frontend) ? "Frontend" : "Backend";
  try {
    this->CoutJson(std::string("[" + head + "] The  requests json data is"), _requests);

    FrontendMessage json_msg(_requests);
    _frontend = json_msg.frontend_;
    GRPCMsg robotend_respond_msg;

    auto responds = [&]() -> bool {   // 响应
        this->CoutJson("[" + head + "] The respond json data is", robotend_respond_msg.data);
        if (_type == CommonEnum::frontend) {
          this->grpc_pub_->publish(robotend_respond_msg);
        } else {
          if (this->use_backend_) {
            auto backend_request_ptr = this->GetRequest(robotend_respond_msg.data);
            BackendMsg backend_response;
            if (!this->RequestsBackend(backend_request_ptr, backend_response)) {
              WARN("[%s] Requests backend feedback is error", head.c_str());
              return false;
            }
          }
        }
        return true;
      };

    auto request = [&](const OperateMsg & _request_msg) -> bool {   // 请求
        bool request_state = false;
        if ((_type == CommonEnum::frontend) &&
          this->use_backend_ &&
          (_request_msg.operate == OperateMsg::OPERATE_INQUIRY) &&
          (_request_msg.target_id.empty()))
        {
          std::string backend_msg = "";
          bool build_backend_msg = false;
          if (_request_msg.type == OperateMsg::TYPE_TASK) {
            build_backend_msg = this->task_.BuildBackendOperate(_frontend, backend_msg);
          } else if (_request_msg.type == OperateMsg::TYPE_MODULE) {
            build_backend_msg = this->module_.BuildBackendOperate(_frontend, backend_msg);
          }
          if (build_backend_msg && !backend_msg.empty()) {
            auto backend_request_ptr = this->GetRequest(backend_msg);
            BackendMsg backend_response;
            if (this->RequestsBackend(backend_request_ptr, backend_response)) {
              OperateMsg backend_frontend;
              for (auto meta : backend_response.data) {
                this->RespondToFrontAndBackEndRequests(CommonEnum::backend, meta, backend_frontend);
              }
            } else {
              WARN("[%s] Requests backend inquiry is error.", head.c_str());
            }
          }
        }
        if (_request_msg.type == OperateMsg::TYPE_TASK) {
          request_state = this->task_.RespondToRequests(_request_msg, robotend_respond_msg);
        } else if (_request_msg.type == OperateMsg::TYPE_MODULE) {
          request_state = this->module_.RespondToRequests(_request_msg, robotend_respond_msg);
        } else if (_request_msg.type == OperateMsg::TYPE_AI) {
          request_state = this->AI_.RespondToRequests(_request_msg, robotend_respond_msg);
        } else if (_request_msg.type == OperateMsg::TYPE_SLAM) {
          request_state = this->SLAM_.RespondToRequests(_request_msg, robotend_respond_msg);
        } else {
          WARN(
            "[%s] Unable to recognize current message type:%s",
            head.c_str(), _request_msg.type.c_str());
        }
        responds();
        if ((_type == CommonEnum::frontend) &&
          this->use_backend_ &&
          (_request_msg.operate != OperateMsg::OPERATE_INQUIRY) &&
          request_state)
        {
          std::string frontend_msg;
          if (OperateMsgToJson(_request_msg, frontend_msg)) {
            CoutJson(this->logger_ + "[Requests Brontend]", frontend_msg);
            auto backend_request_ptr = this->GetRequest(frontend_msg);
            BackendMsg backend_response;
            if (this->RequestsBackend(backend_request_ptr, backend_response)) {
              INFO(
                "[%s] Requests delete (%s %s) backend inquiry is success.",
                head.c_str(),
                _request_msg.target_id.front().c_str(),
                _request_msg.type.c_str());
            } else {
              WARN(
                "[%s] Requests delete (%s %s) backend inquiry is error.",
                head.c_str(),
                _request_msg.target_id.front().c_str(),
                _request_msg.type.c_str());
            }
          }
        }
        return request_state;
      };

    auto requests = [&](const OperateMsg & _request_msg) -> bool {   // 请求
        uint false_size = 0;
        if (_request_msg.operate == OperateMsg::OPERATE_DELETE) {
          OperateMsg single_msg = _request_msg;
          for (const std::string tar_id : _request_msg.target_id) {
            if (tar_id.empty()) {
              continue;
            }
            single_msg.target_id.clear();
            single_msg.target_id.push_back(tar_id);
            false_size += request(single_msg) ? 0 : 1;
          }
        } else {
          // OPERATE_SAVE, OPERATE_DEBUG, OPERATE_RUN,
          // OPERATE_SUSPEND, OPERATE_RECOVER, OPERATE_SHUTDOWN
          false_size += request(_request_msg) ? 0 : 1;
        }
        return false_size == 0;
      };

    if (json_msg.state_ == FrontendMessage::CommonEnum::efficient) {
      if (this->fsm_.RespondToRequests(_frontend, robotend_respond_msg)) {
        if (requests(_frontend)) {
          INFO(
            "[%s] Requests message processing successfully type:%s",
            head.c_str(), _frontend.type.c_str());
        } else {
          INFO(
            "[%s] Requests message processing fail type:%s",
            head.c_str(), _frontend.type.c_str());
        }
      } else {
        responds();
      }
    } else {
      json_msg.getRobotendMsg(robotend_respond_msg);
      responds();
    }
  } catch (const std::exception & e) {
    WARN(
      "[%s] Error responding to requests message:\nmessage:%s\nerror:%s",
      head.c_str(), _requests.c_str(), e.what());
    return false;
  }
  return true;
}

bool Interact::OperateMsgToJson(const OperateMsg & _msg, std::string & _frontend_msg)
{
  try {
    std::string id = GetTime();
    rapidjson::Document frontend_json;
    frontend_json.SetObject();
    rapidjson::Document::AllocatorType & allocator = frontend_json.GetAllocator();
    frontend_json.AddMember("type", rapidjson::StringRef(_msg.type.c_str()), allocator);
    frontend_json.AddMember("id", rapidjson::StringRef(id.c_str()), allocator);
    rapidjson::Value target_id_json(rapidjson::kArrayType);
    for (const auto & tar_id : _msg.target_id) {
      target_id_json.PushBack(rapidjson::StringRef(tar_id.c_str()), allocator);
    }
    frontend_json.AddMember("target_id", target_id_json, allocator);
    frontend_json.AddMember("operate", rapidjson::StringRef(_msg.operate.c_str()), allocator);
    frontend_json.AddMember("describe", rapidjson::StringRef(_msg.describe.c_str()), allocator);
    frontend_json.AddMember("style", rapidjson::StringRef(_msg.style.c_str()), allocator);
    frontend_json.AddMember("mode", rapidjson::StringRef(_msg.mode.c_str()), allocator);
    frontend_json.AddMember("condition", rapidjson::StringRef(_msg.condition.c_str()), allocator);
    frontend_json.AddMember("body", rapidjson::StringRef(_msg.body.c_str()), allocator);
    CyberdogJson::Document2String(frontend_json, _frontend_msg);
  } catch (const std::exception & e) {
    WARN(
      "Error operate to json message:\nerror:%s",
      e.what());
    return false;
  }
  return true;
}
}   // namespace cyberdog_visual_programming_engine
