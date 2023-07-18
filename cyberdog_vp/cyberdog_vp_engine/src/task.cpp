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

#include "cyberdog_vp_engine/task.hpp"

namespace cyberdog_visual_programming_engine
{
Task::Task()
: Base("task")
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

Task::~Task()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool Task::InitList()
{
  try {
    INFO("%s %s().", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      return false;
    }
    const toml::value now_lists = toml::find(registry_toml, this->type_);
    if (!now_lists.is_table()) {return false;}
    std::string mode = "";
    std::string state = "";
    for (const auto & meta : now_lists.as_table()) {
      state = toml::find_or(registry_toml, this->type_, meta.first, "state", "");
      if ((state == OperateMsg::STATE_RUN) ||
        (state == OperateMsg::STATE_SUSPEND))
      {
        mode = toml::find_or(registry_toml, this->type_, meta.first, "mode", "");
        registry_toml[this->type_][meta.first]["state"] = (mode == OperateMsg::MODE_SINGLE) ?
          OperateMsg::STATE_SHUTDOWN : OperateMsg::STATE_RUN_WAIT;
      }
    }
    return registry_toml.is_table() ? cyberdog::common::CyberdogToml::WriteFile(
      this->registry_fil_,
      registry_toml) : false;
  } catch (const std::exception & e) {
    ERROR(
      "%s %s is failed: %s", this->logger_.c_str(), std::string(__FUNCTION__).c_str(),
      e.what());
  }
  return false;
}

bool Task::InitData()
{
  try {
    INFO("%s Initializing ...", this->logger_.c_str());
    this->base_path_ += "/" + OperateMsg::TYPE_TASK;
    this->task_option_pub_ = this->node_ptr_->create_publisher<OperateMsg>(
      toml::find<std::string>(
        this->params_toml_, "vp", "init", "topic", "task_option_request"),
      rclcpp::ParametersQoS());
    return this->InitList() && this->at_.Init() && this->cron_.Init();
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Task::Build(
  const OperateMsg & _msg, std::string & _file,
  std::vector<std::string> & _dependent)
{
  try {
    INFO(
      "%s [%s] Build task ...",
      this->logger_.c_str(),
      _msg.id.c_str());
    int shell_code;
    std::string shell_message;
    auto shell = [&](std::string _cmd) -> bool {
        if (Shell(_cmd, shell_code, shell_message)) {
          INFO(
            "%s [%s] Build task(%s) is ok.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _cmd.c_str());
          return true;
        } else {
          this->state_ = StateEnum::abnormally_build;
          this->describe_ = "Judge task is fail:" + shell_message;
          ERROR(
            "%s [%s] Build task(%s) is fail.\ncode: %d\nmessage:%s",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _cmd.c_str(),
            shell_code,
            shell_message.c_str());
          return false;
        }
      };
    // _file = this->base_path_ + "/" + PYTHON_SRC + "/task_" + _msg.mode +
    //   GetTime(static_cast<int>(TimeMode::_Y_M_D_H_M_S)) +
    //   "_" + _msg.target_id.front();
    _file = this->base_path_ + "/" + PYTHON_SRC + "/" + _msg.target_id.front();
    auto build =
      [&](const std::string _format, const std::vector<std::string> & _header_data,
        const std::string _body_data) -> bool {
        std::string _target_task = _file + _format;
        std::ofstream task_stream(_target_task);
        if (!task_stream) {
          this->state_ = StateEnum::abnormally_open_file;
          this->describe_ = "Canot open task file:" + _target_task;
          ERROR(
            "%s [%s] [Build] %s.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            this->describe_.c_str());
          return false;
        }
        for (auto data : _header_data) {
          task_stream << data << std::endl;
        }
        task_stream << _body_data << std::endl;
        task_stream.close();
        if (!shell(std::string("chmod 777 " + _target_task))) {
          ERROR(
            "%s [%s] [Build] File(%s) permission modification failed.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _target_task.c_str());
          return false;
        }
        return true;
      };
    std::vector<std::string> header_py;
    if (!this->py_interpreter_ptr_->GetTaskHeader(
        _msg.target_id.front(), _msg.body, header_py,
        _dependent))
    {
      ERROR(
        "%s [%s] [Build] Get task header failed.",
        this->logger_.c_str(),
        _msg.id.c_str());
      this->state_ = StateEnum::abnormally_request;
      this->describe_ = "abnormally request";
      return false;
    }
    std::ostringstream body_py;
    if (_msg.operate == OperateMsg::OPERATE_DEBUG) {
      body_py << "cyberdog.set_log(True)" << std::endl;
    }
    body_py <<
      "cyberdog.task.start()" << std::endl <<
      _msg.body << std::endl <<
      "cyberdog.task.stop()" << std::endl <<
      "cyberdog.shutdown()" << std::endl;
    std::string body_str = body_py.str();
    if (this->decorate_body_) {
      if (!this->py_interpreter_ptr_->DecorateBody(body_str)) {
        ERROR(
          "%s [%s] [Build] Decorate body failed.",
          this->logger_.c_str(),
          _msg.id.c_str());
        return false;
      }
    }
    std::string now_file_py = _file + ".py";
    std::string now_file_log;
    if (!this->GetLogFile(_file, now_file_log)) {
      return false;
    }
    std::ostringstream body_sh;
    now_file_log += ".log 2>&1";
    body_sh << "python3 " << now_file_py << " cyberdog_vp_task:=" << _msg.target_id.front() <<
      " > " <<
      now_file_log <<
      std::endl;
    if (!build(".py", header_py, body_str) ||
      !build(".sh", this->header_sh_, body_sh.str()))
    {
      return false;
    }
    sleep(1);
    if (!shell(std::string(this->judge_python_ + now_file_py))) {
      shell(std::string("rm " + now_file_py + "*"));
      return false;
    }
    INFO(
      "%s [%s] Judge task(%s.py) is ok.",
      this->logger_.c_str(),
      _msg.id.c_str(),
      now_file_py.c_str());
    if (_msg.operate == OperateMsg::OPERATE_DEBUG) {
      return this->py_interpreter_ptr_->GenerateDerivativeFile(now_file_py);
    }
  } catch (const std::exception & e) {
    this->state_ = StateEnum::abnormally_build;
    this->describe_ = "Canot build task file.";
    ERROR(
      "%s [%s] [Build] %s.\nerror:%s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      this->describe_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Task::ExecuteRequest(const OperateMsg & _msg, GRPCMsg & msg_)
{
  INFO("%s %s().", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
  if (!_msg.target_id.empty()) {
    std::string task_id = _msg.target_id.front();
    if ((task_id == "id") ||
      (task_id == "terminal_default") ||
      (task_id == "visual_default"))
    {
      return true;
    }
  }
  if ((_msg.operate != OperateMsg::OPERATE_INQUIRY) &&
    (_msg.target_id.empty()))
  {
    this->state_ = StateEnum::abnormally_operate;
    this->describe_ = "[" + _msg.id + "] <" + _msg.operate +
      "> operate is need target_id, but target_id is empty.";
    ERROR(
      "%s %s",
      this->logger_.c_str(),
      this->describe_.c_str());
    return false;
  }
  auto execute = [&](std::function<bool()> const & _request) -> bool {
      try {
        INFO(
          "%s [%s] <%s> operate ...",
          this->logger_.c_str(),
          _msg.id.c_str(),
          _msg.operate.c_str());
        if (_request()) {
          return true;
        } else {
          this->state_ = StateEnum::abnormally_request;
          this->describe_ = "[" + _msg.id + "] <" + _msg.operate +
            "> operate is error.";
          ERROR(
            "%s %s",
            this->logger_.c_str(),
            this->describe_.c_str());
        }
      } catch (const std::exception & e) {
        this->state_ = StateEnum::abnormally_other_errors;
        this->describe_ = "[" + _msg.id + "] <" + _msg.operate +
          "> operate is error to failed.";
        ERROR(
          "%s %s : %s",
          this->logger_.c_str(),
          this->describe_.c_str(),
          e.what());
      }
      return false;
    };
  std::string flow_state = "";
  std::string mode = "";
  std::string condition = "";
  std::string file = "";
  std::vector<std::string> dependent;
  if ((!this->GetFlowState(_msg, flow_state, mode, condition, file))) {
    this->state_ = StateEnum::abnormally_operate;
    this->describe_ = "The current operation(" + _msg.operate + ") is illegal.";
    ERROR(
      "%s [%s] [SetList] %s.",
      this->logger_.c_str(),
      _msg.id.c_str(),
      this->describe_.c_str());
    return false;
  }
  if ((_msg.operate == OperateMsg::OPERATE_START) ||
    (_msg.operate == OperateMsg::OPERATE_STOP))
  {
    return execute(
      [&]() -> bool {
        return this->SetList(_msg, flow_state, file, dependent);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_DEBUG) {
    return execute(
      [&]() -> bool {
        if (this->Build(_msg, file, dependent)) {
          this->at_.Cancellation(_msg.target_id.front());
          if (this->SetList(_msg, flow_state, file, dependent) &&
          this->at_.Registration(_msg.condition, file))
          {
            return true;
          }
        }
        return false;
      });
  } else if (_msg.operate == OperateMsg::OPERATE_SAVE) {
    return execute(
      [&]() -> bool {
        if (!this->Build(_msg, file, dependent)) {
          flow_state = OperateMsg::STATE_ERROR;
        }
        return this->SetList(_msg, flow_state, file, dependent);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_RUN) {
    return execute(
      [&]() -> bool {
        if ((!_msg.mode.empty()) &&
        (_msg.mode != mode))
        {
          mode = _msg.mode;
        }
        if ((!_msg.condition.empty()) &&
        (_msg.condition != condition))
        {
          condition = _msg.condition;
        }
        if (this->SetList(_msg, flow_state, file, dependent) &&
        ( ((mode == OperateMsg::MODE_SINGLE) && this->at_.Registration(condition, file)) ||
        ((mode == OperateMsg::MODE_CYCLE) && this->cron_.Registration(condition, file))))
        {
          return true;
        }
        return false;
      });
  } else if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
    return execute(
      [&]() -> bool {
        return this->GetList(_msg, msg_);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_DELETE) {
    return execute(
      [&]() -> bool {
        if (!this->SetList(_msg, flow_state, file, dependent)) {
          return false;
        }
        int shell_code;
        std::string shell_message;
        std::string cmd = "rm " + file + "*";
        INFO(
          "%s [%s] cmd=<%s>",
          this->logger_.c_str(),
          _msg.id.c_str(),
          cmd.c_str());
        if (!Shell(cmd, shell_code, shell_message)) {
          ERROR(
            "%s [%s] Delete task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(),
            _msg.id.c_str(),
            cmd.c_str(),
            shell_code,
            shell_message.c_str());
          return false;
        }
        std::string log_file;
        if (this->GetLogFile(file, log_file)) {
          cmd = "rm " + log_file + "*";
          INFO(
            "%s [%s] cmd=<%s>",
            this->logger_.c_str(),
            _msg.id.c_str(),
            cmd.c_str());
          if (!Shell(cmd, shell_code, shell_message)) {
            WARN(
              "%s [%s] Delete task log failed:\ncmd:%s\ncode:%d\nreturn:%s",
              this->logger_.c_str(),
              _msg.id.c_str(),
              cmd.c_str(),
              shell_code,
              shell_message.c_str());
          }
        }
        return true;
      });
  } else if (_msg.operate == OperateMsg::OPERATE_SHUTDOWN) {
    return execute(
      [&]() -> bool {
        if (!(((mode == OperateMsg::MODE_SINGLE) &&
        this->at_.Cancellation(_msg.target_id.front())) ||
        ((mode == OperateMsg::MODE_CYCLE) &&
        this->cron_.Cancellation(_msg.target_id.front()))))
        {
          WARN(
            "%s [%s] <%s> operate is failed.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _msg.operate.c_str());
        }
        return this->SetList(_msg, flow_state, file, dependent);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_SUSPEND) {
    return execute(
      [&]() -> bool {
        this->task_option_pub_->publish(_msg);
        return this->SetList(_msg, flow_state, file, dependent);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_RECOVER) {
    return execute(
      [&]() -> bool {
        this->task_option_pub_->publish(_msg);
        return this->SetList(_msg, flow_state, file, dependent);
      });
  } else {
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "[" + _msg.id + "] <" + _msg.operate +
      "> Unable to recognize current message operate.";
    WARN(
      "%s %s",
      this->logger_.c_str(),
      this->describe_.c_str());
  }
  return false;
}

bool Task::GetFlowState(
  const OperateMsg & _msg,
  std::string & _flow_state,
  std::string & _mode,
  std::string & _condition,
  std::string & _file)
{
  try {
    INFO(
      "%s [%s] [GetFlowState] ...",
      this->logger_.c_str(),
      _msg.id.c_str());
    if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
      return true;
    } else {
      if (_msg.target_id.empty()) {
        ERROR(
          "%s [%s] [GetFlowState] need target_id.",
          this->logger_.c_str(),
          _msg.id.c_str());
        return false;
      }
    }
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      return false;
    }
    std::string now_state = toml::find_or(
      registry_toml, _msg.type, _msg.target_id.front(), "state",
      OperateMsg::STATE_NULL);
    if ((now_state == OperateMsg::STATE_NULL) &&
      (_msg.operate != OperateMsg::OPERATE_SAVE) &&
      (_msg.operate != OperateMsg::OPERATE_DEBUG))
    {
      WARN(
        "%s [%s] The task with the target id (%s) in the task registry does not exist.",
        this->logger_.c_str(),
        _msg.id.c_str(),
        _msg.target_id.front().c_str());
      return false;
    }
    _mode = toml::find_or(
      registry_toml, _msg.type, _msg.target_id.front(), "mode",
      _msg.mode);
    if (_mode.empty()) {
      WARN(
        "%s [%s] Task schema with target ID (%s) in task registry is empty.",
        this->logger_.c_str(),
        _msg.id.c_str(),
        _msg.target_id.front().c_str());
      return false;
    }
    if (STATE_FLOW.count(_mode) == 1) {
      if (STATE_FLOW.at(_mode).count(now_state) == 1) {
        if (STATE_FLOW.at(_mode).at(now_state).count(_msg.operate) == 1) {
          _flow_state = STATE_FLOW.at(_mode).at(now_state).at(_msg.operate);
          if ((_msg.operate == OperateMsg::OPERATE_RUN) ||
            (_msg.operate == OperateMsg::OPERATE_DELETE))
          {
            _condition = toml::find_or(
              registry_toml, _msg.type,
              _msg.target_id.front(), "condition", "");
            if (_condition.empty()) {
              WARN(
                "%s [%s] Failed to run task(%s), condition is empty.",
                this->logger_.c_str(),
                _msg.id.c_str(),
                _msg.target_id.front().c_str());
              return false;
            }
            _file = toml::find_or(registry_toml, _msg.type, _msg.target_id.front(), "file", "");
            if (_file.empty()) {
              WARN(
                "%s [%s] Failed to run task(%s), file is empty.",
                this->logger_.c_str(),
                _msg.id.c_str(),
                _msg.target_id.front().c_str());
              return false;
            }
          }
          return true;
        } else {
          ERROR(
            "%s [%s] %s->%s->%s key does not exist in STATE_FLOW",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _mode.c_str(),
            now_state.c_str(),
            _msg.operate.c_str());
        }
      } else {
        ERROR(
          "%s [%s] %s->%s key does not exist in STATE_FLOW",
          this->logger_.c_str(),
          _msg.id.c_str(),
          _mode.c_str(),
          now_state.c_str());
      }
    } else {
      ERROR(
        "%s [%s] %s key does not exist in STATE_FLOW",
        this->logger_.c_str(),
        _msg.id.c_str(),
        _mode.c_str());
    }
  } catch (const std::exception & e) {
    ERROR(
      "%s [%s] GetFlowState is error: %s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      e.what());
  }
  return false;
}

bool Task::GetLogFile(const std::string & _file, std::string & log_file)
{
  log_file = "";
  if (!_file.empty()) {
    log_file = _file;
    std::string target = "/workspace/task/src";
    log_file = log_file.replace(log_file.find(target), target.size(), "/workspace/log");
  }
  if (log_file.empty()) {
    WARN(
      "%s [GetLogFile] Get task(%s) log file failed.",
      this->logger_.c_str(),
      _file.c_str());
  }
  return !log_file.empty();
}
}   // namespace cyberdog_visual_programming_engine
