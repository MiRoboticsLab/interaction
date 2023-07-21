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

#include "cyberdog_vp_engine/module.hpp"

namespace cyberdog_visual_programming_engine
{
Module::Module()
: Base("module")
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

Module::~Module()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool Module::InitData()
{
  try {
    INFO("%s Initializing ...", this->logger_.c_str());
    this->base_path_ += "/" + OperateMsg::TYPE_MODULE;
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Module::JudgeModulesUniquenes(const OperateMsg & _msg, const std::string _neglect)
{
  try {
    INFO(
      "%s [%s] Judge modules uniquenes.",
      this->logger_.c_str(),
      _msg.id.c_str());
    auto get_interface = [&](const std::string _message,
        std::string & interface_name) -> bool {
        std::vector<std::string> mode_vct = GetVector(_message, '(');
        if (mode_vct.empty()) {
          ERROR(
            "%s [%s] get interface(%s) name is fail.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _message.c_str());
          return false;
        }
        interface_name = mode_vct.at(0) + '(';
        return true;
      };
    std::vector<std::string> neglect = {"id"};
    if (!_neglect.empty()) {
      neglect.push_back(_neglect);
    }
    std::string interface_name, now_interface_name;
    if (!get_interface(_msg.condition, interface_name)) {
      return false;
    }
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      ERROR(
        "%s [%s] Toml config file is not in toml format, config file dir:\n%s",
        this->logger_.c_str(),
        _msg.id.c_str(),
        this->registry_fil_.c_str());
      return false;
    }
    const toml::value now_lists = toml::find(registry_toml, OperateMsg::TYPE_MODULE);
    if (!now_lists.is_table()) {return false;}
    std::string id = "";
    for (const auto & meta : now_lists.as_table()) {
      id = meta.first;
      if (std::find(neglect.begin(), neglect.end(), id) != neglect.end()) {
        continue;
      }
      if (!get_interface(toml::find<std::string>(meta.second, "condition"), now_interface_name)) {
        ERROR(
          "%s [%s] Module (%s) interface error:",
          this->logger_.c_str(),
          _msg.id.c_str(),
          id.c_str());
        return false;
      }
      if (id == _msg.target_id.front()) {
        if (now_interface_name != interface_name) {
          this->describe_ = "[" + _msg.id + "] module(" +
            _msg.target_id.front() + ") modifying the interface name:" + interface_name +
            ") -> " + now_interface_name +
            ").\nNot currently supported, the operation is illegal.";
          ERROR(
            "%s %s",
            this->logger_.c_str(),
            this->describe_.c_str());
          return false;         // 支持修改，同文件函数/接口唯一性，避免依赖时发生错乱
        }
      } else if (now_interface_name == interface_name) {
        this->describe_ = "[" + _msg.id + "] module(" +
          _msg.target_id.front() + ") define interface names (" + now_interface_name +
          ")) repeatedly in different module files:" + id +
          " and " + _msg.target_id.front() +
          ".\nNot currently supported, the operation is illegal.";
        ERROR(
          "%s %s",
          this->logger_.c_str(),
          this->describe_.c_str());
        return false;           // 不支持重载（不同文件间函数/接口唯一性）
      }
    }
    return true;
  } catch (const std::exception & e) {
    ERROR(
      "%s [%s] Judge modules uniquenes failed: %s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      e.what());
  }
  return false;
}

bool Module::JudgeModulesLoop(
  const OperateMsg & _msg,
  const std::vector<std::string> & _dependent)
{
  try {
    INFO(
      "%s [%s] Judge modules unidirectional.",
      this->logger_.c_str(),
      _msg.id.c_str());
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      ERROR(
        "%s [%s] Toml config file is not in toml format, config file dir:\n%s",
        this->logger_.c_str(),
        _msg.id.c_str(),
        this->registry_fil_.c_str());
      return false;
    }
    const toml::value now_lists = toml::find(registry_toml, OperateMsg::TYPE_MODULE);
    if (!now_lists.is_table()) {return false;}
    auto get_loop = [](const std::vector<std::string> & _loop) -> std::string {
        std::ostringstream ret;
        ret << "\n╭─╮";
        for (auto meta_ptr = _loop.rbegin(); meta_ptr != _loop.rend(); ++meta_ptr) {
          ret << "\n│ v : " << *meta_ptr;
        }
        ret << "\n╰─╯" << std::endl;
        return ret.str();
      };
    std::vector<std::string> loop;
    std::function<bool(const std::vector<std::string> &)> judge_loop =
      [&](const std::vector<std::string> & _dep) -> bool {
        if (!_dep.empty()) {
          if (std::find(_dep.begin(), _dep.end(), _msg.target_id.front()) != _dep.end()) {
            return true;
          }
          for (const auto & id : _dep) {
            std::vector<std::string> null_dependent;
            std::vector<std::string> dependent = toml::find_or(
              now_lists, id, "dependent",
              null_dependent);
            if (judge_loop(dependent)) {
              loop.push_back(id);
              return true;
            }
          }
        }
        return false;
      };
    if (judge_loop(_dependent)) {
      loop.push_back(_msg.target_id.front());
      this->describe_ = "[" + _msg.id + "] module(" +
        _msg.target_id.front() + ") has loop:" + get_loop(loop);
      ERROR(
        "%s %s",
        this->logger_.c_str(),
        this->describe_.c_str());
      return true;
    }
  } catch (const std::exception & e) {
    ERROR(
      "%s [%s] Judge modules uniquenes failed: %s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      e.what());
  }
  return false;
}

bool Module::Build(
  const OperateMsg & _msg, std::string & _file,
  std::vector<std::string> & _dependent)
{
  try {
    if (_file.empty()) {
      _file = this->base_path_ + "/" + PYTHON_SRC + "/" + _msg.target_id.front() +
        ".py";
    }
    _dependent.clear();
    std::vector<std::string> header_py;
    if (!this->py_interpreter_ptr_->GetModuleHeader(
        _msg.condition, _msg.describe, _msg.body,
        header_py, _dependent))
    {
      ERROR(
        "%s [%s] [Build] Get module header failed.",
        this->logger_.c_str(),
        _msg.id.c_str());
      this->state_ = StateEnum::abnormally_request;
      this->describe_ = "abnormally request";
      return false;
    }
    std::string body_str = _msg.body;
    if (this->decorate_body_) {
      if (!this->py_interpreter_ptr_->DecorateBody(body_str)) {
        this->state_ = StateEnum::abnormally_decorate_body;
        this->describe_ = "decorate body";
        ERROR(
          "%s [%s] [Build] Decorate body failed.",
          this->logger_.c_str(),
          _msg.id.c_str());
        return false;
      }
    }
    if (this->JudgeModulesLoop(_msg, _dependent)) {
      this->state_ = StateEnum::abnormally_request;
      this->describe_ = "abnormally request";
      return false;
    }
    auto build =
      [&](const std::vector<std::string> & _header_data,
        const std::vector<std::string> _body_data) -> bool {
        std::ofstream file_stream(_file);
        if (!file_stream) {
          this->state_ = StateEnum::abnormally_open_file;
          this->describe_ = "Canot open module file:" + _file;
          ERROR(
            "%s [%s] [Build] %s.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            this->describe_.c_str());
          return false;
        }
        for (auto data : _header_data) {
          file_stream << data << std::endl;
        }
        for (auto data : _body_data) {
          file_stream << data << std::endl;
        }
        file_stream.close();
        return true;
      };
    std::vector<std::string> body_py = GetVector(body_str, '\n', "    ");
    if (!build(header_py, body_py)) {
      return false;
    }
    sleep(1);
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
    if (!shell(std::string(this->judge_python_ + _file))) {
      shell(std::string("rm " + _file + "*"));
      return false;
    }
    INFO(
      "%s [%s] Judge modules(%s) is ok.",
      this->logger_.c_str(),
      _msg.id.c_str(),
      _file.c_str());
  } catch (const std::exception & e) {
    this->state_ = StateEnum::abnormally_build;
    this->describe_ = "Canot build modules file.";
    ERROR(
      "%s [%s] [Build] %s.\nerror:%s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      this->describe_.c_str(),
      e.what());
    return false;
  }
  return true;
}

bool Module::DeleteTheModule(const OperateMsg & _msg)
{
  try {
    INFO(
      "%s [%s] Delete the modules(%s).",
      this->logger_.c_str(),
      _msg.id.c_str(),
      _msg.target_id.front().c_str());
    std::string file = "";
    auto shell = [&](std::string _cmd) -> bool {
        int shell_code;
        std::string shell_message;
        if (Shell(_cmd, shell_code, shell_message)) {
          INFO(
            "%s [%s] Judge modules can revise(%s) is ok.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _cmd.c_str());
          return true;
        } else {
          this->state_ = StateEnum::abnormally_build;
          this->describe_ = "Judge modules can revise is fail:" + shell_message;
          ERROR(
            "%s [%s] Judge modules can revise(%s) is fail.\ncode: %d\nmessage:%s",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _cmd.c_str(),
            shell_code,
            shell_message.c_str());
          return false;
        }
      };
    auto judge_delete = [&]() -> bool {
        OperateMsg target_meta;
        if (!GetMeta(_msg.target_id.front(), target_meta) ||
          !target_meta.be_depended.empty())
        {
          this->state_ = StateEnum::abnormally_build;
          this->describe_ = "Judge modules can delete is fail.";
          ERROR(
            "%s [%s] Judge modules can delete(%s) is fail.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _msg.target_id.front().c_str());
          return false;
        }
        file = target_meta.body;
        if (file.empty()) {
          this->describe_ = "Toml config file(" + this->registry_fil_ +
            ") is not have module(" + _msg.target_id.front() + ") file.";
          ERROR(
            "%s [%s] %s",
            this->logger_.c_str(),
            _msg.id.c_str(),
            this->describe_.c_str());
          return false;
        }
        return true;
      };
    if (judge_delete() &&
      shell(std::string("rm " + file)))
    {
      INFO(
        "%s [%s] Delete current module(%s) is ok",
        this->logger_.c_str(),
        _msg.id.c_str(),
        _msg.target_id.front().c_str());
      return true;
    }
  } catch (const std::exception & e) {
    ERROR(
      "%s [%s] Try to delete the module failed: %s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      e.what());
  }
  return false;
}

bool Module::ExecuteRequest(const OperateMsg & _msg, GRPCMsg & msg_)
{
  INFO("%s Execute Request ...", this->logger_.c_str());
  if ((_msg.operate != OperateMsg::OPERATE_INQUIRY) &&
    (_msg.target_id.empty()))
  {
    ERROR(
      "%s [%s] <%s> operate is need target_id, but target_id is empty.",
      this->logger_.c_str(),
      _msg.id.c_str(),
      _msg.operate.c_str());
    return false;
  }
  OperateMsg self_msg(_msg);
  std::string flow_state = OperateMsg::STATE_ERROR;
  std::string file = "";
  std::vector<std::string> dependent;
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
          ERROR(
            "%s [%s] <%s> operate is error.",
            this->logger_.c_str(),
            _msg.id.c_str(),
            _msg.operate.c_str());
          this->state_ = StateEnum::abnormally_request;
          this->describe_ = "abnormally request";
        }
      } catch (const std::exception & e) {
        ERROR(
          "%s [%s] <%s> operate is error:%s",
          this->logger_.c_str(),
          _msg.id.c_str(),
          _msg.operate.c_str(),
          e.what());
        this->state_ = StateEnum::abnormally_other_errors;
        this->describe_ = "abnormally other errors";
      }
      return false;
    };
  if (_msg.operate == OperateMsg::OPERATE_SAVE) {
    return execute(
      [&]() -> bool {
        if (!this->JudgeModulesUniquenes(_msg) ||  // 确保模块唯一
        !this->Build(_msg, file, dependent))
        {
          return false;
        }
        flow_state = OperateMsg::STATE_NORMAL;
        return this->SetList(_msg, flow_state, file, dependent);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
    return execute(
      [&]() -> bool {
        return this->GetList(_msg, msg_);
      });
  } else if (_msg.operate == OperateMsg::OPERATE_DELETE) {
    return execute(
      [&]() -> bool {
        return static_cast<bool>(this->DeleteTheModule(_msg) &&
        this->SetList(_msg, flow_state, file, dependent));
      });
  } else {
    WARN(
      "%s [%s] Unable to recognize current message operate:%s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      _msg.operate.c_str());
    this->state_ = StateEnum::abnormally_other_errors;
    this->describe_ = "abnormally other errors";
  }
  return false;
}
}   // namespace cyberdog_visual_programming_engine
