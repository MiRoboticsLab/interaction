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

#include "cyberdog_vp_engine/base.hpp"

namespace cyberdog_visual_programming_engine
{
Base::Base(std::string _type)
: type_(_type)
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "][" + _type + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

Base::~Base()
{
  INFO("%s Destroy object(node)", this->logger_.c_str());
}

bool Base::Init(
  const rclcpp::Node::SharedPtr & _node_ptr,
  const std::shared_ptr<PythonInterpreter> & _py_interpreter_ptr,
  const toml::value & _params_toml)
{
  try {
    INFO("%s Initializing ...", this->logger_.c_str());
    this->node_ptr_ = _node_ptr;
    this->py_interpreter_ptr_ = _py_interpreter_ptr;
    this->params_toml_ = _params_toml;
    if (!GetWorkspace(this->base_path_)) {
      ERROR("%s Get workspace failed.", this->logger_.c_str());
      return false;
    }
    auto init_registry_fil = [&](std::string & target_fil, const std::string _type) -> bool {
        target_fil = this->base_path_ + "/" + _type + "/" + _type + ".toml";
        if (!JudgeConfileFile(target_fil)) {
          return false;
        }
        return true;
      };
    if (((this->type_ == OperateMsg::TYPE_TASK) &&
      init_registry_fil(this->registry_fil_, OperateMsg::TYPE_TASK) &&
      init_registry_fil(this->registry_related_fil_, OperateMsg::TYPE_MODULE)) ||
      ((this->type_ == OperateMsg::TYPE_MODULE) &&
      init_registry_fil(this->registry_fil_, OperateMsg::TYPE_MODULE) &&
      init_registry_fil(this->registry_related_fil_, OperateMsg::TYPE_TASK)))
    {
      INFO("%s Init registry file is success.", this->logger_.c_str());
    } else {
      ERROR("%s Init registry file is failed.", this->logger_.c_str());
      return false;
    }
    this->header_sh_.insert(this->header_sh_.end(), LICENSES.begin(), LICENSES.end());
    this->header_sh_.insert(
      this->header_sh_.end(),
      BASH_CONFIGURATION.begin(), BASH_CONFIGURATION.end());
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return this->InitData();
}

bool Base::InitData()
{
  try {
    INFO("%s Initializing data...", this->logger_.c_str());
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Base::RespondToRequests(const OperateMsg & _msg, GRPCMsg & msg_)
{
  bool ret = true;
  this->state_ = StateEnum::normally;
  this->describe_ = "normally";
  ret = this->ExecuteRequest(_msg, msg_);
  if (msg_.data.empty()) {
    this->getRobotendMsg(_msg, msg_);
  }
  return ret;
}

void Base::getRobotendMsg(const OperateMsg & _msg, GRPCMsg & msg_)
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

bool Base::ExecuteRequest(const OperateMsg & _msg, GRPCMsg &)
{
  INFO(
    "%s [ExecuteRequest] Business logic to be implemented:%s",
    this->logger_.c_str(), _msg.id.c_str());
  return false;
}

bool Base::GetRegistryToml(toml::value & _registry_toml, bool _is_registry_file)
{
  try {
    if (_is_registry_file) {
      if (cyberdog::common::CyberdogToml::ParseFile(
          this->registry_fil_.c_str(), _registry_toml))
      {
        return true;
      }
    } else {
      if (cyberdog::common::CyberdogToml::ParseFile(
          this->registry_related_fil_.c_str(), _registry_toml))
      {
        return true;
      }
    }
    ERROR(
      "%s Toml config file is not in toml format, config file dir:\n%s",
      this->logger_.c_str(),
      _is_registry_file ?
      this->registry_fil_.c_str() :
      this->registry_related_fil_.c_str());
  } catch (const std::exception & e) {
    ERROR("%s [GetRegistryToml] error:%s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Base::SetRegistryToml(const toml::value & _registry_toml, bool _is_registry_file)
{
  try {
    if (_registry_toml.is_table()) {
      if (_is_registry_file) {
        std::ofstream ofs(this->registry_fil_, std::ofstream::out);
        ofs << _registry_toml;
        ofs.close();
        return true;
      } else {
        std::ofstream ofs(this->registry_related_fil_, std::ofstream::out);
        ofs << _registry_toml;
        ofs.close();
        return true;
      }
    }
  } catch (const std::exception & e) {
    ERROR("%s [SetRegistryToml] error:%s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Base::GetState(const std::string _id, std::string & now_state)
{
  toml::value registry_toml;
  if (!this->GetRegistryToml(registry_toml)) {
    return false;
  }
  now_state = toml::find_or(
    registry_toml, this->type_, _id, "state",
    OperateMsg::STATE_NULL);
  return true;
}

bool Base::SetList(
  const OperateMsg & _msg, const std::string & _state, const std::string & _file,
  std::vector<std::string> & _dependent)
{
  try {
    INFO(
      "%s [%s] SetList(%s, %s, %s).",
      this->logger_.c_str(),
      _msg.id.c_str(),
      _msg.operate.c_str(),
      _state.c_str(),
      _file.c_str());
    std::lock_guard<std::mutex> guard_registry(registry_toml_mutex);
    std::lock_guard<std::mutex> guard_registry_related(registry_related_toml_mutex);
    if (_msg.operate == OperateMsg::OPERATE_INQUIRY) {
      return true;
    }
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      return false;
    }
    auto get_operation = [&]() -> std::string {
        return std::string(
          GetTime(static_cast<int>(TimeMode::STANDARD)) + " >> " + _msg.operate +
          " operation.");
      };
    auto other_operate = [&]() -> bool {
        return static_cast<bool>(
          (_msg.operate == OperateMsg::OPERATE_SHUTDOWN) ||
          (_msg.operate == OperateMsg::OPERATE_SUSPEND) ||
          (_msg.operate == OperateMsg::OPERATE_RECOVER) ||
          (_msg.operate == OperateMsg::OPERATE_START) ||
          (_msg.operate == OperateMsg::OPERATE_STOP));
      };
    auto add_mode_be_depended = [&](toml::value & module_toml) -> bool {
        std::string target_id = _msg.target_id.front();
        const toml::value module_table = toml::find(module_toml, OperateMsg::TYPE_MODULE);
        if (!module_table.is_table()) {return false;}
        auto now_unmap = module_table.as_table();
        for (auto & module_id : _dependent) {
          if (now_unmap.find(module_id) != now_unmap.end()) {
            const toml::value be_depended_array =
              toml::find(module_toml[OperateMsg::TYPE_MODULE][module_id], "be_depended");
            if (!be_depended_array.is_array()) {return false;}
            std::vector<std::string> now_vector =
              toml::find<std::vector<std::string>>(
              module_toml[OperateMsg::TYPE_MODULE][module_id],
              "be_depended");
            if (std::find(now_vector.begin(), now_vector.end(), target_id) == now_vector.end()) {
              module_toml[OperateMsg::TYPE_MODULE][module_id]["be_depended"].push_back(target_id);
            }
          } else {
            WARN(
              "%s [%s] Add be depended is fild. module %s is not find.",
              this->logger_.c_str(),
              _msg.id.c_str(),
              module_id.c_str());
            return false;
          }
        }
        return true;
      };
    auto add_be_depended = [&]() -> bool {
        std::string id = _msg.target_id.front();
        if ((id == "id") ||
          (id == OperateMsg::OPERATE_DEBUG) ||
          (id == "terminal_default") ||
          (id == "visual_default"))
        {
          return true;
        }
        if (this->type_ == OperateMsg::TYPE_TASK) {
          toml::value registry_related_toml;
          if (!this->GetRegistryToml(registry_related_toml, false)) {
            return false;
          }
          if (!add_mode_be_depended(registry_related_toml)) {
            return false;
          }
          return this->SetRegistryToml(registry_related_toml, false);
        } else {
          return add_mode_be_depended(registry_toml);
        }
      };
    auto delete_mode_be_depended = [&](toml::value & module_toml) -> bool {
        std::string target_id = _msg.target_id.front();
        const toml::value module_table = toml::find(module_toml, OperateMsg::TYPE_MODULE);
        if (!module_table.is_table()) {return false;}
        auto now_unmap = module_table.as_table();
        for (auto & module_id : _dependent) {
          if (now_unmap.find(module_id) != now_unmap.end()) {
            const toml::value be_depended_array =
              toml::find(module_toml[OperateMsg::TYPE_MODULE][module_id], "be_depended");
            if (!be_depended_array.is_array()) {return false;}
            std::vector<std::string> now_vector =
              toml::find<std::vector<std::string>>(
              module_toml[OperateMsg::TYPE_MODULE][module_id],
              "be_depended");
            const auto target_id_iterator = std::find(
              now_vector.begin(),
              now_vector.end(), target_id);
            if (target_id_iterator != now_vector.end()) {
              now_vector.erase(target_id_iterator);
              module_toml[OperateMsg::TYPE_MODULE][module_id]["be_depended"] = toml::array();
              for (auto meta : now_vector) {
                module_toml[OperateMsg::TYPE_MODULE][module_id]["be_depended"].push_back(
                  toml::string(
                    meta,
                    toml::string_t::literal));
              }
            }
          } else {
            WARN(
              "%s [%s] Delete be depended is fild. module %s is not find.",
              this->logger_.c_str(),
              _msg.id.c_str(),
              module_id.c_str());
            return false;
          }
        }
        return true;
      };
    auto delete_be_depended = [&]() -> bool {
        if (this->type_ == OperateMsg::TYPE_TASK) {
          toml::value registry_related_toml;
          if (!this->GetRegistryToml(registry_related_toml, false)) {
            return false;
          }
          if (!delete_mode_be_depended(registry_related_toml)) {
            return false;
          }
          return this->SetRegistryToml(registry_related_toml, false);
        } else {
          return delete_mode_be_depended(registry_toml);
        }
      };
    if ((_msg.operate == OperateMsg::OPERATE_SAVE) ||
      (_msg.operate == OperateMsg::OPERATE_DEBUG))
    {
      // toml::string_t::basic    // 默认类型: 会将长字符串xxx转为"""xxx\"""
      // toml::string_t::literal  // 原文类型: 会将长字符串xxx转为'xxx'
      toml::value element;
      element["file"] = _file;
      element["mode"] = _msg.mode;
      element["condition"] = _msg.condition;
      element["style"] = toml::string(_msg.style, toml::string_t::literal);
      element["describe"] = toml::string(_msg.describe, toml::string_t::literal);
      element["state"] = _state;
      element["dependent"] = toml::array();
      for (auto meta : _dependent) {
        element["dependent"].push_back(toml::string(meta, toml::string_t::literal));
      }
      element["be_depended"] = toml::array();
      element["operate"] = toml::array();
      element["operate"].push_back(get_operation());
      registry_toml[this->type_][_msg.target_id.front()] = element;
      add_be_depended();
    } else if (_msg.operate == OperateMsg::OPERATE_DELETE) {
      const toml::value now_lists = toml::find(registry_toml, this->type_);
      if (!now_lists.is_table()) {return false;}
      const toml::value be_depended_array =
        toml::find(registry_toml[this->type_][_msg.target_id.front()], "dependent");
      if (!be_depended_array.is_array()) {return false;}
      _dependent =
        toml::find<std::vector<std::string>>(
        registry_toml[this->type_][_msg.target_id.front()], "dependent");
      auto now_unmap = now_lists.as_table();
      now_unmap.erase(now_unmap.find(_msg.target_id.front()));
      registry_toml[this->type_] = toml::value(now_unmap);
      delete_be_depended();
      // registry_toml[this->type_][_msg.target_id.front()]["state"] = _state;
      // registry_toml[this->type_][_msg.target_id.front()]["operate"].push_back(get_operation());
    } else if ((_msg.operate == OperateMsg::OPERATE_RUN)) {
      if ((!_msg.mode.empty())) {
        registry_toml[this->type_][_msg.target_id.front()]["mode"] = _msg.mode;
      }
      if ((!_msg.condition.empty())) {
        registry_toml[this->type_][_msg.target_id.front()]["condition"] = _msg.condition;
      }
      registry_toml[this->type_][_msg.target_id.front()]["state"] = _state;
      registry_toml[this->type_][_msg.target_id.front()]["operate"].push_back(get_operation());
    } else if (other_operate()) {
      registry_toml[this->type_][_msg.target_id.front()]["state"] = _state;
      registry_toml[this->type_][_msg.target_id.front()]["operate"].push_back(get_operation());
    } else {
      WARN(
        "%s [%s] Unable to recognize current message operate:%s",
        this->logger_.c_str(),
        _msg.id.c_str(),
        _msg.operate.c_str());
    }
    return this->SetRegistryToml(registry_toml);
  } catch (const std::exception & e) {
    this->state_ = StateEnum::abnormally_update_list;
    this->describe_ = "Canot update registry list.";
    ERROR(
      "%s [%s] [SetList] %s.\nerror:%s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      this->describe_.c_str(),
      e.what());
  }
  return false;
}

bool Base::GetList(const OperateMsg & _msg, GRPCMsg & msg_)
{
  try {
    INFO(
      "%s [%s] GetList.",
      this->logger_.c_str(),
      _msg.id.c_str());
    std::lock_guard<std::mutex> guard_registry(registry_toml_mutex);
    std::lock_guard<std::mutex> guard_registry_related(registry_related_toml_mutex);
    using TomlList = std::vector<OperateMsg>;
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      return false;
    }
    const toml::value now_lists = toml::find(registry_toml, this->type_);
    if (!now_lists.is_table()) {return false;}
    TomlList tag_list;
    std::string id = "";
    for (const auto & meta : now_lists.as_table()) {
      id = meta.first;
      if ((id == "id") ||
        (id == "terminal_default") ||
        (id == "visual_default") ||
        (id == OperateMsg::OPERATE_DEBUG) ||
        ((!_msg.target_id.empty()) &&
        (std::find(_msg.target_id.begin(), _msg.target_id.end(), id) == _msg.target_id.end())))
      {
        continue;
      }
      OperateMsg _meta;
      if (!this->GetMeta(registry_toml, id, _meta)) {
        continue;
      }
      if (_meta.operate == OperateMsg::STATE_NULL) {
        continue;
      }
      if (_msg.mode.empty() || (_msg.mode == _meta.mode)) {
        tag_list.push_back(_meta);
      }
    }
    std::sort(
      tag_list.begin(), tag_list.end(),
      [](const OperateMsg a, const OperateMsg b) {
        return a.id < b.id;
      });
    rapidjson::Document robotend_json;
    robotend_json.SetObject();
    rapidjson::Document::AllocatorType & allocator = robotend_json.GetAllocator();
    rapidjson::Value response_json(rapidjson::kObjectType);
    response_json.SetObject();
    response_json.AddMember("type", rapidjson::StringRef(this->type_.c_str()), allocator);
    response_json.AddMember("id", rapidjson::StringRef(_msg.id.c_str()), allocator);
    rapidjson::Value list_json(rapidjson::kArrayType);
    for (const auto & now_meta : tag_list) {
      rapidjson::Value now_meta_json(rapidjson::kObjectType);
      now_meta_json.AddMember("id", rapidjson::StringRef((now_meta.id.c_str())), allocator);
      now_meta_json.AddMember(
        "describe", rapidjson::StringRef(
          (now_meta.describe.c_str())), allocator);
      now_meta_json.AddMember("style", rapidjson::StringRef((now_meta.style.c_str())), allocator);
      now_meta_json.AddMember("mode", rapidjson::StringRef(now_meta.mode.c_str()), allocator);
      now_meta_json.AddMember(
        "condition", rapidjson::StringRef(
          now_meta.condition.c_str()), allocator);
      now_meta_json.AddMember(
        "operate", rapidjson::StringRef((now_meta.operate.c_str())),
        allocator);

      rapidjson::Value dependent_list_json(rapidjson::kArrayType);
      for (auto & meta : now_meta.dependent) {
        dependent_list_json.PushBack(rapidjson::StringRef((meta.c_str())), allocator);
      }
      now_meta_json.AddMember(
        "dependent", dependent_list_json, allocator);

      rapidjson::Value be_depended_list_json(rapidjson::kArrayType);
      for (auto & meta : now_meta.be_depended) {
        be_depended_list_json.PushBack(rapidjson::StringRef((meta.c_str())), allocator);
      }
      now_meta_json.AddMember(
        "be_depended", be_depended_list_json, allocator);
      list_json.PushBack(now_meta_json, allocator);
    }
    response_json.AddMember("list", list_json, allocator);
    robotend_json.AddMember("response", response_json, allocator);
    CyberdogJson::Document2String(robotend_json, msg_.data);
    return true;
  } catch (const std::exception & e) {
    ERROR(
      "%s [%s] Get list data failed: %s",
      this->logger_.c_str(),
      _msg.id.c_str(),
      e.what());
  }
  return false;
}

bool Base::GetMeta(const toml::value & _toml, const std::string & _id, OperateMsg & _meta)
{
  try {
    _meta.type = this->type_;
    _meta.id = std::string(_id);
    _meta.describe = toml::find_or(_toml, this->type_, _id, "describe", "");
    _meta.style = toml::find_or(_toml, this->type_, _id, "style", "");
    _meta.operate = toml::find_or(_toml, this->type_, _id, "state", "");
    _meta.mode = toml::find_or(_toml, this->type_, _id, "mode", "");
    _meta.condition = toml::find_or(_toml, this->type_, _id, "condition", "");
    _meta.body = toml::find_or(_toml, this->type_, _id, "file", "");
    _meta.dependent = toml::find_or<std::vector<std::string>>(
      _toml, this->type_, _id, "dependent",
      std::vector<std::string>());
    _meta.be_depended = toml::find_or<std::vector<std::string>>(
      _toml, this->type_, _id,
      "be_depended",
      std::vector<std::string>());
    return true;
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Base::GetMeta(const std::string & _id, OperateMsg & _meta)
{
  try {
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      return false;
    }
    return this->GetMeta(registry_toml, _id, _meta);
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Base::BuildFrontendOperate(const std::string _operate, std::string & _frontend_msg)
{
  try {
    INFO(
      "%s [%s] BuildFrontendOperate.",
      this->logger_.c_str(),
      _operate.c_str());
    if ((_operate != OperateMsg::OPERATE_SHUTDOWN) &&
      (_operate != OperateMsg::OPERATE_SUSPEND) &&
      (_operate != OperateMsg::OPERATE_RECOVER))
    {
      return false;
    }
    std::vector<std::string> target_task_list;
    toml::value registry_toml;
    if (!this->GetRegistryToml(registry_toml)) {
      return false;
    }
    const toml::value now_lists = toml::find(registry_toml, this->type_);
    if (!now_lists.is_table()) {return false;}
    std::string now_id = "";
    std::string now_state = "";
    for (const auto & meta : now_lists.as_table()) {
      now_id = meta.first;
      if ((now_id == "id") ||
        (now_id == "terminal_default") ||
        (now_id == "visual_default"))
      {
        continue;
      }
      now_state = toml::find_or(registry_toml, this->type_, now_id, "state", "");
      if (( (_operate == OperateMsg::OPERATE_SHUTDOWN) &&
        ( (now_state == OperateMsg::STATE_WAIT_RUN) ||
        (now_state == OperateMsg::STATE_RUN_WAIT) ||
        (now_state == OperateMsg::STATE_RUN) ||
        (now_state == OperateMsg::STATE_SUSPEND))) ||
        ((_operate == OperateMsg::OPERATE_SUSPEND) &&
        ((now_state == OperateMsg::STATE_RUN))) ||
        ((_operate == OperateMsg::OPERATE_RECOVER) &&
        ((now_state == OperateMsg::STATE_SUSPEND))))
      {
        target_task_list.push_back(now_id);
      }
    }
    std::sort(
      target_task_list.begin(), target_task_list.end(),
      [](const std::string a, const std::string b) {
        return a < b;
      });
    std::string id = "ASR_" + GetTime();
    rapidjson::Document frontend_json;
    frontend_json.SetObject();
    rapidjson::Document::AllocatorType & allocator = frontend_json.GetAllocator();
    frontend_json.AddMember("type", rapidjson::StringRef(this->type_.c_str()), allocator);
    frontend_json.AddMember("id", rapidjson::StringRef(id.c_str()), allocator);
    rapidjson::Value target_id_json(rapidjson::kArrayType);
    for (const auto & tar_id : target_task_list) {
      target_id_json.PushBack(rapidjson::StringRef(tar_id.c_str()), allocator);
    }
    frontend_json.AddMember("target_id", target_id_json, allocator);
    frontend_json.AddMember("operate", rapidjson::StringRef(_operate.c_str()), allocator);
    frontend_json.AddMember("describe", rapidjson::StringRef(""), allocator);
    frontend_json.AddMember("style", rapidjson::StringRef(""), allocator);
    frontend_json.AddMember("mode", rapidjson::StringRef(""), allocator);
    frontend_json.AddMember("condition", rapidjson::StringRef(""), allocator);
    frontend_json.AddMember("body", rapidjson::StringRef(""), allocator);
    CyberdogJson::Document2String(frontend_json, _frontend_msg);
    return true;
  } catch (const std::exception & e) {
    ERROR(
      "%s [%s] Get list data failed: %s",
      this->logger_.c_str(),
      _operate.c_str(),
      e.what());
  }
  return false;
}
}   // namespace cyberdog_visual_programming_engine
