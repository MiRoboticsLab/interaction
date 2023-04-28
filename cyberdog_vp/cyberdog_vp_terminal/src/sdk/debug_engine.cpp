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
#include <utility>

#include "cyberdog_vp_terminal/debug_engine.hpp"

namespace cyberdog_visual_programming_terminal
{
DebugEngine::DebugEngine()
: node_config_dir_("/config/debug_engine.toml")
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  DEBUG("%s Creating object(node)", this->logger_.c_str());
}

DebugEngine::~DebugEngine()
{
  DEBUG("%s Destroy object(node)", this->logger_.c_str());
}

bool DebugEngine::Init(const std::string & _namespace)
{
  try {
    DEBUG("%s Initializing ...", this->logger_.c_str());
    this->node_ptr_ = rclcpp::Node::make_shared("cyberdog_vp_terminal_debug_engine", _namespace);
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
    this->grpc_sub_ = this->node_ptr_->create_subscription<GRPCMsg>(
      toml::find<std::string>(
        this->params_toml_, "params", "topic", "robotend"),
      rclcpp::ParametersQoS(),
      std::bind(&DebugEngine::RobotendMsgCallback, this, std::placeholders::_1));
    this->grpc_pub_ = this->node_ptr_->create_publisher<GRPCMsg>(
      toml::find<std::string>(
        this->params_toml_, "params", "topic", "frontend"),
      rclcpp::ParametersQoS());

    std::thread spin_thread([&] {
        DEBUG("%s Spin ...", this->logger_.c_str());
        rclcpp::spin(this->node_ptr_);
        rclcpp::shutdown();
        return false;
      });
    spin_thread.detach();
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s.", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void DebugEngine::RobotendMsgCallback(const GRPCMsg::SharedPtr _robotend_msg_ptr)
{
  try {
    CoutJson(this->logger_ + "[Robotend]", _robotend_msg_ptr->data);
  } catch (const std::exception & e) {
    WARN(
      "Error responding to frontend message:\nmessage:%s\nerror:%s",
      _robotend_msg_ptr->data.c_str(), e.what());
  }
}

void DebugEngine::FrontendMsgPublisher(const OperateMsg & _msg)
{
  try {
    GRPCMsg frontend_msg;
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
    CyberdogJson::Document2String(frontend_json, frontend_msg.data);
    CoutJson(this->logger_ + "[Frontend]", frontend_msg.data);
    this->grpc_pub_->publish(frontend_msg);
  } catch (const std::exception & e) {
    WARN(
      "Error publisher to frontend message:\nerror:%s",
      e.what());
  }
}

void DebugEngine::MockAppRequest(const std::string & _target)
{
  if (_target.empty()) {
    WARN("Mock App request error, _target is empty.");
    return;
  }
  std::vector<std::string> target_keys = GetVector(_target, '.');
  toml::value target_toml = this->params_toml_;
  OperateMsg operate_msg;
  size_t index = 0;
  std::function<bool()> update_target = [&]() -> bool {
      if (!target_toml.is_table()) {return false;}
      if (index < target_keys.size()) {
        if (!target_keys.at(index).empty()) {
          toml::value new_target_toml = toml::find_or(
            target_toml, target_keys.at(
              index), toml::value());
          if (!new_target_toml.is_table()) {
            WARN(
              "Mock App request error, [%s] has illegal parameter: [%s].",
              _target.c_str(),
              target_keys.at(index).c_str());
            return false;
          }
          target_toml = new_target_toml;
        }
        ++index;
        return update_target();
      }
      return true;
    };
  auto get_str = [&](const std::string & _key, std::string & data) -> bool {
      auto now_key = toml::find(target_toml, _key);
      if (now_key.is_string()) {
        data = toml::find_or(target_toml, _key, "");
      } else if (now_key.is_array()) {
        data = "";
        for (const auto & meta : now_key.as_array()) {
          data += "\n" + std::string(meta.as_string());
        }
      } else {
        return false;
      }
      return true;
    };
  auto get_strs = [&](const std::string & _key, std::vector<std::string> & data) -> bool {
      auto now_lists = toml::find(target_toml, _key);
      if (!now_lists.is_array()) {
        return false;
      }
      data.clear();
      for (const auto & meta : now_lists.as_array()) {
        data.push_back(meta.as_string());
      }
      return true;
    };
  auto update_operate = [&]() -> bool {
      // type = "task"
      // id = "requested_1"
      // target_id = ["task_1"]
      // describe = "测试保存任务功能"
      // style = "{\"blocks\":{\"languageVersion\":0}}"
      // operate = "save"
      // mode = "single"
      // condition = "now"
      // body = [
      // "   cyberdog.audio.play('单次任务执行成功。')",
      //     ]
      return static_cast<bool>(get_str("type", operate_msg.type) &&
             get_str("id", operate_msg.id) &&
             get_strs("target_id", operate_msg.target_id) &&
             get_str("describe", operate_msg.describe) &&
             get_str("style", operate_msg.style) &&
             get_str("operate", operate_msg.operate) &&
             get_str("mode", operate_msg.mode) &&
             get_str("condition", operate_msg.condition) &&
             get_str("body", operate_msg.body));
    };

  if (update_target() && update_operate()) {
    this->FrontendMsgPublisher(operate_msg);
  } else {
    WARN("Mock App request error, update and get target is error.");
  }
}

}   // namespace cyberdog_visual_programming_terminal
