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
#include <map>

#include "cyberdog_vp_engine/linux_at.hpp"

namespace cyberdog_visual_programming_engine
{
LinuxAt::LinuxAt()
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

bool LinuxAt::Init()
{
  try {
    std::string base_path;
    if (!GetWorkspace(base_path)) {
      ERROR("%s Get workspace failed.", this->logger_.c_str());
      return false;
    }
    this->gerp_info_ = std::string(" | grep \"python3 ") + base_path + "/" + OperateMsg::TYPE_TASK +
      "/" + std::string("\" | awk '{print $3}' | awk -v FS=':=' '{print $(NF)}'");
    this->registry_fil_ = base_path + "/" + OperateMsg::TYPE_TASK + "/" + OperateMsg::TYPE_TASK +
      ".toml";
    if (!JudgeConfileFile(this->registry_fil_)) {
      ERROR("%s Judge Task confile File failed.", this->logger_.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    ERROR(
      "%s Init failed.%s",
      this->logger_.c_str(),
      e.what());
    return false;
  }
  return true;
}

bool LinuxAt::GetTaskAtMap(TaskAtMap & _task_at)
{
  try {
    INFO("%s %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
    int shell_code;
    std::string shell_message;
    auto shell = [&](std::string _cmd) -> bool {
        if (!Shell(_cmd, shell_code, shell_message)) {
          ERROR(
            "%s Get task list data failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    if (!shell("atq | awk '{print $1}'")) {return false;}
    std::vector<std::string> at_id_vector = GetVector(shell_message, '\n');
    if (!at_id_vector.empty()) {
      at_id_vector.pop_back();
    }
    for (auto at_id : at_id_vector) {
      if (!shell(
          std::string(
            std::string("at -c ") + at_id + this->gerp_info_
            // at -c 18 |
            // grep "python3 /opt/ros2/cyberdog/share/cyberdog_vp/workspace/task/" |
            // awk '{print $3}' | awk -v FS=':=' '{print $(NF)}'
          ))) {return false;}
      std::vector<std::string> task_id_vector = GetVector(shell_message, '\n');
      _task_at.insert(
        std::map<std::string,
        std::string>::value_type(task_id_vector.front(), at_id));
    }
    WARN_EXPRESSION(_task_at.empty(), "%s Now at task is null.", this->logger_.c_str());
    for (auto now : _task_at) {
      INFO("%s <%s> -> <%s>", this->logger_.c_str(), now.first.c_str(), now.second.c_str());
    }
    return true;
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool LinuxAt::Registration(
  const std::string & _condition,
  const std::string & _file)
{
  try {
    INFO(
      "%s %s(%s, %s)", this->logger_.c_str(),
      std::string(__FUNCTION__).c_str(),
      _condition.c_str(), _file.c_str());
    auto shell = [&](std::string _cmd) -> bool {
        int shell_code;
        std::string shell_message;
        INFO("%s cmd=<%s>", this->logger_.c_str(), _cmd.c_str());
        if (!Shell(_cmd, shell_code, shell_message)) {
          ERROR(
            "%s Registration task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    if (!_condition.empty() && !_file.empty() &&
      shell(std::string("at " + _condition + " -f " + _file + ".sh")))
    {
      return true;
    }
    ERROR(
      "%s %s(%s, %s) parameter is invalid", this->logger_.c_str(),
      std::string(__FUNCTION__).c_str(),
      _condition.c_str(), _file.c_str());
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool LinuxAt::Cancellation(const std::string & _id)
{
  try {
    INFO(
      "%s %s(%s)", this->logger_.c_str(),
      std::string(__FUNCTION__).c_str(), _id.c_str());
    auto shell = [&](std::string _cmd) -> bool {
        int shell_code;
        std::string shell_message;
        INFO("%s cmd=<%s>", this->logger_.c_str(), _cmd.c_str());
        if (!Shell(_cmd, shell_code, shell_message)) {
          WARN(
            "%s Cancellation task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    auto kill_task = [&]() -> bool {
        // ps -ef|grep 'cyberdog_vp_task:=task_id'|grep -v grep|awk '{print $2}'|xargs kill -9
        int shell_code;
        std::string shell_message;
        INFO("%s Kill task %s.", this->logger_.c_str(), _id.c_str());
        std::string get_task_pid_cmd = "ps -ef | grep 'cyberdog_vp_task:=" +
          _id + "' | grep -v grep | awk '{print $2}'";
        if (!Shell(get_task_pid_cmd, shell_code, shell_message)) {
          ERROR(
            "%s Kill task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), get_task_pid_cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        std::vector<std::string> shell_return = GetVector(shell_message, '\n');
        if (shell_return.size() < 2) {
          WARN(
            "%s Kill task return size is failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), get_task_pid_cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        if (shell_return.front().empty()) {
          WARN(
            "%s Kill task return  front is empty:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), get_task_pid_cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        get_task_pid_cmd += " | xargs kill -9";
        if (!Shell(get_task_pid_cmd, shell_code, shell_message)) {
          WARN(
            "%s Kill task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), get_task_pid_cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    TaskAtMap task_at;
    if (!_id.empty() &&
      this->GetTaskAtMap(task_at) &&
      (task_at.count(_id) == 1) &&
      shell(std::string("atrm " + task_at.at(_id))) &&
      kill_task())
    {
      return true;
    }
    WARN(
      "%s %s(%s) parameter is invalid", this->logger_.c_str(),
      std::string(__FUNCTION__).c_str(), _id.c_str());
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}
}   // namespace cyberdog_visual_programming_engine
