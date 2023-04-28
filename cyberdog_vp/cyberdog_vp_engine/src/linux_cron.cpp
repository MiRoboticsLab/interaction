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

#include "cyberdog_vp_engine/linux_cron.hpp"

namespace cyberdog_visual_programming_engine
{
LinuxCron::LinuxCron()
{
  this->logger_ = "[" + std::string(__FUNCTION__) + "]";
  INFO("%s Creating object(node)", this->logger_.c_str());
}

bool LinuxCron::Init()
{
  try {
    std::string base_path;
    if (!GetWorkspace(base_path)) {
      ERROR("%s Get workspace failed.", this->logger_.c_str());
      return false;
    }
    this->registry_fil_ = base_path + "/" + OperateMsg::TYPE_TASK + "/" + OperateMsg::TYPE_TASK +
      ".toml";
    this->crontab_file_ = base_path + "/" + OperateMsg::TYPE_TASK + "/cyberdog.crontab";
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

bool LinuxCron::UpdateCrontabFile(
  const std::vector<std::string> & _new_crontab, const bool _create)
{
  try {
    INFO("%s %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
    std::ofstream crontab_stream(this->crontab_file_);
    if (!crontab_stream) {
      ERROR(
        "%s [Update] Canot open task file: %s.",
        this->logger_.c_str(), this->crontab_file_.c_str());
      return false;
    }
    if (_create) {
      for (auto data : LICENSES) {
        crontab_stream << data << std::endl;
      }
      for (auto data : CRONTAB_EXAMPLE) {
        crontab_stream << data << std::endl;
      }
    }
    for (auto now : _new_crontab) {
      INFO("%s new task: <%s>", this->logger_.c_str(), now.c_str());
      if (!now.empty()) {
        crontab_stream << now << std::endl;
      }
    }
    crontab_stream.close();
    sleep(1);
    return true;
  } catch (const std::exception & e) {
    ERROR("%s Init object fail, %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool LinuxCron::GetNowCrontab(std::vector<std::string> & _now_crontab)
{
  try {
    INFO("%s %s()", this->logger_.c_str(), std::string(__FUNCTION__).c_str());
    _now_crontab.clear();
    int shell_code;
    std::string shell_message;
    auto shell = [&](std::string _cmd) -> bool {
        if (!Shell(_cmd, shell_code, shell_message)) {
          WARN(
            "%s Get now crontab task list data failed:\ncmd:%s\ncode:%d\nreturn:%s"
            "\nA new crontab file will be created.",
            this->logger_.c_str(), _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    // 筛选出可视化编程任务
    // if (!shell("crontab -l  | awk '/^[^#](.*)(task_)+(.*).sh$/'")) return false;
    // 筛选出非可视化编程任务
    // if (!shell("crontab -l  | awk '/^[^#](.*)[^(task_(.*).sh)]$/'")) return false;
    // 筛选所有 crontab 任务
    // if (!shell("crontab -l")) {return false;}
    if (shell("crontab -l")) {
      _now_crontab = GetVector(shell_message, '\n');
      if (!_now_crontab.empty()) {
        _now_crontab.pop_back();
      }
      for (auto now : _now_crontab) {
        INFO("%s old task: <%s>", this->logger_.c_str(), now.c_str());
      }
      return true;
    }
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool LinuxCron::Registration(
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
        if (!Shell(_cmd, shell_code, shell_message)) {
          ERROR(
            "%s Register task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    if (_condition.empty() || _file.empty()) {
      ERROR(
        "%s %s(%s, %s) parameter is invalid", this->logger_.c_str(),
        std::string(__FUNCTION__).c_str(),
        _condition.c_str(), _file.c_str());
      return false;
    }
    std::vector<std::string> now_tasks;
    bool creat_cron_file = !this->GetNowCrontab(now_tasks);
    now_tasks.push_back(std::string(_condition + " " + _file + ".sh"));
    if (this->UpdateCrontabFile(now_tasks, creat_cron_file) &&
      shell(std::string("crontab " + this->crontab_file_)))
    {
      return true;
    }
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool LinuxCron::Cancellation(const std::string & _id)
{
  try {
    INFO(
      "%s %s(%s)", this->logger_.c_str(),
      std::string(__FUNCTION__).c_str(), _id.c_str());
    auto shell = [&](std::string _cmd) -> bool {
        int shell_code;
        std::string shell_message;
        if (!Shell(_cmd, shell_code, shell_message)) {
          WARN(
            "%s Register task failed:\ncmd:%s\ncode:%d\nreturn:%s",
            this->logger_.c_str(), _cmd.c_str(), shell_code, shell_message.c_str());
          return false;
        }
        return true;
      };
    if (_id.empty()) {
      ERROR(
        "%s %s(%s) parameter is invalid", this->logger_.c_str(),
        std::string(__FUNCTION__).c_str(), _id.c_str());
      return false;
    }
    std::vector<std::string> now_tasks;
    std::vector<std::string> new_tasks;
    if (this->GetNowCrontab(now_tasks)) {
      for (auto data : now_tasks) {
        if (data.find(_id) == data.npos) {
          new_tasks.push_back(data);
        }
      }
      if (this->UpdateCrontabFile(new_tasks) &&
        shell(std::string("crontab " + this->crontab_file_)))
      {
        return true;
      }
    }
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}
}   // namespace cyberdog_visual_programming_engine
