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

#include "cyberdog_vp_abilityset/log.hpp"

namespace cyberdog_visual_programming_abilityset
{
Log * Log::pinstance_ptr {nullptr};
std::mutex Log::mutex_;

Log * Log::GetInstance(
  rclcpp::Node::SharedPtr _node_ptr, const std::string & _task_id,
  const toml::value & _params_toml)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pinstance_ptr == nullptr) {
    pinstance_ptr = new Log(_node_ptr, _task_id, _params_toml);
  }
  return pinstance_ptr;
}

bool Log::SetData(const toml::value & _params_toml)
{
  try {
    INFO("%s", std::string(__FUNCTION__).c_str());
    auto make_dir = [&]() -> bool {
        std::string workspase = "";
        if (!GetWorkspace(workspase)) {
          ERROR("Get workspace is failed");
          return false;
        }
        this->file_ = workspase + "/log/" + this->task_id_ + ".csv";
        std::string file_path("");
        size_t file_path_lenght = this->file_.find_last_of('/');
        if (file_path_lenght != std::string::npos) {
          file_path = this->file_.substr(0, file_path_lenght);
          if (file_path.empty()) {
            ERROR("Log file (%s) path (%s) is empty.", this->file_.c_str(), file_path.c_str());
            return false;
          }
        }
        if (access(file_path.c_str(), F_OK) == 0) {
          return true;
        }
        if (mkdir(file_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
          ERROR(
            "Log file (%s) path (%s) is failed: %s.", this->file_.c_str(),
            file_path.c_str(), std::strerror(errno));
          return false;
        }
        return true;
      };
    if (this->task_id_.empty() || !make_dir()) {
      ERROR("Build log file is error.");
      return false;
    }
    this->log_file_.exceptions(std::ios_base::badbit);
    this->log_file_.open(this->file_.c_str(), std::ios::trunc);
    if (!this->log_file_.is_open()) {
      ERROR("Log file (%s) open is failed.", this->file_.c_str());
      return false;
    }

    INFO("The log of the current task is saved in a file:%s", this->file_.c_str());
    this->record_ = true;
    this->log_file_ << "level,time,role,message" << std::endl;

    this->publish_log_ = toml::find_or(
      _params_toml, "vp", "init", "environment", "publish_log",
      false);
  } catch (const std::exception & e) {
    ERROR("Set data failed: %s", e.what());
    return false;
  }
  return true;
}

bool Log::SetMechanism(const toml::value & _params_toml)
{
  try {
    INFO("%s", std::string(__FUNCTION__).c_str());
    this->publish_log_cb_group_ =
      this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->publish_log_cb_group_;
    this->publish_log_pub_ = this->node_ptr_->create_publisher<MsgHeader>(
      toml::find_or(
        _params_toml, "vp", "init", "topic", "publish_log", "debug_info"),
      PublisherQos,
      pub_option);
  } catch (const std::exception & e) {
    ERROR("Set mechanism failed: %s", e.what());
    return false;
  }
  return true;
}

char * Log::Trim(char * str)
{
  /*
  标准的空白字符包括：
  ' '     (0x20)    space (SPC) 空格符
  '\t'    (0x09)    horizontal tab (TAB) 水平制表符
  '\n'    (0x0a)    newline (LF) 换行符
  '\v'    (0x0b)    vertical tab (VT) 垂直制表符
  '\f'    (0x0c)    feed (FF) 换页符
  '\r'    (0x0d)    carriage return (CR) 回车符
  //windows \r\n linux \n mac \r
  */
  auto left_trim = [&]() {
      if (str != nullptr && *str != '\0') {
        int len = 0;
        char * p = str;
        while (*p != '\0' && isspace(*p)) {
          ++p;
          ++len;
        }
        memmove(str, p, strlen(str) - len + 1);
      }
    };
  auto right_trim = [&]() {
      if (str != nullptr && *str != '\0') {
        int len = strlen(str);
        char * p = str + len - 1;
        while (p >= str && isspace(*p)) {
          *p = '\0';
          --p;
        }
      }
    };
  left_trim();
  right_trim();
  return str;
}
}   // namespace cyberdog_visual_programming_abilityset
