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
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "connector/uploader_log.hpp"

namespace cyberdog
{
namespace interaction
{
UploaderLog::UploaderLog(const std::string & name)
: Node(name)
{
  INFO("Creating [UploaderLog] object(node)");
}

UploaderLog::~UploaderLog()
{
  INFO("Destroy [UploaderLog] object(node)");
}

bool UploaderLog::Init(const rclcpp::Node::SharedPtr node)
{
  INFO("Initializing data ...");
  try {
    std::string node_config_dir = ament_index_cpp::get_package_share_directory("connector") +
      "/config/connector.toml";
    INFO("Params config file dir:<%s>", node_config_dir.c_str());

    if (access(node_config_dir.c_str(), F_OK)) {
      ERROR("Params config file does not exist");
      return false;
    }

    if (access(node_config_dir.c_str(), R_OK)) {
      ERROR("Params config file does not have read permissions");
      return false;
    }

    if (access(node_config_dir.c_str(), W_OK)) {
      ERROR("Params config file does not have write permissions");
      return false;
    }
    toml::value params_toml;
    if (!cyberdog::common::CyberdogToml::ParseFile(
        node_config_dir.c_str(), params_toml))
    {
      ERROR("Params config file is not in toml format");
      return false;
    }

    this->lcm_log_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->lcm_log_service_ = this->create_service<TriggerSrv>(
      toml::find<std::string>(
        params_toml, "connector", "initialization", "service", "lcm_log"),
      std::bind(&UploaderLog::Uploader, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->lcm_log_cb_group_);

    this->lcm_log_ptr_ = std::make_shared<LcmLogUploader>(node);
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

void UploaderLog::Uploader(
  const std::shared_ptr<TriggerSrv::Request>,
  std::shared_ptr<TriggerSrv::Response> response)
{
  try {
    if (this->lcm_log_ptr_ != nullptr) {
      int state = this->lcm_log_ptr_->checkAndUploadLcmLog();
      response->success = (state == 0) ? true : false;
      response->message = "The current return value of interface checkAndUploadLcmLog() is " +
        std::to_string(state);
    } else {
      WARN("Uploader lcm log is failed: <lcm_log_ptr_ == nullptr>");
    }
  } catch (const std::exception & e) {
    WARN("Connect service is failed: <%s>", e.what());
  }
}
}   // namespace interaction
}   // namespace cyberdog
