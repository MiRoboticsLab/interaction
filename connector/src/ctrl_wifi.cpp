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
#include <map>
#include "connector/ctrl_wifi.hpp"

namespace cyberdog
{
namespace interaction
{
CtrlWifi::CtrlWifi(const std::string name)
: Node(name)
{
  INFO("Creating [CtrlWifi] object(node)");
  if (!this->Init()) {exit(-1);}
}

CtrlWifi::~CtrlWifi()
{
  INFO("Destroy [CtrlWifi] object(node)");
}

bool CtrlWifi::Init()
{
  try {
    INFO("Initializing data ...");
    toml::value params_toml_;
    std::string params_pkg_dir_ = ament_index_cpp::get_package_share_directory("connector");
    std::string node_config_dir_ = params_pkg_dir_ + "/config/connector.toml";
    INFO("Params config file dir:<%s>", node_config_dir_.c_str());
    if (access(node_config_dir_.c_str(), F_OK)) {
      ERROR("Params config file does not exist");
      return false;
    }

    if (access(node_config_dir_.c_str(), R_OK)) {
      ERROR("Params config file does not have read permissions");
      return false;
    }

    if (access(node_config_dir_.c_str(), W_OK)) {
      ERROR("Params config file does not have write permissions");
      return false;
    }

    if (!cyberdog::common::CyberdogToml::ParseFile(
        node_config_dir_.c_str(), params_toml_))
    {
      ERROR("Params config file is not in toml format");
      return false;
    }
    this->client_ = this->create_client<WiFiSrv>(
      toml::find<std::string>(
        params_toml_, "connector", "initialization", "service", "wifi"));

    this->wifi_response_result_.insert(
      std::map<uint8_t, std::string>::value_type(
        WiFiSrv::Response::RESULT_NO_SSID,
        "4:The wifi module did not find the given wifi name."));
    this->wifi_response_result_.insert(
      std::map<uint8_t, std::string>::value_type(
        WiFiSrv::Response::RESULT_ERR_PWD, "5:Password error."));
    this->wifi_response_result_.insert(
      std::map<uint8_t, std::string>::value_type(
        WiFiSrv::Response::RESULT_OTHER, "6:Other situations."));
    this->wifi_response_result_.insert(
      std::map<uint8_t, std::string>::value_type(
        WiFiSrv::Response::RESULT_SUCCESS, "7:Connection succeeded."));
    this->wifi_response_result_.insert(
      std::map<uint8_t, std::string>::value_type(
        WiFiSrv::Response::RESULT_INTERRUPT, "14:Process is interrupted."));
    this->wifi_response_result_.insert(
      std::map<uint8_t, std::string>::value_type(
        WiFiSrv::Response::RESULT_TIMEOUT, "15:The wifi module connection timed out."));
    this->wait_for_service_timeout_s = toml::find_or(
      params_toml_, "connector", "initialization", "timeout_s", "wait_for_service",
      this->wait_for_service_timeout_s);
    this->wait_for_service_response_timeout_s = toml::find_or(
      params_toml_, "connector", "initialization", "timeout_s", "wait_for_wifi_service_response",
      this->wait_for_service_response_timeout_s);
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

uint CtrlWifi::ControlWifi(const std::string name, const std::string password)
{
  /* return:
   * 100: 控制LED成功
   * 101: 客户端在请求服务出现时被打断
   * 102: 等待服务出现（启动）超时
   * 103: 请求WIFI模块失败
   * ---
   * 4: 没找到请求的wifi
   * 5: 密码错误
   * 6: 其它情况
   * 7: 连接成功
   * 14: 过程被中断
   * 15: 连接超时
   * ---
   */
  try {
    INFO(
      "Connect wifi:{name=<%s>, password=<%s>}", name.c_str(), password.c_str());
    auto request = std::make_shared<WiFiSrv::Request>();
    request->ssid = name;
    request->pwd = password;
    if (!rclcpp::ok()) {
      WARN("Client interrupted while requesting for service to appear.");
      return 101;
    }
    if (!this->client_->wait_for_service(std::chrono::seconds(this->wait_for_service_timeout_s))) {
      WARN("Waiting for service to appear(start) timeout.");
      return 102;
    }
    auto result = this->client_->async_send_request(request);
    std::future_status status =
      result.wait_for(std::chrono::seconds(this->wait_for_service_response_timeout_s));
    if (status != std::future_status::ready) {
      WARN(
        "Request led module timedout (%d) or deferred.",
        this->wait_for_service_response_timeout_s);
      return 103;
    }
    auto response_ptr = result.get();
    auto wifi_result_msgs = this->wifi_response_result_.find(response_ptr->result);
    std::string result_msgs =
      static_cast<bool>(wifi_result_msgs != this->wifi_response_result_.end()) ?
      wifi_result_msgs->second.c_str() : "return result is illegal";
    switch (response_ptr->result) {
      case WiFiSrv::Response::RESULT_SUCCESS:
        INFO(
          "Connection WiFi OK: <%d:%s>", static_cast<int>(response_ptr->result),
          result_msgs.c_str());
        break;
      default:
        WARN(
          "Connection WiFi failed: <%d:%s>", static_cast<int>(response_ptr->result),
          result_msgs.c_str());
        break;
    }
    return response_ptr->result;
  } catch (...) {
    WARN("Do connect wifi is failed");
  }
  return 104;
}

}   // namespace interaction
}   // namespace cyberdog
