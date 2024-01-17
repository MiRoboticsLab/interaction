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
#include "connector/ctrl_led.hpp"

namespace cyberdog
{
namespace interaction
{
CtrlLed::CtrlLed(const std::string name)
: Node(name)
{
  INFO("Creating [CtrlLed] object(node)");
  if (!this->Init()) {exit(-1);}
}

CtrlLed::~CtrlLed()
{
  INFO("Destroy [CtrlLed] object(node)");
}

bool CtrlLed::Init()
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

    this->client_ = this->create_client<LedSrv>(
      toml::find<std::string>(
        params_toml_, "connector", "initialization", "service", "led"));
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

uint CtrlLed::EnableLed(
  const bool & _occupation,
  const uint8_t & _target,
  const uint8_t & _mode,
  const uint8_t & _effect,
  const uint8_t & _r,
  const uint8_t & _g,
  const uint8_t & _b)
{
  auto request = std::make_shared<LedSrv::Request>();
  request->client = LedSrv::Request::CONNECTOR;
  request->occupation = _occupation;
  request->target = _target;
  request->mode = _mode;
  request->effect = _effect;
  request->r_value = _r;
  request->g_value = _g;
  request->b_value = _b;
  INFO(
    "Request led module info:\n\t+ request:"
    "\n\t  - client = %s"
    "\n\t  - occupation = %s"
    "\n\t  - target = %d"
    "\n\t  - mode = %d"
    "\n\t  - effect = %d"
    "\n\t  - r_value = %d"
    "\n\t  - g_value = %d"
    "\n\t  - b_value = %d",
    request->client.c_str(),
    std::string(request->occupation ? "true" : "false").c_str(),
    request->target,
    request->mode,
    request->effect,
    request->r_value,
    request->g_value,
    request->b_value);
  if (!rclcpp::ok()) {
    WARN("Client interrupted while requesting for service to appear.");
    return 1;
  }
  if (!this->client_->wait_for_service(std::chrono::seconds(3))) {
    WARN("Waiting for service to appear(start) timeout.");
    return 2;
  }
  auto result = this->client_->async_send_request(request);
  std::future_status status =
    result.wait_for(std::chrono::seconds(3));
  if (status != std::future_status::ready) {
    WARN("Request led module timedout or deferred.");
    return 3;
  }
  auto response_ptr = result.get();
  if (response_ptr->code != LedSrv::Response::SUCCEED) {
    WARN(
      "Control led module failed, response code = %d", static_cast<int>(response_ptr->code));
  } else {
    INFO(
      "Control led module succeeded, response code = %d", static_cast<int>(response_ptr->code));
  }
  return response_ptr->code;
}

uint CtrlLed::ControlLed(const uint16_t & _id)
{
  // PID_WIFI_ENTER_CONNECTION_MODE_0 开启配网功能
  // PID_WIFI_FAILED_PLEASE_RETRY     启动失败请重试
  // PID_WIFI_WAIT_FOR_SCAN_CODE_0    等待扫描二维码(1/5s)
  // PID_WIFI_SCAN_CODE_SUCCEEDED_0   扫码成功，网络连接中
  // PID_WIFI_CONNECTION_SUCCEEDED_0  连网成功
  // PID_WIFI_CONNECTED_UNKNOWN_NET   已连接无密码的网络
  // PID_WIFI_SCAN_CODE_IP_ERROR      二维码信息错误，请使用正确二维码
  // PID_WIFI_CONNECTION_FAILED_0     无线网络名称错误，请修改后重试
  // PID_WIFI_CONNECTION_FAILED_1     无线网络密码错误，请修改后重试
  // PID_WIFI_CONNECTION_FAILED_2     无法连接网络，请检查网络状况并重新尝试
  // PID_WIFI_SCAN_CODE_INFO_ERROR    二维码失效，请重新生成
  // PID_WIFI_EXIT_CONNECTION_MODE_0  关闭配网功能
  INFO("Control led <%d>", _id);
  if ((_id == AudioMsg::PID_WIFI_CONNECTION_SUCCEEDED_0) ||
    (_id == AudioMsg::PID_WIFI_CONNECTED_UNKNOWN_NET) ||
    (_id == AudioMsg::PID_WIFI_FAILED_PLEASE_RETRY) ||
    (_id == AudioMsg::PID_WIFI_EXIT_CONNECTION_MODE_0))
  {
    // this->EnableLed(
    //   true,
    //   LedSrv::Request::HEAD_LED,
    //   LedSrv::Request::SYSTEM_PREDEFINED,
    //   LedSrv::Request::RGB_OFF);
    // this->EnableLed(
    //   true,
    //   LedSrv::Request::TAIL_LED,
    //   LedSrv::Request::SYSTEM_PREDEFINED,
    //   LedSrv::Request::RGB_OFF);
    // this->EnableLed(
    //   true,
    //   LedSrv::Request::MINI_LED,
    //   LedSrv::Request::SYSTEM_PREDEFINED,
    //   LedSrv::Request::MINI_OFF);
    this->EnableLed(
      false,
      LedSrv::Request::HEAD_LED);
    this->EnableLed(
      false,
      LedSrv::Request::TAIL_LED);
    this->EnableLed(
      false,
      LedSrv::Request::MINI_LED);
  } else if (_id == AudioMsg::PID_WIFI_ENTER_CONNECTION_MODE_0) {
    this->EnableLed(
      true,
      LedSrv::Request::HEAD_LED,
      LedSrv::Request::USER_DEFINED,
      LedSrv::Request::BLINK,
      255, 165, 0);
    this->EnableLed(
      true,
      LedSrv::Request::TAIL_LED,
      LedSrv::Request::USER_DEFINED,
      LedSrv::Request::BLINK,
      255, 165, 0);
    this->EnableLed(
      true,
      LedSrv::Request::MINI_LED,
      LedSrv::Request::USER_DEFINED,
      LedSrv::Request::CIRCULAR_BREATH,
      255, 165, 0);
  } else if (_id == 9) {
    this->EnableLed(
      true,
      LedSrv::Request::HEAD_LED,
      LedSrv::Request::USER_DEFINED,
      LedSrv::Request::BLINK_FAST,
      255, 140, 0);
    this->EnableLed(
      true,
      LedSrv::Request::TAIL_LED,
      LedSrv::Request::USER_DEFINED,
      LedSrv::Request::BLINK_FAST,
      255, 140, 0);
    this->EnableLed(
      true,
      LedSrv::Request::MINI_LED,
      LedSrv::Request::USER_DEFINED,
      LedSrv::Request::CIRCULAR_BREATH,
      255, 140, 0);
  } else {
    INFO("Control led : maintain the current state.");
  }
  return 0;
}
}   // namespace interaction
}   // namespace cyberdog
