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
#ifndef CONNECTOR__CTRL_BLUETOOTH_HPP_
#define CONNECTOR__CTRL_BLUETOOTH_HPP_

#include <unistd.h>
#include <string>
#include <iostream>
#include <sstream>
#include <regex>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_set.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/bluetooth_status.hpp"
#include "protocol/srv/bms_cmd.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cyberdog
{
namespace interaction
{
class CtrlBluetooth final : public rclcpp::Node
{
public:
  explicit CtrlBluetooth(const std::string name);
  ~CtrlBluetooth();
  bool CtrlAdvertising(bool enable);

  bool WriteToFile(
    const std::string ssid,
    const std::string ip,
    const std::string type);
  bool IsConnectApp();

private:
  bool Init();
  std::string runCommand(const std::string & cmd);
  std::vector<std::string> parseConnectionsOutput(const std::string & output);
  void BluetoothStatusCallback(
    const protocol::msg::BluetoothStatus::SharedPtr msg);
  bool DisconnectBluetooth();

private:
  int bluetooth_connect_status_ = -1;
  int bluetooth_advertising_status_ = -1;

  rclcpp::Subscription<protocol::msg::BluetoothStatus>::SharedPtr bluetooth_status_sub_ {nullptr};
  rclcpp::Client<protocol::srv::BmsCmd>::SharedPtr control_advertise_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disconnect_app_bt_srv_ {nullptr};
};  // class CtrlBluetooth
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__CTRL_BLUETOOTH_HPP_
