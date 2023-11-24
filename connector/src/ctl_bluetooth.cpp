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

#include "connector/ctrl_bluetooth.hpp"

namespace cyberdog
{
namespace interaction
{
CtrlBluetooth::CtrlBluetooth(const std::string name)
: Node(name)
{
  INFO("Creating [CtrlBluetooth] object(node)");
  if (!this->Init()) {exit(-1);}
}

CtrlBluetooth::~CtrlBluetooth()
{
  INFO("Destroy [CtrlBluetooth] object(node)");
}

bool CtrlBluetooth::Init()
{
  INFO("Initializing CtrlBluetooth ...");
  try {
    this->bluetooth_status_sub_ =
      this->create_subscription<protocol::msg::BluetoothStatus>(
      "bluetooth_status", 1,
      std::bind(&CtrlBluetooth::BluetoothStatusCallback, this, std::placeholders::_1));

    this->control_advertise_client_ =
      this->create_client<protocol::srv::BmsCmd>("ctrl_advertising");

    this->disconnect_app_bt_srv_ = this->create_client<std_srvs::srv::Trigger>(
      "disconnect_app_bt", rmw_qos_profile_services_default);
  } catch (const std::exception & e) {
    ERROR("Init CtrlBluetooth failed: <%s>", e.what());
    return false;
  }
  return true;
}

void CtrlBluetooth::BluetoothStatusCallback(
  const protocol::msg::BluetoothStatus::SharedPtr msg)
{
  this->bluetooth_connect_status_ = msg->connectable;
  this->bluetooth_advertising_status_ = msg->advtiseable;
}

bool CtrlBluetooth::DisconnectBluetooth()
{
  if (!this->disconnect_app_bt_srv_->wait_for_service(std::chrono::seconds(5))) {
    INFO("Failed to call disconnect_app_bt service");
    return false;
  } else {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = this->disconnect_app_bt_srv_->async_send_request(request);
    std::future_status status = future_result.wait_for(std::chrono::seconds(5));
    if (status == std::future_status::ready) {
      auto result = future_result.get();
      if (result->success) {
        INFO("success to call disconnect_app_bt service");
      } else {
        INFO("success to call disconnect_app_bt service,but failed to disconnect app bluetooth");
        return false;
      }
    } else {
      INFO("Failed to call disconnect_app_bt service");
      return false;
    }
  }
}

bool CtrlBluetooth::CtrlAdvertising(bool enable)
{
  std::shared_ptr<protocol::srv::BmsCmd::Request> request =
    std::make_shared<protocol::srv::BmsCmd::Request>();
  INFO("enter CtrlAdvertising");
  if (enable) {
    if (this->bluetooth_connect_status_ == 1) {
      INFO("disconnect app bluetooth");
      if (!this->DisconnectBluetooth()) {
        INFO("Failed to disconnect app bluetooth");
        return false;
      }
    }
    if (this->bluetooth_advertising_status_ == 1) {
      INFO_MILLSECONDS(3000, "is Advertising");
      return true;  // 广播已开启，直接返回
    }
    request->electric_machine_command = 1;  // 开启广播
    sleep(1);
  } else {
    if (this->bluetooth_advertising_status_ == 0) {
      INFO_MILLSECONDS(3000, "Advertising is closed");
      return true;  // 广播已经关闭
    }
    request->electric_machine_command = 2;  // 关闭广播
  }

  auto future_result = this->control_advertise_client_->async_send_request(request);
  std::future_status status = future_result.wait_for(std::chrono::seconds(5));
  if (status == std::future_status::ready) {
    auto result = future_result.get();
    if (result->success) {
      INFO("success to call CtrlAdvertising service");
      return true;
    } else {
      INFO("success to call CtrlAdvertising service,but failed to control advertising");
      return false;
    }
  } else {
    INFO("Failed to call CtrlAdvertising service");
    return false;
  }
  return true;
}
}   // namespace interaction
}   // namespace cyberdog
