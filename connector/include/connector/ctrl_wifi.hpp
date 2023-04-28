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
#ifndef CONNECTOR__CTRL_WIFI_HPP_
#define CONNECTOR__CTRL_WIFI_HPP_

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>

#include <protocol/srv/wifi_connect.hpp>

#include <string>
#include <map>

namespace cyberdog
{
namespace interaction
{
/*! \file       ctrl_wifi.hpp
    \brief      WiFi控制模块。
    \details    创建及初始化WiFi控制模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class CtrlWifi final : public rclcpp::Node
{
  using WiFiSrv = protocol::srv::WifiConnect;               /*!< [service 类型]WiFi 服务 */

public:
  explicit CtrlWifi(const std::string name);
  ~CtrlWifi();
  uint ControlWifi(const std::string, const std::string);   /*!< 控制wifi */

private:
  bool Init();                                              /*!< 初始化 */

private:
  rclcpp::Client<WiFiSrv>::SharedPtr client_ {nullptr};     /*!< [客户端]wifi驱动 */
  std::map<uint8_t, std::string> wifi_response_result_;     /*!< WiFi 连接状态 */
  int wait_for_service_timeout_s {3};                       /*!< 等待 service 出现 */
  int wait_for_service_response_timeout_s {20};             /*!< 等待 service 响应 */
};  // class CtrlWifi
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__CTRL_WIFI_HPP_
