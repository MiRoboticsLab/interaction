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
#ifndef CONNECTOR__CONNECTOR_HPP_
#define CONNECTOR__CONNECTOR_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include <unistd.h>
#include <stdlib.h>
#include <ReadBarcode.h>
#include <TextUtfEncoding.h>
#include <GTIN.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include <file_uploading/lcm_log_uploader.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <protocol/msg/wifi_info.hpp>
#include <protocol/msg/notify_to_app.hpp>
#include <protocol/msg/bluetooth_status.hpp>

#include <protocol/msg/audio_play.hpp>
#include <protocol/msg/connector_status.hpp>
#include <protocol/msg/touch_status.hpp>
#include <protocol/msg/wifi_status.hpp>
#include <protocol/srv/connector.hpp>
#include <protocol/srv/connector_status.hpp>
#include <protocol/srv/led_execute.hpp>
#include <protocol/srv/wifi_connect.hpp>
#include <protocol/srv/camera_service.hpp>

#include <string>
#include <thread>
#include <map>
#include <mutex>
#include <tuple>
#include <memory>
#include <functional>
#include <type_traits>
#include <chrono>
#include <condition_variable>

namespace cyberdog
{
namespace interaction
{
namespace py = pybind11;
static const unsigned int LINE_MAX_SIZE {16384};  /*!< 行最大值:2kb */

/*! \file       connector.hpp
    \brief      连接模块。
    \details    创建及初始化连接模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Connector final : public rclcpp::Node
{
  using CameraMsg = sensor_msgs::msg::Image;                  /*!< [topic 类型]镜头图片消息 */
  using DisConMsg = std_msgs::msg::Bool;                      /*!< [topic 类型]断开连接消息 */
  using AudioMsg = protocol::msg::AudioPlay;                  /*!< [topic 类型]语音消息 */

  using ConnectorStatusMsg = protocol::msg::ConnectorStatus;  /*!< [topic 类型]连接状态消息 */
  using TouchMsg = protocol::msg::TouchStatus;                /*!< [topic 类型]触摸板状态消息 */
  using WiFiMsg = protocol::msg::WifiStatus;                  /*!< [topic 类型]WiFi 消息 */
  using ConnectorSrv = protocol::srv::Connector;              /*!< [service 类型]连接 服务 */
  using WiFiSrv = protocol::srv::WifiConnect;                 /*!< [service 类型]WiFi 服务 */
  using LedSrv = protocol::srv::LedExecute;                   /*!< [service 类型]LED 驱动服务 */
  using CameraSrv = protocol::srv::CameraService;             /*!< [service 类型]相机 驱动服务 */

  using NotifyToAppMsg = protocol::msg::NotifyToApp;  /*!< [topic 类型]通知APP连接状态消息 */
  using WIFIINFOMSG = protocol::msg::WifiInfo;
  using BLUETOOTHSTATUSMSG = protocol::msg::BluetoothStatus;
  using ConnectorStatus = std_msgs::msg::String;

  using TimeType = std::chrono::time_point<std::chrono::system_clock>;    /*!< 超时 */
  enum ShellEnum
  {
    shell = 1993,     /*!< 异常结束, 执行shell命令错误 */
    command,          /*!< 异常结束, 待执行命令错误 */
    command_popen,    /*!< 异常结束, 无法执行命令 */
    command_error,    /*!< 异常结束, 执行命令失败 */
  };

public:
  explicit Connector(const std::string & name);
  ~Connector();
  bool Init(
    const std::function<uint(uint16_t)>,
    const std::function<uint(uint8_t)>,
    const std::function<uint(uint16_t)>,
    const std::function<uint(const std::string, const std::string)>,
    const std::function<bool(bool)>);                                   /*!< 初始化 */

public:
  std::function<uint(uint16_t)> CtrlAudio;                    /*!< [客户端]控制语音 */
  std::function<uint(uint8_t)> CtrlCamera;                    /*!< [客户端]控制相机 */
  std::function<uint(uint16_t)> CtrlLed;                      /*!< [客户端]控制led */
  std::function<uint(const std::string, const std::string)>
  CtrlWifi;                                                   /*!< [客户端]控制wifi */
  std::function<bool(bool)> CtrlAdvertising;                    /*!< [客户端]控制蓝牙 */

private:
  bool InitPythonInterpreter();                               /*!< 初始化 python 解释器 */
  void UpdateStatus();                                        /*!< 更新状态 */
  void ResetSignal();                                         /*!< 重置信号 */
  void WiFiSignalCallback(const WiFiMsg::SharedPtr);          /*!< WiFi信号回调 */
  void DisconnectAppCallback(const DisConMsg::SharedPtr);     /*!< 断开连接App消息回调 */
  void TouchSignalCallback(const TouchMsg::SharedPtr);        /*!< 触摸板信号回调 */
  void CameraSignalCallback(const CameraMsg::SharedPtr);      /*!< 镜头图像回调 */
  bool DoConnect(std::string, std::string, std::string);      /*!< 进行连接 */
  void Connect(
    const std::shared_ptr<ConnectorSrv::Request> request,
    std::shared_ptr<ConnectorSrv::Response> response);        /*!< 进行连接 */
  bool CheckInternet();                                       /*!< 检测因特网 */
  void SaveWiFi(
    const std::string &, const std::string &,
    const std::string &);                                     /*!< 保存WiFi */
  std::string GetWiFiProvider(const std::string & name);      /*!< 获取wifi提供着 */
  std::string GetTime(const TimeType &, const std::string);   /*!< 获取时间 */
  bool Shell(
    const std::string &, int &,
    std::string &);                                           /*!< 执行shell */
  bool Interaction(const uint16_t & _id);                     /*!< 执行交互 */
  bool UserBootsFirstTime();                                  /*!< 用户首次开机 */

private:
  std::string name_ {""};                                     /*!< node 名称 */
  std::string params_pkg_dir_ {""};                           /*!< params 包路径 */
  std::string node_config_dir_ {""};                          /*!< node 配置文件路径 */
  std::string wifi_config_dir_ {""};                          /*!< wifi 配置文件路径 */
  std::string initial_ip_ {"initial_ip_"};                    /*!< 初始 ip */
  std::string provider_ip_ {"initial_ip_"};                   /*!< wifi 请求手机IP */
  toml::value params_toml_;                                   /*!< 配置文件数据 */
  ConnectorStatusMsg state_msg_;                              /*!< 连接状态消息 */
  std::mutex state_msg_mutex_;                                /*!< 连接状态消息占用权限 */
  bool touch_efficient_;                                      /*!< 触摸板是否有效 */
  bool camer_efficient_;                                      /*!< 相机是否有效 */
  int touch_signal_timeout_s {0};                             /*!< 触摸板信号有效时长 */
  int touch_signal_invalid_interval_s {1};                    /*!< 触摸板信号无效间隔 */
  TimeType touch_signal_timeout_;                             /*!< 触摸板信号超时 */
  int srv_code_ {0};                                          /*!< 服务返回码 */
  rclcpp::Time touch_previous_time_;                          /*!< Touch 时间戳 */
  bool judge_first_time_ {true};                              /*!< 第一次判断 */

  rclcpp::TimerBase::SharedPtr update_status_timer_ {nullptr};        /*!< [定时器]更新数据 */
  rclcpp::TimerBase::SharedPtr reset_signal_timer_ {nullptr};         /*!< [定时器]重置触摸板 */
  rclcpp::Publisher<ConnectorStatusMsg>::SharedPtr status_pub_ {nullptr};   /*!< [发布器]连接状态 */
  rclcpp::Subscription<WiFiMsg>::SharedPtr wifi_sub_ {nullptr};       /*!< [监听器]WiFi */
  rclcpp::Subscription<DisConMsg>::SharedPtr disconn_sub_ {nullptr};  /*!< [监听器]断开连接app */
  rclcpp::Subscription<TouchMsg>::SharedPtr touch_sub_ {nullptr};     /*!< [监听器]触摸板状态 */
  rclcpp::Subscription<CameraMsg>::SharedPtr img_sub_ {nullptr};      /*!< [监听器]镜头图像 */
  rclcpp::Service<ConnectorSrv>::SharedPtr connect_service_ {nullptr};  /*!< [服务端]连接 */
  std::string KEY {""};                                               /*!< 秘钥 */
  std::string wifi_name {""};                                         /*!< 名称 */
  std::string wifi_password {""};                                     /*!< 密码 */
  std::string ip {""};                                                /*!< ip */
  rclcpp::CallbackGroup::SharedPtr service_cb_group_ {nullptr};       /*!< [回调组] service */
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_ {nullptr};         /*!< [回调组] timer */
  rclcpp::CallbackGroup::SharedPtr wifi_cb_group_ {nullptr};          /*!< [回调组] wifi */
  rclcpp::CallbackGroup::SharedPtr disconn_cb_group_ {nullptr};       /*!< [回调组] disconn */
  rclcpp::CallbackGroup::SharedPtr touch_cb_group_ {nullptr};         /*!< [回调组] touch */
  rclcpp::CallbackGroup::SharedPtr img_cb_group_ {nullptr};           /*!< [回调组] img */
  rclcpp::CallbackGroup::SharedPtr pub_cb_group_ {nullptr};           /*!< [回调组] pub */

  void APPSendWiFiCallback(const protocol::msg::WifiInfo::SharedPtr msg);
  void AppConnectState(const std_msgs::msg::Bool msg);
  void BtStatusCallback(const protocol::msg::BluetoothStatus::SharedPtr msg);
  bool WriteToFile(
    const std::string ssid,
    const std::string ip,
    const std::string type);

  NotifyToAppMsg notify_to_app_msg_;
  ConnectorStatus connector_status_msg_;
  rclcpp::Subscription<WIFIINFOMSG>::SharedPtr wifi_info_sub_ {nullptr};
  rclcpp::Publisher<NotifyToAppMsg>::SharedPtr notify_to_app_pub_ {nullptr};
  rclcpp::Subscription<BLUETOOTHSTATUSMSG>::SharedPtr bluetooth_status_sub_ {nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connector_init_pub_ {nullptr};
  std::string wifi_record_dir_ {""};   /*!< wifi 类型记录文件路径 */
  bool connect_network_status = true;
  int connect_code = -1;
};  // class Connector
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__CONNECTOR_HPP_
