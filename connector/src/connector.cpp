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
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "connector/connector.hpp"

namespace cyberdog
{
namespace interaction
{
py::object decrypt;   // 解密
Connector::Connector(const std::string & name)
: Node(name),
  name_(name),
  node_config_dir_("/config/connector.toml"),
  wifi_config_dir_("/wifi.toml"),
  initial_ip_("initial_ip_"),
  provider_ip_(initial_ip_),
  KEY("b1950ca92ccc9ffe"),
  wifi_name("n"),
  wifi_password("p"),
  ip("i"),
  wifi_record_dir_("/wifirecord.toml")
{
  INFO("Creating [Connector] object(node)");
}

Connector::~Connector()
{
  INFO("Destroy [Connector] object(node)");
  this->CtrlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
}

bool Connector::Init(
  const std::function<uint(uint16_t)> _control_audio,
  const std::function<uint(uint8_t)> _control_camera,
  const std::function<uint(uint16_t)> _control_led,
  const std::function<uint(const std::string, const std::string)> _control_wifi,
  const std::function<bool(bool)> _control_bluetooth)
{
  INFO("Initializing ...");
  try {
    this->CtrlAudio = _control_audio;
    this->CtrlCamera = _control_camera;
    this->CtrlLed = _control_led;
    this->CtrlWifi = _control_wifi;
    this->CtrlAdvertising = _control_bluetooth;

    this->touch_efficient_ = false;
    this->camer_efficient_ = false;

    this->touch_signal_timeout_ = std::chrono::system_clock::now();

    this->state_msg_.is_connected = false;
    this->state_msg_.is_internet = false;
    this->state_msg_.ssid = "";
    this->state_msg_.robot_ip = "";
    this->state_msg_.strength = 0;
    this->state_msg_.code = 0;
    this->touch_previous_time_ = this->now();

    this->params_pkg_dir_ = ament_index_cpp::get_package_share_directory("connector");
    this->node_config_dir_ = this->params_pkg_dir_ + this->node_config_dir_;
    INFO("Params config file dir:<%s>", this->node_config_dir_.c_str());

    if (access(this->node_config_dir_.c_str(), F_OK)) {
      ERROR("Params config file does not exist");
      return false;
    }

    if (access(this->node_config_dir_.c_str(), R_OK)) {
      ERROR("Params config file does not have read permissions");
      return false;
    }

    if (access(this->node_config_dir_.c_str(), W_OK)) {
      ERROR("Params config file does not have write permissions");
      return false;
    }

    if (!cyberdog::common::CyberdogToml::ParseFile(
        this->node_config_dir_.c_str(), this->params_toml_))
    {
      ERROR("Params config file is not in toml format");
      return false;
    }

    std::string original_file = this->params_pkg_dir_ + "/config" + this->wifi_config_dir_;
    std::string original_wifi_record_file = this->params_pkg_dir_ + "/config" +
      this->wifi_record_dir_;
    std::string workspace = toml::find<std::string>(
      this->params_toml_, "connector", "initialization", "cmd", "workspace");
    this->wifi_config_dir_ = workspace + this->wifi_config_dir_;
    this->wifi_record_dir_ = workspace + this->wifi_record_dir_;
    auto make_dir = [&]() -> bool {
        if (access(workspace.c_str(), F_OK) != 0) {
          if (mkdir(workspace.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
            ERROR("Mkdir <%s> failed: %s.", workspace.c_str(), std::strerror(errno));
            return false;
          }
        }
        return true;
      };

    if (access(workspace.c_str(), F_OK) != 0) {
      int code = -1;
      std::string message;
      if (!make_dir()) {
        if (!this->Shell(std::string("mkdir -p " + workspace), code, message)) {
          return false;
        }
      }
      if (!this->Shell(
          std::string("cp " + original_file + " " + this->wifi_config_dir_), code,
          message))
      {
        return false;
      }
      if (!this->Shell(
          std::string("cp " + original_wifi_record_file + " " + this->wifi_record_dir_), code,
          message))
      {
        return false;
      }
    }

    this->touch_signal_timeout_s = static_cast<int>(toml::find<int>(
        this->params_toml_, "connector", "initialization", "timeout_s", "touch_signal_effective"));
    this->touch_signal_invalid_interval_s = static_cast<int>(toml::find<int>(
        this->params_toml_, "connector", "initialization", "timeout_s",
        "touch_signal_invalid_interval"));

    float hz = toml::find<float>(
      this->params_toml_, "connector", "initialization", "hz", "update_status");
    if (!(hz > 0)) {
      hz = 1;
    }
    this->update_status_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / hz)),
      std::bind(&Connector::UpdateStatus, this));

    hz = toml::find<float>(
      this->params_toml_, "connector", "initialization", "hz", "reset_signal");
    if (!(hz > 0)) {
      hz = 1;
    }
    this->service_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->wifi_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->disconn_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    this->touch_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->img_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->pub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->reset_signal_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / hz)),
      std::bind(&Connector::ResetSignal, this),
      this->timer_cb_group_);

    rclcpp::PublisherOptions pub_option;
    pub_option.callback_group = this->pub_cb_group_;
    this->status_pub_ = this->create_publisher<ConnectorStatusMsg>(
      toml::find<std::string>(
        this->params_toml_, "connector", "initialization", "topic", "connector"),
      1, pub_option);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = this->wifi_cb_group_;
    this->wifi_sub_ = this->create_subscription<WiFiMsg>(
      toml::find<std::string>(
        this->params_toml_, "connector", "initialization", "topic", "wifi"),
      1,
      std::bind(&Connector::WiFiSignalCallback, this, std::placeholders::_1), sub_option);
    sub_option.callback_group = this->disconn_cb_group_;
    this->disconn_sub_ = this->create_subscription<DisConMsg>(
      toml::find<std::string>(
        this->params_toml_, "connector", "initialization", "topic", "disconnect_app"),
      1,
      std::bind(&Connector::DisconnectAppCallback, this, std::placeholders::_1), sub_option);
    sub_option.callback_group = this->touch_cb_group_;
    this->touch_sub_ = this->create_subscription<TouchMsg>(
      toml::find<std::string>(
        this->params_toml_, "connector", "initialization", "topic", "touch"),
      1,
      std::bind(&Connector::TouchSignalCallback, this, std::placeholders::_1), sub_option);
    sub_option.callback_group = this->img_cb_group_;
    this->img_sub_ = this->create_subscription<CameraMsg>(
      toml::find<std::string>(
        this->params_toml_, "connector", "initialization", "topic", "camera"),
      1,
      std::bind(&Connector::CameraSignalCallback, this, std::placeholders::_1), sub_option);
    this->connect_service_ = this->create_service<ConnectorSrv>(
      toml::find<std::string>(
        this->params_toml_, "connector", "initialization", "service", "connection"),
      std::bind(&Connector::Connect, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->service_cb_group_);
    this->bluetooth_status_sub_ = this->create_subscription<protocol::msg::BluetoothStatus>(
      "bluetooth_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&Connector::BtStatusCallback, this, std::placeholders::_1),
      sub_option);
    this->wifi_info_sub_ = this->create_subscription<protocol::msg::WifiInfo>(
      "app_send_network_data", rclcpp::SystemDefaultsQoS(),
      std::bind(&Connector::APPSendWiFiCallback, this, std::placeholders::_1),
      sub_option);
    this->notify_to_app_pub_ = this->create_publisher<protocol::msg::NotifyToApp>(
      "notify_to_app", 1, pub_option);
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return this->InitPythonInterpreter();
}

bool Connector::InitPythonInterpreter()
{
  try {
    // py::module os = py::module::import("os");
    // py::module sys = py::module::import("sys");
    // py::module ament_index_python_packages = py::module::import("ament_index_python.packages");
    // py::object get_package_share_directory = ament_index_python_packages.attr(
    //   "get_package_share_directory");
    // py::object os_path_join = os.attr("path").attr("join");
    // py::object sys_path_append = sys.attr("path").attr("append");
    // sys_path_append(os_path_join(get_package_share_directory("connector"), "script"));
    // // py::print("python path:", sys.attr("path"));
    py::module aes = py::module::import("mi.connector.aes");
    auto set_object = [&](py::object & _obj, std::string _fun) -> bool {
        if (!py::hasattr(aes, _fun.c_str())) {
          ERROR("'%s()' function not found in 'aes' module", _fun.c_str());
          return false;
        }
        _obj = aes.attr(_fun.c_str());
        return true;
      };
    if (!set_object(decrypt, "decrypt")) {
      return false;
    }
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

std::string Connector::GetTime(const TimeType & _time, const std::string _format)
{
  const std::time_t t_time = std::chrono::system_clock::to_time_t(_time);
  std::stringstream t_time_str;
  t_time_str << std::put_time(std::localtime(&t_time), _format.c_str());
  return t_time_str.str();
}

bool Connector::Shell(const std::string & _command, int & _code, std::string & _message)
{
  try {
    INFO("Shell: %s", _command.c_str());
    _message = "";
    if (!_command.empty()) {
      std::string _code_cmd = _command + "; echo $?";  //  | xargs echo
      std::string _code_str = "";
      FILE * fstream_ptr = nullptr;
      fstream_ptr = popen(_code_cmd.c_str(), "r");
      if (fstream_ptr != nullptr) {
        char buffer[LINE_MAX_SIZE];
        while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
          _code_str = buffer;
          memset(buffer, '\0', sizeof(buffer));
          _message += _code_str;
        }
        pclose(fstream_ptr);
        fstream_ptr = nullptr;
        _code = std::atoi(_code_str.c_str());
        if (_code == static_cast<int>(0)) {
          return true;
        } else {
          _code = static_cast<int>(ShellEnum::command_error);
          _code_cmd = _command + " 2> /dev/stdout";
          fstream_ptr = popen(_code_cmd.c_str(), "r");
          if (fstream_ptr != nullptr) {
            _code_str = "";
            while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
              _code_str += std::string(buffer);
              memset(buffer, '\0', sizeof(buffer));
            }
            pclose(fstream_ptr);
            fstream_ptr = nullptr;
          }
          _message = "Shell command is error.\n - command : " + _command +
            "\n - code : " + _message + " - error : " + _code_str;
        }
      } else {
        _code = static_cast<int>(ShellEnum::command_popen);
        _message = "Canot shell command popen.\n - command : " + _command +
          "\n - error : " + strerror(errno);
      }
    } else {
      _code = static_cast<int>(ShellEnum::command);
      _message = "Shell command is empty.\n - command : " + _command;
    }
  } catch (const std::exception & e) {
    _code = static_cast<int>(ShellEnum::shell);
    _message = "Shell command is error.\n - command : " + _command +
      "\n - error : " + e.what();
  }
  ERROR("Shell: %s, %s", _command.c_str(), _message.c_str());
  return false;
}

void Connector::UpdateStatus()
{
  try {
    std::lock_guard<std::mutex> guard(this->state_msg_mutex_);
    this->status_pub_->publish(this->state_msg_);
  } catch (const std::exception & e) {
    WARN("Update status failed: <%s>", e.what());
  }
}

void Connector::ResetSignal()
{
  if (this->touch_efficient_ &&
    (std::chrono::system_clock::now() > this->touch_signal_timeout_))
  {
    INFO("ResetSignal(Exit network mode).");
    this->camer_efficient_ = false;
    this->CtrlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
    int count = 0;
    while (!this->CtrlAdvertising(false) && count < 3) {
      sleep(2);
      count++;
    }
    this->touch_efficient_ = false;
    this->Interaction(AudioMsg::PID_WIFI_EXIT_CONNECTION_MODE_0);
  }
  if (this->judge_first_time_) {
    if (this->UserBootsFirstTime()) {
      TouchMsg::SharedPtr touch_ptr = std::make_shared<TouchMsg>();
      touch_ptr->touch_state = 7;
      this->TouchSignalCallback(touch_ptr);
    }
  }
}

void Connector::DisconnectAppCallback(const DisConMsg::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }
  {
    std::lock_guard<std::mutex> guard(this->state_msg_mutex_);
    this->provider_ip_ = "";
    this->state_msg_.provider_ip = "";
  }
  this->SaveWiFi("", this->state_msg_.ssid, "");
}

void Connector::WiFiSignalCallback(const WiFiMsg::SharedPtr msg)
{
  try {
    std::lock_guard<std::mutex> guard(this->state_msg_mutex_);

    if (msg->is_connected) {
      // this->notify_to_app_msg_.ssid = msg->ssid;
      // this->notify_to_app_msg_.ip = msg->ip;
      if ((!this->state_msg_.is_connected) &&
        (std::chrono::system_clock::now() > this->touch_signal_timeout_))
      {
        // 网络连接已建立: 开机自动连接网络，自动断网后自动连接网络
        this->notify_to_app_msg_.code = 2000;
      }
      if (msg->ssid != this->state_msg_.ssid) {
        this->state_msg_.is_internet = this->CheckInternet();
        if (this->state_msg_.is_internet) {
          // 当前网络能访问互联网
          // this->notify_to_app_msg_.code = 2001;
        } else {
          // 当前网络不能访问互联网
          this->notify_to_app_msg_.code = 2002;
        }
      }
      if (this->provider_ip_ == this->initial_ip_) {
        this->provider_ip_ = this->GetWiFiProvider(msg->ssid);
      }
      this->state_msg_.provider_ip = this->provider_ip_;
      this->connect_network_status = true;

    } else {
      if (this->state_msg_.is_connected) {
        // 网络连接已断开
        this->notify_to_app_msg_.code = 2003;
        if (this->connect_network_status) {
          this->CtrlAudio(17);
          this->CtrlLed(AudioMsg::PID_WIFI_FAILED_PLEASE_RETRY);
          this->connect_network_status = false;
        }
      }
    }
    this->state_msg_.is_connected = msg->is_connected;
    this->state_msg_.ssid = msg->ssid;
    this->state_msg_.strength = msg->strength;
    this->state_msg_.robot_ip = msg->ip;
  } catch (const std::exception & e) {
    WARN("WiFi signal callback failed: <%s>", e.what());
  }
}

void Connector::TouchSignalCallback(const TouchMsg::SharedPtr msg)
{
  try {
    INFO(
      "Touch signal callback:{touch_state=%d, timestamp=%d}", static_cast<int>(msg->touch_state),
      static_cast<int>(msg->timestamp));
    if (msg->touch_state != 7) {  // 长按
      return;
    }
    rclcpp::Time current_time = msg->header.stamp;
    rclcpp::Duration data_interval = current_time - this->touch_previous_time_;
    if (data_interval.seconds() < this->touch_signal_invalid_interval_s) {
      WARN("Touch invalid data interval: %f seconds", data_interval.seconds());
      return;
    }
    this->touch_previous_time_ = msg->header.stamp;
    INFO("Touch data interval: %f seconds", data_interval.seconds());

    if (this->touch_efficient_) {  // 退出配网
      INFO("Exit network mode.");
      this->touch_efficient_ = false;
      this->camer_efficient_ = false;
      this->CtrlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
      this->Interaction(AudioMsg::PID_WIFI_EXIT_CONNECTION_MODE_0);
      return;
    }
    INFO("Enter network mode.");
    this->Interaction(AudioMsg::PID_WIFI_ENTER_CONNECTION_MODE_0);
    if (this->CtrlCamera(CameraSrv::Request::START_IMAGE_PUBLISH)) {  // 打开相机失败
      this->touch_efficient_ = false;
      this->CtrlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
      this->Interaction(AudioMsg::PID_WIFI_FAILED_PLEASE_RETRY);
      return;
    }
    // this->Interaction(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
    this->CtrlAudio(16);
    this->CtrlLed(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
    this->touch_signal_timeout_ = std::chrono::system_clock::now() +
      std::chrono::seconds(this->touch_signal_timeout_s);
    this->touch_efficient_ = true;
    int count = 0;
    while (!this->CtrlAdvertising(true) && count < 3) {
      sleep(3);
      count++;
    }
    this->camer_efficient_ = true;
    INFO(
      "A touch signal is detected at %s, the signal is valid for %d seconds and expires at %s.",
      this->GetTime(std::chrono::system_clock::now(), "%F %T").c_str(),
      static_cast<int>(this->touch_signal_timeout_s),
      this->GetTime(this->touch_signal_timeout_, "%F %T").c_str());
  } catch (const std::exception & e) {
    WARN("Touch signal callback failed: <%s>", e.what());
  }
}

void Connector::CameraSignalCallback(const CameraMsg::SharedPtr msg)
{
  try {
    if (this->camer_efficient_) {
      INFO("Identifying current QR code...");
      auto return_error = [&](const std::string & msg, const std::string & data) {
          WARN("%s:\n<%s>.", msg.c_str(), data.c_str());
          // this->Interaction(AudioMsg::PID_WIFI_SCAN_CODE_IP_ERROR);
          this->CtrlLed(AudioMsg::PID_WIFI_SCAN_CODE_IP_ERROR);
          if (this->touch_efficient_ &&
            (std::chrono::system_clock::now() < this->touch_signal_timeout_))
          {
            // this->Interaction(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
            this->CtrlAudio(16);
            this->CtrlLed(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
          }
        };
      int num_pixels = msg->data.size();
      INFO(
        "image:{step = %d, height = %d, width = %d, data_size = %d}",
        static_cast<int>(msg->step),
        static_cast<int>(msg->height),
        static_cast<int>(msg->width),
        static_cast<int>(msg->data.size()));
      uint8_t * img_buffer = new uint8_t[num_pixels];
      for (int i = 0; i < num_pixels; i++) {
        img_buffer[i] = (msg->data)[i];
      }
      INFO("new img_buffer is ok.");
      ZXing::DecodeHints hints;
      hints.setEanAddOnSymbol(ZXing::EanAddOnSymbol::Read);
      ZXing::ImageView image{img_buffer, static_cast<int>(msg->width),
        static_cast<int>(msg->height), ZXing::ImageFormat::RGB};
      auto results = ZXing::ReadBarcodes(image, hints);
      delete[] img_buffer;
      INFO("ZXing::ReadBarcodes() is return.");
      if (!results.empty()) {
        for (auto && result : results) {
          if (result.status() == ZXing::DecodeStatus::NoError) {
            bool angleEscape = false;
            std::string ciphertext = ZXing::TextUtfEncoding::ToUtf8(
              result.text(),
              angleEscape);
            if (ciphertext.empty()) {
              return_error("Extract the QR code in the field of vision to be empty.", ciphertext);
              break;
            }
            INFO("ciphertext:[%s].", ciphertext.c_str());
            std::string wifi_info = decrypt(KEY.c_str(), ciphertext.c_str()).cast<std::string>();
            INFO("wifi_info:[%s].", wifi_info.c_str());
            if (wifi_info.empty() || wifi_info.size() < 3) {
              return_error(
                "Recognize the QR code ciphertext in the unconstrained field of view",
                ciphertext);
              break;
            }
            INFO(
              "The current frame ciphertext is:\n%s\n"
              "\nThe plaintext of the current frame is:\n%s\n",
              ciphertext.c_str(), wifi_info.c_str());
            if (
              (wifi_info.find(std::string("\"" + wifi_name + "\":")) > wifi_info.length()) ||
              (wifi_info.find(std::string("\"" + wifi_password + "\":")) > wifi_info.length()) ||
              (wifi_info.find(std::string("\"" + ip + "\":")) > wifi_info.length()))
            {
              return_error(
                "Identify non-wifi information in the field of view",
                wifi_info);
              break;
            }
            rapidjson::Document connect_document;
            connect_document.Parse<rapidjson::kParseStopWhenDoneFlag>(wifi_info.c_str());
            if (!connect_document.HasParseError()) {
              if (connect_document.HasMember(wifi_name.c_str()) &&
                connect_document[wifi_name.c_str()].IsString() &&
                connect_document.HasMember(wifi_password.c_str()) &&
                connect_document[wifi_password.c_str()].IsString() &&
                connect_document.HasMember(ip.c_str()) &&
                connect_document[ip.c_str()].IsString())
              {
                // this->Interaction(AudioMsg::PID_WIFI_SCAN_CODE_SUCCEEDED_0);
                // this->CtrlAudio(15);
                // this->CtrlLed(AudioMsg::PID_WIFI_SCAN_CODE_SUCCEEDED_0);
                if (this->DoConnect(
                    connect_document[wifi_name.c_str()].GetString(),
                    connect_document[wifi_password.c_str()].GetString(),
                    connect_document[ip.c_str()].GetString()))
                {
                  this->camer_efficient_ = false;
                }
                break;
              } else {
                return_error(
                  "The QR code is not wifi, the information is",
                  wifi_info);
                break;
              }
            } else {
              ERROR(
                "Error(offset %u): %s",
                static_cast<unsigned>(connect_document.GetErrorOffset()),
                GetParseError_En(connect_document.GetParseError()));
              return_error(
                "The QR code is not json, the information is",
                wifi_info);
              break;
            }
          } else {WARN("QR code recognized Error");}
        }
      } else {WARN("QR code not recognized");}
    }
  } catch (const std::exception & e) {
    WARN("Camera signal callback failed: <%s>", e.what());
  } catch (...) {
    WARN("Camera signal callback failed.");
  }
}

bool Connector::DoConnect(std::string name, std::string password, std::string provider)
{
  this->CtrlAudio(15);
  this->CtrlLed(AudioMsg::PID_WIFI_ENTER_CONNECTION_MODE_0);
  auto return_true = [&](std::string msg, bool same_wifi) -> bool {
      this->provider_ip_ = provider;
      this->SaveWiFi(provider, name, password);
      INFO("%s, %d", msg.c_str(), same_wifi);
      if (password.empty()) {
        this->Interaction(AudioMsg::PID_WIFI_CONNECTED_UNKNOWN_NET);
      } else {
        this->Interaction(AudioMsg::PID_WIFI_CONNECTION_SUCCEEDED_0);
      }
      this->camer_efficient_ = false;
      this->CtrlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
      // this->Interaction(AudioMsg::PID_WIFI_EXIT_CONNECTION_MODE_0);
      this->touch_efficient_ = false;
      this->srv_code_ = ConnectorSrv::Response::CODE_SUCCESS;
      // this->connect_code = 2000;
      return true;
    };
  auto return_false = [&](std::string msg) -> bool {
      WARN("%s", msg.c_str());
      if (this->touch_efficient_ &&
        (std::chrono::system_clock::now() < this->touch_signal_timeout_))
      {
        // this->Interaction(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
        this->CtrlAudio(16);
        this->CtrlLed(AudioMsg::PID_WIFI_WAIT_FOR_SCAN_CODE_0);
      }
      // 释放灯效
      this->CtrlLed(AudioMsg::PID_WIFI_EXIT_CONNECTION_MODE_0);
      return false;
    };
  auto judge_string = [&](std::string msg) -> bool {
      if (msg.length() > 32) {
        WARN(
          "The current string(%s) is larger than 32 bytes, which is illegal.",
          msg.c_str());
        return false;
      }
      return true;
    };
  try {
    INFO(
      "Connect wifi:{name=<%s>, password=<%s>, provider=<%s>}", name.c_str(),
      password.c_str(), provider.c_str());
    if (name.empty()) {
      WARN(
        "The current WiFi(%s) name to be connected is empty, and the security is low",
        name.c_str());
      this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_0);
      this->srv_code_ = ConnectorSrv::Response::CODE_WIFI_NAME_FAIL;
      this->connect_code = 2004;
      return return_false(" Wifi name is empty.");
    } else {
      if (!judge_string(name)) {
        WARN(
          "The current WiFi(%s) name to be connected is invalid, "
          "there is a risk of security breaches.",
          name.c_str());
        this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_0);
        this->srv_code_ = ConnectorSrv::Response::CODE_WIFI_NAME_FAIL;
        this->connect_code = 2004;
        return return_false(" Wifi name is invalid.");
      }
    }
    if (password.empty()) {
      WARN(
        "The current WiFi(%s) password to be connected is empty, and the security is low.",
        name.c_str());
      // 变更Jira参见：https://jira.n.xiaomi.com/browse/CARPO-1031?filter=1426943
      // this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_1);
      // return return_false("Wifi password is empty.");
    } else {
      if (!judge_string(password)) {
        WARN(
          "The current WiFi(%s) password(%s) to be connected is invalid, "
          "there is a risk of security breaches.",
          name.c_str(),
          password.c_str());
        this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_1);
        this->connect_code = 2005;
        return return_false("Wifi password is invalid.");
      }
    }
    if (provider.empty()) {
      // 手机端IP错误，会导致无法与设备通讯
      // this->Interaction(AudioMsg::PID_WIFI_SCAN_CODE_INFO_ERROR);
      this->CtrlLed(AudioMsg::PID_WIFI_SCAN_CODE_INFO_ERROR);
      this->srv_code_ = ConnectorSrv::Response::CODE_WIFI_PROVIDER_IP_FAIL;
      return return_false("Wifi provider is empty.");
    }
    if (name == this->state_msg_.ssid) {
      return return_true(
        "The target wifi and the currently connected wifi are the same wifi.",
        true);
    }
    switch (this->CtrlWifi(name, password)) {
      case WiFiSrv::Response::RESULT_SUCCESS:
        this->connect_code = 2000;
        return return_true("WiFi connection succeeded.", false);
      case WiFiSrv::Response::RESULT_NO_SSID:
        this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_0);
        this->srv_code_ = ConnectorSrv::Response::CODE_WIFI_NAME_FAIL;
        this->connect_code = 2004;
        break;
      case WiFiSrv::Response::RESULT_ERR_PWD:
        this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_1);
        this->srv_code_ = ConnectorSrv::Response::CODE_WIFI_PASSWORD_FAIL;
        this->connect_code = 2005;
        break;
      default:
        this->Interaction(AudioMsg::PID_WIFI_CONNECTION_FAILED_2);
        this->srv_code_ = ConnectorSrv::Response::CODE_CONNECTION_TIMEOUT_FAIL;
        this->connect_code = 2001;
        break;
    }
  } catch (const std::exception & e) {
    WARN("Do connect wifi is failed: <%s>", e.what());
  }
  return return_false("DoConnect is failed.");
}

void Connector::SaveWiFi(
  const std::string & provider,
  const std::string & name, const std::string & password
)
{
  try {
    INFO(
      "Save WiFi :\n\tprovider = %s\n\tname = %s\n\tpassword = %s",
      provider.c_str(),
      name.c_str(),
      password.c_str());
    toml::value wifi_toml;
    if (cyberdog::common::CyberdogToml::ParseFile(
        this->wifi_config_dir_.c_str(), wifi_toml))
    {
      std::string old_provider = toml::find_or(wifi_toml, "wifi", name, "provider", "");
      if (old_provider.empty()) {
        toml::value wifi;
        // wifi["password"] = password;
        wifi["password"] = "";  // 处于个人隐私安全考虑，wifi密码不做保存。
        wifi["provider"] = provider;
        wifi_toml["wifi"][name] = wifi;
      } else {
        // wifi_toml["wifi"][name]["password"] = password;
        wifi_toml["wifi"][name]["password"] = "";  // 处于个人隐私安全考虑，wifi密码不做保存。
        wifi_toml["wifi"][name]["provider"] = provider;
      }
      if (!cyberdog::common::CyberdogToml::WriteFile(this->wifi_config_dir_, wifi_toml)) {
        WARN("WiFi params config file does not have wifi key, write failed");
      }
    } else {
      ERROR(
        "Toml WiFi config file is not in toml format, config file dir:\n%s",
        this->wifi_config_dir_.c_str());
    }
  } catch (const std::exception & e) {
    ERROR("Save WiFi is error:%s", e.what());
  }
}

bool Connector::UserBootsFirstTime()
{
  this->judge_first_time_ = false;
  try {
    // this->judge_first_time_ = false;
    INFO("Judge user boots first time...");
    toml::value wifi_toml;
    if (cyberdog::common::CyberdogToml::ParseFile(
        this->wifi_config_dir_.c_str(), wifi_toml))
    {
      bool first = toml::find_or(wifi_toml, "wifi", "user_boots_first_time", false);
      if (first) {
        wifi_toml["wifi"]["user_boots_first_time"] = false;
        if (cyberdog::common::CyberdogToml::WriteFile(this->wifi_config_dir_, wifi_toml)) {
          return true;
        } else {
          WARN("WiFi params config file does not have wifi key, write failed");
        }
      }
    } else {
      ERROR(
        "Toml WiFi config file is not in toml format, config file dir:\n%s",
        this->wifi_config_dir_.c_str());
    }
  } catch (const std::exception & e) {
    ERROR("Judge user boots first time is error:%s", e.what());
  }

  try {
    toml::value wifi_toml1;
    INFO("Judge user boots first time : %s", this->wifi_record_dir_.c_str());
    if (cyberdog::common::CyberdogToml::ParseFile(
        this->wifi_record_dir_.c_str(), wifi_toml1))
    {
      bool first1 = toml::find_or(wifi_toml1, "wifi", "user_boots_first_time", false);
      if (first1) {
        wifi_toml1["wifi"]["user_boots_first_time"] = false;
        if (cyberdog::common::CyberdogToml::WriteFile(this->wifi_record_dir_, wifi_toml1)) {
          return true;
        } else {
          WARN("111WiFi params config file does not have wifi key, write failed");
        }
      }
    } else {
      ERROR(
        "111Toml WiFi config file is not in toml format, config file dir:\n%s",
        this->wifi_record_dir_.c_str());
    }
  } catch (const std::exception & e) {
    ERROR("111Judge user boots first time is error:%s", e.what());
  }
  return false;
}

std::string Connector::GetWiFiProvider(const std::string & name)
{
  std::string provider;
  try {
    INFO("Get WiFi(%s) provider.", name.c_str());
    toml::value wifi_toml;
    if (cyberdog::common::CyberdogToml::ParseFile(
        this->wifi_config_dir_.c_str(), wifi_toml))
    {
      provider = toml::find_or(wifi_toml, "wifi", name, "provider", "");
    } else {
      ERROR(
        "Toml WiFi config file is not in toml format, config file dir:\n%s",
        this->wifi_config_dir_.c_str());
    }
  } catch (const std::exception & e) {
    ERROR("Save WiFi is error:%s", e.what());
  }
  return provider;
}

bool Connector::CheckInternet()
{
  bool is_internet = false;
  std::string check_internet_cmd =
    toml::find<std::string>(
    this->params_toml_, "connector", "initialization", "cmd", "check_internet_cmd");
  if (!check_internet_cmd.empty()) {
    INFO("Running system(<%s>) ...", check_internet_cmd.c_str());
    int status = system(check_internet_cmd.c_str());

    if (!(status < 0)) {
      if (WIFEXITED(status)) {
        is_internet = !WEXITSTATUS(status);
        INFO(
          "Run system(<%s>) results of the: %d",
          check_internet_cmd.c_str(), is_internet);             // 执行结果
      } else if (WIFSIGNALED(status)) {
        ERROR(
          "Run system(<%s>) interrupted by a signal(%d)",
          check_internet_cmd.c_str(), WTERMSIG(status));       // 被信号中断
      } else if (WIFSTOPPED(status)) {
        ERROR(
          "Run system(<%s>) suspended by a signal(%d)",
          check_internet_cmd.c_str(), WSTOPSIG(status));       // 被信号暂停执行
      }
    } else {
      ERROR(
        "Run system(<%s>) failed: <%s>",
        check_internet_cmd.c_str(), strerror(errno));
    }
  }
  return is_internet;
}

void Connector::Connect(
  const std::shared_ptr<ConnectorSrv::Request> request,
  std::shared_ptr<ConnectorSrv::Response> response)
{
  try {
    response->connected = this->DoConnect(
      request->wifi_name, request->wifi_password, request->provider_ip);
    response->code = this->srv_code_;
  } catch (const std::exception & e) {
    WARN("Connect service is failed: <%s>", e.what());
  }
}

bool Connector::Interaction(const uint16_t & _id)
{
  this->CtrlAudio(_id);
  this->CtrlLed(_id);
  return true;
}

void Connector::APPSendWiFiCallback(
  const protocol::msg::WifiInfo::SharedPtr msg)
{
  INFO("receive wifi info from app, connect wifi");
  this->notify_to_app_msg_.ssid = msg->ssid;
  this->notify_to_app_msg_.ip = msg->ip;
  if (msg->ssid.empty() || msg->ip.empty()) {
    INFO("receive wifi info from app, ssid or ip is empty");
    this->notify_to_app_msg_.code = 2001;
    this->notify_to_app_pub_->publish(this->notify_to_app_msg_);
    return;
  }
  if (this->DoConnect(msg->ssid, msg->pwd, msg->ip)) {
    // 连网成功, 写文件
    this->WriteToFile(msg->ssid, msg->ip, msg->type);
    this->notify_to_app_msg_.code = 2000;
  } else {
    INFO("receive wifi info from app, connect wifi failed");
    this->notify_to_app_msg_.code = 2001;  // 连网失败
  }
  this->notify_to_app_msg_.code = this->connect_code;
  this->notify_to_app_pub_->publish(this->notify_to_app_msg_);
}

bool Connector::WriteToFile(
  const std::string ssid,
  const std::string ip,
  const std::string type)
{
  // 将ssid，ip，type写入到文件中
  try {
    INFO(
      "Save WiFi :\n\tprovider = %s\n\tname = %s\n\ttype = %s",
      ip.c_str(),
      ssid.c_str(),
      type.c_str());
    toml::value wifi_toml;
    if (cyberdog::common::CyberdogToml::ParseFile(
        this->wifi_record_dir_.c_str(), wifi_toml))
    {
      std::string old_provider = toml::find_or(wifi_toml, "wifi", ssid, "provider", "");
      if (old_provider.empty()) {
        toml::value wifi;
        wifi["type"] = type;
        wifi["provider"] = ip;
        wifi_toml["wifi"][ssid] = wifi;
      } else {
        wifi_toml["wifi"][ssid]["type"] = type;
        wifi_toml["wifi"][ssid]["provider"] = ip;
      }
      if (!cyberdog::common::CyberdogToml::WriteFile(this->wifi_record_dir_, wifi_toml)) {
        WARN("WiFi  record file does not have wifi key, write failed");
        return false;
      }
    } else {
      ERROR(
        "Toml WiFi config file is not in toml format, config file dir:\n%s",
        this->wifi_record_dir_.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    ERROR("WriteToFile()-> Save WiFi is error:%s", e.what());
    return false;
  }
}
void Connector::BtStatusCallback(
  const protocol::msg::BluetoothStatus::SharedPtr msg)
{
  if (msg->connectable == 1) {
    INFO_MILLSECONDS(3000, " bluetooth connect(app and cyberdog)");
    if (this->touch_efficient_ == false) {
      // touch没有长按，关闭广播
      int count = 0;
      while (!this->CtrlAdvertising(false) && count < 3) {
        sleep(3);
        count++;
      }
    }
  } else if (msg->connectable == 0) {
    // 未连接状态，开起广播
    INFO_MILLSECONDS(3000, " bluetooth is not connect(app and cyberdog)");
    this->CtrlAdvertising(true);
  }
}
}   // namespace interaction
}   // namespace cyberdog
