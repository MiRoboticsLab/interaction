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
#include <vector>

#include "cyberdog_vp_abilityset/cyberdog.hpp"

namespace cyberdog_visual_programming_abilityset
{
Cyberdog::Cyberdog(std::string _task_id, std::string _namespace, bool _ros, std::string _parameters)
: Base(std::string(__FUNCTION__)),
  ros_(_ros)
{
  try {
    LOGGER_MAIN_INSTANCE("cyberdog_vp_abilityset");
    INFO(
      "%s(%s, %s, %d, %s)", std::string(__FUNCTION__).c_str(),
      _task_id.c_str(), _namespace.c_str(), _ros, _parameters.c_str());
    if (this->ros_) {
      INFO("szwh @ happy Ah Fu.");
      INFO("Init ros ...");
      std::vector<std::string> parameters;
      auto GetVector = [&]() {
          std::stringstream parameters_str;
          parameters_str.str(_parameters);
          std::string elems;
          while (std::getline(parameters_str, elems, ' ')) {
            parameters.push_back(elems);
          }
        };
      GetVector();
      if (parameters.empty()) {
        rclcpp::init(0, nullptr);
      } else {
        int argc = parameters.size();
        char ** argv = new char *[argc];
        for (int i = 0; i < argc; i++) {
          size_t size = parameters.at(i).size() + 1;
          argv[i] = new char[size];
          // strcpy(argv[i], parameters.at(i).c_str());
          snprintf(argv[i], size, "%s", parameters.at(i).c_str());
        }
        rclcpp::init(argc, argv);
      }
    }
    this->task_id_ = _task_id;
    // visual programming abilityset immortal
    std::string name = "vpa_" + this->task_id_ + "_immortal_node";
    this->node_immortal_ptr_ = rclcpp::Node::make_shared(name, _namespace);
    name = "vpa_" + this->task_id_ + "_mortal_node";    // visual programming abilityset mortal
    this->node_mortal_ptr_ = rclcpp::Node::make_shared(name, _namespace);
    if (!this->Start()) {
      if (this->ros_) {
        rclcpp::shutdown();
      }
      // exit(-1);
    }
  } catch (const std::exception & e) {
    ERROR("%s Creating object failed, %s", this->logger_.c_str(), e.what());
  }
}

bool Cyberdog::Start()
{
  try {
    std::string params_pkg_dir = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    std::string node_config_dir = params_pkg_dir + "/config/abilityset.toml";
    toml::value params_toml;
    if (!JudgeToml(node_config_dir) ||
      !this->GetToml(node_config_dir, params_toml))
    {
      return false;
    }
    if (this->Init(
        this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, params_toml))
    {
      this->task_.InitDependent(
        std::bind(&Cyberdog::Shutdown, this, std::placeholders::_1),
        std::bind(&Audio::OnlinePlay, &this->audio_, std::placeholders::_1, std::placeholders::_2));
      this->motion_.InitDependent(
        std::bind(
          &Audio::OnlinePlay, &this->audio_,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &Audio::OfflinePlay, &this->audio_,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &Odometer::IfCumulativeDistance, &this->odometer_,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        std::bind(&Task::GetProcessor, &this->task_, std::placeholders::_1));
      this->navigation_.InitDependent(
        std::bind(
          &Motion::Turn, &this->motion_,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &Audio::OnlinePlay, &this->audio_,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &Audio::OnlineInstantlyPlay, &this->audio_,
          std::placeholders::_1, std::placeholders::_2));
      this->skeleton_.InitDependent(
        std::bind(
          &Audio::OnlinePlay, &this->audio_,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &Audio::OnlineInstantlyPlay, &this->audio_,
          std::placeholders::_1, std::placeholders::_2));

      std::thread spin_thread([&] {
          Debug("Spin ...");
          rclcpp::executors::MultiThreadedExecutor cyberdog_vpa_exec;
          cyberdog_vpa_exec.add_node(this->node_immortal_ptr_);
          cyberdog_vpa_exec.add_node(this->node_mortal_ptr_);
          cyberdog_vpa_exec.spin();
        });
      spin_thread.detach();
      std::this_thread::sleep_for(std::chrono::seconds(2));  // 等待总线注册成功
      return true;
    }
    Error("%s Init failed.", this->logger_.c_str());
  } catch (const std::exception & e) {
    Error("%s Init data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Cyberdog::SetData(const toml::value & _params_toml)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
    return this->network_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->follow_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->motion_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->navigation_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->task_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->train_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->personnel_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->gesture_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->skeleton_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->audio_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->bms_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->led_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->touch_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->imu_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->odometer_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->gps_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->lidar_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->tof_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml) &&
           this->ultrasonic_.Init(
      this->task_id_, this->node_immortal_ptr_, this->node_mortal_ptr_, _params_toml)
    ;
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

bool Cyberdog::SetMechanism(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

void Cyberdog::Shutdown(bool _exit)
{
  try {
    Info(
      "%s Task %s shutdown ...",
      this->logger_.c_str(),
      this->task_id_.c_str());
    this->led_.Freed(SrvLedExecute::Request::HEAD_LED);
    this->led_.Freed(SrvLedExecute::Request::TAIL_LED);
    this->led_.Freed(SrvLedExecute::Request::MINI_LED);
    this->skeleton_.TurnOffRecognition();
    this->gesture_.TurnOffRecognition();
    this->navigation_.CancelNavigation();
    this->personnel_.face_.CancelRecognize();
    if (this->ros_) {
      Info(
        "%s Task %s shutdown ros ...",
        this->logger_.c_str(),
        this->task_id_.c_str());
      rclcpp::shutdown();
    }
  } catch (const std::exception & e) {
    Error(
      "%s Task %s shutdown ros is failed. \n%s",
      this->logger_.c_str(),
      this->task_id_.c_str(),
      e.what());
  }
  Info(
    "%s Task %s exit(0) ...",
    this->logger_.c_str(),
    this->task_id_.c_str());
  if (_exit) {
    exit(0);
  }
}

void Cyberdog::SetLog(const bool _log)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s)", std::string(_log ? "true" : "false").c_str());
  INFO("%s %s ...", this->logger_.c_str(), funs.c_str());
  if (_log) {
    this->log_ptr_->On();
  } else {
    this->log_ptr_->Off();
  }
  this->network_.SetLog(_log);
  this->follow_.SetLog(_log);
  this->motion_.SetLog(_log);
  this->navigation_.SetLog(_log);
  this->task_.SetLog(_log);
  this->train_.SetLog(_log);
  this->personnel_.SetLog(_log);
  this->gesture_.SetLog(_log);
  this->skeleton_.SetLog(_log);
  this->audio_.SetLog(_log);
  this->bms_.SetLog(_log);
  this->led_.SetLog(_log);
  this->touch_.SetLog(_log);
  this->imu_.SetLog(_log);
  this->odometer_.SetLog(_log);
  this->gps_.SetLog(_log);
  this->lidar_.SetLog(_log);
  this->tof_.SetLog(_log);
  this->ultrasonic_.SetLog(_log);
}

State Cyberdog::Ready(const uint16_t _timeout)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT("(%d)", _timeout);
  INFO("%s %s ...", this->logger_.c_str(), funs.c_str());
  try {
    std::chrono::time_point<std::chrono::system_clock> deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(std::abs(_timeout));
    while (rclcpp::ok() && std::chrono::system_clock::now() < deadline) {
      if (this->task_.GetProcessor(Processor::task)) {
        return this->GetState(funs, StateCode::success);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return this->GetState(funs, StateCode::timeout);
  } catch (const std::exception & e) {
    Error(
      "%s %s is failed. \n%s",
      this->logger_.c_str(), funs.c_str(), e.what());
  }
  return this->GetState(funs, StateCode::fail);
}
}   // namespace cyberdog_visual_programming_abilityset
