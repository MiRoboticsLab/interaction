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
#include "connector/ctrl_camera.hpp"

namespace cyberdog
{
namespace interaction
{
CtrlCamera::CtrlCamera(const std::string name)
: Node(name)
{
  INFO("Creating [CtrlCamera] object(node)");
  if (!this->Init()) {exit(-1);}
}

CtrlCamera::~CtrlCamera()
{
  INFO("Destroy [CtrlCamera] object(node)");
  this->ControlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
}

bool CtrlCamera::Init()
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
    this->image_width_ = toml::find_or(
      params_toml_, "connector", "initialization", "image", "width", 2560);
    this->image_height_ = toml::find_or(
      params_toml_, "connector", "initialization", "image", "height", 1920);
    this->image_fps_ = toml::find_or(
      params_toml_, "connector", "initialization", "image", "fps", 10);
    this->client_ = this->create_client<CameraSrv>(
      toml::find<std::string>(
        params_toml_, "connector", "initialization", "service", "camera"));
    this->wait_for_service_timeout_s = toml::find_or(
      params_toml_, "connector", "initialization", "timeout_s", "wait_for_service",
      this->wait_for_service_timeout_s);
    this->wait_for_service_response_timeout_s = toml::find_or(
      params_toml_, "connector", "initialization", "timeout_s", "wait_for_camera_service_response",
      this->wait_for_service_response_timeout_s);
  } catch (const std::exception & e) {
    ERROR("Init data failed: <%s>", e.what());
    return false;
  }
  return true;
}

uint CtrlCamera::ControlCamera(uint8_t _action)
{
  /*
   * return:
   * 0: 控制相机成功
   * 1: 客户端在请求服务出现时被打断
   * 2: 等待服务出现（启动）超时
   * 3: 请求相机模块超时或延迟
   * 4: 控制相机失败
   */
  auto single_control = [&](uint8_t _command) -> uint {
      auto request = std::make_shared<CameraSrv::Request>();
      std::string operate;
      switch (_command) {
        case CameraSrv::Request::SET_PARAMETERS:
          operate = "set parameters";
          break;
        case CameraSrv::Request::TAKE_PICTURE:
          operate = "task picture";
          break;
        case CameraSrv::Request::START_RECORDING:
          operate = "start recoreing";
          break;
        case CameraSrv::Request::STOP_RECORDING:
          operate = "stop recoreing";
          break;
        case CameraSrv::Request::GET_STATE:
          operate = "get state";
          break;
        case CameraSrv::Request::DELETE_FILE:
          operate = "delete file";
          break;
        case CameraSrv::Request::GET_ALL_FILES:
          operate = "get all files";
          break;
        case CameraSrv::Request::START_LIVE_STREAM:
          operate = "start live stream";
          break;
        case CameraSrv::Request::STOP_LIVE_STREAM:
          operate = "stop live stream";
          break;
        case CameraSrv::Request::START_IMAGE_PUBLISH:
          operate = "start publist image";
          request->width = this->image_width_;
          request->height = this->image_height_;
          request->fps = this->image_fps_;
          break;
        case CameraSrv::Request::STOP_IMAGE_PUBLISH:
          operate = "stop publist image";
          break;
        default:
          operate = "undefined";
          break;
      }
      INFO("Control camera <%s>", operate.c_str());
      request->command = _command;
      request->args = "";
      if (!rclcpp::ok()) {
        WARN("Client interrupted while requesting for service to appear.");
        return 1;
      }
      if (!this->client_->wait_for_service(std::chrono::seconds(this->wait_for_service_timeout_s)))
      {
        WARN("Waiting for service to appear(start) timeout.");
        return 2;
      }
      auto result = this->client_->async_send_request(request);
      std::future_status status =
        result.wait_for(std::chrono::seconds(this->wait_for_service_response_timeout_s));
      if (status != std::future_status::ready) {
        WARN("Request camera module timedout or deferred.");
        return 3;
      }
      auto response_ptr = result.get();
      if (response_ptr->result == CameraSrv::Response::RESULT_SUCCESS) {
        INFO(
          "Control camera module succeeded, %d, <%s>", static_cast<int>(response_ptr->result),
          response_ptr->msg.c_str());
        return 0;
      } else {
        WARN(
          "Control camera module failed, %d, <%s>", static_cast<int>(response_ptr->result),
          response_ptr->msg.c_str());
        return 4;
      }
    };
  uint ret = 0;
  if (_action == CameraSrv::Request::START_IMAGE_PUBLISH) {
    ret = single_control(CameraSrv::Request::STOP_IMAGE_PUBLISH);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return (ret == 0) ? single_control(CameraSrv::Request::START_IMAGE_PUBLISH) : ret;
  } else {
    return single_control(_action);
  }
}

}   // namespace interaction
}   // namespace cyberdog
