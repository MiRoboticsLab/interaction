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
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "connector/connector.hpp"
#include "connector/ctrl_audio.hpp"
#include "connector/ctrl_led.hpp"
#include "connector/ctrl_wifi.hpp"
#include "connector/ctrl_camera.hpp"

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("main@connector");
  INFO("Connector node started");
  rclcpp::init(argc, argv);
  pybind11::scoped_interpreter guard{};
  auto connector = std::make_shared<cyberdog::interaction::Connector>("connector");
  auto ctrl_audio = std::make_shared<cyberdog::interaction::CtrlAudio>("connector_ctrl_audio");
  auto ctrl_camera = std::make_shared<cyberdog::interaction::CtrlCamera>("connector_ctrl_camera");
  auto ctrl_led = std::make_shared<cyberdog::interaction::CtrlLed>("connector_ctrl_led");
  auto ctrl_wifi = std::make_shared<cyberdog::interaction::CtrlWifi>("connector_ctrl_wifi");
  if (!connector->Init(
      std::bind(
        &cyberdog::interaction::CtrlAudio::ControlAudio,
        ctrl_audio, std::placeholders::_1),
      std::bind(
        &cyberdog::interaction::CtrlCamera::ControlCamera,
        ctrl_camera, std::placeholders::_1),
      std::bind(
        &cyberdog::interaction::CtrlLed::ControlLed,
        ctrl_led, std::placeholders::_1),
      std::bind(
        &cyberdog::interaction::CtrlWifi::ControlWifi,
        ctrl_wifi, std::placeholders::_1, std::placeholders::_2)
    )) {exit(-1);}
  INFO("Connector node is running ...");
  rclcpp::executors::MultiThreadedExecutor exec_;
  exec_.add_node(connector->get_node_base_interface());
  exec_.add_node(ctrl_audio->get_node_base_interface());
  exec_.add_node(ctrl_camera->get_node_base_interface());
  exec_.add_node(ctrl_led->get_node_base_interface());
  exec_.add_node(ctrl_wifi->get_node_base_interface());
  exec_.spin();
  rclcpp::shutdown();
  return 0;
}
