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
#ifndef CONNECTOR__CTRL_CAMERA_HPP_
#define CONNECTOR__CTRL_CAMERA_HPP_

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>

#include <protocol/srv/camera_service.hpp>

#include <string>

namespace cyberdog
{
namespace interaction
{
/*! \file       ctrl_camera.hpp
    \brief      相机控制模块。
    \details    创建及初始化相机控制模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class CtrlCamera final : public rclcpp::Node
{
  using CameraSrv = protocol::srv::CameraService;           /*!< [service 类型]相机 驱动服务 */

public:
  explicit CtrlCamera(const std::string name);
  ~CtrlCamera();
  uint ControlCamera(uint8_t _command);                     /*!< 控制摄像头 */

private:
  bool Init();                                              /*!< 初始化 */

private:
  uint16_t image_width_ {4208};                             /*!< 图像宽 */
  uint16_t image_height_ {3120};                            /*!< 图像高 */
  uint16_t image_fps_ {10};                                 /*!< 图像频率 */
  int wait_for_service_timeout_s {3};                       /*!< 等待 service 出现 */
  int wait_for_service_response_timeout_s {10};             /*!< 等待 service 响应 */
  rclcpp::Client<CameraSrv>::SharedPtr client_ {nullptr};   /*!< [客户端]相机 灯驱动 */
};  // class CtrlCamera
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__CTRL_CAMERA_HPP_
