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
#ifndef CONNECTOR__UPLOADER_LOG_HPP_
#define CONNECTOR__UPLOADER_LOG_HPP_

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

#include <std_srvs/srv/trigger.hpp>

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
/*! \file       uploader_log.hpp
    \brief      上传日志模块。
    \details    创建及初始化上传日志模块。。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class UploaderLog final : public rclcpp::Node
{
  using TriggerSrv = std_srvs::srv::Trigger;      /*!< [service 类型]日志上传 服务 */

public:
  explicit UploaderLog(const std::string &);
  ~UploaderLog();
  bool Init();                                    /*!< 初始化 */

private:
  void Uploader(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response);                  /*!< 上传日志 */

private:
  std::shared_ptr<LcmLogUploader> lcm_log_ptr_ {nullptr};             /*!< lcm 日志 */
  rclcpp::Service<TriggerSrv>::SharedPtr lcm_log_service_ {nullptr};  /*!< [服务端]日志 */
  rclcpp::CallbackGroup::SharedPtr lcm_log_cb_group_ {nullptr};       /*!< [回调组] service */
};  // class Connector
}  // namespace interaction
}  // namespace cyberdog
#endif  // CONNECTOR__UPLOADER_LOG_HPP_
