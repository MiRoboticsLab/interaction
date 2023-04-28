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
#ifndef CYBERDOG_VP_ENGINE__BACKEND_MESSAGE_HPP_
#define CYBERDOG_VP_ENGINE__BACKEND_MESSAGE_HPP_

#include <string>
#include <unordered_map>

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       backend_message.hpp
    \brief      消息模块。
    \details    负责后端消息的解析及审核。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        注册表有效。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class BackendMessage
{
public:
  explicit BackendMessage(const std::string & msg);
  ~BackendMessage() {}
  void getRobotendMsg(GRPCMsg &);                 /*!< 获取机器人端消息 */
  using CommonEnum = enum
  {
    efficient = 0,                                /*!< 有效 */
    invalid,                                      /*!< 无效json */
    code,                                         /*!< 状态码 */
    message,                                      /*!< 消息 */
    request_id,                                   /*!< 请求id */
    data,                                         /*!< 数据 */
    // type,                                      /*!< 类型 */
    // list,                                      /*!< 列表 */
  };
  const std::unordered_map<CommonEnum, std::string> keys = {
    {CommonEnum::code, "code"},                   /*!< 状态码 */
    {CommonEnum::message, "message"},             /*!< 消息 */
    {CommonEnum::request_id, "request_id"},       /*!< 请求id */
    {CommonEnum::data, "data"},                   /*!< 数据 */
    // {CommonEnum::type, "type"},                /*!< 类型 */
    // {CommonEnum::list, "list"},                /*!< 列表 */
  };
  using HttpCode = enum
  {
    Success = 200,                                /*!< 成功 */
    AccessTimeout = 400,                          /*!< 访问超时 */
    UnauthorizedAccess = 401,                     /*!< 拒绝访问 */
    AccessDenied = 403,                           /*!< 未经授权访问 */
    ResourceDoesNotExist = 404,                   /*!< 资源不存在 */
    NotSupported = 405,                           /*!< 不支持当前请求方法 */
    ServerAbnormally = 500,                       /*!< 服务器运行异常 */
    ParameterEmpty = 10001,                       /*!< 参数不能为空 */
  };
  const std::unordered_map<int, std::string> errors = {
    {HttpCode::Success, "success"},
    {HttpCode::AccessTimeout, "Access Timeout"},
    {HttpCode::UnauthorizedAccess, "Access Denied"},
    {HttpCode::AccessDenied, "Unauthorized Access"},
    {HttpCode::ResourceDoesNotExist, "Resource does not exist"},
    {HttpCode::NotSupported, "The current request method is not supported"},
    {HttpCode::ServerAbnormally, "The server is running abnormally"},
    {HttpCode::ParameterEmpty, "The parameter cannot be empty"},
  };
  CommonEnum state_;                              /*!< 状态 */
  std::string describe_ {""};                     /*!< 描述 */
  BackendMsg backend_;                            /*!< 后端消息 */
};  // class BackendMessage
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__BACKEND_MESSAGE_HPP_
