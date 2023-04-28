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
#ifndef CYBERDOG_VP_ENGINE__FRONTEND_MESSAGE_HPP_
#define CYBERDOG_VP_ENGINE__FRONTEND_MESSAGE_HPP_

#include <string>
#include <unordered_map>

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       frontend_message.hpp
    \brief      消息模块。
    \details    负责前端消息的解析及审核。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        注册表有效。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class FrontendMessage
{
public:
  explicit FrontendMessage(const std::string & msg);
  ~FrontendMessage() {}
  void getRobotendMsg(GRPCMsg &);                 /*!< 获取机器人端消息 */
  using CommonEnum = enum
  {
    efficient = 0,                                /*!< 有效 */
    invalid,                                      /*!< 1:无效json */
    type,                                         /*!< 2:类型 */
    id,                                           /*!< 3:id */
    target_id,                                    /*!< 4:目标 id */
    describe,                                     /*!< 5:描述 */
    style,                                        /*!< 6:样式 */
    operate,                                      /*!< 7:操作 */
    mode,                                         /*!< 8:模式 */
    condition,                                    /*!< 9:条件 */
    body,                                         /*!< 10:主体 */
    /*!< 11~20 */
  };
  const std::unordered_map<CommonEnum, std::string> keys = {
    {CommonEnum::type, "type"},                   /*!< 类型键 */
    {CommonEnum::id, "id"},                       /*!< id */
    {CommonEnum::target_id, "target_id"},         /*!< target_id */
    {CommonEnum::describe, "describe"},           /*!< 描述 */
    {CommonEnum::style, "style"},                 /*!< 描述 */
    {CommonEnum::operate, "operate"},             /*!< 操作 */
    {CommonEnum::mode, "mode"},                   /*!< 模式 */
    {CommonEnum::condition, "condition"},         /*!< 条件 */
    {CommonEnum::body, "body"}                    /*!< 主体 */
  };
  CommonEnum state_;                              /*!< 状态 */
  std::string describe_ {""};                     /*!< 描述 */
  OperateMsg frontend_;                           /*!< 前端消息 */
};  // class FrontendMessage
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__FRONTEND_MESSAGE_HPP_
