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
#ifndef CYBERDOG_VP_ENGINE__LINUX_AT_HPP_
#define CYBERDOG_VP_ENGINE__LINUX_AT_HPP_

#include <string>
#include <map>

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       linux_at.hpp
    \brief      单次任务模块。
    \details    负责单次任务的注册。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        注册表有效。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class LinuxAt
{
  using TaskAtMap = std::map<std::string /*task_id*/, std::string /*at_id*/>;

public:
  LinuxAt();
  ~LinuxAt() {}

private:
  std::string logger_ {""};                       /*!< 日志 */
  std::string gerp_info_ {""};                    /*!< 查找信息 */
  std::string registry_fil_ {""};                 /*!< 配置注册表 */

public:
  bool Init();                                    /*!< 初始化 */
  bool GetTaskAtMap(TaskAtMap &);                 /*!< 获取任务列表 */
  bool Registration(
    const std::string &,
    const std::string &);                         /*!< 注册 */
  bool Cancellation(
    const std::string &);                         /*!< 注销 */
};  // class LinuxAt
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__LINUX_AT_HPP_
