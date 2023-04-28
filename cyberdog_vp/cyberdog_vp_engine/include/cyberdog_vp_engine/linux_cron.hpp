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
#ifndef CYBERDOG_VP_ENGINE__LINUX_CRON_HPP_
#define CYBERDOG_VP_ENGINE__LINUX_CRON_HPP_

#include <string>
#include <vector>

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       linux_cron.hpp
    \brief      周期任务模块。
    \details    负责周期任务的注册。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        注册表有效。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class LinuxCron
{
public:
  LinuxCron();
  ~LinuxCron() {}

private:
  const std::vector<std::string> CRONTAB_EXAMPLE = {
    "# Example of job definition:",
    "# .---------------- minute (0 - 59)",
    "# |  .------------- hour (0 - 23)",
    "# |  |  .---------- day of month (1 - 31)",
    "# |  |  |  .------- month (1 - 12) OR jan,feb,mar,apr ...",
    "# |  |  |  |  .---- day of week (0 - 6) (Sunday=0 or 7) OR sun,mon,tue,wed,thu,fri,sat",
    "# |  |  |  |  |",
    "# *  *  *  *  * command to be executed"
  };                                              /*!< crontab注释 */
  std::string logger_ {""};                       /*!< 日志 */
  std::string registry_fil_ {""};                 /*!< 配置注册表 */
  std::string crontab_file_ {""};                 /*!< 注册文件 */

public:
  bool Init();                                    /*!< 初始化 */
  bool UpdateCrontabFile(
    const std::vector<std::string> &,
    const bool _create = false);                  /*!< 更新 Crontab 文件 */
  bool GetNowCrontab(
    std::vector<std::string> &);                  /*!< 获取当前 Crontab 任务列表 */
  bool Registration(
    const std::string &,
    const std::string &);                         /*!< 注册 */
  bool Cancellation(const std::string &);         /*!< 注销 */
};  // class LinuxCron
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__LINUX_CRON_HPP_
