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
#ifndef CYBERDOG_VP_ENGINE__MODULE_HPP_
#define CYBERDOG_VP_ENGINE__MODULE_HPP_

#include <string>
#include <vector>

#include "cyberdog_vp_engine/common.hpp"
#include "cyberdog_vp_engine/base.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       module.hpp
    \brief      模块编程模块。
    \details    负责模块编程，生成模块文件以便任务编程过程中调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        相互作用有效。
    \bug        模块操作尚待调试
    \warning    留意注册表的稳定性
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Module : public Base
{
public:
  Module();
  ~Module();

private:
  std::string logger_ {""};                       /*!< 日志 */

private:
  bool InitData();                                /*!< 初始化数据 */
  bool JudgeModulesUniquenes(
    const OperateMsg &,
    const std::string _neglect = "");             /*!< 判断模块唯一性 */
  bool JudgeModulesLoop(
    const OperateMsg &,
    const std::vector<std::string> &);            /*!< 判断模块回路 */
  bool Build(
    const OperateMsg &, std::string &,
    std::vector<std::string> &);                  /*!< 构建模块 */
  bool DeleteTheModule(const OperateMsg &);       /*!< 删除模块 */
  bool ExecuteRequest(
    const OperateMsg &, GRPCMsg &);               /*!< 执行请求 */
};  // class Module
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__MODULE_HPP_
