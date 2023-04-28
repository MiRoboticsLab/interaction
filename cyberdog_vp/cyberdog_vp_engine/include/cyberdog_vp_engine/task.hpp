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
#ifndef CYBERDOG_VP_ENGINE__TASK_HPP_
#define CYBERDOG_VP_ENGINE__TASK_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "cyberdog_vp_engine/common.hpp"
#include "cyberdog_vp_engine/base.hpp"
#include "cyberdog_vp_engine/linux_at.hpp"
#include "cyberdog_vp_engine/linux_cron.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       task.hpp
    \brief      任务编程模块。
    \details    负责任务编程，生成任务文件以便任务注册过程中调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        相互作用有效。
    \bug        任务操作尚待调试
    \bug        任务依赖模块的筛查尚待调试
    \bug        依赖模块的任务的编程尚待调试
    \warning    小心注册表的更新及任务的删除操作
    \note       留意注册表的稳定性
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Task : public Base
{
public:
  Task();
  ~Task();

private:
  const std::unordered_map<std::string,
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>>> STATE_FLOW =
  {
    {OperateMsg::MODE_SINGLE, {
        {OperateMsg::STATE_NULL, {
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN}}},
        {OperateMsg::STATE_ERROR, {
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_ERROR}}},
        {OperateMsg::STATE_WAIT_RUN, {
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_RUN, OperateMsg::STATE_RUN_WAIT}}},
        {OperateMsg::STATE_RUN_WAIT, {
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_START, OperateMsg::STATE_RUN}}},
        {OperateMsg::STATE_RUN, {
            {OperateMsg::OPERATE_RUN, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_START, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_RECOVER, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_SUSPEND, OperateMsg::STATE_SUSPEND},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_STOP, OperateMsg::STATE_SHUTDOWN}}},
        {OperateMsg::STATE_SUSPEND, {
            {OperateMsg::OPERATE_SUSPEND, OperateMsg::STATE_SUSPEND},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_SUSPEND},
            {OperateMsg::OPERATE_RECOVER, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_STOP, OperateMsg::STATE_SHUTDOWN}}},
        {OperateMsg::STATE_SHUTDOWN, {
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_RUN, OperateMsg::STATE_RUN_WAIT}}}
      }},
    {OperateMsg::MODE_CYCLE, {
        {OperateMsg::STATE_NULL, {
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN}}},
        {OperateMsg::STATE_ERROR, {
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_ERROR}}},
        {OperateMsg::STATE_WAIT_RUN, {
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_RUN, OperateMsg::STATE_RUN_WAIT}}},
        {OperateMsg::STATE_RUN_WAIT, {
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_START, OperateMsg::STATE_RUN}}},
        {OperateMsg::STATE_RUN, {
            {OperateMsg::OPERATE_RUN, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_START, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_RECOVER, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_SUSPEND, OperateMsg::STATE_SUSPEND},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_STOP, OperateMsg::STATE_RUN_WAIT}}},
        {OperateMsg::STATE_SUSPEND, {
            {OperateMsg::OPERATE_SUSPEND, OperateMsg::STATE_SUSPEND},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_SUSPEND},
            {OperateMsg::OPERATE_RECOVER, OperateMsg::STATE_RUN},
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_STOP, OperateMsg::STATE_RUN_WAIT}}},
        {OperateMsg::STATE_SHUTDOWN, {
            {OperateMsg::OPERATE_SHUTDOWN, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_DEBUG, OperateMsg::STATE_RUN_WAIT},
            {OperateMsg::OPERATE_SAVE, OperateMsg::STATE_WAIT_RUN},
            {OperateMsg::OPERATE_INQUIRY, OperateMsg::STATE_SHUTDOWN},
            {OperateMsg::OPERATE_DELETE, OperateMsg::STATE_NULL},
            {OperateMsg::OPERATE_RUN, OperateMsg::STATE_RUN_WAIT}}}
      }}
  };                                              /*!< 状态流转 */
  std::string logger_ {""};                       /*!< 日志 */
  rclcpp::Publisher<OperateMsg>::SharedPtr
    task_option_pub_ {nullptr};                   /*!< [发布器]OperateMsg 消息:操作任务 */
  LinuxAt at_;                                    /*!< at 指令 */
  LinuxCron cron_;                                /*!< cron 指令 */

private:
  bool InitList();                                /*!< 初始化列表 */
  bool InitData();                                /*!< 初始化数据 */
  bool Build(
    const OperateMsg &, std::string &,
    std::vector<std::string> &);                  /*!< 构建任务 */
  bool ExecuteRequest(
    const OperateMsg &, GRPCMsg &);               /*!< 执行请求 */
  bool GetFlowState(
    const OperateMsg &,
    std::string &,
    std::string &,
    std::string &,
    std::string &);                               /*!< 获取流转状态 */
  bool GetLogFile(
    const std::string &,
    std::string &);                               /*!< 获取日志文件 */
};  // class Task
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__TASK_HPP_
